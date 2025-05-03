#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LittleFS.h>
#include <PCF8574.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------------------------------
// Constants
// -------------------------------
static const uint8_t Zone           = 4;
static const unsigned long WEATHER_INTERVAL = 600000; // 10 minutes

PCF8574 pcfIn(0x22, 4, 15);   //  Inputs
PCF8574 pcfOut(0x24, 4, 15);  //  Relays (P0 - P5)

#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Wi‑Fi & WebServer
WiFiManager wifiManager;
WebServer   server(80);
WiFiClient  client;

// Solenoid channels (PCF8574 pins)
const uint8_t valveChannel[Zone] = { P0, P1, P2, P3 };
const uint8_t mainsChannel           = P4;
const uint8_t tankChannel            = P5;

// On‑board LED & tank sensor
const int LED_PIN  = 2;
const int TANK_PIN = 36;

// -------------------------------
// Globals
// -------------------------------
String apiKey, city;
float  tzOffsetHours      = 0.0;
bool   rainDelayEnabled   = true;
bool   windDelayEnabled   = false;
bool   justUseTank        = false;
bool   justUseMains       = false;
float  windSpeedThreshold = 5.0f;

int    startHour[Zone]   = {0};
int    startMin [Zone]   = {0};
int    startHour2[Zone]  = {0};
int    startMin2 [Zone]  = {0};
int    durationMin[Zone] = {0};
bool   enableStartTime2[Zone] = {false};
bool   days[Zone][7]    = {{false}};

bool   zoneActive[Zone]         = {false};
unsigned long zoneStartMs[Zone] = {0};
bool pendingStart[Zone] = { false };

// how often to refresh the OLED (1 Hz)
const unsigned long SCREEN_REFRESH_MS = 1000;
unsigned long       lastScreenRefresh  = 0;
unsigned long lastWeatherUpdate  = 0;
const unsigned long weatherUpdateInterval = 3600000; // 1 hour
String cachedWeatherData;
float  lastRainAmount = 0.0f;   // mm in last checked period
bool   rainActive     = false;  // true while we have a rain delay
int    lastCheckedMinute[Zone] = { -1, -1, -1, -1 };

// -------------------------------
// Prototypes
// -------------------------------
void wifiCheck();
void loadConfig();
void saveConfig();
void loadSchedule();
void saveSchedule();
void updateCachedWeather();
String fetchWeather();
void HomeScreen();
void updateLCDForZone(int zone);
bool shouldStartZone(int zone);
bool hasDurationCompleted(int zone);
bool isTankLow();
void turnOnZone(int zone);
void turnOffZone(int zone);
void turnOnValveManual(int z);
void turnOffValveManual(int z);
void handleRoot();
void handleSubmit();
void handleSetup();
void handleConfigure();
String getDayName(int d);

// -------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin(4, 15); // I2C pins for KC868-A6
  Serial.println("Initializing Smart Irrigation KC868-A6...");

  for (uint8_t ch = P0; ch <= P5; ++ch) {
    pcfOut.pinMode(ch, OUTPUT);
    pcfOut.digitalWrite(ch, HIGH);  // OFF (active LOW)
    pcfIn .pinMode(ch, INPUT);
    pcfIn .digitalWrite(ch, HIGH);  // pull‑up
  }
  
    // —————— Initialize expanders ——————
  if (!pcfIn.begin()) {
    Serial.println("PCF8574 input init failed");
    while (true) delay(100);
  }
  if (!pcfOut.begin()) {
    Serial.println("PCF8574 output init failed");
    while (true) delay(100);
  }

  pinMode(LED_PIN, OUTPUT);

  // OLED Initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 init failed");
    while (true) delay(100);
  }
  delay(2000);

  // Filesystem, config & schedule
  LittleFS.begin();
  loadConfig();
  loadSchedule();

  display.display();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1.5);
  // Wi‑Fi Setup
  wifiManager.setTimeout(180);
  if (!wifiManager.autoConnect("ESPIrrigationAP")) {
    Serial.println("Failed to connect to WiFi. Restarting...");
    display.clearDisplay();
    display.setCursor(0,10);
    display.print("Connect To");
    display.setCursor(0,20);
    display.print("A6-Irrigation");
    display.setCursor(0,30);
    display.print("Goto: https://192.168.4.1");
    display.display();  
    ESP.restart();
  }

  // WiFi connected feedback
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Connected!");
    display.setCursor(0,20);
    display.print(WiFi.localIP().toString());
    display.display();          
    delay(3000);
  }

  // NTP setup with saved offset
  long gmtOffsetSec     = long(tzOffsetHours * 3600);
  long daylightOffsetSec = 0;
  configTime(gmtOffsetSec, daylightOffsetSec,
             "pool.ntp.org", "time.nist.gov");
  time_t now = time(nullptr);
  while (now < 1000000000) {
    delay(500);
    now = time(nullptr);
  }

  // OTA & Web Server
  ArduinoOTA.begin();
  ArduinoOTA.setHostname("A6-Irrigation");

  server.on("/",          HTTP_GET,  handleRoot);
  server.on("/submit",    HTTP_POST, handleSubmit);
  server.on("/setup",     HTTP_GET,  handleSetup);
  server.on("/configure", HTTP_POST, handleConfigure);
  server.on("/events", HTTP_GET, handleEvents);
  server.on("/clearevents", HTTP_POST, handleClearEvents);


  // Manual control for valves
  for (int i = 0; i < Zone; i++) {
    server.on(String("/valve/on/")  + i, HTTP_POST, [i]() {
      turnOnValveManual(i);
      server.send(200, "text/plain", "OK");
    });
    server.on(String("/valve/off/") + i, HTTP_POST, [i]() {
      turnOffValveManual(i);
      server.send(200, "text/plain", "OK");
    });
  }

  // Backlight toggle
  server.on("/toggleBacklight", HTTP_POST, [](){
    display.invertDisplay(true);
    delay(300);
    display.invertDisplay(false);
    server.send(200, "text/plain", "Backlight toggled");
  });

  server.begin();
}

void loop() {
  unsigned long now = millis();

  ArduinoOTA.handle();
  server.handleClient();
  wifiCheck();

  // Midnight reset of minute‑checks
  time_t nowTime = time(nullptr);
  struct tm* nowTm = localtime(&nowTime);
  if (nowTm->tm_hour == 0 && nowTm->tm_min == 0) {
    memset(lastCheckedMinute, -1, sizeof(lastCheckedMinute));
  }

  // — Determine if any zone is currently active (and record its index) —
  bool anyActive = false;
  int  activeZoneIndex = -1;
  for (int z = 0; z < Zone; ++z) {
    if (zoneActive[z]) {
      anyActive = true;
      activeZoneIndex = z;
      break;
    }
  }

  // ——— Scheduled actions with sequential‑on‑mains logic ———
  for (int z = 0; z < Zone; ++z) {
    if (shouldStartZone(z)) {
  if (!checkWindRain()) {
    pendingStart[z] = true;
    // record rain‐delay event for this zone
    logEvent(z, "RAIN_DELAY", "N/A", true);
    continue;
      }
      if (isTankLow()) {
        // mains only one at a time
        bool someone = false;
        for (int i = 0; i < Zone; ++i) {
          if (zoneActive[i]) { someone = true; break; }
        }
        if (someone) {
          pendingStart[z] = true;
          Serial.printf("Zone %d queued for sequential start\n", z+1);
        } else {
          turnOnZone(z);
        }
      } else {
        // tank full: parallel starts allowed
        turnOnZone(z);
      }
    }
    // auto‑stop when duration completes
    if (zoneActive[z] && hasDurationCompleted(z)) {
      turnOffZone(z);
    }
  }

  // — If nothing is running, start the next queued zone —
  {
    bool someone = false;
    for (int i = 0; i < Zone; ++i) {
      if (zoneActive[i]) { someone = true; break; }
    }
    if (!someone) {
      for (int z = 0; z < Zone; ++z) {
        if (pendingStart[z]) {
          Serial.printf("Starting queued Zone %d\n", z+1);
          pendingStart[z] = false;
          turnOnZone(z);
          break;
        }
      }
    }
  }

  // ——— OLED refresh at 1 Hz ———
  if (now - lastScreenRefresh >= SCREEN_REFRESH_MS) {
  lastScreenRefresh = now;

  if (rainActive) {
    RainScreen();
  }
  else if (anyActive) {
    HomeScreen();
  }
  else {
    HomeScreen();
  }
 }

  delay(500);
}

void wifiCheck() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    delay(1000);
    if (!wifiManager.autoConnect("ESPIrrAP")) {
      Serial.println("Reconnection failed.");
    } else {
      Serial.println("Reconnected.");
    }
  }
}

void printCurrentTime() {
  time_t now = time(nullptr);
  struct tm *tm = localtime(&now);
  Serial.printf("Current time: %02d:%02d:%02d\n", tm->tm_hour, tm->tm_min, tm->tm_sec);
}

void loadConfig() {
  File f = LittleFS.open("/config.txt","r");
  if (!f) return;
  apiKey            = f.readStringUntil('\n'); apiKey.trim();
  city              = f.readStringUntil('\n'); city.trim();
  tzOffsetHours     = f.readStringUntil('\n').toFloat();
  rainDelayEnabled  = (f.readStringUntil('\n').toInt() == 1);
  windSpeedThreshold= f.readStringUntil('\n').toFloat();
  windDelayEnabled  = (f.readStringUntil('\n').toInt() == 1);
  justUseTank       = (f.readStringUntil('\n').toInt() == 1);
  justUseMains      = (f.readStringUntil('\n').toInt() == 1);
  f.close();
}

void saveConfig() {
  File f = LittleFS.open("/config.txt","w");
  if (!f) return;
  f.println(apiKey);
  f.println(city);
  f.println(tzOffsetHours, 2);
  f.println(rainDelayEnabled ? "1" : "0");
  f.println(windSpeedThreshold, 1);
  f.println(windDelayEnabled ? "1" : "0");
  f.println(justUseTank  ? "1" : "0");
  f.println(justUseMains ? "1" : "0");
  f.close();
}

void loadSchedule() {
  File f = LittleFS.open("/schedule.txt", "r");
  if (!f) {
    Serial.println("Failed to open schedule file");
    return;
  }

  Serial.println("Loaded schedule:");
  for (int i = 0; i < Zone; i++) {
    String line = f.readStringUntil('\n');
    if (line.length() == 0) continue;

    int index = 0;
    int nextComma;

    // 1) startHour
    nextComma     = line.indexOf(',', index);
    startHour[i]  = line.substring(index, nextComma).toInt();
    index         = nextComma + 1;

    // 2) startMin
    nextComma     = line.indexOf(',', index);
    startMin[i]   = line.substring(index, nextComma).toInt();
    index         = nextComma + 1;

    // 3) startHour2
    nextComma      = line.indexOf(',', index);
    startHour2[i]  = line.substring(index, nextComma).toInt();
    index          = nextComma + 1;

    // 4) startMin2
    nextComma      = line.indexOf(',', index);
    startMin2[i]   = line.substring(index, nextComma).toInt();
    index          = nextComma + 1;

    // 5) durationMin
    nextComma        = line.indexOf(',', index);
    durationMin[i]   = line.substring(index, nextComma).toInt();
    index            = nextComma + 1;

    // 6) enableStartTime2
    nextComma            = line.indexOf(',', index);
    enableStartTime2[i]  = (line.substring(index, nextComma).toInt() == 1);
    index                 = nextComma + 1;

    // 7) days[7]
    for (int j = 0; j < 7; j++) {
      // each day is separated by a comma; for the last day, indexOf will return -1
      nextComma = line.indexOf(',', index);
      if (nextComma < 0) nextComma = line.length();
      days[i][j] = (line.substring(index, nextComma).toInt() == 1);
      index = nextComma + 1;
    }

    // Debug print
    Serial.printf(
      "Zone %d: %02d:%02d, %02d:%02d, %d mins, Days: %d%d%d%d%d%d%d\n",
      i+1,
      startHour[i],  startMin[i],
      startHour2[i], startMin2[i],
      durationMin[i],
      days[i][0], days[i][1], days[i][2], days[i][3],
      days[i][4], days[i][5], days[i][6]
    );
  }

  f.close();
}

void saveSchedule() {
  File f = LittleFS.open("/schedule.txt","w");
  if (!f) return;
  for (int i = 0; i < Zone; i++) {
    f.print(startHour[i]);   f.print(',');
    f.print(startMin [i]);   f.print(',');
    f.print(startHour2[i]);  f.print(',');
    f.print(startMin2 [i]);  f.print(',');
    f.print(durationMin[i]); f.print(',');
    f.print(enableStartTime2[i] ? '1' : '0');
    for (int d = 0; d < 7; d++) {
      f.print(',');
      f.print(days[i][d] ? '1' : '0');
    }
    f.println();
  }
  f.close();
}

void logEvent(int zone, const char* eventType, const char* source, bool rainDelayed) {
  File f = LittleFS.open("/events.csv", "a");
  if (!f) return;

  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  char buf[128];
  // Format: YYYY-MM-DD HH:MM:SS,Zone#,EVENT,SOURCE,RAIN_DELAY(Y/N)
  sprintf(buf,
    "%04d-%02d-%02d %02d:%02d:%02d,Zone%d,%s,%s,%s\n",
    t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
    t->tm_hour, t->tm_min, t->tm_sec,
    zone + 1,
    eventType,
    source,
    rainDelayed ? "Y" : "N"
  );
  f.print(buf);
  f.close();
}

void handleClearEvents() {
  // Delete the file (or truncate)
  if (LittleFS.exists("/events.csv")) {
    LittleFS.remove("/events.csv");
  }
  // Redirect back to the log page
  server.sendHeader("Location", "/events", true);
  server.send(302, "text/plain", "");
}

void updateCachedWeather() {
  unsigned long currentMillis = millis();
  if (cachedWeatherData == "" || (currentMillis - lastWeatherUpdate >= weatherUpdateInterval)) {
    cachedWeatherData = fetchWeather();
    lastWeatherUpdate = currentMillis;
  }
}

String fetchWeather() {
    HTTPClient http;
  http.setTimeout(5000);
  String url = "http://api.openweathermap.org/data/2.5/weather?id=" + city + "&appid=" + apiKey + "&units=metric";
  http.begin(client, url);
  int httpResponseCode = http.GET();
  String payload = "{}";
  if (httpResponseCode > 0) {
    payload = http.getString();
  } else {
    Serial.println("Error: Unable to fetch weather data.");
  }
  http.end();
  return payload;
}

void HomeScreen() {
  // 1) Refresh weather cache & parse
  updateCachedWeather();
  DynamicJsonDocument js(1024);
  if (!deserializeJson(js, cachedWeatherData)) {
    // okay
  }
  float temp = js["main"]["temp"].as<float>();
  int   hum  = js["main"]["humidity"].as<int>();

  // 2) Read tank level
  int raw = analogRead(TANK_PIN);
  int pct = map(raw, 0, 1023, 0, 100);
  bool low = raw < 150;
  const char* src = low ? "Main" : "Tank";

  // 3) Get current time & date
  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  int day = t->tm_mday;
  int mon = t->tm_mon + 1;

  // 4) Draw everything
  display.clearDisplay();

    // Line 1: time + date
  display.setTextSize(2);
  display.setCursor(0,  0);
  display.printf("%02d:%02d", t->tm_hour, t->tm_min);
  display.setTextSize(1.9);
  display.setCursor(75, 5);
  display.printf("%02d/%02d", day, mon);

  // Line 2: temp & humidity
  display.setCursor(0, 20);
  display.setTextSize(1.9);
  display.printf("Temp:%2.0fC Humidity:%02d%%", temp, hum);

  // Line 3: tank level & source
  display.setCursor(0, 33);
  display.setTextSize(1.9);
  display.printf("Tank:%3d%% (%s)", pct, src);

 // Line 4 (y=40): Zones 1 & 2
  for (int i = 0; i < 2; i++) {
    int x = (i == 0) ? 0 : 64;
    display.setCursor(x, 45);
    display.setTextSize(1.5);

    if (zoneActive[i]) {
      // compute remaining seconds
      unsigned long elapsed = (millis() - zoneStartMs[i]) / 1000;
      unsigned long total   = (unsigned long)durationMin[i] * 60;
      unsigned long rem     = (elapsed < total ? total - elapsed : 0);

      int remM = rem / 60;
      int remS = rem % 60;
      display.printf("Z%d:%02d:%02d", i+1, remM, remS);
    } else {
      display.print   ("Z");
      display.print   (i+1);
      display.print   (":Off ");
    }
  }

  // Line 4 (y=50): Zones 3 & 4
  for (int i = 2; i < 4; i++) {
    int x = (i == 2) ? 0 : 64;
    display.setTextSize(1);
    display.setCursor(x, 55);

    if (zoneActive[i]) {
      unsigned long elapsed = (millis() - zoneStartMs[i]) / 1000;
      unsigned long total   = (unsigned long)durationMin[i] * 60;
      unsigned long rem     = (elapsed < total ? total - elapsed : 0);

      int remM = rem / 60;
      int remS = rem % 60;
      display.printf("Z%d:%02d:%02d", i+1, remM, remS);
    } else {
      display.print   ("Z");
      display.print   (i+1);
      display.print   (":Off ");
    }
  }

  display.display();
}

void toggleBacklight() {
  // If your OLED display supports backlight control via GPIO, add that logic here.
  // Placeholder: Flash screen to simulate toggle
  display.invertDisplay(true);
  delay(300);
  display.invertDisplay(false);
}

void showMainDisplay() {
  HomeScreen();
}

void updateLCDForZone(int zone) {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  // only update once per second
  if (now - lastUpdate < 1000) return;
  lastUpdate = now;

  // compute elapsed, total and remaining seconds
  unsigned long elapsed = (now - zoneStartMs[zone]) / 1000;
  unsigned long total   = (unsigned long)durationMin[zone] * 60;
  unsigned long rem     = (elapsed < total ? total - elapsed : 0);

  // build lines
  String line1 = "Zone " + String(zone + 1)
               + " " + String(elapsed / 60) + ":"
               + (elapsed % 60 < 10 ? "0" : "")
               + String(elapsed % 60);

  String line2;
  if (elapsed < total) {
    line2 = String(rem / 60) + "m Remaining";
  } else {
    line2 = "Complete";
  }

  // clear once, draw both lines, then display
  display.clearDisplay();

  const uint8_t charW = 6;   // default font width in pixels
  const uint8_t charH = 8;   // default font height in pixels
  uint8_t maxCols = display.width() / charW;  // e.g. 128/6 = 21 chars

  // center each line in pixels
  int16_t x1 = ((int16_t)maxCols - (int16_t)line1.length()) / 2 * charW;
  int16_t x2 = ((int16_t)maxCols - (int16_t)line2.length()) / 2 * charW;

  // line 1 at y=0px, line 2 at y=charH
  display.setCursor(x1, 55);
  display.setTextSize(1.9);
  display.print(line1);

  display.setCursor(x2, charH);
  display.setTextSize(1.9);
  display.print(line2);

  display.display();
}

void showZoneDisplay(int zone) {
  updateLCDForZone(zone);
}

bool checkWindRain() {
  DynamicJsonDocument js(1024);
  if (deserializeJson(js, cachedWeatherData)) {
    Serial.println("weather parse error – allowing irrigation");
    rainActive = false;
    return true;
  }

  // 1) Rain check
  if (rainDelayEnabled && js.containsKey("rain")) {
    float rain1h = js["rain"]["1h"].as<float>();
    if (rain1h <= 0.0f) rain1h = js["rain"]["3h"].as<float>();

    if (rain1h > 0.0f) {
      lastRainAmount = rain1h;
      rainActive     = true;
      Serial.printf("Rain-delay active (%.2f mm)\n", rain1h);
      return false;
    }
  }
  // no rain blocking
  rainActive = false;

  // 2) Wind check (unchanged)
  if (windDelayEnabled && js.containsKey("wind")) {
    float windSpd = js["wind"]["speed"].as<float>();
    if (windSpd >= windSpeedThreshold) {
      Serial.printf("Skipping due to high wind: %.2f m/s\n", windSpd);
      return false;
    }
  }

  return true;
}

void RainScreen() {
  display.clearDisplay();

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("Rain Delay");

  display.setTextSize(1.5);
  display.setCursor(0, 20);
  display.printf("Last: %.2f mm", lastRainAmount);

  // Count delayed zones
  int delayed = 0;
  for (int i = 0; i < Zone; i++) {
    if (pendingStart[i]) delayed++;
  }
  display.setCursor(0, 40);
  display.printf("Zones delayed: %d", delayed);

  display.display();
}

bool shouldStartZone(int zone) {
  time_t now = time(nullptr);
  struct tm *t = localtime(&now);
  int wd = t->tm_wday;    // Weekday (0-6, Sunday=0)
  int hr = t->tm_hour;    // Hour (0-23)
  int mn = t->tm_min;     // Minute (0-59)

  // Skip if not a scheduled day for this zone
  if (!days[zone][wd]) return false;

  // Skip if we already checked this minute
  if (lastCheckedMinute[zone] == mn) return false;

  // Check first scheduled time
  bool match1 = (hr == startHour[zone] && mn == startMin[zone]);

  // Check second scheduled time (if enabled)
  bool match2 = enableStartTime2[zone] && 
               (hr == startHour2[zone] && mn == startMin2[zone]);

  if (match1 || match2) {
    lastCheckedMinute[zone] = mn;
    Serial.printf("Zone %d triggered at %02d:%02d\n", zone+1, hr, mn);
    return true;
  }

  return false;
}

bool hasDurationCompleted(int zone) {
  unsigned long elapsed = (millis() - zoneStartMs[zone]) / 1000;
  return (elapsed >= (unsigned long)durationMin[zone] * 60);
}

bool isTankLow() {
  int raw = analogRead(TANK_PIN);
  return raw < 150;
}

void turnOnZone(int z) {
  // PCF8574 outputs are active LOW
  pcfOut.digitalWrite(valveChannel[z], LOW);  // Turn ON valve
  zoneStartMs[z] = millis();
  zoneActive[z] = true;

  // ——— New: force-source overrides or fallback to tank-low logic ———
  if (justUseTank) {
    // Always use tank, ignore mains
    pcfOut.digitalWrite(mainsChannel, HIGH);  // Mains OFF
    pcfOut.digitalWrite(tankChannel, LOW);    // Tank ON
  }
  else if (justUseMains) {
    // Always use mains, ignore tank
    pcfOut.digitalWrite(mainsChannel, LOW);   // Mains ON
    pcfOut.digitalWrite(tankChannel, HIGH);   // Tank OFF
  }
  else if (isTankLow()) {
    // Tank low → switch to mains
    pcfOut.digitalWrite(mainsChannel, LOW);   // Mains ON
    pcfOut.digitalWrite(tankChannel, HIGH);   // Tank OFF
  }
  else {
    // Tank OK → use tank
    pcfOut.digitalWrite(mainsChannel, HIGH);  // Mains OFF
    pcfOut.digitalWrite(tankChannel, LOW);    // Tank ON
  }
  // ————————————————————————————————————————————————————————————

  // determine actual source used
  const char* src;
  if (justUseTank)        src = "Tank";
  else if (justUseMains)  src = "Mains";
  else if (isTankLow())   src = "Mains";
  else                    src = "Tank";

  // record the start event
  logEvent(z, "START", src, rainActive); 
  
  Serial.printf("Zone %d activated\n", z+1);
  display.clearDisplay();
  display.setCursor(3,0);
  display.print("Zone "); display.print(z+1); display.print(" ON");
  display.display();
  delay(1500);
  HomeScreen();
}

void turnOffZone(int z) {
  // PCF8574 outputs are active LOW
  pcfOut.digitalWrite(valveChannel[z], HIGH);  // Turn OFF valve
  pcfOut.digitalWrite(mainsChannel, HIGH);     // Turn OFF mains
  pcfOut.digitalWrite(tankChannel, HIGH);      // Turn OFF tank
  zoneActive[z] = false;

  // determine source (same logic as ON)
   const char* src;
   if (justUseTank)        src = "Tank";
   else if (justUseMains)  src = "Mains";
   else if (isTankLow())   src = "Mains";
   else                    src = "Tank";

  // record the stop event
  logEvent(z, "STOP", src, rainActive);


  Serial.printf("Zone %d deactivated\n", z+1);
  display.clearDisplay();
  display.setCursor(4,0);
  display.print("Zone "); display.print(z+1); display.print(" OFF");
  display.display();
  delay(1500);
}

void turnOnValveManual(int z) {
  if (!zoneActive[z]) {
    pcfOut.digitalWrite(valveChannel[z], LOW);
  if (justUseTank) {
    // Always use tank, ignore mains
    pcfOut.digitalWrite(mainsChannel, HIGH);  // Mains OFF
    pcfOut.digitalWrite(tankChannel, LOW);    // Tank ON
  }
  else if (justUseMains) {
    // Always use mains, ignore tank
    pcfOut.digitalWrite(mainsChannel, LOW);   // Mains ON
    pcfOut.digitalWrite(tankChannel, HIGH);   // Tank OFF
  }
  else if (isTankLow()) {
    // Tank low → switch to mains
    pcfOut.digitalWrite(mainsChannel, LOW);   // Mains ON
    pcfOut.digitalWrite(tankChannel, HIGH);   // Tank OFF
  }
  else {
    // Tank OK → use tank
    pcfOut.digitalWrite(mainsChannel, HIGH);  // Mains OFF
    pcfOut.digitalWrite(tankChannel, LOW);    // Tank ON
  }
    zoneActive[z]  = true;
    zoneStartMs[z] = millis();
    Serial.printf("Manual zone %d ON\n", z+1);
  }
}

void turnOffValveManual(int z) {
  if (zoneActive[z]) {
    pcfOut.digitalWrite(valveChannel[z], HIGH);
    pcfOut.digitalWrite(mainsChannel, HIGH);
    pcfOut.digitalWrite(tankChannel, HIGH);
    zoneActive[z] = false;
    Serial.printf("Manual zone %d OFF\n", z+1);
  }
}

String getDayName(int d) {
  const char* names[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  return String(names[d]);
}

void handleRoot() {
  // --- Time & schedule load ---
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  char timeStr[9];
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", timeinfo);
  String currentTime = String(timeStr);

  loadSchedule();

  // --- Weather parse ---
  String weatherData = cachedWeatherData;
  DynamicJsonDocument jsonResponse(1024);
  deserializeJson(jsonResponse, weatherData);
  float temp     = jsonResponse["main"]["temp"];
  float hum      = jsonResponse["main"]["humidity"];
  float ws       = jsonResponse["wind"]["speed"];
  String cond    = jsonResponse["weather"][0]["main"];
  String cityName= jsonResponse["name"];

  // --- Build HTML ---
  String html = "<!DOCTYPE html><html lang='en'><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Smart Irrigation System</title>";
  html += "<link href='https://fonts.googleapis.com/css?family=Roboto:400,500&display=swap' rel='stylesheet'>";
  html += "<style>";
    // Base styles
    html += "body { font-family: 'Roboto', sans-serif; background: linear-gradient(135deg, #e7f0f8, #ffffff); margin:0; padding:0; }";
    html += "header { background: linear-gradient(90deg, #0073e6, #00aaff); color:#fff; padding:20px; text-align:center; box-shadow:0 2px 4px rgba(0,0,0,0.1);}";
    html += ".container { max-width:800px; margin:20px auto; background:#fff; border-radius:10px; box-shadow:0 4px 12px rgba(0,0,0,0.15); padding:20px; }";
    html += "h1 { margin:0 0 10px; font-size:2em; }";
    html += "p { margin:10px 0; text-align:center; }";
    // Responsive horizontal layout
    html += ".zones-wrapper { display:flex; flex-wrap:wrap; justify-content:center; gap:20px; margin-bottom:20px; }";
    html += ".zone-container { background:#f9fbfd; padding:15px; border-radius:8px; border:1px solid #e0e0e0; box-sizing:border-box;"
            "flex:1 1 calc(45% - 20px); max-width:calc(45% - 20px); }";
    // Other existing styles
    html += ".days-container { display:flex; flex-wrap:wrap; justify-content:center; margin-bottom:10px; }";
    html += ".checkbox-container { margin:5px; }";
    html += ".time-duration-container { display:flex; flex-wrap:wrap; align-items:center; justify-content:center; margin-bottom:15px; }";
    html += ".time-input, .duration-input { margin:0 10px 10px; }";
    html += ".time-input label, .duration-input label { display:block; font-size:0.9em; margin-bottom:5px; }";
    html += "input[type='number'] { padding:5px; border:1px solid #ccc; border-radius:4px; }";
    html += ".enable-input { margin:10px; }";
    html += ".manual-control-container { text-align:center; margin-top:10px; }";
    html += "button { padding:10px 20px; border:none; border-radius:5px; cursor:pointer; margin:5px; transition:background 0.3s ease; }";
    html += ".turn-on-btn { background:#4caf50; color:#fff; } .turn-on-btn:hover { background:#45a044; }";
    html += ".turn-off-btn { background:#0073e6; color:#fff; } .turn-off-btn:hover { background:#0061c2; }";
    html += "button[type='submit'] { background:#2196F3; color:#fff; } button[type='submit']:hover { background:#1976d2; }";
    html += "a { color:#0073e6; text-decoration:none; } a:hover { text-decoration:underline; }";
    // Mobile fallback
    html += "@media (max-width:600px) { .zones-wrapper { flex-direction:column; } .zone-container { max-width:100%; } }";
  html += "</style>";
  html += "</head><body>";

    html += "<header><h1>A6-ESP32 Irrigation System</h1></header>";
    html += "<div class='container'>";
      // Clock & weather
      html += "<p id='clock'>Current Time: " + currentTime + "</p>";
      html += "<p>Location: " + cityName + "</p>";
      html += "<p id='weather-condition'>Condition: " + cond + "</p>";
      html += "<p id='temperature'>Temperature: " + String(temp) + " &#8451;</p>";
      html += "<p id='humidity'>Humidity: " + String(int(hum)) + " %</p>";
      html += "<p id='wind-speed'>Wind Speed: " + String(ws) + " m/s</p>";
      // Tank level
      int tankRaw = analogRead(TANK_PIN);
      int tankPct = map(tankRaw, 0, 1023, 0, 100);
      String tankStatus = (tankRaw < 250) ? "Low - Using Main" : "Normal - Using Tank";
      html += "<p>Tank Level: <progress id='tankLevel' value='" + String(tankPct) + "' max='100'></progress> "
              + String(tankPct) + "% (" + tankStatus + ")</p>";
      // Toggle backlight
      html += "<div style='text-align:center; margin-top:20px;'>"
              "<button type='button' id='toggle-backlight-btn'>Toggle Backlight</button>"
              "</div>";

      // Scripts
      html += "<script>"
               // Clock
               "function updateClock(){"
                 "const now=new Date();"
                 "const h=now.getHours().toString().padStart(2,'0');"
                 "const m=now.getMinutes().toString().padStart(2,'0');"
                 "const s=now.getSeconds().toString().padStart(2,'0');"
                 "document.getElementById('clock').textContent='Current Time: '+h+':'+m+':'+s;"
               "}"
               "setInterval(updateClock,1000);"
               // Weather
               "function fetchWeatherData(){"
                 "fetch('/weather-data').then(r=>r.json()).then(d=>{"
                   "document.getElementById('weather-condition').textContent='Condition: '+d.condition;"
                   "document.getElementById('temperature').textContent='Temperature: '+d.temp+' °C';"
                   "document.getElementById('humidity').textContent='Humidity: '+d.humidity+' %';"
                   "document.getElementById('wind-speed').textContent='Wind Speed: '+d.windSpeed+' m/s';"
                 "}).catch(console.error);"
               "}"
               "setInterval(fetchWeatherData,60000);"
               // Backlight
               "document.getElementById('toggle-backlight-btn').addEventListener('click',()=>{"
                 "fetch('/toggleBacklight',{method:'POST'}).then(r=>r.text()).then(alert).catch(console.error);"
               "});"
               // Manual on/off
               "document.addEventListener('DOMContentLoaded',()=>{"
                 "document.querySelectorAll('.turn-on-btn').forEach(btn=>{"
                   "btn.addEventListener('click',()=>{"
                     "const z=btn.dataset.zone;"
                     "fetch('/valve/on/'+z,{method:'POST'})"
                       ".then(()=>{ btn.disabled=true;"
                                  "document.querySelector('.turn-off-btn[data-zone=\"'+z+'\"').disabled=false; })"
                       ".catch(console.error);"
                   "});"
                 "});"
                 "document.querySelectorAll('.turn-off-btn').forEach(btn=>{"
                   "btn.addEventListener('click',()=>{"
                     "const z=btn.dataset.zone;"
                     "fetch('/valve/off/'+z,{method:'POST'})"
                       ".then(()=>{ btn.disabled=true;"
                                  "document.querySelector('.turn-on-btn[data-zone=\"'+z+'\"').disabled=false; })"
                       ".catch(console.error);"
                   "});"
                 "});"
               "});"
             "</script>";

      // Zones form
      html += "<form action='/submit' method='POST'>";
        html += "<div class='zones-wrapper'>";
 for (int zone = 0; zone < Zone; zone++) {
  html += "<div class='zone-container'>";

    // Zone header + status
    html += "<p><strong>Zone " + String(zone + 1) + ":</strong></p>";
    html += "<p>Status: " + String(zoneActive[zone] ? "Running" : "Off") + "</p>";

    // Days of week checkboxes
    html += "<div class='days-container'>";
      for (int d = 0; d < 7; d++) {
        String chk = days[zone][d] ? "checked" : "";
        html += "<div class='checkbox-container'>"
                "<input type='checkbox' name='day" + String(zone) + "_" + d +
                  "' id='day" + String(zone) + "_" + d + "' " + chk + ">"
                "<label for='day" + String(zone) + "_" + d + "'>" + getDayName(d) + "</label>"
                "</div>";
      }
    html += "</div>";

 // — Start times (1 & 2) + enable below Start 2 —
 html += "<div class='time-duration-container' style='flex-direction:column; align-items:flex-start;'>";
  // Start 1
  html += "<div class='time-input'>"
            "<label for='startHour" + String(zone) + "'>Start Time 1:</label>"
            "<input type='number' name='startHour" + String(zone) + "' min='0' max='23' value='" + String(startHour[zone]) + "' required>"
            "<input type='number' name='startMin"  + String(zone) + "' min='0' max='59' value='" + String(startMin[zone])  + "' required>"
          "</div>";
  // Start 2
  html += "<div class='time-input'>"
            "<label for='startHour2" + String(zone) + "'>Start Time 2:</label>"
            "<input type='number' name='startHour2" + String(zone) + "' min='0' max='23' value='" + String(startHour2[zone]) + "'>"
            "<input type='number' name='startMin2"  + String(zone) + "' min='0' max='59' value='" + String(startMin2[zone])  + "'>"
          "</div>";
  // Enable Start 2 (new row)
  html += "<div class='enable-input'>"
            "<input type='checkbox' id='enableStartTime2" + String(zone) + "' name='enableStartTime2" + String(zone) + "'" +
              (enableStartTime2[zone] ? " checked" : "") + "> "
            "<label for='enableStartTime2" + String(zone) + "'>Start 2 On/Off</label>"
          "</div>";
 html += "</div>";

 // — Duration, below the two start times + enable —
 html += "<div class='time-duration-container' style='justify-content:center;'>"
          "<div class='duration-input'>"
            "<label for='duration" + String(zone) + "'>Duration (min):</label>"
            "<input type='number' name='duration" + String(zone) + "' min='0' value='" + String(durationMin[zone]) + "' required>"
          "</div>"
        "</div>";

    // **Manual On/Off controls**  
    html += "<div class='manual-control-container'>"
              "<button type='button' class='turn-on-btn'  data-zone='" + String(zone) + "'>Turn On</button>"
              "<button type='button' class='turn-off-btn' data-zone='" + String(zone) + "' disabled>Turn Off</button>"
            "</div>";

  html += "</div>";  // close .zone-container
  }
        html += "</div>";
        html += "<button type='submit'>Update Schedule</button>";
      html += "</form>";

      // Footer
      html += "<p style='text-align:center;'><a href='/events'>View Event Log</a></p>";
      html += "<p style='text-align:center;'><a href='/setup'>Setup Page</a></p>";
      html += "<p style='text-align:center;'><a href='https://openweathermap.org/city/" + cityName + "' target='_blank'>View Weather on Openweathermap.org</a></p>";

    html += "</div>";  // .container
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleSubmit() {
  for (int z = 0; z < Zone; z++) {
    for (int d = 0; d < 7; d++) {
      days[z][d] = server.hasArg("day" + String(z) + "_" + String(d));
    }
    if (server.hasArg("startHour"+String(z)))
      startHour[z] = server.arg("startHour"+String(z)).toInt();
    if (server.hasArg("startMin"+String(z)))
      startMin[z]  = server.arg("startMin"+String(z)).toInt();

    if (server.hasArg("startHour2"+String(z)))
      startHour2[z] = server.arg("startHour2"+String(z)).toInt();
    if (server.hasArg("startMin2"+String(z)))
      startMin2[z]  = server.arg("startMin2"+String(z)).toInt();

    if (server.hasArg("duration"+String(z)))
      durationMin[z] = server.arg("duration"+String(z)).toInt();

    enableStartTime2[z] = server.hasArg("enableStartTime2"+String(z));
  }

  // Config fields?
  if (server.hasArg("apiKey"))    apiKey            = server.arg("apiKey");
  if (server.hasArg("city"))      city              = server.arg("city");
  if (server.hasArg("dstOffset")) tzOffsetHours     = server.arg("dstOffset").toFloat();
  rainDelayEnabled  = server.hasArg("rainDelay");
  windDelayEnabled  = server.hasArg("windCancelEnabled");
  if (server.hasArg("windSpeedThreshold"))
    windSpeedThreshold = server.arg("windSpeedThreshold").toFloat();

  saveConfig();
  saveSchedule();
  updateCachedWeather();

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleSetup() {
   String html = "<!DOCTYPE html><html lang='en'><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Setup</title>";
  html += "<link href='https://fonts.googleapis.com/css?family=Roboto:400,500&display=swap' rel='stylesheet'>";
  html += "<style>";
  html += "body { font-family: 'Roboto', sans-serif; background: #f7f9fc; margin: 0; padding: 0; display: flex; align-items: center; justify-content: center; height: 100vh; }";
  html += "form { background: #ffffff; padding: 30px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); width: 100%; max-width: 400px; }";
  html += "h1 { text-align: center; color: #333333; margin-bottom: 20px; }";
  html += "label { display: block; margin-bottom: 5px; color: #555555; }";
  html += "input[type='text'], input[type='number'], select { width: 100%; padding: 10px; margin-bottom: 15px; border: 1px solid #cccccc; border-radius: 5px; font-size: 14px; }";
  html += "input[type='checkbox'] { margin-right: 10px; }";
  html += "input[type='submit'] { width: 100%; padding: 10px; background: #007BFF; border: none; border-radius: 5px; color: #ffffff; font-size: 16px; cursor: pointer; }";
  html += "input[type='submit']:hover { background: #0056b3; }";
  html += "</style>";
  html += "</head><body>";
  html += "<form action='/configure' method='POST'>";
  html += "<h1>Setup</h1>";
  html += "<label for='apiKey'>API Key:</label>";
  html += "<input type='text' id='apiKey' name='apiKey' value='" + apiKey + "'>";
  html += "<label for='city'>City Number:</label>";
  html += "<input type='text' id='city' name='city' value='" + city + "'>";
  html += "<label for='timezone'>City Timezone Offset (hours):</label>";
  html += "<input type='number' id='timezone' name='dstOffset' min='-12' max='14' step='0.50' value='" + String(tzOffsetHours, 2) + "'>";
  html += "<label for='windSpeedThreshold'>Wind Speed Threshold (m/s):</label>";
  html += "<input type='number' id='windSpeedThreshold' name='windSpeedThreshold' min='0' step='0.1' value='" + String(windSpeedThreshold) + "'>";
  html += String("<label for='windCancelEnabled'><input type='checkbox' id='windCancelEnabled' name='windCancelEnabled'") + (windDelayEnabled ? " checked" : "") + "> Enable Wind Delay</label>";
  html += "<label for='rainDelay'><input type='checkbox' id='rainDelay' name='rainDelay' " + String(rainDelayEnabled ? "checked" : "") + "> Enable Rain Delay</label>";
  html += "<label for='justUseTank'>"
          "<input type='checkbox' id='justUseTank' name='justUseTank'";
  if (justUseTank) html += " checked";
  html += "> Only use Tank (Solenoids will start sequentially if times overlap/same)</label>";

  html += "<label for='justUseMains'>"
          "<input type='checkbox' id='justUseMains' name='justUseMains'";
  if (justUseMains) html += " checked";
  html += "> Only use Mains (Solenoids run together if times overlap/same)</label>";
  html += "<input type='submit' value='Submit'>";
  html += "<p style='text-align: center;'><a href='https://openweathermap.org/city/" + city + "' target='_blank'>View Weather Details on OpenWeatherMap</a></p>";
  html += "</form>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleEvents() {
  File f = LittleFS.open("/events.csv", "r");
  if (!f) {
    server.send(404, "text/plain", "No event log found");
    return;
  }

  // Start HTML
  String html = "<!DOCTYPE html><html lang='en'><head>"
                "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>"
                "<title>Event Log</title>"
                "<style>"
                  "body{font-family:Arial,sans-serif;padding:20px;} "
                  "table{width:100%;border-collapse:collapse;} "
                  "th,td{border:1px solid #ccc;padding:8px;text-align:left;} "
                  "th{background:#f0f0f0;} "
                "</style>"
                "</head><body>"
                "<h1>Irrigation Event Log</h1>"
                "<form method='POST' action='/clearevents' style='text-align:right; margin-bottom:10px;'>"
                "<button type='submit' onclick='return confirm(\"Clear all logs?\");'>Clear Log</button>"
                "</form>"
                "<table>"
                  "<tr><th>Timestamp</th><th>Zone</th><th>Event</th><th>Source</th><th>Rain Delay</th></tr>";

  // Read CSV lines
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() < 5) continue;  // skip empty
    // Split on commas
    int idx1 = line.indexOf(',');
    int idx2 = line.indexOf(',', idx1 + 1);
    int idx3 = line.indexOf(',', idx2 + 1);
    int idx4 = line.indexOf(',', idx3 + 1);
    String ts   = line.substring(0, idx1);
    String zone = line.substring(idx1 + 1, idx2);
    String ev   = line.substring(idx2 + 1, idx3);
    String src  = line.substring(idx3 + 1, idx4);
    String rd   = line.substring(idx4 + 1);
    rd.trim();

    html += "<tr><td>" + ts + "</td>"
            "<td>" + zone + "</td>"
            "<td>" + ev   + "</td>"
            "<td>" + src  + "</td>"
            "<td>" + rd   + "</td></tr>";
  }
  f.close();

  // Close HTML
  html += "</table>"
          "<p><a href='/'>Back to Home</a></p>"
          "</body></html>";

  server.send(200, "text/html", html);
}

void handleConfigure() {
  apiKey = server.arg("apiKey");
  city   = server.arg("city");
  tzOffsetHours = server.arg("dstOffset").toFloat();
  rainDelayEnabled  = server.hasArg("rainDelay");
  windDelayEnabled  = server.hasArg("windCancelEnabled");
  windSpeedThreshold= server.arg("windSpeedThreshold").toFloat();
  justUseTank  = server.hasArg("justUseTank");
  justUseMains = server.hasArg("justUseMains");

  saveConfig();

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}
