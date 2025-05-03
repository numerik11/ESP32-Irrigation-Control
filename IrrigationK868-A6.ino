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
int tankEmptyRaw = 100;  // default for empty tank
int tankFullRaw  = 900;  // default for full tank

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
void lcdHomeScreen();
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
  server.on("/tank", HTTP_GET, handleTankCalibration);

  server.on("/setTankEmpty", HTTP_POST, [](){
  tankEmptyRaw = analogRead(TANK_PIN);
  saveConfig();
  server.sendHeader("Location", "/tank", true);
  server.send(302, "text/plain", "");
  });

 server.on("/setTankFull", HTTP_POST, [](){
  tankFullRaw = analogRead(TANK_PIN);
  saveConfig();
  server.sendHeader("Location", "/tank", true);
  server.send(302, "text/plain", "");
  });


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
    lcdHomeScreen();
  }
  else {
    lcdHomeScreen();
  }
 }

  delay(100);
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
  File f = LittleFS.open("/config.txt", "r");
  if (!f) return;

  apiKey            = f.readStringUntil('\n'); apiKey.trim();
  city              = f.readStringUntil('\n'); city.trim();
  tzOffsetHours     = f.readStringUntil('\n').toFloat();
  rainDelayEnabled  = (f.readStringUntil('\n').toInt() == 1);
  windSpeedThreshold= f.readStringUntil('\n').toFloat();
  windDelayEnabled  = (f.readStringUntil('\n').toInt() == 1);
  justUseTank       = (f.readStringUntil('\n').toInt() == 1);
  justUseMains      = (f.readStringUntil('\n').toInt() == 1);
  tankEmptyRaw      = f.readStringUntil('\n').toInt();
  tankFullRaw       = f.readStringUntil('\n').toInt();
  
  f.close();
}

void saveConfig() {
  File f = LittleFS.open("/config.txt", "w");
  if (!f) return;

  f.println(apiKey);
  f.println(city);
  f.println(tankEmptyRaw);
  f.println(tankFullRaw);
  f.println(tzOffsetHours, 2);
  f.println(rainDelayEnabled ? "1" : "0");
  f.println(windSpeedThreshold, 1);
  f.println(windDelayEnabled ? "1" : "0");
  f.println(justUseTank ? "1" : "0");
  f.println(justUseMains ? "1" : "0");
  f.println(tankEmptyRaw);
  f.println(tankFullRaw);
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

void lcdHomeScreen() {
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
  lcdHomeScreen();
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
  lcdHomeScreen();
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
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  char timeStr[9];
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", timeinfo);
  String currentTime = String(timeStr);

  loadSchedule();

  String weatherData = cachedWeatherData;
  DynamicJsonDocument jsonResponse(1024);
  deserializeJson(jsonResponse, weatherData);
  float temp = jsonResponse["main"]["temp"];
  float hum = jsonResponse["main"]["humidity"];
  float ws  = jsonResponse["wind"]["speed"];
  String cond = jsonResponse["weather"][0]["main"];
  String cityName = jsonResponse["name"];

  int tankRaw = analogRead(TANK_PIN);
  int tankPct = map(tankRaw, 0, 1023, 0, 100);
  String tankStatus = (tankRaw < 250) ? "Low - Using Main" : "Normal - Using Tank";

  String html = R"rawliteral(
  <!DOCTYPE html><html lang='en'>
  <head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0'>
    <title>Smart Irrigation System</title>
    <link href='https://fonts.googleapis.com/css?family=Roboto:400,500&display=swap' rel='stylesheet'>
    <script src='https://kit.fontawesome.com/4f508dd61b.js' crossorigin='anonymous'></script>
    <style>
      body {
        font-family: 'Roboto', sans-serif;
        margin: 0; padding: 0;
        background: linear-gradient(135deg, #eef3f9, #ffffff);
      }
      header {
        background: linear-gradient(90deg, #0073e6, #00aaff);
        color: #fff;
        text-align: center;
        padding: 20px;
        box-shadow: 0 2px 6px rgba(0,0,0,0.2);
      }
      .container {
        max-width: 900px;
        margin: 20px auto;
        background: #fff;
        padding: 25px;
        border-radius: 12px;
        box-shadow: 0 6px 15px rgba(0,0,0,0.1);
      }
      h1 { margin: 0; }
      p { text-align: center; margin: 8px 0; }
      .zones-wrapper {
        display: flex; flex-wrap: wrap;
        gap: 20px; justify-content: center;
      }
      .zone-container {
        flex: 1 1 calc(45% - 20px);
        background: #f9fbfd;
        border: 1px solid #ddd;
        border-radius: 8px;
        padding: 15px;
      }
      .days-container, .time-duration-container {
        display: flex; flex-wrap: wrap;
        justify-content: center;
        gap: 10px;
        margin: 10px 0;
      }
      .checkbox-container {
        display: flex;
        align-items: center;
        gap: 4px;
      }
      input[type='number'] {
        padding: 6px;
        border: 1px solid #ccc;
        border-radius: 4px;
        width: 60px;
      }
      button {
        padding: 10px 18px;
        border: none;
        border-radius: 5px;
        cursor: pointer;
        margin: 5px;
        transition: background 0.3s ease;
      }
      .turn-on-btn { background: #4caf50; color: #fff; }
      .turn-off-btn { background: #f44336; color: #fff; }
      button[type='submit'] {
        display: block;
        background: #2196F3;
        color: #fff;
        width: 100%;
        font-weight: bold;
        margin-top: 20px;
      }
      .manual-control-container {
        text-align: center;
        margin-top: 10px;
      }
      .enable-input {
        display: flex;
        align-items: center;
        justify-content: center;
        margin-top: 8px;
      }
      a {
        color: #0073e6;
        display: block;
        text-align: center;
        margin-top: 12px;
        text-decoration: none;
      }
      a:hover { text-decoration: underline; }
      @media (max-width: 600px) {
        .zone-container {
          max-width: 100%;
          flex: 1 1 100%;
        }
      }
    </style>
  </head>
  <body>
    <header><h1><i class='fas fa-seedling'></i> A6-ESP32 Irrigation System</h1></header>
    <div class='container'>
      <p><i class='fas fa-clock'></i> Current Time: )rawliteral" + currentTime + R"rawliteral(</p>
      <p><i class='fas fa-map-marker-alt'></i> Location: )rawliteral" + cityName + R"rawliteral(</p>
      <p><i class='fas fa-cloud'></i> Condition: )rawliteral" + cond + R"rawliteral(</p>
      <p><i class='fas fa-thermometer-half'></i> Temperature: )rawliteral" + String(temp) + R"rawliteral( &#8451;</p>
      <p><i class='fas fa-tint'></i> Humidity: )rawliteral" + String((int)hum) + R"rawliteral(%</p>
      <p><i class='fas fa-wind'></i> Wind Speed: )rawliteral" + String(ws) + R"rawliteral( m/s</p>
      <p><i class='fas fa-flask'></i> Tank: <progress value=')rawliteral" + String(tankPct) + R"rawliteral(' max='100'></progress> )rawliteral" + String(tankPct) + R"rawliteral(% - )rawliteral" + tankStatus + R"rawliteral(</p>

      <div style='text-align:center; margin-top:20px;'>
        <button id='toggle-backlight-btn'><i class='fas fa-lightbulb'></i> Toggle Backlight</button>
      </div>

      <form action='/submit' method='POST'>
        <div class='zones-wrapper'>
  )rawliteral";

  // Add all zone blocks
  for (int zone = 0; zone < Zone; zone++) {
    html += "<div class='zone-container'>";
    html += "<p><strong><i class='fas fa-water'></i> Zone " + String(zone + 1) + "</strong></p>";
    html += "<p>Status: " + String(zoneActive[zone] ? "Running" : "Off") + "</p>";

    html += "<div class='days-container'>";
    for (int d = 0; d < 7; d++) {
      String chk = days[zone][d] ? "checked" : "";
      html += "<div class='checkbox-container'>"
              "<input type='checkbox' name='day" + String(zone) + "_" + d + "' id='day" + String(zone) + "_" + d + "' " + chk + ">"
              "<label for='day" + String(zone) + "_" + d + "'>" + getDayName(d) + "</label>"
              "</div>";
    }
    html += "</div>";

    html += "<div class='time-duration-container'>";
    html += "<div><label>Start 1:</label><input type='number' name='startHour" + String(zone) + "' min='0' max='23' value='" + String(startHour[zone]) + "'> : ";
    html += "<input type='number' name='startMin" + String(zone) + "' min='0' max='59' value='" + String(startMin[zone]) + "'></div>";
    html += "<div><label>Start 2:</label><input type='number' name='startHour2" + String(zone) + "' min='0' max='23' value='" + String(startHour2[zone]) + "'> : ";
    html += "<input type='number' name='startMin2" + String(zone) + "' min='0' max='59' value='" + String(startMin2[zone]) + "'></div>";
    html += "</div>";

    html += "<div class='enable-input'><input type='checkbox' name='enableStartTime2" + String(zone) + "'" +
            (enableStartTime2[zone] ? " checked" : "") + "> Enable Start 2</div>";

    html += "<div class='time-duration-container'><label>Duration (min):</label><input type='number' name='duration" + String(zone) + "' min='0' value='" + String(durationMin[zone]) + "'></div>";

    html += "<div class='manual-control-container'>"
            "<button type='button' class='turn-on-btn' data-zone='" + String(zone) + "'><i class='fas fa-play'></i> Turn On</button>"
            "<button type='button' class='turn-off-btn' data-zone='" + String(zone) + "' disabled><i class='fas fa-stop'></i> Turn Off</button>"
            "</div>";

    html += "</div>"; // close .zone-container
  }

  html += R"rawliteral(
        </div>
        <button type='submit'><i class='fas fa-save'></i> Update Schedule</button>
      </form>

      <a href='/events'><i class='fas fa-list'></i> View Event Log</a>
      <a href='/setup'><i class='fas fa-cogs'></i> Setup Page</a>
      <a href='https://openweathermap.org/city/)" + cityName + R"rawliteral(' target='_blank'><i class='fas fa-cloud'></i> OpenWeatherMap</a>
    </div>

    <script>
      setInterval(() => {
        const now = new Date();
        document.querySelector('#clock').textContent = 'Current Time: ' + now.toLocaleTimeString();
      }, 1000);

      document.getElementById('toggle-backlight-btn').addEventListener('click', () => {
        fetch('/toggleBacklight', {method: 'POST'}).then(res => res.text()).then(alert);
      });

      document.querySelectorAll('.turn-on-btn').forEach(btn => {
        btn.addEventListener('click', () => {
          let zone = btn.dataset.zone;
          fetch('/valve/on/' + zone, {method: 'POST'}).then(() => {
            btn.disabled = true;
            document.querySelector('.turn-off-btn[data-zone="' + zone + '"]').disabled = false;
          });
        });
      });
      document.querySelectorAll('.turn-off-btn').forEach(btn => {
        btn.addEventListener('click', () => {
          let zone = btn.dataset.zone;
          fetch('/valve/off/' + zone, {method: 'POST'}).then(() => {
            btn.disabled = true;
            document.querySelector('.turn-on-btn[data-zone="' + zone + '"]').disabled = false;
          });
        });
      });
    </script>
  </body>
  </html>
  )rawliteral";

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

String generateCheckbox(const String& id, const String& name, bool checked, const String& label) {
  return "<div class='checkbox-row'>"
         "<input type='checkbox' id='" + id + "' name='" + name + "'" + (checked ? " checked" : "") + ">"
         "<label for='" + id + "'>" + label + "</label>"
         "</div>";
}

void handleSetup() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html lang='en' data-theme='light'>
  <head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0'>
    <title>Setup</title>
    <link href='https://fonts.googleapis.com/css?family=Roboto:400,500&display=swap' rel='stylesheet'>
    <script src='https://kit.fontawesome.com/4f508dd61b.js' crossorigin='anonymous'></script>
    <style>
      :root {
        --bg: #fff; --text: #000; --card: #fefefe; --accent: #0073e6;
      }
      [data-theme='dark'] {
        --bg: #121212; --text: #f0f0f0; --card: #1f1f1f; --accent: #64b5f6;
      }
      body {
        font-family: 'Roboto', sans-serif;
        background: var(--bg);
        color: var(--text);
        margin: 0;
        padding: 0;
        display: flex;
        justify-content: center;
        align-items: center;
        min-height: 100vh;
      }
      form {
        background: var(--card);
        padding: 30px;
        border-radius: 12px;
        box-shadow: 0 6px 15px rgba(0,0,0,0.2);
        width: 100%;
        max-width: 500px;
      }
      h1 {
        text-align: center;
        margin-bottom: 25px;
      }
      label {
        display: block;
        margin: 12px 0 6px;
        font-weight: 500;
      }
      input[type='text'],
      input[type='number'],
      select {
        width: 100%;
        padding: 10px;
        border: 1px solid #ccc;
        border-radius: 6px;
        margin-bottom: 10px;
      }
      .checkbox-row {
        display: flex;
        align-items: center;
        gap: 10px;
        margin: 12px 0;
      }
      input[type='checkbox'] {
        width: 18px;
        height: 18px;
      }
      input[type='submit'] {
        width: 100%;
        padding: 12px;
        background: var(--accent);
        color: #fff;
        border: none;
        border-radius: 6px;
        font-weight: 500;
        cursor: pointer;
        margin-top: 20px;
        transition: background 0.3s ease;
      }
      input[type='submit']:hover {
        background: #005bb5;
      }
      #theme-toggle {
        position: absolute;
        top: 15px;
        right: 20px;
        background: none;
        border: none;
        color: var(--text);
        font-size: 1.3em;
        cursor: pointer;
      }
      a {
        display: block;
        text-align: center;
        margin-top: 15px;
        color: var(--accent);
        text-decoration: none;
      }
      a:hover {
        text-decoration: underline;
      }
    </style>
  </head>
  <body>
    <button id='theme-toggle'><i class='fas fa-moon'></i></button>
    <form method='POST' action='/configure'>
      <h1><i class='fas fa-cog'></i> Setup</h1>

      <label for='apiKey'><i class='fas fa-key'></i> API Key</label>
      <input type='text' id='apiKey' name='apiKey' value=')rawliteral" + apiKey + R"rawliteral('>

      <label for='city'><i class='fas fa-city'></i> City Number</label>
      <input type='text' id='city' name='city' value=')rawliteral" + city + R"rawliteral('>

      <label for='dstOffset'><i class='fas fa-clock'></i> Timezone Offset (hrs)</label>
      <input type='number' id='dstOffset' name='dstOffset' step='0.5' min='-12' max='14' value=')rawliteral" + String(tzOffsetHours, 2) + R"rawliteral('>

      <label for='windSpeedThreshold'><i class='fas fa-wind'></i> Wind Threshold (m/s)</label>
      <input type='number' id='windSpeedThreshold' name='windSpeedThreshold' step='0.1' min='0' value=')rawliteral" + String(windSpeedThreshold) + R"rawliteral('>
  )rawliteral";

  // Append checkboxes inside the form
  html += generateCheckbox("windCancelEnabled", "windCancelEnabled", windDelayEnabled, "Enable Wind Delay");
  html += generateCheckbox("rainDelay", "rainDelay", rainDelayEnabled, "Enable Rain Delay");
  html += generateCheckbox("justUseTank", "justUseTank", justUseTank, "Only use Tank (zones sequential)");
  html += generateCheckbox("justUseMains", "justUseMains", justUseMains, "Only use Mains (zones parallel)");

  html += R"rawliteral(
      <input type='submit' value='Save Settings'>
      <a href='/tank'><i class='fas fa-flask'></i> Tank Calibration Tool</a>
      <a href='https://openweathermap.org/city/)" + city + R"rawliteral(' target='_blank'><i class='fas fa-cloud'></i> View Weather on OpenWeatherMap</a>
    </form>
    <script>
      document.getElementById('theme-toggle').onclick = () => {
        let r = document.documentElement;
        let dark = r.getAttribute('data-theme') === 'dark';
        r.setAttribute('data-theme', dark ? 'light' : 'dark');
        localStorage.setItem('theme', dark ? 'light' : 'dark');
        document.getElementById('theme-toggle').innerHTML = dark ? '<i class="fas fa-moon"></i>' : '<i class="fas fa-sun"></i>';
      };
      window.onload = () => {
        let saved = localStorage.getItem('theme') || 'light';
        document.documentElement.setAttribute('data-theme', saved);
        document.getElementById('theme-toggle').innerHTML = saved === 'dark' ? '<i class="fas fa-sun"></i>' : '<i class="fas fa-moon"></i>';
      };
    </script>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleEvents() {
  File f = LittleFS.open("/events.csv", "r");
  if (!f) {
    server.send(404, "text/plain", "No event log found");
    return;
  }

  String html = "<!DOCTYPE html><html lang='en'><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Event Log</title>";
  html += "<link href='https://fonts.googleapis.com/css?family=Roboto:400,700&display=swap' rel='stylesheet'>";
  html += "<style>";
  html += "body { font-family: 'Roboto', sans-serif; background: #f0f4f8; margin: 0; }";
  html += "header { background:#007BFF; color:white; text-align:center; padding:20px; font-size:1.6em; font-weight:500; }";
  html += ".container { max-width:900px; margin:20px auto; background:white; border-radius:8px; box-shadow:0 4px 8px rgba(0,0,0,0.1); padding:20px; }";
  html += "h2 { color:#007BFF; margin-top:0; }";
  html += "table { width:100%; border-collapse:collapse; margin-top:20px; }";
  html += "th, td { padding:10px; border:1px solid #ccc; text-align:left; font-size:0.95em; }";
  html += "th { background:#e0ecf7; }";
  html += "tr:nth-child(even) { background:#f9f9f9; }";
  html += "form { text-align:right; margin-bottom:15px; }";
  html += "button { padding:10px 15px; background:#d9534f; color:white; border:none; border-radius:4px; cursor:pointer; }";
  html += "button:hover { background:#c9302c; }";
  html += "a { display:inline-block; margin-top:15px; color:#007BFF; text-decoration:none; }";
  html += "a:hover { text-decoration:underline; }";
  html += "</style></head><body>";

  html += "<header>📜 Irrigation Event Log</header>";
  html += "<div class='container'>";
  html += "<h2>Recent Events</h2>";

  html += "<form method='POST' action='/clearevents'>";
  html += "<button type='submit' onclick='return confirm(\"Clear all logs?\");'>🗑 Clear Log</button>";
  html += "</form>";

  html += "<table>";
  html += "<tr><th>Timestamp</th><th>Zone</th><th>Event</th><th>Source</th><th>Rain Delay</th></tr>";

  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() < 5) continue;

    int idx1 = line.indexOf(',');
    int idx2 = line.indexOf(',', idx1 + 1);
    int idx3 = line.indexOf(',', idx2 + 1);
    int idx4 = line.indexOf(',', idx3 + 1);

    String ts   = line.substring(0, idx1);
    String zone = line.substring(idx1 + 1, idx2);
    String ev   = line.substring(idx2 + 1, idx3);
    String src  = line.substring(idx3 + 1, idx4);
    String rd   = line.substring(idx4 + 1); rd.trim();

    html += "<tr><td>" + ts + "</td><td>" + zone + "</td><td>" + ev + "</td><td>" + src + "</td><td>" + rd + "</td></tr>";
  }
  f.close();

  html += "</table>";
  html += "<a href='/'>⬅ Back to Home</a>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

void handleTankCalibration() {
  int raw = analogRead(TANK_PIN);

  // Calculate % based on calibration range
  int pct = map(raw, tankEmptyRaw, tankFullRaw, 0, 100);
  pct = constrain(pct, 0, 100);

  String html = "<!DOCTYPE html><html lang='en'><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Tank Calibration</title>";
  html += "<link href='https://fonts.googleapis.com/css?family=Roboto:400,500&display=swap' rel='stylesheet'>";
  html += "<style>";
  html += "body { font-family: 'Roboto', sans-serif; background: #f0f4f8; margin: 0; padding: 0; display: flex; align-items: center; justify-content: center; height: 100vh; }";
  html += ".card { background: #ffffff; padding: 30px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); width: 100%; max-width: 400px; text-align: center; }";
  html += "h1 { color: #007BFF; margin-bottom: 20px; }";
  html += "p { margin: 10px 0; font-size: 1.1em; }";
  html += "form { margin: 10px 0; }";
  html += "button { padding: 10px 20px; border: none; border-radius: 5px; background: #007BFF; color: #ffffff; font-size: 16px; cursor: pointer; }";
  html += "button:hover { background: #0056b3; }";
  html += "a { display: block; margin-top: 15px; color: #007BFF; text-decoration: none; }";
  html += "a:hover { text-decoration: underline; }";
  html += ".alert { color: #cc0000; font-weight: bold; }";
  html += "</style></head><body>";

  html += "<div class='card'>";
  html += "<h1>Tank Calibration</h1>";
  html += "<p>📟 Raw Sensor Value: <strong>" + String(raw) + "</strong></p>";
  html += "<p>🧪 Calibrated Range: <strong>" + String(tankEmptyRaw) + " (Empty)</strong> → <strong>" + String(tankFullRaw) + " (Full)</strong></p>";
  html += "<p>🚰 Calculated Level: <strong>" + String(pct) + "%</strong></p>";

  // Show warning if raw is out of expected range
  if (raw < tankEmptyRaw || raw > tankFullRaw) {
    html += "<p class='alert'>⚠ Outside calibration range</p>";
  }

  html += "<form method='POST' action='/setTankEmpty'>";
  html += "<button type='submit'>Set as Empty</button>";
  html += "</form>";

  html += "<form method='POST' action='/setTankFull'>";
  html += "<button type='submit'>Set as Full</button>";
  html += "</form>";

  html += "<a href='/setup'>⬅ Back to Setup</a>";
  html += "<a href='/'>🏠 Home</a>";

  html += "</div>";

  html += "<script>setTimeout(() => location.reload(), 2000);</script>"; // auto-refresh every 2s
  html += "</body></html>";

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
