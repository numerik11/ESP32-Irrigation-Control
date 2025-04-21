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

unsigned long lastWeatherUpdate  = 0;
const unsigned long weatherUpdateInterval = 3600000; // 1 hour
String cachedWeatherData;
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
void drawHomeScreen();
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
  display.setTextSize(1);
  // Wi‑Fi Setup
  wifiManager.setTimeout(180);
  if (!wifiManager.autoConnect("ESPIrrigationAP")) {
    Serial.println("Failed to connect to WiFi. Restarting...");
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("ESPIrrigation");
    display.setCursor(0,1);
    display.print("IP: 192.168.4.1");
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
    display.setCursor(0,1);
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
  ArduinoOTA.handle();
  server.handleClient();
  wifiCheck();

  // Midnight reset
  time_t nowTime = time(nullptr);
  struct tm* nowTm = localtime(&nowTime);
  if (nowTm->tm_hour==0 && nowTm->tm_min==0) {
    memset(lastCheckedMinute, -1, sizeof(lastCheckedMinute));
  }

  // Toggle display when no zone active
  bool anyActive = false;
  for (int z=0; z<Zone; z++) if (zoneActive[z]) { anyActive=true; break; }
  if (!anyActive) {
  drawHomeScreen();
  }

  // ——— Scheduled actions with sequential‑on‑mains logic ———
  for (int z = 0; z < Zone; ++z) {
    if (shouldStartZone(z)) {
       if (!weatherAllowsIrrigation()) {
    pendingStart[z] = true;   // optionally queue to try again later
    continue;
      }
      if (isTankLow()) {
        // Running off mains: only start immediately if no other zone is active
        bool anyActive = false;
        for (int i = 0; i < Zone; ++i) {
          if (zoneActive[i]) { anyActive = true; break; }
        }
        if (anyActive) {
          pendingStart[z] = true;
          Serial.printf("Zone %d queued for sequential start\n", z+1);
        } else {
          turnOnZone(z);
        }
      } else {
        // Plenty in the tank: allow parallel starts
        turnOnZone(z);
      }
    }
    // Check for completion regardless of tank status
    if (zoneActive[z] && hasDurationCompleted(z)) {
      turnOffZone(z);
    }
  }

  // If no zone is currently running, fire the next queued one (if any)
  {
    bool anyActive = false;
    for (int i = 0; i < Zone; ++i) {
      if (zoneActive[i]) { anyActive = true; break; }
    }
    if (!anyActive) {
      for (int z = 0; z < Zone; ++z) {
        if (pendingStart[z]) {
          Serial.printf("Starting queued Zone %d\n", z+1);
          pendingStart[z] = false;
          turnOnZone(z);
          break;  // only one at a time
        }
      }
    }
  }

  delay(50);
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

void drawHomeScreen() {
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

  // Line 0: time (large) + date (smaller)
  display.setTextSize(2);
  display.setCursor(0,  0);
  display.printf("%02d:%02d", t->tm_hour, t->tm_min);

  display.setTextSize(1);
  display.setCursor(96, 2);           // adjust X if it overlaps
  display.printf("%02d/%02d", day, mon);

  // Line 1 (y=10): temperature & humidity
  display.setTextSize(1);
  display.setCursor(0,  10);
  display.printf("T:%2.0fC   H:%02d%%", temp, hum);

  // Line 2 (y=20): tank level & source
  display.setCursor(0,  20);
  display.printf("Tank: %3d%% (%s)", pct, src);

  // Line 3 (y=30): Zone 1 & 2 status
  display.setCursor(0,  30);
  display.printf("Z1:%s", zoneActive[0] ? "On " : "Off");
  display.setCursor(64, 30);
  display.printf("Z2:%s", zoneActive[1] ? "On " : "Off");

  // Line 4 (y=40): Zone 3 & 4 status
  display.setCursor(0,  40);
  display.printf("Z3:%s", zoneActive[2] ? "On " : "Off");
  display.setCursor(64, 40);
  display.printf("Z4:%s", zoneActive[3] ? "On " : "Off");

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
  drawHomeScreen();
}

void updateLCDForZone(int zone) {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < 1000) return;
  last = now;

  unsigned long elapsed = (now - zoneStartMs[zone]) / 1000;
  unsigned long total   = (unsigned long)durationMin[zone] * 60;
  unsigned long rem     = (elapsed < total ? total - elapsed : 0);

  String line1 = "Zone " + String(zone+1)
               + " " + String(elapsed/60) + ":"
               + (elapsed%60 < 10 ? "0" : "")
               + String(elapsed%60);
  display.clearDisplay();
  display.setCursor((16 - line1.length())/2, 0);
  display.print(line1);
  display.display();  

  if (elapsed < total) {
    String line2 = String(rem/60) + "m rem";
    display.clearDisplay();
    display.setCursor((16 - line2.length())/2, 1);
    display.print(line2);
    display.display();  
  } else {
    display.clearDisplay();
    display.setCursor(4,0);
    display.print("Complete");
    display.display();  
    delay(2000);
  }
}

void showZoneDisplay(int zone) {
  updateLCDForZone(zone);
}

bool weatherAllowsIrrigation() {
  // parse the last‐fetched weather JSON
  DynamicJsonDocument js(1024);
  if (deserializeJson(js, cachedWeatherData) || js.overflowed()) {
    Serial.println("weather parse error – allowing irrigation");
    return true;
  }

  // 1) Rain check (OpenWeatherMap might report rain["1h"])
  if (rainDelayEnabled && js.containsKey("rain")) {
    float rain1h = js["rain"]["1h"]   // mm in last hour
                  | js["rain"]["3h"]   // fallback to 3h
                  | 0.0f;
    if (rain1h > 0.0f) {
      Serial.printf("Skipping due to recent rain: %.2f mm\n", rain1h);
      return false;
    }
  }

  // 2) Wind check
  if (windDelayEnabled) {
    float ws = js["wind"]["speed"].as<float>();
    if (ws >= windSpeedThreshold) {
      Serial.printf("Skipping due to high wind: %.1f m/s >= %.1f\n",
                    ws, windSpeedThreshold);
      return false;
    }
  }

  return true;
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

  // Control water source based on tank level
  if (isTankLow()) {
    pcfOut.digitalWrite(mainsChannel, LOW);   // Turn ON mains
    pcfOut.digitalWrite(tankChannel, HIGH);   // Turn OFF tank
  } else {
    pcfOut.digitalWrite(mainsChannel, HIGH);  // Turn OFF mains
    pcfOut.digitalWrite(tankChannel, LOW);    // Turn ON tank
  }

  Serial.printf("Zone %d activated\n", z+1);
  display.clearDisplay();
  display.setCursor(3,0);
  display.print("Zone "); display.print(z+1); display.print(" ON");
  display.display();
  delay(1500);
}

void turnOffZone(int z) {
  // PCF8574 outputs are active LOW
  pcfOut.digitalWrite(valveChannel[z], HIGH);  // Turn OFF valve
  pcfOut.digitalWrite(mainsChannel, HIGH);     // Turn OFF mains
  pcfOut.digitalWrite(tankChannel, HIGH);      // Turn OFF tank
  zoneActive[z] = false;

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
    if (isTankLow()) {
      pcfOut.digitalWrite(mainsChannel, LOW);
      pcfOut.digitalWrite(tankChannel, HIGH);
    } else {
      pcfOut.digitalWrite(mainsChannel, HIGH);
      pcfOut.digitalWrite(tankChannel, LOW);
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
  float ws = jsonResponse["wind"]["speed"];
  String cond = jsonResponse["weather"][0]["main"];
  String cityName = jsonResponse["name"];

  if (WiFi.status() == WL_CONNECTED) {
    display.clearDisplay();
    int textLength = cond.substring(0, 10).length();
    int startPos = (textLength < 16) ? (16 - textLength) / 2 : 0;
    display.setCursor(startPos, 0);
    display.print(cond.substring(0, 10));
    display.setCursor(0, 1);
    display.print("Te:");
    display.print(temp);
    display.print("C Hu:");
    display.print(int(hum));
    display.print("%");
  }

  String html = "<!DOCTYPE html><html lang='en'><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Smart Irrigation System</title>";
  html += "<link href='https://fonts.googleapis.com/css?family=Roboto:400,500&display=swap' rel='stylesheet'>";
  html += "<style>";
  html += "body { font-family: 'Roboto', sans-serif; background: linear-gradient(135deg, #e7f0f8, #ffffff); margin: 0; padding: 0; }";
  html += "header { background: linear-gradient(90deg, #0073e6, #00aaff); color: #fff; padding: 20px; text-align: center; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
  html += ".container { max-width: 800px; margin: 20px auto; background: #fff; border-radius: 10px; box-shadow: 0 4px 12px rgba(0,0,0,0.15); padding: 20px; }";
  html += "h1 { margin: 0 0 10px; font-size: 2em; }";
  html += "p { margin: 10px 0; text-align: center; }";
  html += ".zone-container { background: #f9fbfd; padding: 15px; border-radius: 8px; margin-bottom: 20px; border: 1px solid #e0e0e0; }";
  html += ".days-container { display: flex; flex-wrap: wrap; justify-content: center; margin-bottom: 10px; }";
  html += ".checkbox-container { margin: 5px; }";
  html += ".time-duration-container { display: flex; flex-wrap: wrap; align-items: center; justify-content: center; margin-bottom: 15px; }";
  html += ".time-input, .duration-input { margin: 0 10px 10px; }";
  html += ".time-input label, .duration-input label { display: block; font-size: 0.9em; margin-bottom: 5px; }";
  html += "input[type='number'] { padding: 5px; border: 1px solid #ccc; border-radius: 4px; }";
  html += ".enable-input { margin: 10px; }";
  html += ".manual-control-container { text-align: center; margin-top: 10px; }";
  html += "button { padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; margin: 5px; transition: background 0.3s ease; }";
  html += ".turn-on-btn { background: #4caf50; color: #fff; }";
  html += ".turn-on-btn:hover { background: #45a044; }";
  html += ".turn-off-btn { background: #0073e6; color: #fff; }";
  html += ".turn-off-btn:hover { background: #0061c2; }";
  html += "button[type='submit'] { background: #2196F3; color: #fff; }";
  html += "button[type='submit']:hover { background: #1976d2; }";
  html += "a { color: #0073e6; text-decoration: none; }";
  html += "a:hover { text-decoration: underline; }";
  html += "@media (max-width: 600px) { .container { width: 100%; padding: 10px; } .zone-container { margin-bottom: 15px; } .time-duration-container { flex-direction: column; align-items: flex-start; } .time-input, .duration-input { width: 100%; } }";
  html += "</style></head><body>";

 html += "<header><h1>Smart Irrigation System</h1></header>";
  html += "<div class='container'>";
  html += "<p id='clock'>Current Time: " + currentTime + "</p>";
  html += "<p>Location: " + cityName + "</p>";
  html += "<p id='weather-condition'>Condition: " + cond + "</p>";
  html += "<p id='temperature'>Temperature: " + String(temp) + " &#8451;</p>";
  html += "<p id='humidity'>Humidity: " + String(int(hum)) + " %</p>";
  html += "<p id='wind-speed'>Wind Speed: " + String(ws) + " m/s</p>";
  int tankRaw = analogRead(TANK_PIN);
  int tankPercentage = map(tankRaw, 0, 1023, 0, 100);
  String tankStatus = (tankRaw < 250) ? "Low - Using Main" : "Normal - Using Tank";
  html += "<p>Tank Level: <progress id='tankLevel' value='" + String(tankPercentage) + "' max='100'></progress> " + String(tankPercentage) + "% (" + tankStatus + ")</p>";

  // Add the new toggle backlight button here
  html += "<div style='text-align: center; margin-top: 20px;'>";
  html += "<button type='button' id='toggle-backlight-btn'>Toggle Backlight</button>";
  html += "</div>";
  
  html += "<script>";
  html += "function updateClock() {";
  html += "  var now = new Date();";
  html += "  var hours = now.getHours().toString().padStart(2, '0');";
  html += "  var minutes = now.getMinutes().toString().padStart(2, '0');";
  html += "  var seconds = now.getSeconds().toString().padStart(2, '0');";
  html += "  document.getElementById('clock').textContent = 'Current Time: ' + hours + ':' + minutes + ':' + seconds;";
  html += "} setInterval(updateClock, 1000);";
  html += "function fetchWeatherData() {";
  html += "  fetch('/weather-data').then(response => response.json()).then(data => {";
  html += "    document.getElementById('weather-condition').textContent = 'Condition: ' + data.condition;";
  html += "    document.getElementById('temperature').textContent = 'Temperature: ' + data.temp + ' &#8451;';";
  html += "    document.getElementById('humidity').textContent = 'Humidity: ' + data.humidity + ' %';";
  html += "    document.getElementById('wind-speed').textContent = 'Wind Speed: ' + data.windSpeed + ' m/s';";
  html += "  }).catch(error => console.error('Error fetching weather data:', error));";
  html += "} setInterval(fetchWeatherData, 60000);";
  // Backlight toggle button script
  html += "document.getElementById('toggle-backlight-btn').addEventListener('click', function() {";
  html += "  fetch('/toggleBacklight', { method: 'POST' })";
  html += "    .then(response => response.text())";
  html += "    .then(data => {";
  html += "      console.log(data);";
  html += "      alert('Backlight toggled');";
  html += "    })";
  html += "    .catch(error => console.error('Error toggling backlight:', error));";
  html += "});";
  html += "</script>";
  
  html += "<form action='/submit' method='POST'>";
  for (int zone = 0; zone < Zone; zone++) {    html += "<div class='zone-container'>";
    html += "<p><strong>Zone " + String(zone + 1) + ":</strong></p>";
    
    // --- Status display for each valve ---
    html += "<p>Status: ";
    if (zoneActive[zone]) {
   html += "Running";
  } else {
   html += "Off";
  }
    html += "</p>";
    // ------------------------------------

    html += "<div class='days-container'>";
    for (int i = 0; i < 7; i++) {
      String dayLabel = getDayName(i);
      String checked = days[zone][i] ? "checked" : "";
      html += "<div class='checkbox-container'>";
      html += "<input type='checkbox' name='day" + String(zone) + "_" + String(i) + "' id='day" + String(zone) + "_" + String(i) + "' " + checked + ">";
      html += "<label for='day" + String(zone) + "_" + String(i) + "'>" + dayLabel + "</label>";
      html += "</div>";
    }
    html += "</div>";

    html += "<div class='time-duration-container'>";
    html += "<div class='time-input'>";
    html += "<label for='startHour" + String(zone) + "'>Start Time 1:</label>";
    html += "<input type='number' name='startHour" + String(zone) + "' id='startHour" + String(zone) + "' min='0' max='23' value='" + String(startHour[zone]) + "' required>";
    html += "<input type='number' name='startMin" + String(zone) + "' id='startMin" + String(zone) + "' min='0' max='59' value='" + String(startMin[zone]) + "' required>";
    html += "</div>";
    html += "<div class='duration-input'>";
    html += "<label for='duration" + String(zone) + "'>Duration (min):</label>";
    html += "<input type='number' name='duration" + String(zone) + "' id='duration" + String(zone) + "' min='0' value='" + String(durationMin[zone]) + "' required>";
    html += "</div>";
    html += "</div>";

    html += "<div class='time-duration-container'>";
    html += "<div class='time-input'>";
    html += "<label for='startHour2" + String(zone) + "'>Start Time 2:</label>";
    html += "<input type='number' name='startHour2" + String(zone) + "' id='startHour2" + String(zone) + "' min='0' max='23' value='" + String(startHour2[zone]) + "' required>";
    html += "<input type='number' name='startMin2" + String(zone) + "' id='startMin2" + String(zone) + "' min='0' max='59' value='" + String(startMin2[zone]) + "' required>";
    html += "</div>";
    html += "<div class='enable-input'>";
    html += "<input type='checkbox' name='enableStartTime2" + String(zone) + "' id='enableStartTime2" + String(zone) + "'" + (enableStartTime2[zone] ? " checked" : "") + ">";
    html += "<label for='enableStartTime2" + String(zone) + "'>Enable Start Time 2</label>";
    html += "</div>";
    html += "</div>";

    html += "<div class='manual-control-container'>";
    html += "  <button type='button' class='turn-on-btn' data-zone='"  + String(zone) + "'>Turn On</button>";
    html += "  <button type='button' class='turn-off-btn' data-zone='" + String(zone) + "' disabled>Turn Off</button>";
    html += "</div>";

    html += "</div>";
  }
  
  html += "<button type='submit'>Update Schedule</button>";
  html += "</form>";
  html += "<p style='text-align: center;'>Click <a href='/setup'>HERE</a> to enter API key, City, Time Zone offset, and Wind Settings.</p>";
  html += "<p style='text-align: center;'><a href='https://openweathermap.org/city/" + city + "' target='_blank'>View Weather Details on OpenWeatherMap</a></p>";

  html += "<script>";
  html += "document.addEventListener('DOMContentLoaded', function() {";
  html += "  document.querySelectorAll('.turn-on-btn').forEach(function(btn) {";
  html += "    btn.addEventListener('click', function() {";
  html += "      const z = this.dataset.zone;";
  html += "      fetch('/valve/on/' + z, { method: 'POST' })";
  html += "        .then(() => {";
  html += "          this.disabled = true;";
  html += "          document.querySelector('.turn-off-btn[data-zone=\"'+z+'\"]').disabled = false;";
  html += "        })";
  html += "        .catch(console.error);";
  html += "    });";
  html += "  });";
  html += "  document.querySelectorAll('.turn-off-btn').forEach(function(btn) {";
  html += "    btn.addEventListener('click', function() {";
  html += "      const z = this.dataset.zone;";
  html += "      fetch('/valve/off/' + z, { method: 'POST' })";
  html += "        .then(() => {";
  html += "          this.disabled = true;";
  html += "          document.querySelector('.turn-on-btn[data-zone=\"'+z+'\"]').disabled = false;";
  html += "        })";
  html += "        .catch(console.error);";
  html += "    });";
  html += "  });";
  html += "});";
  html += "</script>";
  
  html += "</div></body></html>";
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
  html += "<input type='submit' value='Submit'>";
  html += "<p style='text-align: center;'><a href='https://openweathermap.org/city/" + city + "' target='_blank'>View Weather Details on OpenWeatherMap</a></p>";
  html += "</form>";
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

  saveConfig();

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}
