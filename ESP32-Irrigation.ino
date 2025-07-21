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
#include "esp_log.h"

// -------------------------------
// Constants
// -------------------------------
static const uint8_t Zone           = 4;
static const unsigned long WEATHER_INTERVAL = 600000; // 10 minutes

PCF8574 pcfIn(0x22, 4, 15);   //  Digital Inputs (P0 - P5)
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

// === GPIO Fallback Configuration ===
bool useGpioFallback = false;
// Direct GPIO pin assignments for fallback (adjust to your board)
uint8_t zonePins[Zone] = {16, 17, 18, 19};
uint8_t mainsPin       = 21;
uint8_t tankPin        = 22;

// A6 Solenoid channels (PCF8574 pins)
const uint8_t valveChannel[Zone] = { P0, P1, P2, P3 };
const uint8_t mainsChannel        = P4;
const uint8_t tankChannel         = P5;

// On‑board LED & tank sensor
const int LED_PIN  = 2;
const int TANK_PIN = 36; // A1

// -------------------------------
// Globals
// -------------------------------

String apiKey, city;
String cachedWeatherData;
bool   rainDelayEnabled   = true;
bool   windDelayEnabled   = false;
bool   justUseTank        = false;
bool   justUseMains       = false;
bool   enableStartTime2[Zone] = {false};
bool   days[Zone][7]    = {{false}};
bool   zoneActive[Zone]         = {false};
bool   pendingStart[Zone] = { false };
bool   rainActive     = false;
bool   windActive = false;   // ← new
float  tzOffsetHours      = 0.0;
float  windSpeedThreshold = 5.0f;
float  lastRainAmount = 0.0f;
int    startHour[Zone]   = {0};
int    startMin [Zone]   = {0};
int    startHour2[Zone]  = {0};
int    startMin2 [Zone]  = {0};
int    durationMin[Zone] = {0};
int    durationSec[Zone] = {0};   // ← new: seconds for each zone
int    lastCheckedMinute[Zone] = { -1, -1, -1, -1 };
int    tankEmptyRaw = 100;
int    tankFullRaw  = 900;
unsigned long zoneStartMs[Zone] = {0};
unsigned long lastScreenRefresh  = 0;
unsigned long lastWeatherUpdate  = 0;
const unsigned long SCREEN_REFRESH_MS = 1000;
const unsigned long weatherUpdateInterval = 3600000; // 1 hour
const uint8_t expanderAddrs[] = { 0x22, 0x24 };
const uint8_t I2C_HEALTH_DEBOUNCE = 5;  // how many bad loops before fallback
uint8_t i2cFailCount = 0;

// -------------------------------
// Prototypes
// -------------------------------

void wifiCheck();
void loadConfig();
void saveConfig();
void loadSchedule();
void saveSchedule();
void updateCachedWeather();
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
void handleSetupPage();
void handleLogPage();
void handleConfigure();
String getDayName(int d);
String fetchWeather();

// -------------------------------

void setup() {

  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  esp_log_level_set("i2c",        ESP_LOG_NONE);

  Serial.begin(115200);
  Wire.begin(4, 15);

 if (!LittleFS.begin()) {
    Serial.println("⚠ LittleFS mount failed; formatting...");
    if (LittleFS.format() && LittleFS.begin()) {
      Serial.println("✔ LittleFS reformatted and mounted");
    } else {
      Serial.println("‼ LittleFS format/remount failed; halting");
      while (true) delay(1000);
    }
  }

  loadConfig();
  if (!LittleFS.exists("/schedule.txt")) {
    Serial.println("No schedule.txt found—creating default");
    saveSchedule();    // writes zeros/default entries
  }
  loadSchedule();

  // —— PCF8574 init or GPIO fallback ——
  if (!pcfIn.begin() || !pcfOut.begin()) {
    Serial.println("⚠ PCF8574 init failed. Switching to GPIO fallback.");
    initGpioFallback();    // ← this sets all your zonePins[], mainsPin, tankPin
  } else {
    // Original expander setup
    for (uint8_t ch = P0; ch <= P5; ++ch) {
      pcfOut.pinMode(ch, OUTPUT);
      pcfOut.digitalWrite(ch, HIGH);
      pcfIn .pinMode(ch, INPUT);
      pcfIn .digitalWrite(ch, HIGH);
    }
  }

  pinMode(LED_PIN, OUTPUT);

  // OLED Initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 init failed");
    while (true) delay(100);
  }
  delay(2000);

  display.display();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1.5);
    display.clearDisplay();
    display.setCursor(0,10);
    display.print("Connect To");
    display.setCursor(0,20);
    display.print("A6-Irrigation");
    display.setCursor(0,30);
    display.print("Goto: https://192.168.4.1");
    display.display();  
    
  // Wi‑Fi Setup
  wifiManager.setTimeout(180);
  if (!wifiManager.autoConnect("ESPIrrigationAP")) {
    Serial.println("Failed to connect to WiFi. Restarting...");
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

  // NTP setup with precise offset handling
  int gmtOffsetSec = round(tzOffsetHours * 3600);
  int daylightOffsetSec = 0;

  configTime(gmtOffsetSec, daylightOffsetSec,
           "pool.ntp.org", "time.nist.gov");
  time_t now = time(nullptr);
  while (now < 1000000000) {
  delay(500);
  now = time(nullptr);
 }

  // OTA & Web Server
  ArduinoOTA.begin();
  ArduinoOTA.setHostname("ESP32 Irrigation");

  server.on("/",          HTTP_GET,  handleRoot);
  server.on("/submit",    HTTP_POST, handleSubmit);
  server.on("/setup",     HTTP_GET,  handleSetupPage);
  server.on("/configure", HTTP_POST, handleConfigure);
  server.on("/events", HTTP_GET, handleLogPage);
  server.on("/clearevents", HTTP_POST, handleClearEvents);
  server.on("/tank", HTTP_GET, handleTankCalibration);
  server.on("/status", HTTP_GET, [](){
  DynamicJsonDocument doc(256);
  doc["rainDelayActive"] = rainActive;
  doc["windDelayActive"] = windActive;     // ← new

  JsonArray zones = doc.createNestedArray("zones");
  for (int i = 0; i < Zone; i++) {
    JsonObject z = zones.createNestedObject();
    z["active"]    = zoneActive[i];
    unsigned long rem = 0;
    if (zoneActive[i]) {
      unsigned long elapsed = (millis() - zoneStartMs[i]) / 1000;
      unsigned long total   = (unsigned long)durationMin[i]*60 + durationSec[i];
      rem = (elapsed < total ? total - elapsed : 0);
    }
    z["remaining"] = rem;
  }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
  });

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
  
  if (!useGpioFallback) {
    checkI2CHealth();
  }

  if (rainActive || windActive) {
  for (int z = 0; z < Zone; ++z) {
    if (zoneActive[z]) {
      turnOffZone(z);
      Serial.printf("Zone %d forcibly stopped due to %s\n",
                    z+1,
                    rainActive ? "rain" : "wind");
    }
  }
  // Skip scheduling logic until clear
  delay(1000);
  return;
  }

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
    logEvent(z, "RAIN DELAY", "N/A", true);
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

void checkI2CHealth() {
  delay(50);
  bool anyErr = false;
  for (auto addr : expanderAddrs) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) {
      anyErr = true;
      break;
    }
  }

  if (anyErr) {
    i2cFailCount++;
    if (i2cFailCount >= I2C_HEALTH_DEBOUNCE) {
      Serial.println("⚠ Multiple I2C errors, switching to GPIO fallback");
      useGpioFallback = true;
      // re-init fallback pins
      for (uint8_t i = 0; i < Zone; i++) {
        pinMode(zonePins[i], OUTPUT);
        digitalWrite(zonePins[i], HIGH);
      }
      pinMode(mainsPin, OUTPUT);   digitalWrite(mainsPin, HIGH);
      pinMode(tankPin, OUTPUT);    digitalWrite(tankPin, HIGH);
    }
  } else {
    // healthy bus → reset counter
    i2cFailCount = 0;
  }
}

void initGpioFallback() {
  useGpioFallback = true;
  for (uint8_t i = 0; i < Zone; i++) {
    pinMode(zonePins[i], OUTPUT);
    digitalWrite(zonePins[i], HIGH);
  }
  pinMode(mainsPin, OUTPUT);  digitalWrite(mainsPin, HIGH);
  pinMode(tankPin, OUTPUT);   digitalWrite(tankPin, HIGH);
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
  tzOffsetHours = f.readStringUntil('\n').toFloat();
  rainDelayEnabled  = (f.readStringUntil('\n').toInt() == 1);
  windSpeedThreshold= f.readStringUntil('\n').toFloat();
  windDelayEnabled  = (f.readStringUntil('\n').toInt() == 1);
  justUseTank       = (f.readStringUntil('\n').toInt() == 1);
  justUseMains      = (f.readStringUntil('\n').toInt() == 1);
  tankEmptyRaw      = f.readStringUntil('\n').toInt();
  tankFullRaw       = f.readStringUntil('\n').toInt();
   for (int i = 0; i < Zone; i++) {
    zonePins[i] = f.readStringUntil('\n').toInt();
  }
  mainsPin = f.readStringUntil('\n').toInt();
  tankPin  = f.readStringUntil('\n').toInt();
  f.close();
}

void saveConfig() {
  File f = LittleFS.open("/config.txt", "w");
  if (!f) return;

  f.println(apiKey);
  f.println(city);
  f.println(tzOffsetHours, 2);
  f.println(rainDelayEnabled ? "1" : "0");
  f.println(windSpeedThreshold, 1);
  f.println(windDelayEnabled ? "1" : "0");
  f.println(justUseTank ? "1" : "0");
  f.println(justUseMains ? "1" : "0");
  f.println(tankEmptyRaw);
  f.println(tankFullRaw);
  for (int i = 0; i < Zone; i++) {
    f.println(zonePins[i]);
  }
  f.println(mainsPin);
  f.println(tankPin);
  f.close();

}

void loadSchedule() {
  File f = LittleFS.open("/schedule.txt", "r");
  if (!f) {
    Serial.println("Failed to open schedule file");
    return;
  }

  Serial.println("Watering Schedule:");
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

    //  durationSec:
    nextComma       = line.indexOf(',', index);
    durationSec[i]  = line.substring(index, nextComma).toInt();
    index           = nextComma + 1;

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
      durationSec[i],
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
    f.print(durationSec[i]); f.print(',');
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
  updateCachedWeather();
  DynamicJsonDocument js(512);
  float temp = NAN, wind = NAN;
  int hum = 0;
  String cond = "?";
  if (!deserializeJson(js, cachedWeatherData)) {
    temp = js["main"]["temp"].as<float>();
    hum  = js["main"]["humidity"].as<int>();
    wind = js["wind"]["speed"].as<float>();
    cond = js["weather"][0]["main"].as<const char*>();
  }

  // --- Log to CSV with weather ---
  File f = LittleFS.open("/events.csv", "a");
  if (!f) return;
  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  char buf[200];
  // Add weather columns: temp, hum, wind, cond
  sprintf(buf,
    "%04d-%02d-%02d %02d:%02d:%02d,Zone%d,%s,%s,%s,%.1f,%d,%.1f,%s\n",
    t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
    t->tm_hour, t->tm_min, t->tm_sec,
    zone + 1,
    eventType,
    source,
    rainDelayed ? "Active" : "Off",
    temp,
    hum,
    wind,
    cond.c_str()
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
      unsigned long total = (unsigned long)durationMin[i] * 60
                          + (unsigned long)durationSec[i];
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
  updateCachedWeather();

  DynamicJsonDocument js(1024);
  if (deserializeJson(js, cachedWeatherData)) {
    rainActive = false;
    windActive = false;
    return true;
  }

  // 1) Rain check
  if (rainDelayEnabled && js.containsKey("rain")) {
    rainActive = true;
    Serial.println("Rain-delay active");
    return false;
  }
  rainActive = false;

  // 2) Wind check
  if (windDelayEnabled && js.containsKey("wind")) {
    float windSpd = js["wind"]["speed"].as<float>();
    if (windSpd >= windSpeedThreshold) {
      windActive = true;
      Serial.printf("Wind-delay active (%.2f m/s)\n", windSpd);
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
  unsigned long total   = (unsigned long)durationMin[zone] * 60
                         + (unsigned long)durationSec[zone];
  return (elapsed >= total);
}

bool isTankLow() {
  int raw = analogRead(TANK_PIN);
  return raw < 150;
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

void turnOnZone(int z) {
  // 1) Refresh rain/wind flags immediately
  checkWindRain();

  // 2) If rain‐delay is active, queue and abort
  if (rainActive) {
    pendingStart[z] = true;
    logEvent(z, "RAIN DELAY", "N/A", true);
    Serial.printf("Zone %d delayed by rain\n", z + 1);
    return;
  }

  // 3) If wind‐delay is active, queue and abort
  if (windActive) {
    pendingStart[z] = true;
    logEvent(z, "WIND DELAY", "N/A", false);
    Serial.printf("Zone %d delayed by wind\n", z + 1);
    return;
  }

  // 4) No delays → actually start
  zoneStartMs[z] = millis();
  zoneActive[z]  = true;

  // 5) Open valve and select source
  if (useGpioFallback) {
    digitalWrite(zonePins[z], LOW);
    if (justUseTank) {
      digitalWrite(mainsPin, HIGH);
      digitalWrite(tankPin, LOW);
    } else if (justUseMains) {
      digitalWrite(mainsPin, LOW);
      digitalWrite(tankPin, HIGH);
    } else if (isTankLow()) {
      digitalWrite(mainsPin, LOW);
      digitalWrite(tankPin, HIGH);
    } else {
      digitalWrite(mainsPin, HIGH);
      digitalWrite(tankPin, LOW);
    }
  } else {
    pcfOut.digitalWrite(valveChannel[z], LOW);
    if (justUseTank) {
      pcfOut.digitalWrite(mainsChannel, HIGH);
      pcfOut.digitalWrite(tankChannel, LOW);
    } else if (justUseMains) {
      pcfOut.digitalWrite(mainsChannel, LOW);
      pcfOut.digitalWrite(tankChannel, HIGH);
    } else if (isTankLow()) {
      pcfOut.digitalWrite(mainsChannel, LOW);
      pcfOut.digitalWrite(tankChannel, HIGH);
    } else {
      pcfOut.digitalWrite(mainsChannel, HIGH);
      pcfOut.digitalWrite(tankChannel, LOW);
    }
  }

  // 6) Log the START event
  const char* src = justUseTank   ? "Tank"
                     : justUseMains ? "Mains"
                     : isTankLow()   ? "Mains"
                                    : "Tank";
  logEvent(z, "START", src, false);
  Serial.printf("Zone %d activated\n", z + 1);

  // 7) OLED confirmation then back to Home
  display.clearDisplay();
  display.setCursor(3, 0);
  display.print("Zone "); display.print(z + 1); display.print(" ON");
  display.display();
  delay(1500);
  HomeScreen();
}

void turnOffZone(int z) {
  // 1) Figure out the water source as before
  const char* src = justUseTank   ? "Tank"
                   : justUseMains ? "Mains"
                   : isTankLow()   ? "Mains"
                                  : "Tank";

  // 2) Detect if this stop was caused by a delay (rain or wind)
  bool wasDelayed = rainActive || windActive;

  // 3) Log STOP, passing wasDelayed as the 'rainDelayed' field
  logEvent(z, "STOPPED", src, wasDelayed);

  // 4) Print a clearer Serial message
  if      (rainActive) Serial.printf("Zone %d stopped due to rain\n", z+1);
  else if (windActive) Serial.printf("Zone %d stopped due to wind\n", z+1);
  else                 Serial.printf("Zone %d deactivated\n", z+1);

  // 5) Physically turn everything off
  if (useGpioFallback) {
    digitalWrite(zonePins[z], HIGH);
    digitalWrite(mainsPin,   HIGH);
    digitalWrite(tankPin,    HIGH);
  } else {
    pcfOut.digitalWrite(valveChannel[z], HIGH);
    pcfOut.digitalWrite(mainsChannel,    HIGH);
    pcfOut.digitalWrite(tankChannel,     HIGH);
  }

  zoneActive[z] = false;

  // 6) Optional OLED feedback
  display.clearDisplay();
  display.setCursor(4,0);
  display.print("Zone "); display.print(z+1);
  display.print(wasDelayed ? " STOP" : " OFF");
  display.display();
  delay(1500);
}

void turnOnValveManual(int z) {
  if (!zoneActive[z]) {
    zoneStartMs[z] = millis();
    zoneActive[z]  = true;

   if (useGpioFallback) {
      digitalWrite(zonePins[z], LOW);
      // replicate source logic from turnOnZone
      if (justUseTank) {
        digitalWrite(mainsPin, HIGH);
        digitalWrite(tankPin, LOW);
      } else if (justUseMains) {
        digitalWrite(mainsPin, LOW);
        digitalWrite(tankPin, HIGH);
      } else if (isTankLow()) {
        digitalWrite(mainsPin, LOW);
        digitalWrite(tankPin, HIGH);
      } else {
        digitalWrite(mainsPin, HIGH);
        digitalWrite(tankPin, LOW);
      }
    } else {
      pcfOut.digitalWrite(valveChannel[z], LOW);
      if (justUseTank) {
        pcfOut.digitalWrite(mainsChannel, HIGH);
        pcfOut.digitalWrite(tankChannel, LOW);
      } else if (justUseMains) {
        pcfOut.digitalWrite(mainsChannel, LOW);
        pcfOut.digitalWrite(tankChannel, HIGH);
      } else if (isTankLow()) {
        pcfOut.digitalWrite(mainsChannel, LOW);
        pcfOut.digitalWrite(tankChannel, HIGH);
      } else {
        pcfOut.digitalWrite(mainsChannel, HIGH);
        pcfOut.digitalWrite(tankChannel, LOW);
      }
    }

    Serial.printf("Manual zone %d ON\n", z+1);
  }
}

void turnOffValveManual(int z) {
  if (!zoneActive[z]) return;

  // 1) Turn off this valve
  if (useGpioFallback) {
    digitalWrite(zonePins[z], HIGH);
  } else {
    pcfOut.digitalWrite(valveChannel[z], HIGH);
  }
  zoneActive[z] = false;
  Serial.printf("Manual zone %d OFF\n", z+1);

  // 2) Check if any other zone is still active
  bool anyStillOn = false;
  for (int i = 0; i < Zone; i++) {
    if (zoneActive[i]) {
      anyStillOn = true;
      break;
    }
  }

  // 3) Only shut off the source solenoid if *none* are running
  if (!anyStillOn) {
    if (useGpioFallback) {
      digitalWrite(mainsPin, HIGH);
      digitalWrite(tankPin,  HIGH);
    } else {
      pcfOut.digitalWrite(mainsChannel, HIGH);
      pcfOut.digitalWrite(tankChannel,  HIGH);
    }
    Serial.println("Source solenoid OFF (no zones running)");
  }
}

String getDayName(int d) {
  const char* names[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  return String(names[d]);
}

void handleRoot() {
  // --- Time ---
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  char timeStr[9], dateStr[11];
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", timeinfo);
  strftime(dateStr, sizeof(dateStr), "%d/%m/%Y", timeinfo);

  loadSchedule();

  // --- Weather ---
  String weatherData = cachedWeatherData;
  DynamicJsonDocument jsonResponse(1024);
  DeserializationError werr = deserializeJson(jsonResponse, weatherData);

  float temp = werr ? NAN : jsonResponse["main"]["temp"] | NAN;
  float hum  = werr ? NAN : jsonResponse["main"]["humidity"] | NAN;
  float ws   = werr ? NAN : jsonResponse["wind"]["speed"] | NAN;
  String cond     = werr ? "-" : String(jsonResponse["weather"][0]["main"].as<const char*>());
  if (cond == "") cond = "-";
  String cityName = werr ? "-" : String(jsonResponse["name"].as<const char*>());
  if (cityName == "") cityName = "-";

  // --- Tank sensor ---
  int tankRaw = analogRead(TANK_PIN);
  int tankPct = map(tankRaw, tankEmptyRaw, tankFullRaw, 0, 100);
  tankPct = constrain(tankPct, 0, 100);
  bool tankLow = tankRaw < (tankEmptyRaw + (tankFullRaw-tankEmptyRaw) * 0.15f);
  String tankStatusStr = tankLow
    ? "<span class='tank-status-low'>Low — Using Main</span>"
    : "<span class='tank-status-normal'>Normal — Using Tank</span>";

  // --- HTML ---
  String html;
  html.reserve(28 * 1024);
  html += R"(
 <!DOCTYPE html>
 <html lang='en'>
 <head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1.0'>
  <title>ESP32 Irrigation System</title>
  <link href='https://fonts.googleapis.com/css?family=Roboto:400,500&display=swap' rel='stylesheet'>
  <style>
 :root {
  --primary: #1976d2;
  --danger: #e03e36;
  --success: #32c366;
  --neutral: #bed9ff;
  --bg: #f5f7fa;
  --card: #fff;
  --font: #1a1a1a;
  --footer: #fafdff4b;
  --tank-gradient: linear-gradient(90deg, #32c366 0%, #ffe94b 65%, #e03e36 100%);
  --shadow: 0 2px 16px rgba(80,160,255,.08);
 }
 body {
  font-family:'Roboto',sans-serif;
  background:var(--bg);
  color:var(--font);
  margin:0;
  transition: background .2s, color .2s;
 }
 header {
  display:flex;flex-direction:column;align-items:center;justify-content:center;
  gap:10px;padding:22px 0 10px 0;background:var(--primary);color:#fff;
  border-radius:0 0 18px 18px;box-shadow:0 2px 8px #1976d241;
 }
 .container {
  max-width: 1040px;
  margin: 0 auto;
  padding: 22px 10px 14px 10px;
  display: flex;
  flex-direction: column;
  align-items: center;
 }
 .summary-row {
  display: flex;
  flex-wrap: wrap;
  gap: 18px;
  justify-content: center;
  align-items: flex-start;
  width: 100%;
  margin-bottom: 32px;
 }
 .summary-block {
  background: var(--card);
  border-radius: 14px;
  box-shadow: var(--shadow);
  padding: 17px 13px 15px 13px;
  min-width: 110px;
  text-align: center;
  font-size: 1.18em;
  font-weight: 500;
  position: relative;
  border: 1.5px solid var(--neutral);
  margin-bottom: 0;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
 }
 .summary-block .icon { font-size:2.1em; margin-bottom: 3px; display:block; }
 .tank-row progress[value] {
  width: 60px; height: 13px; vertical-align:middle; border-radius:8px; background:#e9ecef;
  box-shadow: 0 1.5px 8px #32c36624 inset;
  margin-bottom: 3px;
 }
 body[data-theme='dark'] .tank-row progress[value],
 body[data-theme='dark'] .tank-row progress[value]::-webkit-progress-bar {
 background: #222 !important;
 }
 .tank-row progress[value]::-webkit-progress-bar { background: #e9ecef; border-radius:8px; }
 .tank-row progress[value]::-webkit-progress-value {
  background: var(--tank-gradient); border-radius:8px; transition:width .4s;
  animation: tankPulse 2.5s linear infinite alternate;
 }
 .tank-row progress[value]::-moz-progress-bar {
  background: var(--tank-gradient); border-radius:8px;
 }
 @keyframes tankPulse {
  0% { box-shadow:0 0 0 #32c36622;}
  60%{ box-shadow:0 0 12px #ffe94b77;}
  100%{ box-shadow:0 0 16px #e03e3665;}
 }
 .active-badge {
  color: var(--success);
  font-weight:700;
  padding:2px 10px;
  border-radius:10px;
  background: #e6ffe7;
  box-shadow: 0 0 12px 2px #32c36688;
  animation: statusGlow 1.2s ease-in-out infinite alternate;
  display:inline-block;
  font-size:1em;
  margin-top:2px;
 }
 body[data-theme='dark'] .active-badge { background:#1e4220; }
 @keyframes statusGlow {
  0% { box-shadow: 0 0 8px #32c36677;}
  100% { box-shadow: 0 0 16px #32c366cc, 0 0 25px #ffe94b66;}
 }
 .inactive-badge {
  color: #ccc;
  font-weight:700;
  background: #f4f4f4;
  border-radius:10px;
  padding:2px 10px;
  font-size:1em;
  margin-top:2px;
 }
 body[data-theme='dark'] .inactive-badge { background:#24262c;color:#666;}
 .zone-status-on {
  color: var(--success);
  font-weight: 600;
  animation: statusGlow 1.5s ease-in-out infinite alternate;
  text-shadow:0 0 6px #32c36699;
 }
 .zone-status-off { color:#aaa; }
 .zones-wrapper {
  display: flex;
  flex-wrap: wrap;
  gap: 22px;
  justify-content: center;
  width: 100%;
 }
 .zone-container {
  border: 1.6px solid var(--neutral);
  background: var(--card);
  border-radius: 13px;
  box-shadow: 0 2px 14px #1976d21a;
  padding: 15px 13px 13px 13px;
  display: flex;
  flex-direction: column;
  gap: 9px;
  align-items: center;
  min-width: 295px;
  max-width: 340px;
  margin-bottom: 8px;
  text-align: center;
 }
 .time-duration-container {
  display: flex;
  flex-direction: column;
  gap: 9px;
  width: 100%;
  align-items: center;
  margin: 8px 0;
 }
 .enable-input {
  display: flex;
  flex-direction: row;
  align-items: center;
  gap: 7px;
  justify-content: center;
  width: 100%;
  margin-bottom: 0;
 }
 .days-container, .manual-control-container {
  justify-content: center;
  width: 100%;
  display: flex;
 }
 .days-container { gap:7px; flex-wrap:wrap; }
 .checkbox-container {
  display: flex;
  align-items: center;
  gap: 4px;
  font-size: 0.97em;
  border-radius: 6px;
  padding: 2px 7px;
  transition: background .19s;
  justify-content: center;
 }
 .checkbox-container:hover, .checkbox-container:focus-within {
  background: #e9f2fe;
 }
 body[data-theme='dark'] .checkbox-container:hover, body[data-theme='dark'] .checkbox-container:focus-within {
  background: #263241;
 }
 .checkbox-container input[type='checkbox']:checked+label {
  background: #1976d2; color:#fff; border-radius:5px; padding:2px 7px;
  font-weight:600; box-shadow:0 1px 5px #1976d261;
  transition:background .17s;
 }
 .checkbox-container label { cursor:pointer; padding:2px 2px; transition:background .14s, color .14s;}
 input[type='checkbox'] { accent-color: #1976d2; }
 .enable-input input[type='checkbox'] { width:1.2em;height:1.2em;}
 input[type='number'] {
  font-size:1.07em; padding:4px 7px; border-radius:6px; border:1.1px solid #b6c8e2; width:4.3em; margin-left:4px;
  text-align: center;
  background: var(--card);
  color: var(--font);
  transition: background .2s, color .2s, border .15s;
 }
 label { margin-right:2px; }
 button, input[type='submit'] {
  background: var(--primary); color: #fff; font-weight: 500; border: none;
  border-radius: 9px; padding: 8px 20px; font-size: 1em; cursor: pointer;
  transition: background .16s, box-shadow .17s, transform .11s;
  box-shadow:0 2px 6px #1976d224;
  text-align: center;
 }
 button:active { transform:scale(0.97);}
 button[disabled], .manual-control-container button[disabled] {
  background: #d4dbe0; color: #888; cursor:not-allowed; box-shadow:none;
 }
 .manual-control-container .turn-on-btn:not([disabled]) {background: var(--success);}
 .manual-control-container .turn-off-btn:not([disabled]) {background: var(--danger);}
 .footer-links {
  border-top: 1.5px solid #dae7f5;
  margin: 38px 0 0 0; padding-top:18px;
  text-align:center; font-size:1.09em;
  background: var(--footer);
  border-radius:0 0 12px 12px;
  transition:background .2s;
 }
 @media(max-width:820px){
  .container{padding:9px;}
  .zones-wrapper{flex-direction: column;align-items:center;}
  .summary-row{gap:10px;}
 }
 @media(max-width:520px){
  .summary-block{min-width:80px;font-size:1em;}
  .zone-container{padding:7px 4px;}
  .footer-links{font-size:.99em;}
 }
 /* --- PLACE THESE LAST! --- */
 body[data-theme='dark'], body[data-theme='dark'] :root {
  --bg: #181a1b !important;
  --card: #20252a !important;
  --font: #f5f7fa !important;
  --footer: #202830 !important;
  --neutral: #394b5b !important;
  --shadow: 0 2px 16px rgba(20,32,45,.12) !important;
 }
 body:not([data-theme='dark']), body:not([data-theme='dark']) :root {
  --bg: #f5f7fa !important;
  --card: #fff !important;
  --font: #1a1a1a !important;
  --footer: #fafdff4b !important;
  --neutral: #bed9ff !important;
  --shadow: 0 2px 16px rgba(80,160,255,.08) !important;
 }
  </style> 
 </head>
 <body>
  <header>
    <span style='font-size:1.75em;font-weight:700;'>💧ESP32 Irrigation Dashboard</span>
    <span style='font-size:1.13em;'>
      🕒 <span id='clock'>)" + String(timeStr) + R"(</span>
      &nbsp;&nbsp; 🗓 <span id='date'>)" + String(dateStr) + R"(</span>
    </span>
  </header>
  <div class='container'>
    <button id='toggle-darkmode-btn' style='margin:16px auto 8px auto;display:block;min-width:125px;'>🌗 Dark Mode</button>
    <div class='summary-row'>
      <div class='summary-block'><span class='icon'>📍</span><br>)" + cityName + R"(</div>
      <div class='summary-block'><span class='icon'>🌬</span><br>)" + (isnan(ws) ? "--" : String(ws,1) + " m/s") + R"(</div>
      <div class='summary-block'><span class='icon'>🌡</span><br>)" + (isnan(temp) ? "--" : String(temp,1) + " ℃") + R"(</div>
      <div class='summary-block'><span class='icon'>💧</span><br>)" + (isnan(hum) ? "--" : String(hum) + " %") + R"(</div>
      <div class='summary-block'><span class='icon'>🌤</span><br>)" + cond + R"(</div>
      <div class='summary-block tank-row'><span class='icon'>🚰</span><progress value=')" + String(tankPct) + R"(' max='100'></progress>)" + String(tankPct) + R"(%<br>)" + tankStatusStr + R"(</div>
      <div class='summary-block'><span class='icon' title='Rain Delay'>🌧</span><br>)" +
        (rainActive ? "<span class='active-badge'>Active</span>" : "<span class='inactive-badge'>Off</span>") + R"(</div>
      <div class='summary-block'><span class='icon' title='Wind Delay'>💨</span><br>)" +
        (windActive ? "<span class='active-badge'>Active</span>" : "<span class='inactive-badge'>Off</span>") + R"(</div>
    </div>
    <form action='/submit' method='POST'><div class='zones-wrapper'>)";

  // --- ZONE EDITORS ---
  static const char* dayNames[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  for (int zone = 0; zone < Zone; zone++) {
    html += "<div class='zone-container'>"
            "<p style='margin:0 0 4px 0'><strong>Zone " + String(zone + 1) + ":</strong></p>"
            "<p>Status: <span class='" + String(zoneActive[zone] ? "zone-status-on" : "zone-status-off") + "'>"
            + String(zoneActive[zone] ? "Running" : "Off") + "</span></p>";

    // Days checkboxes
    html += "<div class='days-container'>";
    for (int d = 0; d < 7; d++) {
      String chk = days[zone][d] ? "checked" : "";
      html += "<div class='checkbox-container'>"
              "<input type='checkbox' name='day" + String(zone) + "_" + d + "' id='day" + String(zone) + "_" + d + "' " + chk + ">"
              "<label for='day" + String(zone) + "_" + d + "'>" + dayNames[d] + "</label>"
              "</div>";
    }
    html += "</div>";

    // Times and durations (column layout)
    html += "<div class='time-duration-container'>"
      "<div class='enable-input'>"
        "<label>Start Time 1:</label>"
        "<input type='number' name='startHour" + String(zone) + "' min='0' max='23' value='" + String(startHour[zone]) + "' required>"
        "<span>:</span>"
        "<input type='number' name='startMin"  + String(zone) + "' min='0' max='59' value='" + String(startMin[zone]) + "' required>"
      "</div>"
      "<div class='enable-input'>"
        "<label>Start Time 2:</label>"
        "<input type='number' name='startHour2" + String(zone) + "' min='0' max='23' value='" + String(startHour2[zone]) + "'>"
        "<span>:</span>"
        "<input type='number' name='startMin2"  + String(zone) + "' min='0' max='59' value='" + String(startMin2[zone]) + "'>"
      "</div>"
      "<div class='enable-input' style='justify-content:center;'>"
        "<input type='checkbox' id='enableStartTime2" + String(zone) + "' name='enableStartTime2" + String(zone) + "'" + (enableStartTime2[zone] ? " checked" : "") + ">"
        "<label for='enableStartTime2" + String(zone) + "'>Start 2 On/Off</label>"
      "</div>"
      "<div class='enable-input'>"
        "<label>Run Time (Min):</label>"
        "<input type='number' name='durationMin"  + String(zone) + "' min='0' value='" + String(durationMin[zone]) + "' required>"
        "<label style='margin-left:8px;'>Sec:</label>"
        "<input type='number' name='durationSec"  + String(zone) + "' min='0' max='59' value='" + String(durationSec[zone]) + "' required>"
      "</div>"
    "</div>";

    // Manual controls
    html += "<div class='manual-control-container'>";
    if (zoneActive[zone]) {
      html += "<button type='button' class='turn-on-btn'  data-zone='" + String(zone) + "' disabled>▶️ On</button>";
      html += "<button type='button' class='turn-off-btn' data-zone='" + String(zone) + "'>⏹ Off</button>";
    } else {
      html += "<button type='button' class='turn-on-btn'  data-zone='" + String(zone) + "'>▶️ On</button>";
      html += "<button type='button' class='turn-off-btn' data-zone='" + String(zone) + "' disabled>⏹ Off</button>";
    }
    html += "</div></div>";
  }
  html += "</div><div style='display:flex;justify-content:center;width:100%;'><button type='submit'>Update Schedule</button></div></form>";

  // LCD Backlight toggle
  html += "<div style='text-align:center; margin:25px 0 0 0;'>"
        "<button type='button' id='toggle-backlight-btn' style='min-width:170px;'>Toggle LCD Backlight</button>"
        "</div>";

  // Footer Links (with card style)
  html += "<div class='footer-links'>"
          "<a href='/events'>📒 Event Log</a>"
          "<span style='margin:0 9px;color:#b3b3b3;'>|</span>"
          "<a href='/tank'>🚰 Tank Calibration</a>"
          "<span style='margin:0 9px;color:#b3b3b3;'>|</span>"
          "<a href='/setup'>⚙️ Setup</a>"
          "<span style='margin:0 9px;color:#b3b3b3;'>|</span>"
          "<a href='https://openweathermap.org/city/" + cityName + "' target='_blank'>🌤 Weather</a>"
          "</div></div>";

  // --- SCRIPTS ---
  html += R"(<script>
 function updateClockAndDate(){
  const now=new Date();
  const h=now.getHours().toString().padStart(2,'0');
  const m=now.getMinutes().toString().padStart(2,'0');
  const s=now.getSeconds().toString().padStart(2,'0');
  document.getElementById('clock').textContent=h+':'+m+':'+s;
  const d=now.getDate().toString().padStart(2,'0');
  const mo=(now.getMonth()+1).toString().padStart(2,'0');
  const y=now.getFullYear();
  document.getElementById('date').textContent=d+'/'+mo+'/'+y;
 }
 setInterval(updateClockAndDate,1000);

 // --- Robust Dark Mode Toggle ---
 function setTheme(theme) {
  if(theme==='dark') {
    document.body.setAttribute('data-theme','dark');
    localStorage.setItem('theme', 'dark');
    document.getElementById('toggle-darkmode-btn').textContent = "☀️ Light Mode";
  } else {
    document.body.removeAttribute('data-theme');
    localStorage.setItem('theme', 'light');
    document.getElementById('toggle-darkmode-btn').textContent = "🌗 Dark Mode";
  }
 }
 function autoTheme() {
  const stored = localStorage.getItem('theme');
  if(stored==='dark') setTheme('dark');
  else if(stored==='light') setTheme('light');
  else if(window.matchMedia('(prefers-color-scheme: dark)').matches) setTheme('dark');
  else setTheme('light');
 }
 window.addEventListener('DOMContentLoaded',()=>{
  autoTheme();
  document.getElementById('toggle-darkmode-btn').onclick = function() {
    const cur = document.body.getAttribute('data-theme')==='dark' ? 'dark' : 'light';
    setTheme(cur==='dark' ? 'light' : 'dark');
  };

  document.querySelectorAll('.turn-on-btn').forEach(btn=>{
    btn.addEventListener('click',()=>{
      const z=btn.dataset.zone;
      fetch('/valve/on/'+z,{method:'POST'}).then(()=>location.reload());
    });
  });
  document.querySelectorAll('.turn-off-btn').forEach(btn=>{
    btn.addEventListener('click',()=>{
      const z=btn.dataset.zone;
      fetch('/valve/off/'+z,{method:'POST'}).then(()=>location.reload());
    });
  });
  const tb=document.getElementById('toggle-backlight-btn');
  if(tb){tb.addEventListener('click',()=>{fetch('/toggleBacklight',{method:'POST'}).then(r=>r.text()).then(alert);});}
 });
    </script></body></html>)";
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

    if (server.hasArg("durationMin"+String(z)))
  durationMin[z] = server.arg("durationMin"+String(z)).toInt();
    
    if (server.hasArg("durationSec"+String(z)))
  durationSec[z] = server.arg("durationSec"+String(z)).toInt();

    enableStartTime2[z] = server.hasArg("enableStartTime2"+String(z));
  }

  saveSchedule();
  updateCachedWeather();  // (optional, to prime the cache)
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleSetupPage() {
  loadConfig();
  String html;

  // Header and CSS
  html += "<!DOCTYPE html><html lang=\"en\"><head>";
  html += "<meta charset=\"UTF-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  html += "<title>Setup – Irrigation System</title>";
  html += "<link href=\"https://fonts.googleapis.com/css?family=Roboto:400,500&display=swap\" rel=\"stylesheet\">";
  html += R"rawliteral(
  <style>
  :root {
    --primary: #2684d7;
    --primary-light: #49b4fa;
    --bg: #f5f7fa;
    --card-bg: #fff;
    --text: #1d2328;
    --shadow: 0 6px 22px rgba(0,0,0,0.12);
    --header: #20242f;
    --success: #30be72;
    --danger: #d34242;
    --input-bg: #f9fcff;
  }
  body {
    margin:0;padding:0;
    font-family:'Roboto',sans-serif;
    background:var(--bg);
    color:var(--text);
    transition: background .22s, color .18s;
  }
  header {
    position:sticky;top:0;
    background:var(--header);
    color:#fff;
    padding:14px 0;
    text-align:center;
    font-size:1.4em;
    font-weight:500;
    box-shadow:0 2px 8px rgba(0,0,0,0.2);
  }
  #toggle-darkmode-btn {
    position:fixed;top:12px;right:12px;
    background:none;border:none;cursor:pointer;
    width:40px;height:24px;
  }
  .theme-switch {
    width:34px;height:16px;
    background:#e4ebf0;
    border-radius:12px;
    border:2px solid #b1c9dd;
    position:relative;
    transition:background .2s, border-color .2s;
  }
  .theme-switch[data-dark="1"] {
    background:#141c25;
    border-color:var(--primary);
  }
  .theme-switch .dot {
    position:absolute;
    top:1px;left:1px;
    width:14px;height:14px;
    background:#ffe94b;
    border-radius:50%;
    transition:left .2s, background .2s;
  }
  .theme-switch[data-dark="1"] .dot {
    left:18px;
    background:var(--primary);
  }
  main {
    display:flex;
    justify-content:center;
    padding:20px 10px;
  }
  .setup-card {
    background:var(--card-bg);
    padding:24px;
    border-radius:8px;
    box-shadow:var(--shadow);
    width:100%;max-width:420px;
  }
  section {
    margin-bottom:20px;
  }
  section legend {
    font-weight:500;
    font-size:1.05em;
    color:var(--primary);
    margin-bottom:6px;
    padding-left:6px;
    border-left:3px solid var(--primary);
  }
  .form-group { margin-bottom:12px; }
  .form-row { display:flex; gap:8px; align-items:center; }
  label { min-width:80px; }
  input[type=text],input[type=number] {
    flex:1;
    padding:8px;
    border:1px solid #ccd8e6;
    border-radius:4px;
    background:var(--input-bg);
    transition:border-color .15s;
  }
  input:focus { border-color:var(--primary); outline:none; }
  input.full-width { width:100%; }
  input.small-input { width:6ch; }
  .tooltip { position:relative; }
  .tooltip .tip {
    visibility:hidden;
    position:absolute; top:24px; left:0;
    background:#333; color:#fff;
    padding:6px; border-radius:4px;
    font-size:.88em;
    white-space:nowrap;
  }
  .tooltip:hover .tip { visibility:visible; }
  .checkbox-group {
    display:flex; align-items:center; gap:6px;
    margin-bottom:10px;
  }
  input[type=checkbox] { accent-color:var(--primary); }
  .zone-pin-row {
    display:flex; align-items:center; gap:6px; margin-bottom:6px;
  }
  .zone-pin-row label { width:60px; }
  .zone-pin-input { width:6ch; }
  .btn-row {
    display:flex; gap:8px; flex-wrap:wrap; margin-top:16px;
    justify-content:space-between;
  }
  .btn, .cancel-btn, .restore-btn {
    flex:1; min-width:100px;
    padding:10px 0;
    border:none;border-radius:4px;
    font-weight:500;
    cursor:pointer;
    transition:background .18s;
  }
  .btn { background:var(--primary); color:#fff; }
  .btn:hover { background:var(--primary-light); }
  .cancel-btn { background:#e0e4e9; color:var(--text); }
  .cancel-btn:hover { background:#d0d4da; }
  .restore-btn { background:#ffaa3c; color:#fff; }
  .restore-btn:hover { background:#ffb95c; } 
  body[data-theme='dark'] {
    background:#181a1b; color:#e3e7ea;
  }
  body[data-theme='dark'] .setup-card {
    background:#23272e;
  }
  body[data-theme='dark'] input {
    background:#2a2d34; color:#e3e7ea; border-color:#393d46;
  }
  body[data-theme='dark'] .cancel-btn {
    background:#2a2d34; color:#e3e7ea;
  }
  #setup-msg {
    margin-top:12px; text-align:center; font-weight:500;
  }
  #setup-msg.success { color:var(--success); }
  #setup-msg.error   { color:var(--danger); }
  </style>
  )rawliteral";

  // --- BODY ---
  html += "</head><body>";
  html += R"rawliteral(
 <header>System Setup</header>
 <button id="toggle-darkmode-btn" aria-label="Toggle dark mode">
  <span id="switchbox" class="theme-switch"><span class="dot"></span></span>
 </button>
 <main>
  <div class="setup-card">
    <form id="setupForm" action="/configure" method="POST" autocomplete="off">

      <section>
        <legend>Weather Settings</legend>
        <div class="form-group form-row">
          <label for="apiKey">API Key</label>
          <input class="full-width" type="text" id="apiKey" name="apiKey" value=")rawliteral"
       + apiKey +
       R"rawliteral(" required maxlength="64" placeholder="API Key">
          <span class="tooltip"><span class="tip">Your weather API key.</span></span>
        </div>
        <div class="form-group form-row">
          <label for="city">City ID</label>
          <input class="full-width" type="text" id="city" name="city" value=")rawliteral"
       + city +
       R"rawliteral(" required maxlength="32" placeholder="City ID">
          <span class="tooltip"><span class="tip">City ID or provider location ID.</span></span>
        </div>
      </section>

      <section>
        <legend>Time & Location</legend>
        <div class="form-group form-row">
          <label for="dstOffset">UTC Offset</label>
          <input class="small-input" type="number" id="dstOffset" name="dstOffset"
            min="-12" max="14" step="0.5" value=")rawliteral"
       + String(tzOffsetHours,1) +
       R"rawliteral(" required placeholder="UTC Offset">
          <span class="tooltip"><span class="tip">e.g. +9.5</span></span>
        </div>
      </section>

      <section>
        <legend>Feature Toggles</legend>
        <div class="checkbox-group">
          <input type="checkbox" id="windCancelEnabled" name="windCancelEnabled" 
            )rawliteral" + String(windDelayEnabled ? " checked" : "") + R"rawliteral(>
          <label for="windCancelEnabled">Enable Wind Delay</label>
          <span class="tooltip"><span class="tip">Pause when wind is high.</span></span>
        </div>
        <div class="form-group form-row">
          <label for="windSpeedThreshold">Wind Threshold</label>
          <input class="small-input" type="number" id="windSpeedThreshold" name="windSpeedThreshold"
            min="0" step="0.1" value=")rawliteral"
       + String(windSpeedThreshold,1) +
       R"rawliteral(" required placeholder="m/s">
          <span class="tooltip"><span class="tip">Threshold in m/s.</span></span>
        </div>
        <div class="checkbox-group">
          <input type="checkbox" id="rainDelay" name="rainDelay"
            )rawliteral" + String(rainDelayEnabled ? " checked" : "") + R"rawliteral(>
          <label for="rainDelay">Enable Rain Delay</label>
          <span class="tooltip"><span class="tip">Pause on rain.</span></span>
        </div>
        <div class="checkbox-group">
          <input type="checkbox" id="justUseTank" name="justUseTank"
            )rawliteral" + String(justUseTank ? " checked" : "") + R"rawliteral(>
          <label for="justUseTank">Only Use Tank</label>
        </div>
        <div class="checkbox-group">
          <input type="checkbox" id="justUseMains" name="justUseMains"
            )rawliteral" + String(justUseMains ? " checked" : "") + R"rawliteral(>
          <label for="justUseMains">Only Use Mains</label>
        </div>
      </section>

      <section>
        <legend>Zone Pins</legend>
 )rawliteral";
  for (uint8_t i = 0; i < Zone; i++) {
    html += "<div class=\"zone-pin-row\">"
            "<label>Zone " + String(i+1) + ":</label>"
            "<input class=\"zone-pin-input\" type=\"number\" name=\"zonePin" + String(i) +
            "\" min=\"0\" max=\"39\" value=\"" + String(zonePins[i]) + "\">"
            "</div>";
  }
  html += R"rawliteral(
        <div class="zone-pin-row">
          <label>Main:</label>
          <input class="zone-pin-input" type="number" id="mainsPin" name="mainsPin"
            min="0" max="39" value=")rawliteral" + String(mainsPin) + R"rawliteral(">
        </div>
        <div class="zone-pin-row">
          <label>Tank:</label>
          <input class="zone-pin-input" type="number" id="tankPin" name="tankPin"
            min="0" max="39" value=")rawliteral" + String(tankPin) + R"rawliteral(">
        </div>
      </section>

  <div class="btn-row">
    <button type="submit" class="btn">Save</button>
    <button type="button" class="cancel-btn" onclick="location.href='/'">Home</button>
    <button type="button" class="restore-btn" onclick="location.href='/tank'">Calibrate Tank</button>
  </div>
    </form>
    <div id="setup-msg"></div>
  </div>
 </main>
 )rawliteral";

  // --- SCRIPTS: Dark mode, validation ---
  html += R"rawliteral(
 <script>
  function setTheme(theme) {
    if(theme==='dark') {
      document.body.setAttribute('data-theme','dark');
      localStorage.setItem('theme', 'dark');
      document.getElementById('toggle-darkmode-btn').title = "Switch to light mode";
      document.getElementById('toggle-darkmode-btn').innerHTML = "<span id='switchbox' class='theme-switch' data-dark='1'><span class='dot'></span></span>";
    } else {
      document.body.removeAttribute('data-theme');
      localStorage.setItem('theme', 'light');
      document.getElementById('toggle-darkmode-btn').title = "Switch to dark mode";
      document.getElementById('toggle-darkmode-btn').innerHTML = "<span id='switchbox' class='theme-switch'><span class='dot'></span></span>";
    }
  }
  function autoTheme() {
    const stored = localStorage.getItem('theme');
    if(stored==='dark') setTheme('dark');
    else if(stored==='light') setTheme('light');
    else if(window.matchMedia('(prefers-color-scheme: dark)').matches) setTheme('dark');
    else setTheme('light');
  }
  window.addEventListener('DOMContentLoaded',()=>{
    autoTheme();
    document.getElementById('toggle-darkmode-btn').onclick = function() {
      const cur = document.body.getAttribute('data-theme')==='dark' ? 'dark' : 'light';
      setTheme(cur==='dark' ? 'light' : 'dark');
    };
  });

  // Simple feedback/validation (optional)
  document.getElementById('setupForm').addEventListener('input', e => {
    // highlight empty required
    ['apiKey','city'].forEach(id => {
      const el = document.getElementById(id);
      el.style.borderColor = el.value.trim() ? '' : 'var(--danger)';
    });
  });
 </script>
 </body></html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleLogPage() {  
  File f = LittleFS.open("/events.csv", "r");
  if (!f) {
    server.send(404, "text/plain", "No event log found");
    return;
  }

  String html = "<!DOCTYPE html><html lang='en'><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>Event Log</title>"
    "<link href='https://fonts.googleapis.com/css?family=Roboto:400,700&display=swap' rel='stylesheet'>"
    "<style>"
      "body { font-family:'Roboto',sans-serif; background:linear-gradient(135deg,#eaf6fb,#d8ecff); margin:0; }"
      "header { background:linear-gradient(90deg,#0059b2 0%,#48c6ef 100%); color:#fff; text-align:center; padding:28px 0 14px; font-size:1.6em; border-bottom-left-radius:18px; border-bottom-right-radius:18px; box-shadow:0 2px 10px rgba(0,60,120,0.08); margin-bottom:0; }"
      ".container { max-width:950px; margin:38px auto 24px; background:#fff; border-radius:18px; box-shadow:0 6px 32px rgba(0,0,0,0.08); padding:30px 28px 18px 28px; min-height:60vh; }"
      "h2 { margin-top:0; color:#0073e6; letter-spacing:0.5px; font-weight:500; font-size:1.25em; }"
      "form { margin-bottom: 16px; text-align:right; }"
      "button { padding:9px 16px; background:#e53935; color:#fff; border:none; border-radius:8px; cursor:pointer; font-size:1em; font-weight:500; box-shadow:0 2px 7px rgba(210,100,100,0.09); transition:background .17s; }"
      "button:hover { background:#b71c1c; }"
      "table { width:100%; border-collapse:collapse; margin-top:18px; background:#f5fcff; border-radius:12px; box-shadow:0 2px 16px rgba(120,180,240,0.07); overflow:hidden; }"
      "th, td { padding:11px 8px; border:1px solid #d8e7fa; text-align:left; font-size:0.97em; }"
      "th { background:#e0ecf7; font-weight:600; color:#1976d2; }"
      "tr:nth-child(even) { background:#f8fafc; }"
      "tr:hover { background:#e2f1ff; }"
      ".icon { font-size:1.09em; vertical-align:middle; margin-right:5px; }"
      ".details-cell { font-family:monospace; font-size:0.99em; }"
      ".back-link { display:inline-block; margin-top:22px; color:#1976d2; text-decoration:none; font-size:1.08em; }"
      ".back-link:hover { text-decoration:underline; }"
      "@media (max-width:650px) {"
        ".container { padding:10px 3vw; }"
        "th,td { font-size:0.95em; padding:8px 3px; }"
        "header { font-size:1.2em; padding:16px 0 10px; }"
      "}"
    "</style></head><body>"
    "<header>📜 Irrigation Event Log</header>"
    "<div class='container'>"
    "<h2>Recent Events</h2>"
    "<form method='POST' action='/clearevents'>"
      "<button type='submit' onclick='return confirm(\"Clear all logs?\");'>🗑 Clear Log</button>"
    "</form>"
    "<table>"
      "<tr>"
        "<th>🕒 Timestamp</th>"
        "<th>💧 Zone</th>"
        "<th>⚡ Event</th>"
        "<th>🔗 Source</th>"
        "<th>🌧 Rain Delay</th>"
        "<th>📋 Details</th>"
      "</tr>";

  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() < 5) continue;

    int idx1 = line.indexOf(',');
    int idx2 = line.indexOf(',', idx1 + 1);
    int idx3 = line.indexOf(',', idx2 + 1);
    int idx4 = line.indexOf(',', idx3 + 1);
    int idx5 = line.indexOf(',', idx4 + 1);
    int idx6 = line.indexOf(',', idx5 + 1);
    int idx7 = line.indexOf(',', idx6 + 1);
    int idx8 = line.indexOf(',', idx7 + 1);

    String ts   = line.substring(0, idx1);
    String zone = line.substring(idx1 + 1, idx2);
    String ev   = line.substring(idx2 + 1, idx3);
    String src  = line.substring(idx3 + 1, idx4);
    String rd   = line.substring(idx4 + 1, idx5);

    // Weather fields (may be missing for old logs)
    String temp = (idx6 > idx5) ? line.substring(idx5 + 1, idx6) : "";
    String hum  = (idx7 > idx6) ? line.substring(idx6 + 1, idx7) : "";
    String wind = (idx8 > idx7) ? line.substring(idx7 + 1, idx8) : "";
    String cond = (idx8 > 0)    ? line.substring(idx8 + 1) : "";

    String details;
    if (temp.length()) {
      details = "T=" + temp + "°C, H=" + hum + "%, W=" + wind + "m/s, " + cond;
    } else {
      details = "n/a";
    }

    html += "<tr>"
              "<td>" + ts         + "</td>"
              "<td>" + zone       + "</td>"
              "<td>" + ev         + "</td>"
              "<td>" + src        + "</td>"
              "<td>" + rd         + "</td>"
              "<td class='details-cell'>" + details + "</td>"
            "</tr>";
  }

  f.close();

  html += "</table>"
          "<a class='back-link' href='/'>⬅ Back to Home</a>"
          "</div></body></html>";

  server.send(200, "text/html", html);
}

void handleConfigure() {
  // --- Save the old API key to detect changes
  String oldApiKey = apiKey;
  

  // 1) Parse POST fields (field names must match your form!)
  if (server.hasArg("apiKey")) apiKey = server.arg("apiKey");
  if (server.hasArg("city"))   city   = server.arg("city");
  if (server.hasArg("dstOffset")) tzOffsetHours = server.arg("dstOffset").toFloat();

  // Checkbox logic
  rainDelayEnabled   = server.hasArg("rainDelay");
  windDelayEnabled   = server.hasArg("windCancelEnabled");
  justUseTank        = server.hasArg("justUseTank");
  justUseMains       = server.hasArg("justUseMains");

  windSpeedThreshold = server.arg("windSpeedThreshold").toFloat();

  for (int i = 0; i < Zone; i++) {
    if (server.hasArg("zonePin" + String(i))) {
      zonePins[i] = server.arg("zonePin" + String(i)).toInt();
    }
  }
  if (server.hasArg("mainsPin")) mainsPin = server.arg("mainsPin").toInt();
  if (server.hasArg("tankPin"))  tankPin  = server.arg("tankPin").toInt();

  // --- Only restart if the API key changed
  if (apiKey != oldApiKey) shouldRestart = true;

  saveConfig();
  loadConfig();

  Serial.printf("[CONFIG] Saved - RainDelay:%d WindDelay:%d JustTank:%d JustMains:%d  RESTART:%d\n",
    rainDelayEnabled, windDelayEnabled, justUseTank, justUseMains, shouldRestart);

  // --- Confirmation and redirect back to setup
  String html = "<!DOCTYPE html><html lang='en'><head>"
                "<meta charset='UTF-8'>"
                "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
                "<meta http-equiv='refresh' content='2;url=/setup' />"
                "<title>Settings Saved</title>"
                "<style>body{font-family:Arial,sans-serif;text-align:center;padding:40px;}h1{color:#2E86AB;}p{font-size:1.1em;}</style>"
                "</head><body>"
                "<h1>✅ Settings Saved</h1>"
                "<p>Settings have been saved.<br>You’ll be returned to Setup.</p>"
                "</body></html>";
  server.send(200, "text/html", html);


      delay(1500);  // Let browser finish receiving the response
    ESP.restart();
  }








