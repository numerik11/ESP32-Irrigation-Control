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
float  tzOffsetHours      = 0.0;
float  windSpeedThreshold = 5.0f;
float  lastRainAmount = 0.0f;
int    startHour[Zone]   = {0};
int    startMin [Zone]   = {0};
int    startHour2[Zone]  = {0};
int    startMin2 [Zone]  = {0};
int    durationMin[Zone] = {0};
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
  
    updateCachedWeather();

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
  rainActive = false;

  // 2) Wind check
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
  zoneStartMs[z] = millis();
  zoneActive[z] = true;

  if (useGpioFallback) {
    digitalWrite(zonePins[z], LOW); // ON (active LOW)
    // Source selection logic
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
    // PCF8574 active-low outputs
    pcfOut.digitalWrite(valveChannel[z], LOW);
    // Source selection
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

  // Determine actual source
  const char* src = justUseTank   ? "Tank"
                     : justUseMains ? "Mains"
                     : isTankLow()   ? "Mains"
                                    : "Tank";

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
  if (useGpioFallback) {
    digitalWrite(zonePins[z], HIGH);
    digitalWrite(mainsPin, HIGH);
    digitalWrite(tankPin, HIGH);
  } else {
    pcfOut.digitalWrite(valveChannel[z], HIGH);
    pcfOut.digitalWrite(mainsChannel, HIGH);
    pcfOut.digitalWrite(tankChannel, HIGH);
  }

  zoneActive[z] = false;

  // Determine source
  const char* src = justUseTank   ? "Tank"
                     : justUseMains ? "Mains"
                     : isTankLow()   ? "Mains"
                                    : "Tank";

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
  if (zoneActive[z]) {
    if (useGpioFallback) {
      digitalWrite(zonePins[z], HIGH);
      digitalWrite(mainsPin, HIGH);
      digitalWrite(tankPin, HIGH);
    } else {
      pcfOut.digitalWrite(valveChannel[z], HIGH);
      pcfOut.digitalWrite(mainsChannel, HIGH);
      pcfOut.digitalWrite(tankChannel, HIGH);
    }
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
  html += "<title>ESP32 Irrigation System</title>";
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

    html += "<header><h1>💧ESP32 Irrigation System💧</h1></header>";
    html += "<div class='container'>";
      // Clock & weather
  html += "<p id='clock'>🕒 Current Time: " + currentTime + "</p>";
  html += "<p>📍 Location: " + cityName + "</p>";
  html += "<p id='weather-condition'>🌤 Condition: " + cond + "</p>";
  html += "<p id='temperature'>🌡 Temperature: " + String(temp) + " ℃</p>";
  html += "<p id='humidity'>💧 Humidity: " + String((int)hum) + " %</p>";
  html += "<p id='wind-speed'>🌬 Wind Speed: " + String(ws) + " m/s</p>";
      // Tank level
      int tankRaw = analogRead(TANK_PIN);
      int tankPct = map(tankRaw, 0, 1023, 0, 100);
      String tankStatus = (tankRaw < 250) ? "Low - Using Main" : "Normal - Using Tank";
      html += "<p> Tank Level: "
          "<progress id='tankLevel' value='" + String(tankPct) + "' max='100'></progress> "
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
      html += "<p style='text-align:center;'><a href='/tank'>Tank Calibration Tool</a></p>";
      html += "<p style='text-align:center;'><a href='/setup'>Setup Page</a></p>";
      html += "<p style='text-align:center;'><a href='https://openweathermap.org/city/" + city + "' target='_blank'>View Weather on Openweathermap.org</a></p>";

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

  saveSchedule();
  updateCachedWeather();  // (optional, to prime the cache)
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}


void handleSetupPage() {
  String html = "";

  // — START HTML —
  html += "<!DOCTYPE html><html lang=\"en\"><head>";
  html += "<meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  html += "<title>Setup - Irrigation System</title>";
  html += "<link href=\"https://fonts.googleapis.com/css?family=Roboto:400,500&display=swap\" rel=\"stylesheet\">";
  html += "<style>"
          ":root{--primary:#2E86AB;--primary-light:#379BD5;--bg:#f4f7fa;"
          "--card-bg:#fff;--text:#333;--accent:#F2994A;}"
          "body{font-family:'Roboto',sans-serif;background:var(--bg);"
          "color:var(--text);display:flex;justify-content:center;"
          "align-items:center;padding:20px;}"
          ".container{background:var(--card-bg);padding:30px;"
          "border-radius:12px;box-shadow:0 8px 24px rgba(0,0,0,0.1);"
          "width:100%;max-width:450px;}"
          ".container h1{text-align:center;color:var(--primary);"
          "margin-bottom:20px;}"
          ".form-group{margin-bottom:15px;}"
          ".form-group label{display:block;margin-bottom:6px;}"
          "input[type=text],input[type=number]{padding:10px;"
          "border:1px solid #ccc;border-radius:6px;}"
          /* full-width by default */
          "input.full-width{width:100%;}"
          /* narrow helpers */
          ".small-input{width:6ch;}"
          ".medium-input{width:8ch;}"
          ".checkbox-group{display:flex;align-items:center;"
          "margin-bottom:12px;}"
          ".checkbox-group label{margin-left:8px;}"
          ".btn{width:100%;padding:12px;background:var(--primary);"
          "color:#fff;border:none;border-radius:6px;cursor:pointer;"
          "transition:background .3s;}"
          ".btn:hover{background:var(--primary-light);}"
          "</style></head><body>";

  html += "<div class=\"container\"><h1>⚙️ System Setup</h1>";
  html += "<form action=\"/configure\" method=\"POST\">";

  // API Key (full-width)
  html += "<div class=\"form-group\"><label for=\"apiKey\">API Key</label>";
  html += "<input class=\"full-width\" type=\"text\" id=\"apiKey\" name=\"apiKey\" "
          "value=\"" + apiKey + "\" required></div>";

  // City ID (full-width)
  html += "<div class=\"form-group\"><label for=\"city\">City ID</label>";
  html += "<input class=\"full-width\" type=\"text\" id=\"city\" name=\"city\" "
          "value=\"" + city + "\" required></div>";

  // Timezone Offset (small)
  html += "<div class=\"form-group\"><label for=\"dstOffset\">Timezone Offset (hrs)</label>";
  html += "<input class=\"small-input\" type=\"number\" id=\"dstOffset\" name=\"dstOffset\" "
          "min=\"-12\" max=\"14\" step=\"0.5\" "
          "value=\"" + String(tzOffsetHours, 1) + "\" required></div>";

  // Wind Speed Threshold (small)
  html += "<div class=\"form-group\"><label for=\"windSpeedThreshold\">Wind Speed Threshold (m/s)</label>";
  html += "<input class=\"small-input\" type=\"number\" id=\"windSpeedThreshold\" name=\"windSpeedThreshold\" "
          "min=\"0\" step=\"0.1\" "
          "value=\"" + String(windSpeedThreshold, 1) + "\" required></div>";

  // Checkboxes
  html += "<div class=\"checkbox-group\">"
          "<input type=\"checkbox\" id=\"windCancelEnabled\" name=\"windCancelEnabled\""
          + String(windDelayEnabled ? " checked" : "") + ">"
          "<label for=\"windCancelEnabled\">Enable Wind Delay</label></div>";

  html += "<div class=\"checkbox-group\">"
          "<input type=\"checkbox\" id=\"rainDelay\" name=\"rainDelay\""
          + String(rainDelayEnabled ? " checked" : "") + ">"
          "<label for=\"rainDelay\">Enable Rain Delay</label></div>";

  html += "<div class=\"checkbox-group\">"
          "<input type=\"checkbox\" id=\"justUseTank\" name=\"justUseTank\""
          + String(justUseTank ? " checked" : "") + ">"
          "<label for=\"justUseTank\">Only Use Tank</label></div>";

  html += "<div class=\"checkbox-group\">"
          "<input type=\"checkbox\" id=\"justUseMains\" name=\"justUseMains\""
          + String(justUseMains ? " checked" : "") + ">"
          "<label for=\"justUseMains\">Only Use Mains</label></div>";

  // Zone pin configuration (medium)
  html += "<div class=\"form-group\"><label>Zone pins (If not using A6) Tank Pin IO36(Default)</label>";
  for (uint8_t i = 0; i < Zone; i++) {
    html += "Zone " + String(i+1) + ": "
         + "<input class=\"medium-input\" type=\"number\" name=\"zonePin" + String(i) + "\" "
         + "min=\"0\" max=\"39\" value=\"" + String(zonePins[i]) + "\"><br>";
  }
  html += "</div>";

  // Mains & Tank pins (medium)
  html += "<div class=\"form-group\"><label for=\"mainsPin\">Mains-source pin (GPIO)</label>";
  html += "<input class=\"medium-input\" type=\"number\" id=\"mainsPin\" name=\"mainsPin\" "
          "min=\"0\" max=\"39\" value=\"" + String(mainsPin) + "\"></div>";

  html += "<div class=\"form-group\"><label for=\"tankPin\">Tank-source pin (GPIO)</label>";
  html += "<input class=\"medium-input\" type=\"number\" id=\"tankPin\" name=\"tankPin\" "
          "min=\"0\" max=\"39\" value=\"" + String(tankPin) + "\"></div>";

  // Submit button
  html += "<button type=\"submit\" class=\"btn\">Save Settings</button>";

  // Close out
  html += "</form></div></body></html>";
  // — END HTML —

  server.send(200, "text/html", html);
}

void handleLogPage() {
  // 1) Make sure weather data is fresh, then parse it
  updateCachedWeather();
  DynamicJsonDocument js(2048);
  deserializeJson(js, cachedWeatherData);
  float temp      = js["main"]["temp"].as<float>();
  int   hum       = js["main"]["humidity"].as<int>();
  float ws        = js["wind"]["speed"].as<float>();
  String cond     = js["weather"][0]["main"].as<String>();
  String cityName = js["name"].as<String>();

  // Build a one-line weather info string
  String weatherInfo = "T=" + String(temp,1)  + "°C, "
                     + "H=" + String(hum)     + "%, "
                     + "W=" + String(ws,1)    + "m/s, "
                     + cond;

  // 2) Open the log file
  File f = LittleFS.open("/events.csv", "r");
  if (!f) {
    server.send(404, "text/plain", "No event log found");
    return;
  }

  // 3) Emit the HTML header + table setup
  String html = "<!DOCTYPE html><html lang='en'><head>"
                "<meta charset='UTF-8'>"
                "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
                "<title>Event Log</title>"
                "<link href='https://fonts.googleapis.com/css?family=Roboto:400,700&display=swap' rel='stylesheet'>"
                "<style>"
                  "body{font-family:'Roboto',sans-serif;background:#f0f4f8;margin:0;}"
                  "header{background:#007BFF;color:#fff;text-align:center;padding:20px;font-size:1.6em;}"
                  ".container{max-width:900px;margin:20px auto;background:#fff;border-radius:8px;"
                             "box-shadow:0 4px 8px rgba(0,0,0,0.1);padding:20px;}"
                  "table{width:100%;border-collapse:collapse;margin-top:20px;}"
                  "th,td{padding:10px;border:1px solid #ccc;text-align:left;font-size:0.95em;}"
                  "th{background:#e0ecf7;}tr:nth-child(even){background:#f9f9f9;}"
                  "button{padding:10px 15px;background:#d9534f;color:#fff;border:none;border-radius:4px;"
                         "cursor:pointer;}button:hover{background:#c9302c;}"
                  "a{display:inline-block;margin-top:15px;color:#007BFF;text-decoration:none;}"
                  "a:hover{text-decoration:underline;}"
                "</style></head><body>"
                "<header>📜 Irrigation Event Log</header>"
                "<div class='container'>"
                "<h2>Recent Events</h2>"
                "<form method='POST' action='/clearevents'>"
                  "<button type='submit' onclick='return confirm(\"Clear all logs?\");'>🗑 Clear Log</button>"
                "</form>"
                "<table>"
                  "<tr>"
                    "<th>Timestamp</th>"
                    "<th>Zone</th>"
                    "<th>Event</th>"
                    "<th>Source</th>"
                    "<th>Rain Delay</th>"
                    "<th>Details</th>"
                  "</tr>";

  // 4) Read each line, parse CSV fields + append weatherInfo in Details
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() < 5) continue;

    int idx1 = line.indexOf(',');
    int idx2 = line.indexOf(',', idx1 + 1);
    int idx3 = line.indexOf(',', idx2 + 1);
    int idx4 = line.indexOf(',', idx3 + 1);
    int idx5 = line.indexOf(',', idx4 + 1);

    String ts   = line.substring(0, idx1);
    String zone = line.substring(idx1 + 1, idx2);
    String ev   = line.substring(idx2 + 1, idx3);
    String src  = line.substring(idx3 + 1, idx4);
    String rd   = line.substring(idx4 + 1, idx5);

    html += "<tr>"
            "<td>" + ts         + "</td>"
            "<td>" + zone       + "</td>"
            "<td>" + ev         + "</td>"
            "<td>" + src        + "</td>"
            "<td>" + rd         + "</td>"
            "<td>" + weatherInfo+ "</td>"
          "</tr>";
  }
  f.close();

  // 5) Close out
  html += "</table>"
          "<a href='/'>⬅ Back to Home</a>"
          "</div></body></html>";

  server.send(200, "text/html", html);
}

void handleConfigure() {
  // 1) Parse POSTed fields
  apiKey             = server.arg("apiKey");
  city               = server.arg("city");
  tzOffsetHours      = server.arg("dstOffset").toFloat();
  rainDelayEnabled   = server.hasArg("rainDelay");
  windDelayEnabled   = server.hasArg("windCancelEnabled");
  windSpeedThreshold = server.arg("windSpeedThreshold").toFloat();
  justUseTank        = server.hasArg("justUseTank");
  justUseMains       = server.hasArg("justUseMains");
  for (int i = 0; i < Zone; i++) {
    if (server.hasArg("zonePin" + String(i)))
      zonePins[i] = server.arg("zonePin" + String(i)).toInt();
  }
  if (server.hasArg("mainsPin")) mainsPin = server.arg("mainsPin").toInt();
  if (server.hasArg("tankPin"))  tankPin  = server.arg("tankPin").toInt();

  // 2) Save to SPIFFS (no restart in saveConfig anymore)
  saveConfig();

  // 3) Build a confirmation page with a 5-second redirect back to "/"
  String html = "<!DOCTYPE html><html lang=\"en\"><head>"
              "<meta charset=\"UTF-8\">"
              "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
              "<meta http-equiv=\"refresh\" content=\"5;url=/\" />"
              "<title>Settings Saved</title>"
              "<style>"
                "body{font-family:Arial,sans-serif;text-align:center;padding:40px;}"
                "h1{color:#2E86AB;}p{font-size:1.1em;}"
              "</style>"
              "</head><body>"
              "<h1>✅ Settings Saved</h1>"
              "<p>Restarting… You’ll be returned to the home page shortly.</p>"
              "</body></html>";

  server.send(200, "text/html", html);

  // 4) Give the browser a moment to receive the page
  delay(500);

  // 5) Then reboot
  ESP.restart();
}


