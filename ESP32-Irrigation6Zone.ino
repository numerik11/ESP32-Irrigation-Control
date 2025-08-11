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
#include <math.h>

// -------------------------------
// Constants
// -------------------------------
static const uint8_t Zone = 6;

// KC868-A6 I2C pins (ESP32)
constexpr uint8_t I2C_SDA = 4;
constexpr uint8_t I2C_SCL = 15;

// Single shared I2C bus (bus 0), used by OLED + both PCF8574s
TwoWire I2Cbus = TwoWire(0);

// PCF8574 devices on A6 (inputs=0x22, relays=0x24)
PCF8574 pcfIn (&I2Cbus, 0x22, I2C_SDA, I2C_SCL);   // Digital Inputs
PCF8574 pcfOut(&I2Cbus, 0x24, I2C_SDA, I2C_SCL);   // Relays

#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET    -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Cbus, OLED_RESET);

// Wi-Fi & WebServer
WiFiManager wifiManager;
WebServer   server(80);
WiFiClient  client;

// --- Relay polarity control (flip to false if your board is active-HIGH) ---
static const bool RELAY_ACTIVE_LOW_PCF = true;  // true = LOW turns relay ON

// A6 Solenoid channels (PCF8574 pins)
// Zones on P0..P5; keep P6/P7 for Mains/Tank
const uint8_t valveChannel[Zone] = { P0, P1, P2, P3, P4, P5 };
const uint8_t mainsChannel       = P6;
const uint8_t tankChannel        = P7;

// === GPIO Fallback Configuration ===
bool useGpioFallback = false;
uint8_t zonePins[Zone] = {16, 17, 18, 19, 23, 25};
uint8_t mainsPin       = 26;
uint8_t tankPin        = 27;

// On-board LED & tank sensor
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
bool   days[Zone][7]          = {{false}};
bool   zoneActive[Zone]       = {false};
bool   pendingStart[Zone]     = {false};
bool   rainActive             = false;
bool   windActive             = false;
float  tzOffsetHours          = 0.0f;
float  windSpeedThreshold     = 5.0f;
float  lastRainAmount         = 0.0f;
int    startHour[Zone]        = {0};
int    startMin [Zone]        = {0};
int    startHour2[Zone]       = {0};
int    startMin2 [Zone]       = {0};
int    durationMin[Zone]      = {0};
int    durationSec[Zone]      = {0};
int    lastCheckedMinute[Zone] = { -1, -1, -1, -1, -1, -1 };
int    tankEmptyRaw           = 100;
int    tankFullRaw            = 900;

unsigned long zoneStartMs[Zone] = {0};
unsigned long lastScreenRefresh  = 0;
unsigned long lastWeatherUpdate  = 0;

const unsigned long SCREEN_REFRESH_MS = 1000;
const unsigned long WEATHER_INTERVAL = 3600000; // 1 hour

const uint8_t expanderAddrs[] = { 0x22, 0x24 }; // 0x22=input, 0x24=relays
const uint8_t I2C_HEALTH_DEBOUNCE = 10;
uint8_t i2cFailCount = 0;

// --- Efficiency timers ---
static const uint32_t I2C_CHECK_MS      = 1000;

// --- Internal state for throttling (GLOBAL; do not shadow in loop) ---
static uint32_t lastI2cCheck      = 0;

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
void handleClearEvents();
void handleTankCalibration();
String getDayName(int d);
String fetchWeather();
bool checkWindRain();
void RainScreen();
void checkI2CHealth();
void initGpioFallback();
bool initExpanders();
void toggleBacklight();
void printCurrentTime();
void logEvent(int zone, const char* eventType, const char* source, bool rainDelayed);
inline void selectWaterSource();

// --- Helpers to unify relay control ---
inline void driveZoneRelay(int z, bool on) {
  if (useGpioFallback) {
    digitalWrite(zonePins[z], on ? HIGH : LOW); // fallback active-HIGH
  } else {
    pcfOut.digitalWrite(
      valveChannel[z],
      (RELAY_ACTIVE_LOW_PCF ? (on ? LOW : HIGH) : (on ? HIGH : LOW))
    );
  }
}

inline void driveSourceRelays(bool useMains) {
  if (useGpioFallback) {
    digitalWrite(mainsPin, useMains ? HIGH : LOW);
    digitalWrite(tankPin,  useMains ? LOW  : HIGH);
  } else {
    // default both OFF
    pcfOut.digitalWrite(mainsChannel, RELAY_ACTIVE_LOW_PCF ? HIGH : LOW);
    pcfOut.digitalWrite(tankChannel,  RELAY_ACTIVE_LOW_PCF ? HIGH : LOW);
    if (useMains) {
      pcfOut.digitalWrite(mainsChannel, RELAY_ACTIVE_LOW_PCF ? LOW : HIGH);
    } else {
      pcfOut.digitalWrite(tankChannel,  RELAY_ACTIVE_LOW_PCF ? LOW : HIGH);
    }
  }
}

inline void selectWaterSource() {
  if (justUseTank)  { driveSourceRelays(false); return; } // Tank ON
  if (justUseMains) { driveSourceRelays(true);  return; } // Mains ON
  // Default when neither override is set: use Mains, ignore tank level
  driveSourceRelays(true);
}

void pulseRelay(int ch, int ms=150) {
  driveZoneRelay(ch, true);
  delay(ms);
  driveZoneRelay(ch, false);
}

static bool i2cPing(uint8_t addr) {
  I2Cbus.beginTransmission(addr);
  return (I2Cbus.endTransmission() == 0);
}

bool initExpanders() {
  // Probe first so we don’t fail on a transient library begin()
  bool haveIn  = i2cPing(0x22);
  bool haveOut = i2cPing(0x24);
  Serial.printf("[I2C] ping 0x22=%d  0x24=%d (1=ok)\n", haveIn, haveOut);
  if (!haveOut) return false;

  // Try a few times—some boards need a moment after power up
  for (int i = 0; i < 3 && !pcfOut.begin(); ++i) delay(5);
  for (int i = 0; i < 3 && !pcfIn.begin();  ++i) delay(5);

  // Relays default OFF by polarity (P0..P7)
  for (uint8_t ch = P0; ch <= P7; ch++) {
    pcfOut.pinMode(ch, OUTPUT);
    pcfOut.digitalWrite(ch, RELAY_ACTIVE_LOW_PCF ? HIGH : LOW);  // OFF
  }

  // Inputs with “pull-up like” behaviour (keep P0..P5 as your board uses)
  for (uint8_t ch = P0; ch <= P5; ch++) {
    pcfIn.pinMode(ch, INPUT);
    pcfIn.digitalWrite(ch, HIGH);
  }
  return true;
}

// --------------------------------

void setup() {
  Serial.begin(115200);
  I2Cbus.begin(I2C_SDA, I2C_SCL, 100000); // PCF8574 is 100kHz

  // ADC config for cleaner tank readings (doesn't affect control)
  analogReadResolution(12);
  analogSetPinAttenuation(TANK_PIN, ADC_11db);

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
    saveSchedule();
  }
  loadSchedule();

  // —— PCF8574 init or GPIO fallback ——  
  if (!initExpanders()) {
    Serial.println("⚠ PCF8574 relays not found at 0x24. Switching to GPIO fallback.");
    initGpioFallback();
    useGpioFallback = true;
  } else {
    useGpioFallback = false;
    checkI2CHealth();
  }

  pinMode(LED_PIN, OUTPUT);

  // OLED Initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 init failed");
    while (true) delay(100);
  }
  delay(200);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,10);
  display.print("Connect To");
  display.setCursor(0,30);
  display.print("ESPIrrigationAP");
  display.setCursor(0,50);
  display.setTextSize(1);
  display.print("Goto: http://192.168.4.1");
  display.display();

  // Wi-Fi Setup
  wifiManager.setTimeout(180);
  if (!wifiManager.autoConnect("ESPIrrigationAP")) {
    Serial.println("Failed to connect to WiFi. Restarting...");
    ESP.restart();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0,0);
    display.print("Connected!");
    display.setTextSize(1); // fix: was 1.5 (invalid)
    display.setCursor(0,20);
    display.print(WiFi.localIP().toString());
    display.display();
    delay(1500);
  }

  // NTP setup with precise offset handling
  int gmtOffsetSec = (int)lroundf(tzOffsetHours * 3600.0f);
  configTime(gmtOffsetSec, 0, "pool.ntp.org", "time.nist.gov");
  time_t now = time(nullptr);
  while (now < 1000000000) { delay(500); now = time(nullptr); }

  // OTA
  ArduinoOTA.setHostname("ESP32 Irrigation");
  ArduinoOTA.begin();

  // Routes
  server.on("/",            HTTP_GET,  handleRoot);
  server.on("/submit",      HTTP_POST, handleSubmit);
  server.on("/setup",       HTTP_GET,  handleSetupPage);
  server.on("/configure",   HTTP_POST, handleConfigure);
  server.on("/events",      HTTP_GET,  handleLogPage);
  server.on("/clearevents", HTTP_POST, handleClearEvents);
  server.on("/tank",        HTTP_GET,  handleTankCalibration);

  // Quick source toggle endpoints
  server.on("/source/mains", HTTP_POST, [](){
    justUseMains = true; justUseTank = false; saveConfig();
    driveSourceRelays(true);
    server.send(200, "text/plain", "Mains selected");
  });
  server.on("/source/tank",  HTTP_POST, [](){
    justUseTank = true;  justUseMains = false; saveConfig();
    driveSourceRelays(false);
    server.send(200, "text/plain", "Tank selected");
  });

  // Status JSON (REFRESHABLE)
  server.on("/status", HTTP_GET, [](){
  DynamicJsonDocument doc(512);

  // existing flags
  doc["rainDelayActive"] = rainActive;
  doc["windDelayActive"] = windActive;

  // tank snapshot (adds live tank info)
  int tankRaw = analogRead(TANK_PIN);
  int tankPct = map(tankRaw, tankEmptyRaw, tankFullRaw, 0, 100);
  tankPct = constrain(tankPct, 0, 100);
  bool tankLow = tankRaw < (tankEmptyRaw + (tankFullRaw - tankEmptyRaw) * 0.15f);
  doc["tankPct"] = tankPct;
  doc["tankLow"] = tankLow;

  // zones
  JsonArray zones = doc.createNestedArray("zones");
  for (int i = 0; i < Zone; i++) {
    JsonObject z = zones.createNestedObject();
    z["active"] = zoneActive[i];
    z["source"] = (justUseTank ? "Tank"
                 : (justUseMains ? "Mains"
                 : (isTankLow() ? "Mains" : "Tank")));
    unsigned long rem = 0;
    if (zoneActive[i]) {
      unsigned long elapsed = (millis() - zoneStartMs[i]) / 1000UL;
      unsigned long total   = (unsigned long)durationMin[i]*60UL + (unsigned long)durationSec[i];
      rem = (elapsed < total ? total - elapsed : 0UL);
    }
    z["remaining"] = rem;  // seconds
  }

  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
  });


  // Tank calibration set points
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
    server.on(String("/valve/on/")  + i, HTTP_POST, [i]() { turnOnValveManual(i); server.send(200, "text/plain", "OK");});
    server.on(String("/valve/off/") + i, HTTP_POST, [i]() { turnOffValveManual(i); server.send(200, "text/plain", "OK");});
  }

  // Stop all
  server.on("/stopall", HTTP_POST, [](){
    for (int z=0; z<Zone; ++z) if (zoneActive[z]) turnOffZone(z);
    server.send(200, "text/plain", "OK");
  });

  // Backlight toggle
  server.on("/toggleBacklight", HTTP_POST, [](){
    display.invertDisplay(true);
    delay(300);
    display.invertDisplay(false);
    server.send(200, "text/plain", "Backlight toggled");
  });

  // I2C quick test endpoint (pulse ALL P0..P7)
  server.on("/i2c-test", HTTP_GET, [](){
    if (useGpioFallback) { server.send(500, "text/plain", "Fallback active"); return; }
    for (uint8_t ch = P0; ch <= P7; ch++) {
      pcfOut.digitalWrite(ch, RELAY_ACTIVE_LOW_PCF ? LOW : HIGH);  delay(150);
      pcfOut.digitalWrite(ch, RELAY_ACTIVE_LOW_PCF ? HIGH: LOW );  delay(80);
    }
    server.send(200, "text/plain", "PCF8574 P0..P7 pulse OK");
  });

  // I2C scanner (debug)
  server.on("/i2c-scan", HTTP_GET, [](){
    String s = "I2C scan:\n";
    byte count = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      I2Cbus.beginTransmission(addr);
      if (I2Cbus.endTransmission() == 0) {
        s += " - Found 0x" + String(addr, HEX) + "\n";
        count++; delay(2);
      }
    }
    if (!count) s += " (no devices)\n";
    server.send(200, "text/plain", s);
  });

  // Outputs peek (debug)
  server.on("/outputs", HTTP_GET, [](){
    if (useGpioFallback) { server.send(200, "text/plain", "Fallback active (no PCF8574)"); return; }
    String s;
    for (uint8_t ch = 0; ch <= 7; ch++) {
      s += "P" + String(ch) + "=" + String(pcfOut.digitalRead(ch)) + "\n";
    }
    server.send(200, "text/plain", s);
  });

  // Download events CSV
  server.on("/download/events.csv", HTTP_GET, [](){
    if (LittleFS.exists("/events.csv")) {
      File f = LittleFS.open("/events.csv", "r");
      server.streamFile(f, "text/csv");
      f.close();
    } else {
      server.send(404, "text/plain", "No event log");
    }
  });

  server.begin();
}

// =================== Loop ===================
void loop() {
  const uint32_t now = millis();

  ArduinoOTA.handle();
  server.handleClient();
  wifiCheck();

  // throttle I²C health checks
  if (!useGpioFallback && (now - lastI2cCheck >= I2C_CHECK_MS)) {
    lastI2cCheck = now;
    checkI2CHealth();
  }

  // Update rain/wind status
  (void)checkWindRain();

  // Hard stop if rain/wind active
  if (rainActive || windActive) {
    for (int z = 0; z < Zone; ++z) {
      if (zoneActive[z]) {
        turnOffZone(z);
        Serial.printf("Zone %d forcibly stopped due to %s\n", z+1, rainActive ? "rain" : "wind");
      }
    }
    // non-blocking UI update
    if (now - lastScreenRefresh >= SCREEN_REFRESH_MS) {
      lastScreenRefresh = now;
      RainScreen();
    }
    delay(20);
    return;
  }

  // Midnight reset of minute checks
  time_t nowTime = time(nullptr);
  struct tm* nowTm = localtime(&nowTime);
  if (nowTm->tm_hour == 0 && nowTm->tm_min == 0) {
    memset(lastCheckedMinute, -1, sizeof(lastCheckedMinute));
  }

  // Scheduling
  for (int z = 0; z < Zone; ++z) {
    if (shouldStartZone(z)) {
      if (!checkWindRain()) {
        pendingStart[z] = true;
        logEvent(z, "RAIN/WIND DELAY", "N/A", true);
        continue;
      }
      // No tank-based logic — just start it
      turnOnZone(z);
    }

    if (zoneActive[z] && hasDurationCompleted(z)) {
      turnOffZone(z);
    }
  }

  // Start next queued zone if and only if nobody is currently active (live check)
  bool anyActiveLive = false;
  for (int i = 0; i < Zone; ++i) { if (zoneActive[i]) { anyActiveLive = true; break; } }
  if (!anyActiveLive) {
    for (int z = 0; z < Zone; ++z) {
      if (pendingStart[z]) {
        pendingStart[z] = false;
        Serial.printf("Starting queued Zone %d\n", z+1);
        turnOnZone(z);
        break;
      }
    }
  }

  // OLED refresh at 1Hz
  if (now - lastScreenRefresh >= SCREEN_REFRESH_MS) {
    lastScreenRefresh = now;
    HomeScreen();
  }

  delay(20);
}

// =================== Helpers ===================
void wifiCheck() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    delay(1000);
    if (!wifiManager.autoConnect("ESPIrrigationAP")) {
      Serial.println("Reconnection failed.");
    } else {
      Serial.println("Reconnected.");
    }
  }
}

void checkI2CHealth() {
  delay(2);
  bool anyErr = false;

  // Use the same bus you initialised (I2Cbus), not Wire
  for (auto addr : expanderAddrs) {
    I2Cbus.beginTransmission(addr);
    if (I2Cbus.endTransmission() != 0) { anyErr = true; break; }
  }

  if (anyErr) {
    i2cFailCount++;
    if (i2cFailCount >= I2C_HEALTH_DEBOUNCE) {
      Serial.println("⚠ Multiple I2C errors, switching to GPIO fallback");
      useGpioFallback = true;

      // Ensure ALL outputs are safe when switching
      initGpioFallback();
    }
  } else {
    i2cFailCount = 0;
  }
}

void initGpioFallback() {
  useGpioFallback = true;

  // Zones OFF
  for (uint8_t i = 0; i < Zone; i++) {
    pinMode(zonePins[i], OUTPUT);
    digitalWrite(zonePins[i], LOW); // OFF (active-HIGH fallback)
  }

  // Sources OFF
  pinMode(mainsPin, OUTPUT);
  pinMode(tankPin,  OUTPUT);
  digitalWrite(mainsPin, LOW);
  digitalWrite(tankPin,  LOW);
}

void printCurrentTime() {
  time_t now = time(nullptr);
  struct tm *tm = localtime(&now);
  Serial.printf("Current time: %02d:%02d:%02d\n", tm->tm_hour, tm->tm_min, tm->tm_sec);
}

// =================== Config & Schedule ===================
void loadConfig() {
  File f = LittleFS.open("/config.txt", "r");
  if (!f) return;

  apiKey             = f.readStringUntil('\n'); apiKey.trim();
  city               = f.readStringUntil('\n'); city.trim();
  tzOffsetHours      = f.readStringUntil('\n').toFloat();
  rainDelayEnabled   = (f.readStringUntil('\n').toInt() == 1);
  windSpeedThreshold = f.readStringUntil('\n').toFloat();
  windDelayEnabled   = (f.readStringUntil('\n').toInt() == 1);
  justUseTank        = (f.readStringUntil('\n').toInt() == 1); // kept for compat
  justUseMains       = (f.readStringUntil('\n').toInt() == 1); // kept for compat
  tankEmptyRaw       = f.readStringUntil('\n').toInt();
  tankFullRaw        = f.readStringUntil('\n').toInt();
  for (int i = 0; i < Zone; i++) {
    zonePins[i] = f.readStringUntil('\n').toInt();
  }
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
  // keep writing these for backward compat
  f.println(justUseTank ? "1" : "0");
  f.println(justUseMains ? "1" : "0");
  f.println(tankEmptyRaw);
  f.println(tankFullRaw);
  for (int i = 0; i < Zone; i++) {
    f.println(zonePins[i]);
  }
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

    nextComma     = line.indexOf(',', index);
    startHour[i]  = line.substring(index, nextComma).toInt();
    index         = nextComma + 1;

    nextComma     = line.indexOf(',', index);
    startMin[i]   = line.substring(index, nextComma).toInt();
    index         = nextComma + 1;

    nextComma      = line.indexOf(',', index);
    startHour2[i]  = line.substring(index, nextComma).toInt();
    index          = nextComma + 1;

    nextComma      = line.indexOf(',', index);
    startMin2[i]   = line.substring(index, nextComma).toInt();
    index          = nextComma + 1;

    nextComma        = line.indexOf(',', index);
    durationMin[i]   = line.substring(index, nextComma).toInt();
    index            = nextComma + 1;

    nextComma       = line.indexOf(',', index);
    durationSec[i]  = line.substring(index, nextComma).toInt();
    index           = nextComma + 1;

    nextComma            = line.indexOf(',', index);
    enableStartTime2[i]  = (line.substring(index, nextComma).toInt() == 1);
    index                 = nextComma + 1;

    for (int j = 0; j < 7; j++) {
      nextComma = line.indexOf(',', index);
      if (nextComma < 0) nextComma = line.length();
      days[i][j] = (line.substring(index, nextComma).toInt() == 1);
      index = nextComma + 1;
    }

    Serial.printf(
      "Zone %d: %02d:%02d, %02d:%02d, %d min %02d sec, Days: %d%d%d%d%d%d%d\n",
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

// =================== Weather ===================
void updateCachedWeather() {
  const unsigned long currentMillis = millis();
  if (cachedWeatherData == "" || (currentMillis - lastWeatherUpdate >= WEATHER_INTERVAL)) {
    cachedWeatherData = fetchWeather();
    lastWeatherUpdate = currentMillis;

    // Parse rain amount (if available)
    DynamicJsonDocument js(1024);
    if (!deserializeJson(js, cachedWeatherData)) {
      if (js["rain"]["1h"].is<float>()) {
        lastRainAmount = js["rain"]["1h"].as<float>();
      } else {
        lastRainAmount = 0.0f;
      }
    }
  }
}

String fetchWeather() {
  if (apiKey.length() < 5 || city.length() < 1) return "{}"; // guard
  HTTPClient http;
  http.setTimeout(5000);
  String url = "http://api.openweathermap.org/data/2.5/weather?id=" + city + "&appid=" + apiKey + "&units=metric";
  http.begin(client, url);
  int httpResponseCode = http.GET();
  String payload = "{}";
  if (httpResponseCode > 0) payload = http.getString();
  else Serial.println("Error: Unable to fetch weather data.");
  http.end();
  return payload;
}

// =================== OLED ===================
void HomeScreen() {
  // --- Weather (safe parse) ---
  updateCachedWeather();
  DynamicJsonDocument js(1024);
  bool ok = (deserializeJson(js, cachedWeatherData) == DeserializationError::Ok);
  float temp = ok ? js["main"]["temp"].as<float>()      : NAN;
  int   hum  = ok ? js["main"]["humidity"].as<int>()    : -1;

  // --- Tank level using calibration (informational only) ---
  int raw = analogRead(TANK_PIN);
  int pct = map(raw, tankEmptyRaw, tankFullRaw, 0, 100);
  pct = constrain(pct, 0, 100);
  const char* src = justUseTank ? "Tank" : "Main"; // ignore tank level

  // --- Time & date ---
  time_t nowT = time(nullptr);
  struct tm* t = localtime(&nowT);
  int day = t ? t->tm_mday : 1;
  int mon = t ? (t->tm_mon + 1) : 1;
  int hr  = t ? t->tm_hour : 0;
  int mn  = t ? t->tm_min  : 0;

  // --- Draw ---
  display.clearDisplay();

  // Line 1: time + date
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.printf("%02d:%02d", hr, mn);
  display.setTextSize(1);
  display.setCursor(80, 3);
  display.printf("%02d/%02d", day, mon);

  // Line 2: temp & humidity
  display.setCursor(0, 17);
  if (!isnan(temp) && hum >= 0) {
    display.printf("Temp:%2.0fC Hum:%02d%%", temp, hum);
  } else {
    display.print("Temp:--C Hum:--%");
  }

  // Line 3: tank level & source
  display.setCursor(0, 28);
  display.printf("Tank:%3d%% (%s)", pct, src);

  // Helper to draw a pair of zones on one row
  auto drawPair = [&](int zA, int zB, int y){
    for (int k = 0; k < 2; ++k) {
      int i = (k == 0) ? zA : zB;
      if (i >= Zone) break;          // safety if Zone < 6
      int x = (k == 0) ? 0 : 64;
      display.setCursor(x, y);
      display.setTextSize(1);
      if (zoneActive[i]) {
        unsigned long elapsed = (millis() - zoneStartMs[i]) / 1000UL;
        unsigned long total   = (unsigned long)durationMin[i] * 60UL + (unsigned long)durationSec[i];
        unsigned long rem     = (elapsed < total) ? (total - elapsed) : 0UL;
        unsigned int  remM    = (unsigned int)(rem / 60UL);
        unsigned int  remS    = (unsigned int)(rem % 60UL);
        display.printf("Z%d:%02u:%02u", i + 1, remM, remS);
      } else {
        display.printf("Z%d:Off", i + 1);
      }
    }
  };

  // Rows: 1–2 @ y=44, 3–4 @ y=52, 5–6 @ y=60
  drawPair(0, 1, 39);
  drawPair(2, 3, 48);
  drawPair(4, 5, 56);

  display.display();
}

void toggleBacklight() {
  display.invertDisplay(true);
  delay(300);
  display.invertDisplay(false);
}

void updateLCDForZone(int zone) {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  if (now - lastUpdate < 1000) return;
  lastUpdate = now;

  unsigned long elapsed = (now - zoneStartMs[zone]) / 1000;
  unsigned long total   = (unsigned long)durationMin[zone] * 60 + (unsigned long)durationSec[zone]; // include seconds
  unsigned long rem     = (elapsed < total ? total - elapsed : 0);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("Zone %d  %02lu:%02lu", zone + 1, elapsed/60, elapsed%60);

  display.setCursor(0, 12);
  display.print(elapsed < total ? "Remaining: " : "Complete   ");
  display.printf("%02lu:%02lu", rem/60, rem%60);
  display.display();
}

void RainScreen() {
  display.clearDisplay();

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("Rain Delay");

  display.setTextSize(1);
  display.setCursor(0, 22);
  display.printf("Last: %.2f mm", lastRainAmount);

  int delayed = 0;
  for (int i = 0; i < Zone; i++) if (pendingStart[i]) delayed++;
  display.setCursor(0, 38);
  display.printf("Zones delayed: %d", delayed);

  display.display();
}

// =================== Weather-based guards ===================
bool checkWindRain() {
  updateCachedWeather();

  DynamicJsonDocument js(1024);
  if (deserializeJson(js, cachedWeatherData)) {
    rainActive = false;
    windActive = false;
    return true; // fail-open on parse error
  }

  // Rain check (be stricter than "rain" key exists)
  bool raining = false;
  if (rainDelayEnabled) {
    float r1h = js["rain"]["1h"] | 0.0f;
    String w  = js["weather"][0]["main"] | "";
    raining   = (r1h > 0.0f) || (w == "Rain" || w == "Drizzle" || w == "Thunderstorm");
    lastRainAmount = r1h;
  }
  rainActive = raining;

  // Wind check
  windActive = false;
  if (windDelayEnabled) {
    float spd = js["wind"]["speed"] | 0.0f;
    windActive = spd >= windSpeedThreshold;
  }

  return !(rainActive || windActive);
}

// =================== Scheduling & Control ===================
bool shouldStartZone(int zone) {
  time_t now = time(nullptr);
  struct tm *t = localtime(&now);
  int wd = t->tm_wday;
  int hr = t->tm_hour;
  int mn = t->tm_min;

  if (!days[zone][wd]) return false;
  if (lastCheckedMinute[zone] == mn) return false;

  bool match1 = (hr == startHour[zone]  && mn == startMin[zone]);
  bool match2 = enableStartTime2[zone] && (hr == startHour2[zone] && mn == startMin2[zone]);

  if (match1 || match2) {
    lastCheckedMinute[zone] = mn;
    Serial.printf("Zone %d triggered at %02d:%02d\n", zone+1, hr, mn);
    return true;
  }
  return false;
}

bool hasDurationCompleted(int zone) {
  unsigned long elapsed = (millis() - zoneStartMs[zone]) / 1000;
  unsigned long total   = (unsigned long)durationMin[zone] * 60 + (unsigned long)durationSec[zone];
  return (elapsed >= total);
}

bool isTankLow() {
  int raw = analogRead(TANK_PIN);
  int lowThresh = tankEmptyRaw + (int)((tankFullRaw - tankEmptyRaw) * 0.15f); // calibrated threshold
  return raw <= lowThresh;
}

void turnOnZone(int z) {
  if (!checkWindRain()) { // just-in-time guard
    pendingStart[z] = true;
    logEvent(z, "RAIN/WIND DELAY", "N/A", true);
    return;
  }

  zoneStartMs[z] = millis();
  zoneActive[z]  = true;

  selectWaterSource(); // ensure source is selected for this run

  if (useGpioFallback) digitalWrite(zonePins[z], HIGH); // ON
  else                 pcfOut.digitalWrite(valveChannel[z], LOW); // active-LOW

  logEvent(z, "START", "-", false);
  Serial.printf("Zone %d activated\n", z + 1);

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(3, 0);
  display.print("Zone "); display.print(z + 1); display.print(" ON");
  display.display();
  delay(400);
}

void turnOffZone(int z) {
  logEvent(z, "OFF", "-", (rainActive || windActive));

  if (useGpioFallback) digitalWrite(zonePins[z], LOW); // OFF
  else                 pcfOut.digitalWrite(valveChannel[z], HIGH);

  zoneActive[z] = false;

  // if no more zones are active, turn sources off
  bool any = false;
  for (int i = 0; i < Zone; ++i) { if (zoneActive[i]) { any = true; break; } }
  if (!any) {
    if (useGpioFallback) {
      digitalWrite(mainsPin, LOW);
      digitalWrite(tankPin,  LOW);
    } else {
      pcfOut.digitalWrite(mainsChannel, RELAY_ACTIVE_LOW_PCF ? HIGH : LOW);
      pcfOut.digitalWrite(tankChannel,  RELAY_ACTIVE_LOW_PCF ? HIGH : LOW);
    }
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(4,0);
  display.print("Zone "); display.print(z+1);
  display.print((rainActive || windActive) ? " STOP" : " OFF");
  display.display();
  delay(300);
}

void turnOnValveManual(int z) {
  if (zoneActive[z]) return;

  zoneStartMs[z] = millis();
  zoneActive[z]  = true;

  selectWaterSource();

  if (useGpioFallback) digitalWrite(zonePins[z], HIGH);
  else                 pcfOut.digitalWrite(valveChannel[z], LOW);

  Serial.printf("Manual zone %d ON\n", z+1);
}

void turnOffValveManual(int z) {
  if (!zoneActive[z]) return;

  if (useGpioFallback) digitalWrite(zonePins[z], LOW);
  else                 pcfOut.digitalWrite(valveChannel[z], HIGH);

  zoneActive[z] = false;

  // if no more zones are active, turn sources off
  bool any = false;
  for (int i = 0; i < Zone; ++i) { if (zoneActive[i]) { any = true; break; } }
  if (!any) {
    if (useGpioFallback) {
      digitalWrite(mainsPin, LOW);
      digitalWrite(tankPin,  LOW);
    } else {
      pcfOut.digitalWrite(mainsChannel, RELAY_ACTIVE_LOW_PCF ? HIGH : LOW);
      pcfOut.digitalWrite(tankChannel,  RELAY_ACTIVE_LOW_PCF ? HIGH : LOW);
    }
  }

  Serial.printf("Manual zone %d OFF\n", z+1);
}

// =================== Web UI: root ===================
String getDayName(int d) {
  const char* names[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  return String(names[d]);
}

void handleRoot() {
  // Time
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  char timeStr[9], dateStr[11];
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", timeinfo);
  strftime(dateStr, sizeof(dateStr), "%d/%m/%Y", timeinfo);

  loadSchedule();

  // Weather (use cached)
  String weatherData = cachedWeatherData;
  DynamicJsonDocument jsonResponse(1024);
  DeserializationError werr = deserializeJson(jsonResponse, weatherData);

  float temp = werr ? NAN : (jsonResponse["main"]["temp"]     | NAN);
  float hum  = werr ? NAN : (jsonResponse["main"]["humidity"] | NAN);
  float ws   = werr ? NAN : (jsonResponse["wind"]["speed"]    | NAN);
  String cond     = werr ? "-" : String(jsonResponse["weather"][0]["main"].as<const char*>());
  if (cond == "") cond = "-";
  String cityName = werr ? "-" : String(jsonResponse["name"].as<const char*>());
  if (cityName == "") cityName = "-";

  // Tank
  int tankRaw = analogRead(TANK_PIN);
  int tankPct = map(tankRaw, tankEmptyRaw, tankFullRaw, 0, 100);
  tankPct = constrain(tankPct, 0, 100);
  bool tankLow = tankRaw < (tankEmptyRaw + (tankFullRaw - tankEmptyRaw) * 0.15f);
  String tankStatusStr = tankLow
    ? "<span class='tank-status-low'>Tank Level Low</span>"
    : "<span class='tank-status-normal'>Tank Level</span>";

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
 :root { --primary:#1976d2; --danger:#e03e36; --success:#32c366; --neutral:#bed9ff; --bg:#f5f7fa; --card:#fff; --font:#1a1a1a; --footer:#fafdff4b; --tank-gradient:linear-gradient(90deg,#32c366 0%,#ffe94b 65%,#e03e36 100%); --shadow:0 2px 16px rgba(80,160,255,.08); }
 body { font-family:'Roboto',sans-serif; background:var(--bg); color:var(--font); margin:0; transition: background .2s, color .2s; }
 header { display:flex;flex-direction:column;align-items:center;justify-content:center; gap:10px;padding:22px 0 10px 0;background:var(--primary);color:#fff; border-radius:0 0 18px 18px;box-shadow:0 2px 8px #1976d241; }
 .container { max-width: 1200px; margin: 0 auto; padding: 22px 10px 14px 10px; display: flex; flex-direction: column; align-items: center; }
 .summary-row { display: flex; flex-wrap: wrap; gap: 18px; justify-content: center; align-items: flex-start; width: 100%; margin-bottom: 32px; }
 .summary-block { background: var(--card); border-radius: 14px; box-shadow: var(--shadow); padding: 17px 13px 15px 13px; min-width: 110px; text-align: center; font-size: 1.18em; font-weight: 500; position: relative; border: 1.5px solid var(--neutral); margin-bottom: 0; display: flex; flex-direction: column; align-items: center; justify-content: center; }
 .summary-block .icon { font-size:2.1em; margin-bottom: 3px; display:block; }
 .tank-row progress[value] { width: 60px; height: 13px; vertical-align:middle; border-radius:8px; background:#e9ecef; box-shadow: 0 1.5px 8px #32c36624 inset; margin-bottom: 3px; }
 body[data-theme='dark'] .tank-row progress[value], body[data-theme='dark'] .tank-row progress[value]::-webkit-progress-bar { background: #222 !important; }
 .tank-row progress[value]::-webkit-progress-bar { background: #e9ecef; border-radius:8px; }
 .tank-row progress[value]::-webkit-progress-value { background: var(--tank-gradient); border-radius:8px; transition:width .4s; animation: tankPulse 2.5s linear infinite alternate; }
 .tank-row progress[value]::-moz-progress-bar { background: var(--tank-gradient); border-radius:8px; }
 @keyframes tankPulse { 0% { box-shadow:0 0 0 #32c36622;} 60%{ box-shadow:0 0 12px #ffe94b77;} 100%{ box-shadow:0 0 16px #e03e3665;} }
 .active-badge { color: var(--success); font-weight:700; padding:2px 10px; border-radius:10px; background: #e6ffe7; box-shadow: 0 0 12px 2px #32c36688; animation: statusGlow 1.2s ease-in-out infinite alternate; display:inline-block; font-size:1em; margin-top:2px; }
 body[data-theme='dark'] .active-badge { background:#1e4220; }
 @keyframes statusGlow { 0% { box-shadow: 0 0 8px #32c36677;} 100% { box-shadow: 0 0 16px #32c366cc, 0 0 25px #ffe94b66;} }
 .inactive-badge { color: #ccc; font-weight:700; background: #f4f4f4; border-radius:10px; padding:2px 10px; font-size:1em; margin-top:2px; }
 body[data-theme='dark'] .inactive-badge { background:#24262c;color:#666;}
 .zone-status-on { color: var(--success); font-weight: 600; animation: statusGlow 1.5s ease-in-out infinite alternate; text-shadow:0 0 6px #32c36699; }
 .zone-status-off { color:#aaa; }
 .rem{color:#888;font-size:.95em;}
 .zones-wrapper { display: flex; flex-wrap: wrap; gap: 22px; justify-content: center; width: 100%; }
 .zone-container { border: 1.6px solid var(--neutral); background: var(--card); border-radius: 13px; box-shadow: 0 2px 14px #1976d21a; padding: 15px 13px 13px 13px; display: flex; flex-direction: column; gap: 9px; align-items: center; min-width: 295px; max-width: 340px; margin-bottom: 8px; text-align: center; }
 .time-duration-container { display: flex; flex-direction: column; gap: 9px; width: 100%; align-items: center; margin: 8px 0; }
 .enable-input { display: flex; flex-direction: row; align-items: center; gap: 7px; justify-content: center; width: 100%; margin-bottom: 0; }
 .days-container, .manual-control-container { justify-content: center; width: 100%; display: flex; }
 .days-container { gap:7px; flex-wrap:wrap; }
 .checkbox-container { display: flex; align-items: center; gap: 4px; font-size: 0.97em; border-radius: 6px; padding: 2px 7px; transition: background .19s; justify-content: center; }
 .checkbox-container:hover, .checkbox-container:focus-within { background: #e9f2fe; }
 body[data-theme='dark'] .checkbox-container:hover, body[data-theme='dark'] .checkbox-container:focus-within { background: #263241; }
 .checkbox-container input[type='checkbox']:checked+label { background: #1976d2; color:#fff; border-radius:5px; padding:2px 7px; font-weight:600; box-shadow:0 1px 5px #1976d261; transition:background .17s; }
 .checkbox-container label { cursor:pointer; padding:2px 2px; transition:background .14s, color .14s;}
 input[type='checkbox'] { accent-color: #1976d2; }
 .enable-input input[type='checkbox'] { width:1.2em;height:1.2em;}
 input[type='number'] { font-size:1.07em; padding:4px 7px; border-radius:6px; border:1.1px solid #b6c8e2; width:4.3em; margin-left:4px; text-align: center; background: var(--card); color: var(--font); transition: background .2s, color .2s, border .15s; }
 label { margin-right:2px; }
 button, input[type='submit'] { background: var(--primary); color: #fff; font-weight: 500; border: none; border-radius: 9px; padding: 8px 20px; font-size: 1em; cursor: pointer; transition: background .16s, box-shadow .17s, transform .11s; box-shadow:0 2px 6px #1976d224; text-align: center; }
 button:active { transform:scale(0.97);}
 button[disabled], .manual-control-container button[disabled] { background: #d4dbe0; color: #888; cursor:not-allowed; box-shadow:none; }
 .manual-control-container .turn-on-btn:not([disabled]) {background: var(--success);}
 .manual-control-container .turn-off-btn:not([disabled]) {background: var(--danger);}
 .footer-links { border-top: 1.5px solid #dae7f5; margin: 38px 0 0 0; padding-top:18px; text-align:center; font-size:1.09em; background: var(--footer); border-radius:0 0 12px 12px; transition:background .2s; }
 @media(max-width:1220px){ .zones-wrapper{flex-wrap:wrap;} }
 @media(max-width:820px){ .container{padding:9px;} .zones-wrapper{flex-direction: column;align-items:center;} .summary-row{gap:10px;} }
 @media(max-width:520px){ .summary-block{min-width:80px;font-size:1em;} .zone-container{padding:7px 4px;} .footer-links{font-size:.99em;} }
 body[data-theme='dark'], body[data-theme='dark'] :root { --bg: #181a1b !important; --card: #20252a !important; --font: #f5f7fa !important; --footer: #202830 !important; --neutral: #394b5b !important; --shadow: 0 2px 16px rgba(20,32,45,.12) !important; }
 body:not([data-theme='dark']), body:not([data-theme='dark']) :root { --bg: #f5f7fa !important; --card: #fff !important; --font: #1a1a1a !important; --footer: #fafdff4b !important; --neutral: #bed9ff !important; --shadow: 0 2px 16px rgba(80,160,255,.08) !important; }
  </style>
 </head>
 <body>
  <header>
    <span style='font-size:1.75em;font-weight:700;'>💧ESP32 Irrigation Dashboard</span>
    <span style='font-size:1.13em;'>
      🕒 <span id='clock'>)"+ String(timeStr) + R"(</span>
      &nbsp;&nbsp; 🗓 <span id='date'>)"+ String(dateStr) + R"(</span>
    </span>
  </header>
  <div class='container'>
    <button id='toggle-darkmode-btn' style='margin:16px auto 8px auto;display:block;min-width:125px;'>🌗 Dark Mode</button>
    <div class='summary-row'>
      <div class='summary-block'><span class='icon'>📍</span><br>)"+ cityName + R"(</div>
      <div class='summary-block'><span class='icon'>🌬</span><br>)"+ (isnan(ws) ? "--" : String(ws,1) + " m/s") + R"(</div>
      <div class='summary-block'><span class='icon'>🌡</span><br>)"+ (isnan(temp) ? "--" : String(temp,1) + " ℃") + R"(</div>
      <div class='summary-block'><span class='icon'>💧</span><br>)"+ (isnan(hum) ? "--" : String((int)hum) + " %") + R"(</div>
      <div class='summary-block'><span class='icon'>🌤</span><br>)"+ cond + R"(</div>
      <div class='summary-block tank-row'><span class='icon'>🚰</span>
        <progress id='tank-bar' value=')" + String(tankPct) + R"(' max='100'></progress>
        <span id='tank-pct'>)" + String(tankPct) + R"(%</span><br>
        <span id='tank-status'>)" + tankStatusStr + R"(</span>
      </div>
      <div class='summary-block'><span class='icon' title='Rain Delay'>🌧</span><br>)";
  html += (rainActive ? "<span id='rain-badge' class='active-badge'>Active</span>" : "<span id='rain-badge' class='inactive-badge'>Off</span>");
  html += R"(</div>
      <div class='summary-block'><span class='icon' title='Wind Delay'>💨</span><br>)";
  html += (windActive ? "<span id='wind-badge' class='active-badge'>Active</span>" : "<span id='wind-badge' class='inactive-badge'>Off</span>");
  html += R"(</div>
    </div>
    <form action='/submit' method='POST'><div class='zones-wrapper'>)";

  static const char* dayNames[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  for (int zone = 0; zone < Zone; zone++) {
    html += "<div class='zone-container'>"
            "<p style='margin:0 0 4px 0'><strong>Zone " + String(zone + 1) + ":</strong></p>"
            "<p>Status: <span id='zone-status-" + String(zone) + "' class='" + String(zoneActive[zone] ? "zone-status-on" : "zone-status-off") + "'>"
            + String(zoneActive[zone] ? "Running" : "Off") + "</span> "
            "<span id='zone-rem-" + String(zone) + "' class='rem'></span></p>";

    html += "<div class='days-container'>";
    for (int d = 0; d < 7; d++) {
      String chk = days[zone][d] ? "checked" : "";
      html += "<div class='checkbox-container'>"
              "<input type='checkbox' name='day" + String(zone) + "_" + d + "' id='day" + String(zone) + "_" + d + "' " + chk + ">"
              "<label for='day" + String(zone) + "_" + d + "'>" + dayNames[d] + "</label>"
              "</div>";
    }
    html += "</div>";

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

  html += "<div style='text-align:center; margin:25px 0 0 0;'>"
          "<button type='button' id='toggle-backlight-btn' style='min-width:170px;'>Toggle LCD Backlight</button>"
          "</div>";

  // Weather link uses city ID
  html += "<div class='footer-links'>"
          "<a href='/events'>📒 Event Log</a>"
          "<span style='margin:0 9px;color:#b3b3b3;'>|</span>"
          "<a href='/tank'>🚰 Tank Calibration</a>"
          "<span style='margin:0 9px;color:#b3b3b3;'>|</span>"
          "<a href='/setup'>⚙️ Setup</a>"
          "<span style='margin:0 9px;color:#b3b3b3;'>|</span>"
          "<a href='https://openweathermap.org/city/" + city + "' target='_blank'>🌤 Weather</a>"
          "</div></div>";

  html += R"(<script>
 // clock / date
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

 // theme
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

 // live status refresh
 function fmtMMSS(totalSec){
  totalSec = Math.max(0, Math.floor(totalSec||0));
  const m = String(Math.floor(totalSec/60)).padStart(2,'0');
  const s = String(totalSec%60).padStart(2,'0');
  return `${m}:${s}`;
 }
 function setBadge(el, active){
  if(!el) return;
  el.textContent = active ? 'Active' : 'Off';
  el.classList.toggle('active-badge',  !!active);
  el.classList.toggle('inactive-badge', !active);
 }
 function updateLive(){
  fetch('/status')
    .then(r=>r.json())
    .then(st=>{
      // rain / wind badges
      setBadge(document.getElementById('rain-badge'), !!st.rainDelayActive);
      setBadge(document.getElementById('wind-badge'), !!st.windDelayActive);

      // tank
      const pct = Number(st.tankPct||0);
      const low = !!st.tankLow;
      const tankBar = document.getElementById('tank-bar');
      const tankPct = document.getElementById('tank-pct');
      const tankStatus = document.getElementById('tank-status');
      if(tankBar) tankBar.value = pct;
      if(tankPct) tankPct.textContent = pct + '%';
      if(tankStatus) {
        tankStatus.innerHTML = low
          ? "<span class='tank-status-low'>Tank Level Low</span>"
          : "<span class='tank-status-normal'>Tank Level</span>";
      }

      // zones
      if(Array.isArray(st.zones)){
        st.zones.forEach((z, i)=>{
          const stEl = document.getElementById('zone-status-' + i);
          const rmEl = document.getElementById('zone-rem-' + i);
          if(stEl){
            stEl.textContent = z.active ? 'Running' : 'Off';
            stEl.classList.toggle('zone-status-on',  !!z.active);
            stEl.classList.toggle('zone-status-off', !z.active);
          }
          if(rmEl){
            rmEl.textContent = z.active ? `(${fmtMMSS(z.remaining)} left)` : '';
          }
        });
      }
    })
    .catch(e=>console.warn('status refresh failed', e));
 }

 window.addEventListener('DOMContentLoaded',()=>{
  autoTheme();
  updateLive();
  setInterval(updateLive, 2000);

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


// =================== Web UI: submit/setup/config/log ===================
void handleSubmit() {
  for (int z = 0; z < Zone; z++) {
    for (int d = 0; d < 7; d++) {
      days[z][d] = server.hasArg("day" + String(z) + "_" + String(d));
    }
    if (server.hasArg("startHour" + String(z))) startHour[z] = server.arg("startHour" + String(z)).toInt();
    if (server.hasArg("startMin"  + String(z))) startMin [z] = server.arg("startMin"  + String(z)).toInt();

    if (server.hasArg("startHour2" + String(z))) startHour2[z] = server.arg("startHour2" + String(z)).toInt();
    if (server.hasArg("startMin2"  + String(z))) startMin2 [z] = server.arg("startMin2"  + String(z)).toInt();

    if (server.hasArg("durationMin" + String(z))) durationMin[z] = server.arg("durationMin" + String(z)).toInt();
    if (server.hasArg("durationSec" + String(z))) durationSec[z] = server.arg("durationSec" + String(z)).toInt();

    enableStartTime2[z] = server.hasArg("enableStartTime2" + String(z));
  }

  saveSchedule();
  updateCachedWeather();
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleSetupPage() {
  loadConfig();
  String html;

  html += "<!DOCTYPE html><html lang='en'><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Setup – Irrigation System</title>";
  html += "<link href='https://fonts.googleapis.com/css?family=Roboto:400,500&display=swap' rel='stylesheet'>";

  html += R"rawliteral(
  <style>
    body{font-family:'Roboto',sans-serif;margin:0;background:#f5f7fa;color:#1a1a1a}
    header{padding:18px 0;text-align:center;background:#1976d2;color:#fff;font-weight:600;font-size:1.2rem}
    main{max-width:900px;margin:20px auto;padding:0 12px}
    .setup-card{background:#fff;border-radius:12px;box-shadow:0 2px 14px rgba(25,118,210,.12);padding:18px}
    .form-group{margin:10px 0}
    .form-row{display:flex;gap:8px;align-items:center}
    .full-width{width:100%}
    .small-input{width:7em}
    .checkbox-group{display:flex;align-items:center;gap:8px;margin:8px 0}
    .zone-pin-row{display:flex;gap:10px;align-items:center;margin:6px 0}
    .zone-pin-input{width:6em}
    .btn-row{display:flex;gap:10px;margin-top:14px}
    .btn,.cancel-btn,.restore-btn{padding:8px 16px;border:none;border-radius:8px;color:#fff;background:#1976d2;cursor:pointer}
    .cancel-btn{background:#455a64}
    .restore-btn{background:#2e7d32}
    #toggle-darkmode-btn{position:fixed;top:10px;right:10px;border:none;background:transparent;cursor:pointer}
    .theme-switch{display:inline-block;width:50px;height:26px;background:#cfd8dc;border-radius:13px;position:relative}
    .theme-switch .dot{position:absolute;top:3px;left:3px;width:20px;height:20px;background:#fff;border-radius:50%;transition:left .18s}
    .theme-switch[data-dark='1']{background:#263238}
    .theme-switch[data-dark='1'] .dot{left:27px}
  </style>
  )rawliteral";

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
          </div>
          <div class="form-group form-row">
            <label for="city">City ID</label>
            <input class="full-width" type="text" id="city" name="city" value=")rawliteral"
  + city +
  R"rawliteral(" required maxlength="32" placeholder="City ID">
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
          </div>
        </section>
        <section>
          <legend>Feature Toggles</legend>
          <div class="checkbox-group">
            <input type="checkbox" id="windCancelEnabled" name="windCancelEnabled" )rawliteral" + String(windDelayEnabled ? " checked" : "") + R"rawliteral(>
            <label for="windCancelEnabled">Enable Wind Delay</label>
          </div>
          <div class="form-group form-row">
            <label for="windSpeedThreshold">Wind Threshold</label>
            <input class="small-input" type="number" id="windSpeedThreshold" name="windSpeedThreshold"
              min="0" step="0.1" value=")rawliteral"
  + String(windSpeedThreshold,1) +
  R"rawliteral(" required placeholder="m/s">
          </div>
          <div class="checkbox-group">
            <input type="checkbox" id="rainDelay" name="rainDelay" )rawliteral" + String(rainDelayEnabled ? " checked" : "") + R"rawliteral(>
            <label for="rainDelay">Enable Rain Delay</label>
          </div>
          <div class="checkbox-group">
            <input type="checkbox" id="useMainsOnly" name="useMainsOnly" %MAINSCHK%>
            <label for="useMainsOnly">Use Mains only</label>
          </div>
          <div class="checkbox-group">
            <input type="checkbox" id="useTankOnly" name="useTankOnly" %TANKCHK%>
            <label for="useTankOnly">Use Tank only</label>
          </div>
        </section>
        <section>
          <legend>Zone Pins</legend>
  )rawliteral";
  for (uint8_t i = 0; i < Zone; i++) {
    html += "<div class='zone-pin-row'><label>Zone " + String(i+1) + ":</label>"
            "<input class='zone-pin-input' type='number' name='zonePin" + String(i) +
            "' min='0' max='39' value='" + String(zonePins[i]) + "'></div>";
  }
  html += R"rawliteral(
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
      document.getElementById('setupForm').addEventListener('input', e => {
        ['apiKey','city'].forEach(id => {
          const el = document.getElementById(id);
          el.style.borderColor = el.value.trim() ? '' : '#d34242';
        });
      });
    });
  </script>
  </body></html>
  )rawliteral";

  // Apply checkbox states
  html.replace("%MAINSCHK%", justUseMains ? "checked" : "");
  html.replace("%TANKCHK%",  justUseTank  ? "checked" : "");

  server.send(200, "text/html", html);
}

void handleConfigure() {
  String oldApiKey = apiKey;
  String oldCity   = city;
  float  oldTz     = tzOffsetHours;

  if (server.hasArg("apiKey")) apiKey = server.arg("apiKey");
  if (server.hasArg("city"))   city   = server.arg("city");
  if (server.hasArg("dstOffset")) tzOffsetHours = server.arg("dstOffset").toFloat();

  rainDelayEnabled   = server.hasArg("rainDelay");
  windDelayEnabled   = server.hasArg("windCancelEnabled");
  if (server.hasArg("windSpeedThreshold")) windSpeedThreshold = server.arg("windSpeedThreshold").toFloat();

  // Source overrides (mutually exclusive; mains wins if both checked)
  bool uiMains = server.hasArg("useMainsOnly");
  bool uiTank  = server.hasArg("useTankOnly");
  justUseMains = uiMains;
  justUseTank  = uiTank && !uiMains;

  for (int i = 0; i < Zone; i++) {
    if (server.hasArg("zonePin" + String(i))) zonePins[i] = server.arg("zonePin" + String(i)).toInt();
  }

  saveConfig();
  loadConfig();

  Serial.printf("[CONFIG] Saved - RainDelay:%d WindDelay:%d Mains:%d Tank:%d\n", rainDelayEnabled, windDelayEnabled, justUseMains, justUseTank);

  bool needsRestart = (apiKey != oldApiKey) || (city != oldCity) || (fabs(tzOffsetHours - oldTz) > 0.0001f);

  if (needsRestart) {
    String html = "<!DOCTYPE html><html lang='en'><head>"
      "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>"
      "<meta http-equiv='refresh' content='2;url=/setup' />"
      "<title>Restarting</title>"
      "<style>body{font-family:Arial,sans-serif;text-align:center;padding:40px;}h1{color:#2E86AB;}p{font-size:1.1em;}</style>"
      "</head><body><h1>⚡ Restart Required</h1><p>Network or timezone settings changed.<br>Restarting system...</p></body></html>";
    server.send(200, "text/html", html);
    delay(1500);
    ESP.restart();
  } else {
    String html = "<!DOCTYPE html><html lang='en'><head>"
      "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>"
      "<meta http-equiv='refresh' content='2;url=/setup' />"
      "<title>Settings Saved</title>"
      "<style>body{font-family:Arial,sans-serif;text-align:center;padding:40px;}h1{color:#2E86AB;}p{font-size:1.1em;}</style>"
      "</head><body><h1>✅ Settings Saved</h1><p>Settings have been saved.<br>You’ll be returned to Setup.</p></body></html>";
    server.send(200, "text/html", html);
  }
}

// =================== Event Log UI ===================
void handleLogPage() {  
  File f = LittleFS.open("/events.csv", "r");
  if (!f) {
    server.send(404, "text/plain", "No event log found");
    return;
  }

  String html = "<!DOCTYPE html><html lang='en'><head>"
    "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>"
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
      "@media (max-width:650px) { .container { padding:10px 3vw; } th,td { font-size:0.95em; padding:8px 3px; } header { font-size:1.2em; padding:16px 0 10px; } }"
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

    String temp = (idx6 > idx5) ? line.substring(idx5 + 1, idx6) : "";
    String hum  = (idx7 > idx6) ? line.substring(idx6 + 1, idx7) : "";
    String wind = (idx8 > idx7) ? line.substring(idx7 + 1, idx8) : "";
    String cond = (idx8 > 0)    ? line.substring(idx8 + 1) : "";

    String details = temp.length()
      ? "T=" + temp + "°C, H=" + hum + "%, W=" + wind + "m/s, " + cond
      : "n/a";

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

  html += "</table><a class='back-link' href='/'>⬅ Back to Home</a></div></body></html>";
  server.send(200, "text/html", html);
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

  File f = LittleFS.open("/events.csv", "a");
  if (!f) return;
  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  char buf[240];
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
  if (LittleFS.exists("/events.csv")) LittleFS.remove("/events.csv");
  server.sendHeader("Location", "/events", true);
  server.send(302, "text/plain", "");
}

// =================== Tank Calibration ===================
void handleTankCalibration() {
  int raw = analogRead(TANK_PIN);
  int pct = map(raw, tankEmptyRaw, tankFullRaw, 0, 100);
  pct = constrain(pct, 0, 100);

  String html = "<!DOCTYPE html><html lang='en'><head>"
  "<meta charset='UTF-8'>"
  "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
  "<title>Tank Calibration</title>"
  "<link href='https://fonts.googleapis.com/css?family=Roboto:400,500&display=swap' rel='stylesheet'>"
  "<style>"
  "body { font-family: 'Roboto', sans-serif; background: #f0f4f8; margin: 0; padding: 0; display: flex; align-items: center; justify-content: center; height: 100vh; }"
  ".card { background: #ffffff; padding: 30px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); width: 100%; max-width: 400px; text-align: center; }"
  "h1 { color: #007BFF; margin-bottom: 20px; }"
  "p { margin: 10px 0; font-size: 1.1em; }"
  "form { margin: 10px 0; }"
  "button { padding: 10px 20px; border: none; border-radius: 5px; background: #007BFF; color: #ffffff; font-size: 16px; cursor: pointer; }"
  "button:hover { background: #0056b3; }"
  "a { display: block; margin-top: 15px; color: #007BFF; text-decoration: none; }"
  "a:hover { text-decoration: underline; }"
  ".alert { color: #cc0000; font-weight: bold; }"
  "</style></head><body>";

  html += "<div class='card'>"
          "<h1>Tank Calibration</h1>"
          "<p>📟 Raw Sensor Value: <strong>" + String(raw) + "</strong></p>"
          "<p>🧪 Calibrated Range: <strong>" + String(tankEmptyRaw) + " (Empty)</strong> → <strong>" + String(tankFullRaw) + " (Full)</strong></p>"
          "<p>🚰 Calculated Level: <strong>" + String(pct) + "%</strong></p>";

  if (raw < tankEmptyRaw || raw > tankFullRaw) {
    html += "<p class='alert'>⚠ Outside calibration range</p>";
  }

  html += "<form method='POST' action='/setTankEmpty'><button type='submit'>Set as Empty</button></form>"
          "<form method='POST' action='/setTankFull'><button type='submit'>Set as Full</button></form>"
          "<a href='/setup'>⬅ Back to Setup</a>"
          "<a href='/'>🏠 Home</a>"
          "</div>"
          "<script>setTimeout(()=>location.reload(),2000);</script>"
          "</body></html>";

  server.send(200, "text/html", html);
}

