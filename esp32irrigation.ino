#ifndef ENABLE_OTA
  #define ENABLE_OTA 0   
#endif
#include <Arduino.h>
#include <ArduinoJson.h>
#if ENABLE_OTA
  #include <ArduinoOTA.h>
#endif
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
#include <esp_event.h>
#include <math.h>
extern "C" {
  #include "esp_log.h"
}
#include <time.h>
#include <ESPmDNS.h> 
#include <PubSubClient.h>   // MQTT

// ---------- Hardware ----------
static const uint8_t MAX_ZONES = 6;
constexpr uint8_t I2C_SDA = 4;
constexpr uint8_t I2C_SCL = 15;

TwoWire I2Cbus = TwoWire(0);
PCF8574 pcfIn (&I2Cbus, 0x22, I2C_SDA, I2C_SCL);
PCF8574 pcfOut(&I2Cbus, 0x24, I2C_SDA, I2C_SCL);

#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Cbus, OLED_RESET);

WiFiManager wifiManager;
WebServer server(80);
WiFiClient client;

// PCF mapping
const uint8_t ALL_P = 6;
const uint8_t PCH[ALL_P] = { P0, P1, P2, P3, P4, P5 };
uint8_t mainsChannel = P4; // 4-zone mode
uint8_t tankChannel  = P5; // 4-zone mode
uint8_t zonesCount   = 4;  // 4 or 6

// GPIO fallback (active-HIGH)
bool useGpioFallback = false;
uint8_t zonePins[MAX_ZONES] = {18, 19, 12, 13, 25, 26};
uint8_t mainsPin = 25;
uint8_t tankPin  = 26;

const int LED_PIN  = 2;
const int TANK_PIN = 36; // ADC1_CH0

// Physical rain sensor
bool rainSensorEnabled = false;
bool rainSensorInvert  = false;
int  rainSensorPin     = 27;

// ---------- Config / State ----------
String apiKey, city; // OpenWeather (city = city ID)
String cachedWeatherData;
unsigned long lastWeatherUpdate = 0;
const unsigned long weatherUpdateInterval = 60UL * 60UL * 1000UL; // 1h

// Forecast cache / metrics
String cachedForecastData;
unsigned long lastForecastUpdate = 0;
const unsigned long forecastUpdateInterval = 30UL * 60UL * 1000UL;
float rainNext12h_mm = NAN;
float rainNext24h_mm = NAN;
int   popNext12h_pct = -1;
int   nextRainIn_h   = -1;
float maxGust24h_ms  = NAN;
float todayMin_C     = NAN, todayMax_C = NAN;
time_t todaySunrise  = 0,   todaySunset = 0;

// Delay controls
bool  rainDelayEnabled = true;
bool  windDelayEnabled = false;
bool  justUseTank = false;
bool  justUseMains = false;

// New saved features
bool     systemPaused = false;
uint32_t pauseUntilEpoch = 0;
bool     rainDelayFromForecastEnabled = true;  // gate for forecast-based rain

// NEW Master/Cooldown/Threshold
bool     systemMasterEnabled = true;     // Master On/Off
uint32_t rainCooldownUntilEpoch = 0;     // when > now => block starts
int      rainCooldownMin = 60;           // minutes to wait after rain clears
int      rainThreshold24h_mm = 5;        // forecast 24h total triggers delay

// NEW Run mode: sequential (false) or concurrent (true)
bool runZonesConcurrent = false;

// Scheduling
bool enableStartTime2[MAX_ZONES] = {false};
bool days[MAX_ZONES][7] = {{false}};
bool zoneActive[MAX_ZONES] = {false};
bool pendingStart[MAX_ZONES] = {false};

bool rainActive = false; // combined
bool windActive = false;
bool rainByWeatherActive = false;
bool rainBySensorActive  = false;

// legacy (kept for file compat; not used in logic)
float tzOffsetHours = 9.5f;

float windSpeedThreshold = 5.0f;
float lastRainAmount = 0.0f;
uint8_t tankLowThresholdPct = 10;

int startHour [MAX_ZONES] = {0};
int startMin  [MAX_ZONES] = {0};
int startHour2[MAX_ZONES] = {0};
int startMin2 [MAX_ZONES] = {0};
int durationMin[MAX_ZONES] = {0};
int durationSec[MAX_ZONES] = {0};
int lastCheckedMinute[MAX_ZONES] = { -1, -1, -1, -1, -1, -1 };

int tankEmptyRaw = 100;
int tankFullRaw  = 900;
String zoneNames[MAX_ZONES] = {"Zone 1","Zone 2","Zone 3","Zone 4","Zone 5","Zone 6"};

unsigned long zoneStartMs[MAX_ZONES] = {0};
unsigned long lastScreenRefresh = 0;

const uint8_t expanderAddrs[] = { 0x22, 0x24 };
const uint8_t I2C_HEALTH_DEBOUNCE = 10;
uint8_t i2cFailCount = 0;

// Debug
bool dbgForceRain = false;
bool dbgForceWind = false;

// Timing
static const uint32_t LOOP_SLEEP_MS    = 20;
static const uint32_t I2C_CHECK_MS     = 1000;
static const uint32_t TIME_QUERY_MS    = 1000;
static const uint32_t SCHEDULE_TICK_MS = 1000;
static uint32_t lastI2cCheck     = 0;
static uint32_t lastTimeQuery    = 0;
static uint32_t lastScheduleTick = 0;
static bool midnightDone = false;
static tm cachedTm = {};

// Uptime
uint32_t bootMillis = 0;

// ---------- NEW: Actual rainfall history (rolling 24h) + globals for 1h/3h ----------
static float rainHist[24] = {0};   // last 24 hourly buckets (mm/hour)
static int   rainIdx = 0;          // points to most recent bucket
static time_t lastRainHistHour = 0;

float rain1hNow = 0.0f;  // mm from /weather last 1h
float rain3hNow = 0.0f;  // mm from /weather last 3h

// ---------- Prototypes ----------
void wifiCheck();
void loadConfig();
void saveConfig();
void loadSchedule();
void saveSchedule();
void updateCachedWeather();
void HomeScreen();
void RainScreen();
void updateLCDForZone(int zone);
bool shouldStartZone(int zone);
bool hasDurationCompleted(int zone);
void turnOnZone(int zone);
void turnOffZone(int zone);
void turnOnValveManual(int z);
void turnOffValveManual(int z);
void handleRoot();
void handleSubmit();
void handleSetupPage();
void handleConfigure();
void handleLogPage();
void handleClearEvents();
void handleTankCalibration();
String fetchWeather();
String fetchForecast(float lat, float lon);
bool checkWindRain();
void checkI2CHealth();
void initGpioFallback();
bool initExpanders();
void toggleBacklight();
void logEvent(int zone, const char* eventType, const char* source, bool rainDelayed);
void printCurrentTime();

// ===================== Timezone config =====================
enum TZMode : uint8_t { TZ_POSIX = 0, TZ_IANA = 1, TZ_FIXED = 2 };
TZMode tzMode = TZ_POSIX;
String tzPosix = "ACST-9:30ACDT-10:30,M10.1.0/2,M4.1.0/3"; // default
String tzIANA  = "Australia/Adelaide";
int16_t tzFixedOffsetMin = 570;

static void applyTimezoneAndSNTP() {
  const char* ntp1 = "pool.ntp.org";
  const char* ntp2 = "time.google.com";
  const char* ntp3 = "time.cloudflare.com";

  switch (tzMode) {
    case TZ_IANA: configTzTime(tzIANA.c_str(), ntp1, ntp2, ntp3); break;
    case TZ_POSIX: configTzTime(tzPosix.c_str(), ntp1, ntp2, ntp3); break;
    case TZ_FIXED: {
      long offSec = (long)tzFixedOffsetMin * 60L;
      configTime(offSec, 0, ntp1, ntp2, ntp3);
      int m = tzFixedOffsetMin; int sign = (m >= 0) ? -1 : 1; m = abs(m);
      int hh = m/60, mm = m%60; char buf[32];
      snprintf(buf, sizeof(buf), "GMT%+d:%02d", sign*hh, sign*mm);
      setenv("TZ", buf, 1); tzset(); break;
    }
  }
  time_t now = time(nullptr);
  for (int i=0; i<50 && now < 1000000000; ++i) { delay(200); now = time(nullptr); }
}

// ---------- MQTT ----------
bool   mqttEnabled = false;
String mqttBroker  = "";
uint16_t mqttPort  = 1883;
String mqttUser    = "";
String mqttPass    = "";
String mqttBase    = "espirrigation";

WiFiClient   _mqttNetCli;
PubSubClient _mqtt(_mqttNetCli);
uint32_t     _lastMqttPub = 0;

void mqttSetup(){
  if (!mqttEnabled || mqttBroker.length()==0) return;
  _mqtt.setServer(mqttBroker.c_str(), mqttPort);
  _mqtt.setCallback([](char* topic, byte* payload, unsigned int len){
    String t(topic), msg; msg.reserve(len);
    for (unsigned i=0;i<len;i++) msg += (char)payload[i];

    if (t.endsWith("/cmd/master")) {
      systemMasterEnabled = (msg=="on"||msg=="ON"||msg=="1");
      saveConfig();
    } else if (t.endsWith("/cmd/pause")) {
      uint32_t sec = msg.toInt();
      systemPaused = true; pauseUntilEpoch = sec ? (time(nullptr)+sec) : 0; saveConfig();
    } else if (t.endsWith("/cmd/resume")) {
      systemPaused=false; pauseUntilEpoch=0; saveConfig();
    } else if (t.endsWith("/cmd/stop_all")) {
      for (int z=0; z<(int)zonesCount; ++z) if (zoneActive[z]) turnOffZone(z);
    } else if (t.indexOf("/cmd/zone/")!=-1) {
      int p=t.lastIndexOf('/'); int z=(p>=0)? t.substring(p+1).toInt():-1;
      if (z>=0 && z<(int)zonesCount){
        if (msg=="on"||msg=="ON"||msg=="1") turnOnValveManual(z);
        else                                turnOffValveManual(z);
      }
    }
  });
}
void mqttEnsureConnected(){
  if (!mqttEnabled) return;
  if (_mqtt.connected()) return;
  String cid = "espirrigation-" + WiFi.macAddress();
  if (_mqtt.connect(cid.c_str(),
      mqttUser.length()?mqttUser.c_str():nullptr,
      mqttPass.length()?mqttPass.c_str():nullptr)) {
    _mqtt.subscribe( (mqttBase + "/cmd/#").c_str() );
  }
}
void mqttPublishStatus(){
  if (!mqttEnabled || !_mqtt.connected()) return;
  if (millis() - _lastMqttPub < 3000) return;
  _lastMqttPub = millis();

  DynamicJsonDocument d(1024 + MAX_ZONES*64);
  d["masterOn"] = systemMasterEnabled;
  d["paused"]   = (systemPaused && (pauseUntilEpoch==0 || time(nullptr)<(time_t)pauseUntilEpoch));
  d["cooldownRemaining"] = (rainCooldownUntilEpoch>time(nullptr)? (rainCooldownUntilEpoch - time(nullptr)) : 0);
  d["rainActive"] = rainActive;
  d["windActive"] = windActive;
  d["tankPct"]    = tankPercent();
  d["sourceMode"] = sourceModeText();
  d["rain24hActual"] = last24hActualRain();  // NEW
  d["runConcurrent"] = runZonesConcurrent;   // NEW
  JsonArray arr = d.createNestedArray("zones");
  for (int i=0;i<zonesCount;i++){
    JsonObject z = arr.createNestedObject();
    z["name"] = zoneNames[i];
    z["active"] = zoneActive[i];
  }
  String out; serializeJson(d,out);
  _mqtt.publish( (mqttBase + "/status").c_str(), out.c_str(), true);
}

// ---------- Helpers ----------
static inline int i_min(int a, int b) { return (a < b) ? a : b; }

static inline bool isPausedNow() {
  time_t now = time(nullptr);
  return systemPaused && (pauseUntilEpoch == 0 || now < (time_t)pauseUntilEpoch);
}

static inline bool isBlockedNow(){
  if (!systemMasterEnabled) return true;
  if (isPausedNow()) return true;
  time_t now = time(nullptr);
  if (rainCooldownUntilEpoch && now < (time_t)rainCooldownUntilEpoch) return true;
  return false;
}

static bool i2cPing(uint8_t addr) {
  I2Cbus.beginTransmission(addr);
  return (I2Cbus.endTransmission() == 0);
}

static String cleanName(String s) {
  s.trim(); s.replace("\r",""); s.replace("\n","");
  if (s.length() > 32) s = s.substring(0,32);
  return s;
}

int tankPercent() {
  const int N=8; uint32_t acc=0;
  for (int i=0;i<N;i++){ acc += analogRead(TANK_PIN); delayMicroseconds(200); }
  int raw=acc/N;
  int pct = map(raw, tankEmptyRaw, tankFullRaw, 0, 100);
  return constrain(pct, 0, 100);
}

bool isTankLow() {
  if (zonesCount == 6) return false;
  return tankPercent() <= tankLowThresholdPct;
}

bool physicalRainNowRaw() {
  if (!rainSensorEnabled) return false;
  pinMode(rainSensorPin, INPUT_PULLUP);
  int v = digitalRead(rainSensorPin); // LOW=dry, HIGH=wet (NC default)
  bool wet = (v == HIGH);
  if (rainSensorInvert) wet = !wet;
  return wet;
}

String sourceModeText() {
  if (zonesCount == 6) return "6-Zone";
  if (justUseTank)  return "Force:Tank";
  if (justUseMains) return "Force:Mains";
  return isTankLow() ? "Auto:Mains" : "Auto:Tank";
}

String rainDelayCauseText() {
  if (!rainDelayEnabled) return "Disabled";
  if (!rainActive) {
    time_t now=time(nullptr);
    if (rainCooldownUntilEpoch && now<(time_t)rainCooldownUntilEpoch) return "Cooldown";
    if (!systemMasterEnabled) return "MasterOff";
    if (isPausedNow()) return "Paused";
    return "No Rain";
  }
  if (rainByWeatherActive && rainBySensorActive) return "Both";
  if (rainByWeatherActive) return "Forecast/Now";
  if (rainBySensorActive)  return "Sensor";
  return "Active";
}

static const char* kHost = "espirrigation";

static void mdnsStart() {
  MDNS.end(); // in case it was running
  if (MDNS.begin(kHost)) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("[mDNS] started: http://espirrigation.local/");
  } else {
    Serial.println("[mDNS] begin() failed");
  }
}

// ---------- Next Water type + forward decl ----------
struct NextWaterInfo {
  time_t   epoch;    // local epoch for the next start
  int      zone;     // zone index
  uint32_t durSec;   // duration in seconds
};
static NextWaterInfo computeNextWatering();


// ---------- I2C init ----------
bool initExpanders() {
  bool haveIn  = i2cPing(0x22);
  bool haveOut = i2cPing(0x24);
  Serial.printf("[I2C] ping 0x22=%d 0x24=%d\n", haveIn, haveOut);
  if (!haveOut) return false;

  for (int i=0;i<3 && !pcfOut.begin();++i) delay(5);
  for (int i=0;i<3 && !pcfIn.begin(); ++i) delay(5);

  for (uint8_t ch=P0; ch<=P5; ch++) { pcfOut.pinMode(ch, OUTPUT); pcfOut.digitalWrite(ch, HIGH); }
  for (uint8_t ch=P0; ch<=P5; ch++) { pcfIn.pinMode (ch, INPUT);  pcfIn.digitalWrite(ch, HIGH); }
  return true;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  // Clamp noisy logs (IDF)
  esp_log_level_set("*", ESP_LOG_WARN);
  esp_log_level_set("i2c", ESP_LOG_NONE);
  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  esp_log_level_set("i2c_master", ESP_LOG_NONE);

  // I2C bus
  I2Cbus.begin(I2C_SDA, I2C_SCL, 100000);
  I2Cbus.setTimeOut(20);

  bootMillis = millis();

  // LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed; formatting…");
    if (!(LittleFS.format() && LittleFS.begin())) {
      Serial.println("LittleFS unavailable; halt.");
      while (true) delay(1000);
    }
  }

  // Config + schedule
  loadConfig();
  if (!LittleFS.exists("/schedule.txt")) saveSchedule();
  loadSchedule();

  mainsChannel = P4; 
  tankChannel  = P5;

  // PCF8574 expanders (or GPIO fallback)
  if (!initExpanders()) {
    Serial.println("PCF8574 relays not found; GPIO fallback.");
    initGpioFallback();
    useGpioFallback = true;
  } else {
    useGpioFallback = false;
    checkI2CHealth();
  }

  pinMode(LED_PIN, OUTPUT);

  // OLED boot splash
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 init failed");
    while (true) delay(100);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2); display.setCursor(0, 8);  display.print("Irrigation");
  display.setTextSize(1); display.setCursor(0, 36); display.print("AP: ESPIrrigationAP");
  display.setCursor(0, 50); display.print("http://192.168.4.1");
  display.display();

  // Hostname + WiFi events
  WiFi.setHostname(kHost);

  // WiFiManager connect
  wifiManager.setTimeout(180);
  if (!wifiManager.autoConnect("ESPIrrigationAP")) {
    ESP.restart();
  }

  // Connected screen
  display.clearDisplay();
  display.setTextSize(2); display.setCursor(0, 0);  display.print("Connected!");
  display.setTextSize(1); display.setCursor(0, 20); display.print(WiFi.localIP().toString());
  display.setCursor(0, 32); display.print("espirrigation.local");
  display.display();
  delay(5000);

  // Timezone + SNTP
  delay(250);
  applyTimezoneAndSNTP();
  {
    time_t now = time(nullptr);
    struct tm tcheck; localtime_r(&now, &tcheck);
    Serial.printf("[TIME] NTP ok: %04d-%02d-%02d %02d:%02d:%02d tz=%s mode=%d\n",
      tcheck.tm_year+1900, tcheck.tm_mon+1, tcheck.tm_mday,
      tcheck.tm_hour, tcheck.tm_min, tcheck.tm_sec,
      (tcheck.tm_isdst>0) ? "DST" : "STD", (int)tzMode);
  }

  // OTA
  #if ENABLE_OTA
    ArduinoOTA.setHostname("ESP32-Irrigation-OTA");
    ArduinoOTA.begin();
  #endif

  mdnsStart();

  // -------- Routes --------
  server.on("/", HTTP_GET, handleRoot);
  server.on("/submit", HTTP_POST, handleSubmit);

  server.on("/setup", HTTP_GET, handleSetupPage);
  server.on("/configure", HTTP_POST, handleConfigure);

  server.on("/events", HTTP_GET, handleLogPage);
  server.on("/clearevents", HTTP_POST, handleClearEvents);

  server.on("/tank", HTTP_GET, handleTankCalibration);

  // /status JSON
  server.on("/status", HTTP_GET, [](){
    DynamicJsonDocument doc(4096 + MAX_ZONES * 160);
    updateCachedWeather();

    doc["rainDelayActive"] = rainActive;
    doc["windDelayActive"] = windActive;
    doc["rainDelayCause"]  = rainDelayCauseText();
    doc["zonesCount"]      = zonesCount;
    doc["tankPct"]         = tankPercent();
    doc["sourceMode"]      = sourceModeText();
    doc["rssi"]            = WiFi.RSSI();
    doc["uptimeSec"]       = (millis() - bootMillis) / 1000;

    // Current rain (actuals) — globals populated in updateCachedWeather()
    doc["rain1hNow"] = rain1hNow;
    doc["rain3hNow"] = rain3hNow;

    // Forecast fields
    doc["rain12h"]     = isnan(rainNext12h_mm) ? 0.0f : rainNext12h_mm;
    doc["rain24h"]     = isnan(rainNext24h_mm) ? 0.0f : rainNext24h_mm;
    doc["pop12h"]      = (popNext12h_pct < 0 ? 0 : popNext12h_pct);
    doc["nextRainInH"] = (nextRainIn_h < 0 ? 255 : nextRainIn_h);
    doc["gust24h"]     = isnan(maxGust24h_ms) ? 0.0f : maxGust24h_ms;
    doc["tmin"]        = isnan(todayMin_C) ? 0.0f : todayMin_C;
    doc["tmax"]        = isnan(todayMax_C) ? 0.0f : todayMax_C;
    doc["sunrise"]     = (uint32_t)todaySunrise;
    doc["sunset"]      = (uint32_t)todaySunset;

    // local vs UTC offset
    time_t nowEpoch = time(nullptr);
    struct tm ltm; localtime_r(&nowEpoch, &ltm);
    struct tm gtm; gmtime_r(&nowEpoch,  &gtm);
    int localMin = ltm.tm_hour*60 + ltm.tm_min;
    int utcMin   = gtm.tm_hour*60 + gtm.tm_min;
    int deltaMin = localMin - utcMin;
    if (deltaMin >  12*60) deltaMin -= 24*60;
    if (deltaMin < -12*60) deltaMin += 24*60;

    auto hhmm = [](time_t t){
      char b[6]; struct tm tt; localtime_r(&t,&tt);
      strftime(b,sizeof(b),"%H:%M",&tt); return String(b);
    };

    doc["deviceEpoch"]  = (uint32_t)nowEpoch;
    doc["utcOffsetMin"] = deltaMin;
    doc["isDST"]        = (ltm.tm_isdst > 0);
    doc["tzAbbrev"]     = (ltm.tm_isdst>0) ? "DST" : "STD";
    doc["sunriseLocal"] = hhmm((time_t)todaySunrise);
    doc["sunsetLocal"]  = hhmm((time_t)todaySunset);

    // New feature gates
    doc["masterOn"]          = systemMasterEnabled;
    doc["cooldownUntil"]     = rainCooldownUntilEpoch;
    doc["cooldownRemaining"] = (rainCooldownUntilEpoch>nowEpoch) ? (rainCooldownUntilEpoch - nowEpoch) : 0;
    doc["rainThresh24h"]     = rainThreshold24h_mm;
    doc["rainCooldownMin"]   = rainCooldownMin;
    doc["rainCooldownHours"] = rainCooldownMin / 60;

    // NEW: expose run mode
    doc["runConcurrent"] = runZonesConcurrent;

    // Zones snapshot
    JsonArray zones = doc.createNestedArray("zones");
    for (int i=0; i<zonesCount; i++){
      JsonObject z = zones.createNestedObject();
      z["active"] = zoneActive[i];
      z["name"]   = zoneNames[i];
      unsigned long rem = 0;
      if (zoneActive[i]) {
        unsigned long elapsed = (millis() - zoneStartMs[i]) / 1000;
        unsigned long total   = (unsigned long)durationMin[i] * 60 + durationSec[i];
        rem = (elapsed < total ? total - elapsed : 0);
      }
      z["remaining"] = rem;
      z["totalSec"]  = (unsigned long)durationMin[i] * 60 + durationSec[i];
    }

    // Current weather pass-through
    {
      DynamicJsonDocument js(2048);
      if (deserializeJson(js, cachedWeatherData) == DeserializationError::Ok) {
        doc["temp"]       = js["main"]["temp"]       | 0.0f;
        doc["feels_like"] = js["main"]["feels_like"] | 0.0f;
        doc["humidity"]   = js["main"]["humidity"]   | 0;
        doc["pressure"]   = js["main"]["pressure"]   | 0;
        doc["wind"]       = js["wind"]["speed"]      | 0.0f;
        doc["gustNow"]    = js["wind"]["gust"]       | 0.0f;
        doc["condMain"]   = js["weather"][0]["main"]        | "";
        doc["condDesc"]   = js["weather"][0]["description"] | "";
        doc["icon"]       = js["weather"][0]["icon"]        | "";
        doc["cityName"]   = js["name"]                      | "";
        doc["owmTzSec"]   = js["timezone"]                  | 0;
      }
    }

    // Next Water (queue-first)
    {
      NextWaterInfo nw = computeNextWatering();
      if (nw.zone >= 0) {
        doc["nextWaterEpoch"]  = (uint32_t)nw.epoch;
        doc["nextWaterZone"]   = nw.zone;
        doc["nextWaterName"]   = zoneNames[nw.zone];
        doc["nextWaterDurSec"] = nw.durSec;
      } else {
        doc["nextWaterEpoch"]  = 0;
        doc["nextWaterZone"]   = 255;
        doc["nextWaterName"]   = "";
        doc["nextWaterDurSec"] = 0;
      }
    }

    // Pause / delay toggles
    doc["systemPaused"]          = isPausedNow();
    doc["pauseUntil"]            = pauseUntilEpoch;
    doc["rainForecastEnabled"]   = rainDelayFromForecastEnabled;
    doc["rainSensorEnabled"]     = rainSensorEnabled;

    // NEW: actual rolling 24h rainfall
    doc["rain24hActual"] = last24hActualRain();

    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  // Time API
  server.on("/api/time", HTTP_GET, [](){
    time_t nowEpoch = time(nullptr);
    struct tm lt; localtime_r(&nowEpoch, &lt);
    struct tm gt; gmtime_r(&nowEpoch,  &gt);
    DynamicJsonDocument d(512);
    d["epoch"] = (uint32_t)nowEpoch;
    char buf[32];
    strftime(buf,sizeof(buf),"%Y-%m-%d %H:%M:%S",&lt); d["local"] = buf;
    strftime(buf,sizeof(buf),"%Y-%m-%d %H:%M:%S",&gt); d["utc"]   = buf;
    d["isDST"] = (lt.tm_isdst>0);
    d["tz"]    = (lt.tm_isdst>0) ? "DST" : "STD";
    String out; serializeJson(d,out);
    server.send(200,"application/json",out);
  });

  // Tank calibration endpoints
  server.on("/setTankEmpty", HTTP_POST, []() {
    tankEmptyRaw = analogRead(TANK_PIN);
    saveConfig();
    server.sendHeader("Location", "/tank", true);
    server.send(302, "text/plain", "");
  });
  server.on("/setTankFull", HTTP_POST, []() {
    tankFullRaw = analogRead(TANK_PIN);
    saveConfig();
    server.sendHeader("Location", "/tank", true);
    server.send(302, "text/plain", "");
  });

  // Manual control per zone
  for (int i=0; i<MAX_ZONES; i++){
    server.on(String("/valve/on/")+i,  HTTP_POST, [i](){ turnOnValveManual(i);  server.send(200,"text/plain","OK"); });
    server.on(String("/valve/off/")+i, HTTP_POST, [i](){ turnOffValveManual(i); server.send(200,"text/plain","OK"); });
  }
  server.on("/stopall", HTTP_POST, [](){
    for (int z=0; z<MAX_ZONES; ++z) if (zoneActive[z]) turnOffZone(z);
    server.send(200,"text/plain","OK");
  });
  server.on("/toggleBacklight", HTTP_POST, [](){
    static bool inverted=false; inverted=!inverted;
    display.invertDisplay(inverted);
    server.send(200,"text/plain","Backlight toggled");
  });

  // I²C tools
  server.on("/i2c-test", HTTP_GET, [](){
    if (useGpioFallback) { server.send(500,"text/plain","Fallback active"); return; }
    for (uint8_t ch : PCH) { pcfOut.digitalWrite(ch, LOW); delay(100); pcfOut.digitalWrite(ch, HIGH); delay(60); }
    server.send(200,"text/plain","PCF8574 pulse OK");
  });
  server.on("/i2c-scan", HTTP_GET, [](){
    String s="I2C scan:\n"; byte count=0;
    for (uint8_t addr=1; addr<127; addr++){
      I2Cbus.beginTransmission(addr);
      if (I2Cbus.endTransmission()==0){ s+=" - Found 0x"+String(addr,HEX)+"\n"; count++; delay(2); }
    }
    if (!count) s += " (no devices)\n";
    server.send(200,"text/plain",s);
  });

  // Downloads / Admin
  server.on("/download/config.txt", HTTP_GET, [](){
    if (LittleFS.exists("/config.txt")){ File f=LittleFS.open("/config.txt","r"); server.streamFile(f,"text/plain"); f.close(); }
    else server.send(404,"text/plain","missing");
  });
  server.on("/download/schedule.txt", HTTP_GET, [](){
    if (LittleFS.exists("/schedule.txt")){ File f=LittleFS.open("/schedule.txt","r"); server.streamFile(f,"text/plain"); f.close(); }
    else server.send(404,"text/plain","missing");
  });
  server.on("/download/events.csv", HTTP_GET, [](){
    if (LittleFS.exists("/events.csv")){ File f=LittleFS.open("/events.csv","r"); server.streamFile(f,"text/csv"); f.close(); }
    else server.send(404,"text/plain","No event log");
  });
  server.on("/reboot", HTTP_POST, [](){
    server.send(200,"text/plain","restarting"); delay(200); ESP.restart();
  });
  server.on("/whereami", HTTP_GET, [](){
    String s;
    s = "IP: " + WiFi.localIP().toString() + "\n";
    s+= "Host: espirrigation.local\n";
    s+= "SSID: " + WiFi.SSID() + "\n";
    s+= "RSSI: " + String(WiFi.RSSI()) + " dBm\n";
    s+= "Mode: " + sourceModeText() + "\n";
    server.send(200,"text/plain",s);
  });

  // Pause/Resume/Delays/Forecast toggle
  server.on("/clear_delays", HTTP_POST, [](){
    for (int z=0; z<(int)zonesCount; ++z) pendingStart[z] = false;
    for (int z=0; z<(int)zonesCount; ++z) if (zoneActive[z]) turnOffZone(z);
    dbgForceRain = false; dbgForceWind = false;
    for (int z=0; z<(int)zonesCount; ++z) lastCheckedMinute[z] = -1;
    server.send(200,"text/plain","OK");
  });
  server.on("/pause", HTTP_POST, [](){
    time_t nowEp = time(nullptr);
    String secStr = server.arg("sec");
    uint32_t sec = secStr.length()? secStr.toInt() : (24u*3600u);
    systemPaused = true;
    pauseUntilEpoch = sec ? (nowEp + sec) : 0;
    saveConfig();
    server.send(200,"text/plain","OK");
  });
  server.on("/resume", HTTP_POST, [](){
    systemPaused = false; pauseUntilEpoch = 0; saveConfig();
    server.send(200,"text/plain","OK");
  });
  server.on("/set_rain_forecast", HTTP_POST, [](){
    rainDelayFromForecastEnabled = server.hasArg("on");
    saveConfig();
    server.send(200,"text/plain", rainDelayFromForecastEnabled ? "on" : "off");
  });

  // Master and cooldown
  server.on("/master", HTTP_POST, [](){
    systemMasterEnabled = server.hasArg("on");  // Dashboard only (not in Setup)
    saveConfig();
    server.send(200,"text/plain", systemMasterEnabled ? "on" : "off");
  });
  server.on("/clear_cooldown", HTTP_POST, [](){
    rainCooldownUntilEpoch = 0;
    server.send(200,"text/plain","OK");
  });

  server.begin();

  // MQTT
  mqttSetup();
  mqttEnsureConnected();
}


// ---------- Loop ----------
void loop() {
  const uint32_t now = millis();
  #if ENABLE_OTA
  ArduinoOTA.handle();
  #endif

  server.handleClient();
  wifiCheck();
  checkWindRain();

  // Hard block while paused/master off/cooldown
  if (isBlockedNow()) {
    for (int z=0; z<(int)zonesCount; ++z) if (zoneActive[z]) turnOffZone(z);
    if (now - lastScreenRefresh >= 1000) { lastScreenRefresh = now; RainScreen(); }
    delay(15); return;
  }

  if (rainActive || windActive) {
    for (int z=0; z<(int)zonesCount; ++z) if (zoneActive[z]) turnOffZone(z);
    if (now - lastScreenRefresh >= 1000) { lastScreenRefresh = now; RainScreen(); }
    delay(15); return;
  }

  if (now - lastTimeQuery >= TIME_QUERY_MS) {
    lastTimeQuery = now;
    time_t nowTime = time(nullptr);
    localtime_r(&nowTime, &cachedTm);
  }

  if (cachedTm.tm_hour == 0 && cachedTm.tm_min == 0) {
    if (!midnightDone) {
      memset(lastCheckedMinute, -1, sizeof(lastCheckedMinute));
      midnightDone = true;
    }
  } else midnightDone = false;

  if (!useGpioFallback && (now - lastI2cCheck >= I2C_CHECK_MS)) {
    lastI2cCheck = now; checkI2CHealth();
  }

  bool anyActive=false;
  for (int z=0; z<(int)zonesCount; z++) if (zoneActive[z]) { anyActive=true; break; }

  // -------- SCHEDULER (supports sequential or concurrent) --------
  if (now - lastScheduleTick >= SCHEDULE_TICK_MS) {
    lastScheduleTick = now;

    if (!isBlockedNow()) {
      for (int z=0; z<(int)zonesCount; z++) {
        if (shouldStartZone(z)) {
          if (rainActive) { pendingStart[z]=true; logEvent(z,"RAIN DELAY","N/A",true); continue; }
          if (windActive) { pendingStart[z]=true; logEvent(z,"WIND DELAY","N/A",false); continue; }

          if (!runZonesConcurrent) {
            // sequential: only start if nothing is running
            if (zonesCount==4 && isTankLow()) {
              if (anyActive) pendingStart[z]=true;
              else { turnOnZone(z); anyActive=true; }
            } else {
              if (anyActive) pendingStart[z]=true;
              else { turnOnZone(z); anyActive=true; }
            }
          } else {
            // concurrent: start regardless of other active zones
            turnOnZone(z); anyActive = true;
          }
        }
        if (zoneActive[z] && hasDurationCompleted(z)) turnOffZone(z);
      }

      if (!runZonesConcurrent) {
        // sequential: drain one queued zone if nothing currently running
        if (!anyActive) {
          for (int z=0; z<(int)zonesCount; z++) if (pendingStart[z]) { pendingStart[z]=false; turnOnZone(z); break; }
        }
      }
      // concurrent: queued starts (due to delays/blocks) will fire when delays clear
    }
  }

  if (now - lastScreenRefresh >= 1000) { lastScreenRefresh = now; HomeScreen(); }
  delay(LOOP_SLEEP_MS);
}

// ---------- Connectivity / I2C health ----------
void wifiCheck() {
  if (WiFi.status()!=WL_CONNECTED) {
    Serial.println("Reconnecting WiFi…");
    WiFi.disconnect(); delay(600);
    if (!wifiManager.autoConnect("ESPIrrigationAP")) Serial.println("Reconnection failed.");
    else Serial.println("Reconnected.");
  }
}

void checkI2CHealth() {
  delay(20);
  bool anyErr=false;
  for (uint8_t addr : expanderAddrs) {
    I2Cbus.beginTransmission(addr);
    if (I2Cbus.endTransmission()!=0) { anyErr=true; break; }
  }
  if (anyErr) {
    i2cFailCount++;
    if (i2cFailCount >= I2C_HEALTH_DEBOUNCE) {
      Serial.println("I2C unstable → GPIO fallback");
      useGpioFallback = true; initGpioFallback();
    }
  } else i2cFailCount = 0;
}

void initGpioFallback() {
  useGpioFallback = true;
  for (uint8_t i=0;i<MAX_ZONES;i++){ pinMode(zonePins[i],OUTPUT); digitalWrite(zonePins[i],LOW); }
  pinMode(mainsPin,OUTPUT); digitalWrite(mainsPin,LOW);
  pinMode(tankPin, OUTPUT); digitalWrite(tankPin, LOW);
}

// ---------- Weather / Forecast ----------
String fetchWeather() {
  if (apiKey.length()<5 || city.length()<1) return "{}";
  HTTPClient http; http.setTimeout(5000);
  String url="http://api.openweathermap.org/data/2.5/weather?id="+city+"&appid="+apiKey+"&units=metric";
  http.begin(client,url);
  int code=http.GET();
  String payload=(code>0)?http.getString():"{}";
  http.end(); return payload;
}

String fetchForecast(float lat, float lon) {
  if (apiKey.length() < 5) return "{}";
  HTTPClient http; 
  http.setTimeout(6500);

  String url = "http://api.openweathermap.org/data/2.5/onecall?lat=" + String(lat,6) +
               "&lon=" + String(lon,6) +
               "&appid=" + apiKey +
               "&units=metric&exclude=minutely,alerts";

  http.begin(client, url);
  int code = http.GET();
  String payload = (code > 0) ? http.getString() : "{}";
  http.end();
  return payload;
}

// ---------- NEW helpers for rain history ----------
static void tickActualRainHistory() {
  time_t now = time(nullptr);
  struct tm lt; localtime_r(&now, &lt);
  if (lt.tm_min == 0 && lt.tm_sec < 5 && (now - lastRainHistHour) > 300) {
    float v = (!isnan(rain1hNow) && rain1hNow > 0.0f) ? rain1hNow
             : ((!isnan(rain3hNow) && rain3hNow > 0.0f) ? (rain3hNow / 3.0f) : 0.0f);
    rainIdx = (rainIdx + 1) % 24;
    rainHist[rainIdx] = v;
    lastRainHistHour = now;
  }
}
static float last24hActualRain() {
  float s = 0.0f;
  for (int i = 0; i < 24; ++i) s += rainHist[i];
  return s;
}

void updateCachedWeather() {
  unsigned long nowms = millis();
  bool needCur = (cachedWeatherData == "" || (nowms - lastWeatherUpdate >= weatherUpdateInterval));
  bool haveCoord = false; float lat = NAN, lon = NAN;

  if (needCur) { cachedWeatherData = fetchWeather(); lastWeatherUpdate = nowms; }

  // Extract coordinates for OneCall
  {
    DynamicJsonDocument js(1024);
    if (deserializeJson(js, cachedWeatherData) == DeserializationError::Ok) {
      if (js["coord"]["lat"].is<float>() && js["coord"]["lon"].is<float>()) {
        lat = js["coord"]["lat"].as<float>();
        lon = js["coord"]["lon"].as<float>();
        haveCoord = true;
      }
      // NEW: populate global rain1hNow / rain3hNow from /weather
      float r1 = 0.0f, r3 = 0.0f;
      JsonVariant rv = js["rain"];
      if (!rv.isNull()) {
        if (rv["1h"].is<float>() || rv["1h"].is<double>() || rv["1h"].is<int>()) r1 = rv["1h"].as<float>();
        if (r1 == 0.0f && (rv.is<float>() || rv.is<double>() || rv.is<int>())) r1 = rv.as<float>();
      }
      rain1hNow = r1;
      rain3hNow = r3;
    }
  }

  // ---- Forecast fetch / parse ----
  if (haveCoord && (cachedForecastData == "" || (nowms - lastForecastUpdate >= forecastUpdateInterval))) {
    cachedForecastData = fetchForecast(lat, lon);
    lastForecastUpdate = nowms;

    rainNext12h_mm = 0; 
    rainNext24h_mm = 0; 
    popNext12h_pct = 0; 
    nextRainIn_h   = -1;
    maxGust24h_ms  = 0; 
    todayMin_C     = NAN; 
    todayMax_C     = NAN; 
    todaySunrise   = 0;   
    todaySunset    = 0;

    DynamicJsonDocument fc(14 * 1024);
    if (deserializeJson(fc, cachedForecastData) == DeserializationError::Ok) {
      // Daily snapshot (min/max & rise/set)
      if (fc["daily"].is<JsonArray>() && fc["daily"].size() > 0) {
        JsonObject d0 = fc["daily"][0];
        todayMin_C   = d0["temp"]["min"] | NAN;
        todayMax_C   = d0["temp"]["max"] | NAN;
        todaySunrise = (time_t)(d0["sunrise"] | 0);
        todaySunset  = (time_t)(d0["sunset"]  | 0);
      }

      // Helper: accept either number or {"1h":x}
      auto read1h = [](JsonVariant v) -> float {
        if (v.isNull()) return 0.0f;
        if (v.is<float>() || v.is<double>() || v.is<int>()) return v.as<float>();
        JsonVariant one = v["1h"];
        return one.isNull() ? 0.0f : one.as<float>();
      };

      // Prefer hourly accumulation
      if (fc["hourly"].is<JsonArray>()) {
        JsonArray hr = fc["hourly"].as<JsonArray>();
        int hrs = hr.size();
        int L24 = min(24, hrs);
        int L12 = min(12, hrs);

        for (int i = 0; i < L24; i++) {
          JsonObject h = hr[i];

          float r = 0.0f;
          r += read1h(h["rain"]);
          r += read1h(h["snow"]);  // treat snow as mm water equivalent

          if (i < L12) {
            rainNext12h_mm += r;
            int pop = (int)roundf(100.0f * (h["pop"] | 0.0f));
            if (pop > popNext12h_pct) popNext12h_pct = pop;
          }
          rainNext24h_mm += r;

          if (nextRainIn_h < 0) {
            float popf = h["pop"] | 0.0f;
            if (r > 0.01f || popf >= 0.5f) nextRainIn_h = i;
          }

          float g = h["wind_gust"] | 0.0f;
          if (g > maxGust24h_ms) maxGust24h_ms = g;
        }
      }

      // Fallback: if hourly sum is zero, use daily totals (rain + snow)
      if (rainNext24h_mm <= 0.0f && fc["daily"].is<JsonArray>() && fc["daily"].size() > 0) {
        JsonObject d0 = fc["daily"][0];
        float dailyTotal = 0.0f;

        if (!d0["rain"].isNull()) {
          if (d0["rain"].is<float>() || d0["rain"].is<double>() || d0["rain"].is<int>())
            dailyTotal += d0["rain"].as<float>();
        }
        if (!d0["snow"].isNull()) {
          if (d0["snow"].is<float>() || d0["snow"].is<double>() || d0["snow"].is<int>())
            dailyTotal += d0["snow"].as<float>();
        }

        if (dailyTotal > 0.0f) {
          rainNext24h_mm = dailyTotal;
          if (rainNext12h_mm <= 0.0f) rainNext12h_mm = dailyTotal * 0.5f; // rough split
        }
      }
    }

    // Defensive clamp
    if (isnan(rainNext12h_mm) || rainNext12h_mm < 0) rainNext12h_mm = 0.0f;
    if (isnan(rainNext24h_mm) || rainNext24h_mm < 0) rainNext24h_mm = 0.0f;
    if (isnan(maxGust24h_ms)  || maxGust24h_ms  < 0) maxGust24h_ms  = 0.0f;
  }

  // ---- Fallback sunrise/sunset & min/max from current weather ----
  DynamicJsonDocument cur(2048);
  if (deserializeJson(cur, cachedWeatherData) == DeserializationError::Ok) {
    time_t sr = (time_t)(cur["sys"]["sunrise"] | 0);
    time_t ss = (time_t)(cur["sys"]["sunset"]  | 0);
    if (sr > 0) todaySunrise = sr;
    if (ss > 0) todaySunset  = ss;

    if (isnan(todayMin_C)) todayMin_C = cur["main"]["temp_min"] | NAN;
    if (isnan(todayMax_C)) todayMax_C = cur["main"]["temp_max"] | NAN;
  }

  // NEW: tick the rolling actual history at top of hour (based on globals)
  tickActualRainHistory();
}

// ---------- Rain/Wind logic with cooldown & threshold ----------
bool checkWindRain() {
  updateCachedWeather();
  DynamicJsonDocument js(1024);
  bool weatherOk = (deserializeJson(js, cachedWeatherData) == DeserializationError::Ok);

  // Current-conditions rain
  rainByWeatherActive = false; 
  lastRainAmount = 0.0f;
  if (weatherOk && rainDelayEnabled && rainDelayFromForecastEnabled) {
    float r1 = js["rain"]["1h"] | 0.0f;
    String wmain  = js["weather"][0]["main"] | "";
    lastRainAmount = r1;
    rainByWeatherActive = (r1 > 0.0f) || (wmain=="Rain" || wmain=="Thunderstorm" || wmain=="Drizzle");
  }

  // Physical sensor
  rainBySensorActive = (rainSensorEnabled ? physicalRainNowRaw() : false);

  // Forecast accumulation threshold (24h)
  bool forecastThresholdActive = false;
  if (rainDelayEnabled && rainDelayFromForecastEnabled) {
    if (!isnan(rainNext24h_mm) && rainNext24h_mm >= (float)rainThreshold24h_mm) {
      forecastThresholdActive = true;
    }
  }

  // Combined (+ debug)
  bool rainForce = dbgForceRain;
  bool windForce = dbgForceWind;

  static bool prevRain = false;
  bool newRainActive =
      rainDelayEnabled && (rainByWeatherActive || rainBySensorActive || forecastThresholdActive || rainForce);
  bool newWindActive = false;

  if (weatherOk && windDelayEnabled) {
    float windSpd = js["wind"]["speed"] | 0.0f;
    newWindActive = (windSpd >= windSpeedThreshold) || windForce;
  }

  // Start cooldown when rain clears
  if (prevRain && !newRainActive) {
    if (rainCooldownMin > 0) {
      time_t now = time(nullptr);
      rainCooldownUntilEpoch = (uint32_t)(now + (uint32_t)rainCooldownMin * 60u);
    }
  }
  prevRain = newRainActive;

  rainActive = newRainActive;
  windActive = newWindActive;

  return !(rainActive || windActive);
}

// ---------- Event log ----------
void logEvent(int zone, const char* eventType, const char* source, bool rainDelayed) {
  updateCachedWeather();
  DynamicJsonDocument js(512);
  float temp=NAN, wind=NAN; int hum=0; String cond="?", cname="-";
  if (deserializeJson(js,cachedWeatherData)==DeserializationError::Ok) {
    temp = js["main"]["temp"].as<float>();
    hum  = js["main"]["humidity"].as<int>();
    wind = js["wind"]["speed"].as<float>();
    cond = js["weather"][0]["main"].as<const char*>();
    cname= js["name"].as<const char*>();
  }

  File f = LittleFS.open("/events.csv","a");
  if (!f) return;

  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  char ts[32];
  sprintf(ts,"%04d-%02d-%02d %02d:%02d:%02d", t->tm_year+1900,t->tm_mon+1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);

  String line; line.reserve(200);
  line += ts; line += ","; line += (zone>=0?zoneNames[zone]:String("n/a")); line += ",";
  line += eventType; line += ","; line += source; line += ",";
  line += (rainDelayed?"Active":"Off"); line += ",";
  line += String(temp,1); line += ","; line += String(hum); line += ",";
  line += String(wind,1); line += ","; line += cond; line += ","; line += cname; line += "\n";

  f.print(line); f.close();
}

// ---------- OLED UI ----------
void toggleBacklight(){ static bool inverted=false; inverted=!inverted; display.invertDisplay(inverted); }

void updateLCDForZone(int zone) {
  static unsigned long lastUpdate=0; unsigned long now=millis();
  if (now - lastUpdate < 1000) return; lastUpdate = now;

  unsigned long elapsed=(now - zoneStartMs[zone]) / 1000;
  unsigned long total=(unsigned long)durationMin[zone]*60 + (unsigned long)durationSec[zone];
  unsigned long rem = (elapsed<total ? total - elapsed : 0);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0); display.print(zoneNames[zone]); display.print(" ");
  display.print(elapsed/60); display.print(":"); if ((elapsed%60)<10) display.print('0'); display.print(elapsed%60);

  display.setCursor(0,12);
  if (elapsed<total){ display.print(rem/60); display.print("m Remaining"); }
  else display.print("Complete");
  display.display();
}

void RainScreen(){
  display.clearDisplay();
  display.setTextSize(2); display.setCursor(0,0); display.print(isPausedNow()? "System Pause" : "Rain/Wind");
  display.setTextSize(1);
  display.setCursor(0,20); display.printf("Last: %.2f mm", lastRainAmount);
  display.setCursor(0,32); display.print("Cause: "); display.print(rainDelayCauseText());
  int delayed=0; for (int i=0;i<(int)zonesCount;i++) if (pendingStart[i]) delayed++;
  display.setCursor(0,46); display.printf("Queued: %d", delayed);
  display.display();
}

void HomeScreen() {
  updateCachedWeather();
  DynamicJsonDocument js(1024); (void)deserializeJson(js,cachedWeatherData);

  float temp = js["main"]["temp"].as<float>();
  int   hum  = js["main"]["humidity"].as<int>();
  int   pct  = tankPercent();

  time_t now = time(nullptr);
  struct tm* t = localtime(&now);

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0); display.printf("%02d:%02d", t->tm_hour, t->tm_min);
  display.setTextSize(1);
  display.setCursor(80,3); display.printf("%02d/%02d", t->tm_mday, t->tm_mon+1);
  display.setCursor(66,3); display.print( (t->tm_isdst>0) ? "DST" : "STD" );

  display.setCursor(0,20);
  if (!isnan(temp) && hum>=0) display.printf("T:%2.0fC H:%02d%%", temp, hum);
  else display.print("T:-- H:--");

  display.setCursor(0,32);
  if (zonesCount==4) { display.printf("Tank:%3d%% ", pct); display.print(isTankLow()? "(Mains)":"(Tank)"); }
  else { display.print("Zones: "); display.print(zonesCount); }

  for (int i=0;i<i_min(2,(int)zonesCount);i++){
    int x=(i==0)?0:64;
    display.setCursor(x,44);
    if (zoneActive[i]) {
      unsigned long elapsed=(millis()-zoneStartMs[i])/1000;
      unsigned long total=(unsigned long)durationMin[i]*60 + durationSec[i];
      unsigned long rem=(elapsed<total?total-elapsed:0);
      display.printf("Z%d:%02d:%02d", i+1, (int)(rem/60),(int)(rem%60));
    } else display.printf("Z%d:Off", i+1);
  }
  for (int i=2;i<i_min(4,(int)zonesCount);i++){
    int x=(i==2)?0:64;
    display.setCursor(x,56);
    if (zoneActive[i]) {
      unsigned long elapsed=(millis()-zoneStartMs[i])/1000;
      unsigned long total=(unsigned long)durationMin[i]*60 + durationSec[i];
      unsigned long rem=(elapsed<total?total-elapsed:0);
      display.printf("Z%d:%02d:%02d", i+1, (int)(rem/60),(int)(rem%60));
    } else display.printf("Z%d:Off", i+1);
  }
  display.display();
}

bool shouldStartZone(int zone) {
  if (zone < 0 || zone >= (int)MAX_ZONES) return false;

  time_t now = time(nullptr);
  struct tm* tt = localtime(&now);
  if (!tt) return false;

  const int wd = tt->tm_wday; // Sun=0..Sat=6
  const int hr = tt->tm_hour;
  const int mn = tt->tm_min;

  if (lastCheckedMinute[zone] == mn) return false;      // avoid dup triggers
  if (!days[zone][wd]) return false;                    // day not enabled
  if (durationMin[zone] == 0 && durationSec[zone] == 0) return false; // zero duration

  const bool match1 = (hr == startHour[zone]  && mn == startMin[zone]);
  const bool match2 = (enableStartTime2[zone] && hr == startHour2[zone] && mn == startMin2[zone]);

  if (match1 || match2) { lastCheckedMinute[zone] = mn; return true; }
  return false;
}

bool hasDurationCompleted(int zone) {
  unsigned long elapsed=(millis()-zoneStartMs[zone])/1000;
  unsigned long total=(unsigned long)durationMin[zone]*60 + (unsigned long)durationSec[zone];
  return (elapsed >= total);
}

// ---------- Valve control ----------
void turnOnZone(int z) {
  checkWindRain();
  Serial.printf("[VALVE] Request ON Z%d rain=%d wind=%d blocked=%d\n", z+1, rainActive?1:0, windActive?1:0, isBlockedNow()?1:0);

  if (isBlockedNow())  { pendingStart[z]=true; logEvent(z,"BLOCKED","PAUSE/COOLDOWN/MASTER",false); return; }
  if (rainActive)      { pendingStart[z]=true; logEvent(z,"RAIN DELAY","N/A",true);  return; }
  if (windActive)      { pendingStart[z]=true; logEvent(z,"WIND DELAY","N/A",false); return; }

  bool anyOn=false; for (int i=0;i<(int)zonesCount;i++) if (zoneActive[i]) { anyOn=true; break; }
  if (!runZonesConcurrent && anyOn) { pendingStart[z]=true; logEvent(z,"QUEUED","ACTIVE RUN",false); return; }

  zoneStartMs[z] = millis();
  zoneActive[z] = true;
  const char* src = "None";

  if (useGpioFallback) {
    digitalWrite(zonePins[z], HIGH);
    if (zonesCount==4) {
      if (justUseTank){ src="Tank";  digitalWrite(mainsPin,LOW);  digitalWrite(tankPin,HIGH); }
      else if (justUseMains){ src="Mains"; digitalWrite(mainsPin,HIGH); digitalWrite(tankPin,LOW); }
      else if (isTankLow()) { src="Mains"; digitalWrite(mainsPin,HIGH); digitalWrite(tankPin,LOW); }
      else { src="Tank";  digitalWrite(mainsPin,LOW);  digitalWrite(tankPin,HIGH); }
    }
  } else {
    pcfOut.digitalWrite(PCH[z], LOW); // active-LOW
    if (zonesCount==4) {
      if (justUseTank){ src="Tank";  pcfOut.digitalWrite(mainsChannel,HIGH); pcfOut.digitalWrite(tankChannel,LOW); }
      else if (justUseMains){ src="Mains"; pcfOut.digitalWrite(mainsChannel,LOW);  pcfOut.digitalWrite(tankChannel,HIGH); }
      else if (isTankLow()) { src="Mains"; pcfOut.digitalWrite(mainsChannel,LOW);  pcfOut.digitalWrite(tankChannel,HIGH); }
      else { src="Tank";  pcfOut.digitalWrite(mainsChannel,HIGH); pcfOut.digitalWrite(tankChannel,LOW); }
    }
  }

  logEvent(z,"START",src,false);

  display.clearDisplay();
  display.setTextSize(2); display.setCursor(2,0); display.print(zoneNames[z]); display.print(" ON");
  display.display(); delay(350); HomeScreen();
}

void turnOffZone(int z) {
  Serial.printf("[VALVE] Request OFF Z%d\n", z+1);
  const char* src="None";
  if (zonesCount==4) src = justUseTank ? "Tank" : justUseMains ? "Mains" : isTankLow() ? "Mains" : "Tank";

  bool wasDelayed = rainActive || windActive || isPausedNow() || !systemMasterEnabled || (rainCooldownUntilEpoch>time(nullptr));
  logEvent(z,"STOPPED",src,wasDelayed);

  if (useGpioFallback) {
    digitalWrite(zonePins[z], LOW);
    if (zonesCount==4) { digitalWrite(mainsPin,LOW); digitalWrite(tankPin,LOW); }
  } else {
    pcfOut.digitalWrite(PCH[z], HIGH);
    if (zonesCount==4) { pcfOut.digitalWrite(mainsChannel,HIGH); pcfOut.digitalWrite(tankChannel,HIGH); }
  }

  zoneActive[z] = false;

  display.clearDisplay();
  display.setTextSize(2); display.setCursor(4,0); display.print(zoneNames[z]); display.print(" OFF");
  display.display(); delay(300);
}

void turnOnValveManual(int z) {
  if (z>=zonesCount) return;
  if (zoneActive[z]) return;

  for (int i=0;i<(int)zonesCount;i++){
    if (zoneActive[i]) {
      if (!runZonesConcurrent) {
        Serial.println("[VALVE] Manual start blocked: another zone is running");
        return;
      }
      break; // concurrent: allow additional zone
    }
  }
  if (isBlockedNow() || rainActive || windActive) { pendingStart[z]=true; return; }

  zoneStartMs[z] = millis();
  zoneActive[z] = true;
  const char* src="None";

  if (useGpioFallback) {
    digitalWrite(zonePins[z], HIGH);
    if (zonesCount==4) {
      if (justUseTank){ src="Tank";  digitalWrite(mainsPin,LOW);  digitalWrite(tankPin,HIGH); }
      else if (justUseMains){ src="Mains"; digitalWrite(mainsPin,HIGH); digitalWrite(tankPin,LOW); }
      else if (isTankLow()) { src="Mains"; digitalWrite(mainsPin,HIGH); digitalWrite(tankPin,LOW); }
      else { src="Tank";  digitalWrite(mainsPin,LOW);  digitalWrite(tankPin,HIGH); }
    }
  } else {
    pcfOut.digitalWrite(PCH[z], LOW);
    if (zonesCount==4) {
      if (justUseTank){ src="Tank";  pcfOut.digitalWrite(mainsChannel,HIGH); pcfOut.digitalWrite(tankChannel,LOW); }
      else if (justUseMains){ src="Mains"; pcfOut.digitalWrite(mainsChannel,LOW);  pcfOut.digitalWrite(tankChannel,HIGH); }
      else if (isTankLow()) { src="Mains"; pcfOut.digitalWrite(mainsChannel,LOW);  pcfOut.digitalWrite(tankChannel,HIGH); }
      else { src="Tank";  pcfOut.digitalWrite(mainsChannel,HIGH); pcfOut.digitalWrite(tankChannel,LOW); }
    }
  }
  logEvent(z,"MANUAL START",src,false);
}

void turnOffValveManual(int z) {
  if (z>=MAX_ZONES) return;
  if (!zoneActive[z]) return;

  if (useGpioFallback)  digitalWrite(zonePins[z], LOW);
  else                  pcfOut.digitalWrite(PCH[z], HIGH);

  zoneActive[z] = false;

  bool anyStillOn=false;
  for (int i=0;i<(int)zonesCount;i++) if (zoneActive[i]) { anyStillOn=true; break; }

  if (!anyStillOn && zonesCount==4) {
    if (useGpioFallback){ digitalWrite(mainsPin,LOW); digitalWrite(tankPin,LOW); }
    else { pcfOut.digitalWrite(mainsChannel,HIGH); pcfOut.digitalWrite(tankChannel,HIGH); }
  }
}

// ---------- Next Water (queue-first) ----------
static NextWaterInfo computeNextWatering() {
  NextWaterInfo best{0, -1, 0};

  for (int z=0; z<(int)zonesCount; ++z) {
    if (pendingStart[z]) {
      best.epoch = time(nullptr);
      best.zone  = z;
      best.durSec = (uint32_t)durationMin[z]*60u + (uint32_t)durationSec[z];
      return best;
    }
  }

  time_t now = time(nullptr);
  struct tm base; localtime_r(&now, &base);

  auto consider = [&](int z, int hr, int mn, bool enabled) {
    if (!enabled) return;
    if (durationMin[z] == 0 && durationSec[z] == 0) return;

    struct tm cand = base;
    cand.tm_sec  = 0;
    cand.tm_hour = hr;
    cand.tm_min  = mn;

    time_t t = mktime(&cand);
    if (t <= now) t += 24*60*60;

    for (int k = 0; k < 8; ++k) {
      struct tm tmp; localtime_r(&t, &tmp);
      int wd = tmp.tm_wday;
      if (days[z][wd]) {
        if (best.zone < 0 || t < best.epoch) {
          best.epoch = t;
          best.zone  = z;
          best.durSec = (uint32_t)durationMin[z]*60u + (uint32_t)durationSec[z];
        }
        return;
      }
      t += 24*60*60;
    }
  };

  for (int z=0; z<(int)zonesCount; ++z) {
    consider(z, startHour[z],  startMin[z],  true);
    consider(z, startHour2[z], startMin2[z], enableStartTime2[z]);
  }
  return best;
}


 // ---------- Main Page ----------
void handleRoot() {  
  // --- Precompute state / snapshots ---
  checkWindRain();

  time_t now = time(nullptr);
  struct tm* ti = localtime(&now);
  char timeStr[9], dateStr[11];
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", ti);
  strftime(dateStr, sizeof(dateStr), "%d/%m/%Y", ti);

  updateCachedWeather();
  DynamicJsonDocument js(1024);
  DeserializationError werr = deserializeJson(js, cachedWeatherData);

  // Safe reads
  float temp = NAN, hum = NAN, ws = NAN;
  if (!werr) {
    if (js["main"]["temp"].is<float>())     temp = js["main"]["temp"].as<float>();
    if (js["main"]["humidity"].is<float>()) hum  = js["main"]["humidity"].as<float>();
    if (js["wind"]["speed"].is<float>())    ws   = js["wind"]["speed"].as<float>();
  }

  String cond = werr ? "-" : String(js["weather"][0]["main"].as<const char*>());
  if (cond == "") cond = "-";
  String cityName = werr ? "-" : String(js["name"].as<const char*>());
  if (cityName == "") cityName = "-";

  const int tankPct = tankPercent();
  const String causeText = rainDelayCauseText();

  // --- HTML ---
  String html; html.reserve(34000);
  html += F("<!doctype html><html lang='en' data-theme='light'><head>");
  html += F("<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>ESP32 Irrigation</title>");
  html += F("<style>");
  html += F(".center{max-width:1120px;margin:0 auto}");
  html += F(":root[data-theme='light']{--bg:#ecf1f8;--bg2:#f6f8fc;--glass:rgba(255,255,255,.55);--glass-brd:rgba(140,158,190,.35);");
  html += F("--card:#ffffff;--text:#0f172a;--muted:#667084;--primary:#1c74d9;--primary-2:#1160b6;--ok:#16a34a;--warn:#d97706;--bad:#dc2626;");
  html += F("--chip:#eef4ff;--chip-brd:#cfe1ff;--ring:#dfe8fb;--ring2:#a4c6ff;--shadow:0 18px 40px rgba(19,33,68,.15)}");
  html += F(":root[data-theme='dark']{--bg:#0a0f18;--bg2:#0e1624;--glass:rgba(16,26,39,.6);--glass-brd:rgba(96,120,155,.28);");
  html += F("--card:#101826;--text:#e8eef6;--muted:#9fb0ca;--primary:#52a7ff;--primary-2:#2f7fe0;--ok:#22c55e;--warn:#f59e0b;--bad:#ef4444;");
  html += F("--chip:#0f2037;--chip-brd:#223a5e;--ring:#172a46;--ring2:#2c4f87;--shadow:0 18px 40px rgba(0,0,0,.45)}");
  html += F("*{box-sizing:border-box}html,body{margin:0;background:radial-gradient(1200px 600px at 10% -5%,var(--bg2),transparent),");
  html += F("radial-gradient(1200px 700px at 100% 0%,var(--ring),transparent),radial-gradient(900px 500px at -10% 80%,var(--ring2),transparent),var(--bg);");
  html += F("color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif}");

  // Top nav
  html += F(".nav{position:sticky;top:0;z-index:10;padding:10px 12px 14px;background:linear-gradient(180deg,rgba(0,0,0,.25),transparent),var(--primary-2);box-shadow:0 16px 36px rgba(0,0,0,.25)}");
  html += F(".nav .in{max-width:1120px;margin:0 auto;display:flex;align-items:center;justify-content:space-between;gap:12px;color:#fff}");
  html += F(".brand{display:flex;align-items:center;gap:10px;font-weight:800;letter-spacing:.2px}.dot{width:10px;height:10px;border-radius:999px;background:#84ffb5;box-shadow:0 0 14px #84ffb5}");
  html += F(".nav .meta{display:flex;gap:8px;flex-wrap:wrap;align-items:center;font-weight:650}.pill{background:rgba(255,255,255,.16);border:1px solid rgba(255,255,255,.28);border-radius:999px;padding:6px 10px}");
  html += F(".btn-ghost{background:rgba(255,255,255,.14);border:1px solid rgba(255,255,255,.35);color:#fff;border-radius:10px;padding:8px 12px;font-weight:700;cursor:pointer}");

  // Cards and grids
  html += F(".wrap{max-width:1120px;margin:16px auto;padding:0 12px}");
  html += F(".glass{background:var(--glass);backdrop-filter:blur(12px);-webkit-backdrop-filter:blur(12px);border:1px solid var(--glass-brd);border-radius:16px;box-shadow:var(--shadow)}");
  html += F(".section{padding:14px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:14px}");
  html += F(".card{background:var(--card);border:1px solid var(--glass-brd);border-radius:16px;box-shadow:var(--shadow);padding:14px}");
  html += F(".card h3{margin:0 0 8px 0;font-size:1rem;color:var(--muted)}");
  html += F(".chip{display:inline-flex;align-items:center;gap:6px;background:var(--chip);border:1px solid var(--chip-brd);border-radius:999px;padding:6px 10px;font-weight:650;white-space:nowrap}");
  html += F(".big{font-weight:800;font-size:1.22rem}.hint{opacity:.7;font-size:.9em;margin-top:6px}.sub{opacity:.8}");
  html += F(".meter{position:relative;height:14px;border-radius:999px;background:linear-gradient(180deg,rgba(0,0,0,.12),transparent);border:1px solid var(--glass-brd);overflow:hidden}");
  html += F(".fill{position:absolute;inset:0 0 0 0;width:0%;height:100%;background:linear-gradient(90deg,#30d1ff,#4da3ff,#1c74d9);box-shadow:0 0 30px rgba(77,163,255,.35) inset;transition:width .45s ease}");
  html += F(".badge{display:inline-flex;align-items:center;gap:8px;padding:6px 10px;border-radius:999px;border:1px solid var(--glass-brd)}");
  html += F(".b-ok{background:rgba(34,197,94,.12);border-color:rgba(34,197,94,.35)}.b-warn{background:rgba(245,158,11,.12);border-color:rgba(245,158,11,.35)}.b-bad{background:rgba(239,68,68,.12);border-color:rgba(239,68,68,.38)}");
  html += F(".toolbar{display:flex;gap:10px;flex-wrap:wrap;margin:10px 0 0}.btn{background:var(--primary);color:#fff;border:none;border-radius:12px;padding:10px 14px;font-weight:800;cursor:pointer;box-shadow:0 10px 24px rgba(0,0,0,.25)}");
  html += F(".btn:disabled{background:#7f8aa1;cursor:not-allowed;box-shadow:none}.btn-danger{background:var(--bad)}");
  html += F(".zones{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,360px));gap:14px;justify-content:center;justify-items:stretch}");

  // Centered two-up schedule grid
  html += F(".sched-ctr{--schedW:360px;--gap:14px;max-width:calc(2*var(--schedW) + var(--gap));margin:16px auto}");
  html += F(".sched-grid{display:grid;grid-template-columns:repeat(2,minmax(var(--schedW),1fr));gap:var(--gap);justify-items:stretch;align-items:stretch}");
  html += F(".sched-card{display:flex;flex-direction:column}");
  html += F(".sched-card .row{display:flex;gap:8px;align-items:center;margin:6px 0}");
  html += F(".sched-card label{min-width:86px}");
  html += F(".sched-card .in{border:1px solid #d0d7e3;border-radius:10px;padding:8px 10px}");
  html += F(".day{display:inline-flex;gap:6px;border:1px solid #d0d7e3;border-radius:999px;padding:6px 10px}");
  html += F("@media (max-width: 820px){.sched-ctr{--schedW:min(320px,100%);max-width:var(--schedW)}.sched-grid{grid-template-columns:1fr}}");

  html += F("</style></head><body>");

  // --- Nav ---
  html += F("<div class='nav'><div class='in'>");
  html += F("<div class='brand'><span class='dot'></span>ESP32 Irrigation</div>");
  html += F("<div class='meta'>");
  html += F("<span class='pill' id='clock'>"); html += timeStr; html += F("</span>");
  html += F("<span class='pill'>"); html += dateStr; html += F("</span>");
  html += F("<span class='pill'>"); html += ((ti && ti->tm_isdst>0) ? "DST" : "STD"); html += F("</span>");
  html += F("<span class='pill'>espirrigation.local</span>");
  html += F("<button id='btn-master' class='pill' style='cursor:pointer' aria-pressed='");
  html += (systemMasterEnabled ? "true" : "false");
  html += F("' title='Toggle master enable/disable'>Master: <b id='master-state'>");
  html += (systemMasterEnabled ? "On" : "Off");
  html += F("</b></button>");
  html += F("<button id='themeBtn' class='btn-ghost' title='Toggle theme'>Theme</button>");
  html += F("</div></div></div>");

  // --- Summary cards ---
  html += F("<div class='wrap'><div class='glass section'><div class='grid'>");

  html += F("<div class='card'><h3>Location</h3><div class='chip'>🏙️ <b id='cityName'>");
  html += cityName; html += F("</b></div></div>");

  html += F("<div class='card'><h3>Uptime</h3><div id='upChip' class='big'>--:--:--</div><div class='hint'>Since last boot</div></div>");

  html += F("<div class='card'><h3>Signal Strength</h3><div id='rssiChip' class='big'>");
  html += String(WiFi.RSSI()); html += F(" dBm</div><div class='hint'>Wi-Fi RSSI</div></div>");

  html += F("<div class='card'><h3>Tank Level</h3><div class='zone-head'>");
  html += F("<div class='big'><span id='tankPctLabel'>"); html += String(tankPct); html += F("%</span></div>");
  html += F("<div id='srcChip' class='sub'>"); html += sourceModeText(); html += F("</div></div>");
  html += F("<div class='meter'><div id='tankFill' class='fill' style='width:"); html += String(tankPct); html += F("%'></div></div></div>");
  
  html += F("<div class='card'><h3>Weather</h3>");
  html += F("<div class='grid' style='grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:25px'>");
  html += F("<div class='chip'>🌡️ "); html += (isnan(temp) ? String("--") : String(temp,1)+" ℃"); html += F("</div>");
  html += F("<div class='chip'>💧 ");  html += (isnan(hum)  ? String("--") : String((int)hum)+" %"); html += F("</div>");
  html += F("<div class='chip'>🌬️ "); html += (isnan(ws)   ? String("--") : String(ws,1)+" m/s"); html += F("</div>");
  html += F("<div class='chip'>☁️ <span class='sub'>Cond</span>&nbsp;<b id='cond'>");
  html += cond.length() ? cond : String("--");
  html += F("</b></div></div></div>");

  // Delays card
  html += F("<div class='card'><h3>Delays</h3><div class='grid' style='grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:10px'>");
  html += F("<div id='rainBadge' class='badge "); html += (rainActive ? "b-bad" : "b-ok"); html += F("'>🌧️ Rain: <b>"); html += (rainActive?"Active":"Off"); html += F("</b></div>");
  html += F("<div id='windBadge' class='badge "); html += (windActive ? "b-warn" : "b-ok"); html += F("'>💨 Wind: <b>"); html += (windActive?"Active":"Off"); html += F("</b></div>");
  html += F("<div class='badge'>Cause: <b id='rainCauseBadge'>"); html += causeText; html += F("</b></div>");
  html += F("<div class='badge'>1h (now): <b id='acc1h'>--</b> mm</div>");
  html += F("<div class='badge'>24h (actual): <b id='acc24'>--</b> mm</div>");
  html += F("</div></div>");

  html += F("<div class='card'><h3>Weather Stats</h3>");
  html += F("<div class='grid' style='grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px'>");
  html += F("<div class='chip'><b>Low</b>&nbsp;<b id='tmin'>---</b> ℃</div>");
  html += F("<div class='chip'><b>High</b>&nbsp;<b id='tmax'>---</b> ℃</div>");
  html += F("<div class='chip'><b>Pressure</b>&nbsp;<b id='press'>--</b> hPa</div>");
  html += F("<div class='chip'>🌅 <span class='sub'>Sunrise</span>&nbsp;<b id='sunr'>--:--</b></div>");
  html += F("<div class='chip'>🌇 <span class='sub'>Sunset</span>&nbsp;<b id='suns'>--:--</b></div>");
  html += F("</div></div>");

  html += F("<div class='card'><h3>Next Water</h3>");
  html += F("<div class='grid' style='grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:10px'>");
  html += F("<div class='chip'>🧭 <b id='nwZone'>--</b></div>");
  html += F("<div class='chip'>🕒 <b id='nwTime'>--:--</b></div>");
  html += F("<div class='chip'>⏳ In <b id='nwETA'>--</b></div>");
  html += F("<div class='chip'>🧮 Dur <b id='nwDur'>--</b></div>");
  html += F("</div><div class='hint'>Queued starts take priority; otherwise shows schedule.</div></div>");

  html += F("</div></div>"); // end glass / grid

  // --- Live Zones ---
  html += F("<div class='center' style='margin-top:16px'><div class='card'>");
  html += F("<h3>Zones</h3><div class='zones'>");
  for (int z=0; z<(int)zonesCount; z++) {
    unsigned long rem=0, total=(unsigned long)durationMin[z]*60 + durationSec[z];
    if (zoneActive[z]) {
      unsigned long elapsed=(millis()-zoneStartMs[z])/1000;
      rem=(elapsed<total?total-elapsed:0);
    }
    int pctDone = (zoneActive[z] && total>0) ? (int)((100UL*(total-rem))/total) : 0;

    html += F("<div class='card'><div class='zone-head'>");
    html += F("<div class='big'>"); html += zoneNames[z]; html += F("</div>");
    html += F("<div id='zone-"); html += String(z); html += F("-state' class='badge ");
    html += (zoneActive[z] ? "b-ok" : ""); html += F("'>");
    html += (zoneActive[z] ? "▶︎ Running" : "⏹ Off"); html += F("</div></div>");

    html += F("<div class='sub' style='margin:6px 0'><span id='zone-"); html += String(z); html += F("-rem'>");
    if (zoneActive[z]) { int rm=rem/60, rs=rem%60; html += String(rm)+"m "+(rs<10?"0":"")+String(rs)+"s left"; }
    else html += F("—");
    html += F("</span></div>");

    html += F("<div class='meter' title='Progress'><div id='zone-"); html += String(z); html += F("-bar' class='fill' style='width:");
    html += String(pctDone); html += F("%'></div></div>");

    html += F("<div class='toolbar' style='margin-top:10px'>");
    if (zoneActive[z]) {
      html += F("<button type='button' class='btn' disabled>▶️ On</button>");
      html += F("<button type='button' class='btn btn-danger' onclick='toggleZone("); html += String(z); html += F(",0)'>⏹ Off</button>");
    } else {
      html += F("<button type='button' class='btn' onclick='toggleZone("); html += String(z); html += F(",1)'>▶️ On</button>");
      html += F("<button type='button' class='btn btn-danger' disabled>⏹ Off</button>");
    }
    html += F("</div></div>");
  }
  html += F("</div></div></div>"); // Close zones block

  // --- Centered per-zone schedules (two-up grid) ---
  static const char* DLBL[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  html += F("<div class='sched-ctr'><div class='sched-grid'>");
  for (int z=0; z<(int)zonesCount; ++z) {
    html += F("<div class='card sched-card'>");
    html += F("<h3>Schedule — "); html += zoneNames[z]; html += F("</h3>");
    html += F("<form method='POST' action='/submit'>");
    html += F("<input type='hidden' name='onlyZone' value='"); html += String(z); html += F("'>");

    // Name
    html += F("<div class='row'><label>Name</label>");
    html += F("<input class='in' type='text' name='zoneName"); html += String(z);
    html += F("' value='"); html += zoneNames[z]; html += F("' maxlength='32' style='flex:1;min-width:160px'></div>");

    // Start 1
    html += F("<div class='row'><label>Start 1</label>");
    html += F("<input class='in' type='number' min='0' max='23' name='startHour"); html += String(z);
    html += F("' value='"); html += String(startHour[z]); html += F("' style='width:70px'> : ");
    html += F("<input class='in' type='number' min='0' max='59' name='startMin"); html += String(z);
    html += F("' value='"); html += String(startMin[z]); html += F("' style='width:70px'></div>");

    // Start 2
    html += F("<div class='row'><label>Start 2</label>");
    html += F("<input class='in' type='number' min='0' max='23' name='startHour2"); html += String(z);
    html += F("' value='"); html += String(startHour2[z]); html += F("' style='width:70px'> : ");
    html += F("<input class='in' type='number' min='0' max='59' name='startMin2"); html += String(z);
    html += F("' value='"); html += String(startMin2[z]); html += F("' style='width:70px'>");
    html += F("<label style='margin-left:8px'><input type='checkbox' name='enableStartTime2"); html += String(z);
    html += F("' "); html += (enableStartTime2[z] ? "checked" : ""); html += F("> Enable</label></div>");

    // Duration
    html += F("<div class='row'><label>Duration</label>");
    html += F("<input class='in' type='number' min='0' max='600' name='durationMin"); html += String(z);
    html += F("' value='"); html += String(durationMin[z]); html += F("' style='width:60px'> m : ");
    html += F("<input class='in' type='number' min='0' max='59' name='durationSec"); html += String(z);
    html += F("' value='"); html += String(durationSec[z]); html += F("' style='width:60px'> s</div>");

    // Days
    html += F("<div class='row' style='flex-wrap:wrap'><label>Days</label><div class='days' style='display:flex;gap:6px;flex-wrap:wrap'>");
    for (int d=0; d<7; ++d) {
      html += F("<label class='day'><input type='checkbox' name='day"); html += String(z); html += F("_"); html += String(d);
      html += F("' "); html += (days[z][d] ? "checked" : ""); html += F("> "); html += DLBL[d]; html += F("</label>");
    }
    html += F("</div></div>");

    // Actions
    html += F("<div class='toolbar' style='margin-top:10px'><button class='btn' type='submit'>Save Zone</button></div>");

    html += F("</form></div>");
  }
  html += F("</div></div>"); // end schedules

  // --- Tools / System Controls ---
  html += F("<div class='grid center' style='margin:16px auto 24px'><div class='card' style='grid-column:1/-1'>");
  html += F("<h3>Tools</h3><div class='toolbar'>");
  html += F("<button class='btn' id='btn-save-all' title='Save all zone schedules on this page'>Save All</button>");
  html += F("<a class='btn' href='/setup'>Setup</a>");
  html += F("<a class='btn' href='/events'>Events</a>");
  html += F("<a class='btn' href='/tank'>Calibrate Tank</a>");
  html += F("<a class='btn' href='/whereami' role='button' class='btn'>Connection Stats</a>");
  html += F("</div></div></div>");

  html += F("<div class='grid center' style='margin:16px auto 24px'><div class='card' style='grid-column:1/-1'>");
  html += F("<h3>System Controls</h3><div class='toolbar'>");
  html += F("<button class='btn' id='btn-pause-24' title='Pause all starts for 24 hours'>Pause 24h</button>");
  html += F("<button class='btn' id='btn-pause-7d' title='Pause 7d'>Pause 7d</button>");
  html += F("<button class='btn' id='btn-resume' title='Resume now'>Resume</button>");
  html += F("<button class='btn' id='btn-clear-delays' title='Clear queued starts and delays'>Clear Delays</button>");
  html += F("<button class='btn' id='toggle-backlight-btn' title='Invert OLED (quick night toggle)'>LCD Toggle</button>");
  html += F("<button class='btn btn-danger' id='rebootBtn'>Reboot</button>");
  html += F("</div></div></div>");

  // --- JS ---
  html += F("<script>");
  html += F("function pad(n){return n<10?'0'+n:n;}");
  html += F("let _devEpoch=null; let _tickTimer=null;");
  html += F("function startDeviceClock(seedSec){_devEpoch=seedSec; if(_tickTimer)clearInterval(_tickTimer);");
  html += F("const draw=()=>{if(_devEpoch==null)return; const d=new Date(_devEpoch*1000);");
  html += F("const h=pad(d.getHours()),m=pad(d.getMinutes()),s=pad(d.getSeconds()); const el=document.getElementById('clock'); if(el) el.textContent=h+':'+m+':'+s; _devEpoch++;};");
  html += F("draw(); _tickTimer=setInterval(draw,1000);} ");

  html += F("function fmtUptime(sec){const h=Math.floor(sec/3600),m=Math.floor((sec%3600)/60),s=sec%60;return pad(h)+':'+pad(m)+':'+pad(s);} ");

  html += F("let _busy=false; async function postJson(url,payload){const body=payload?JSON.stringify(payload):\"{}\";");
  html += F("return fetch(url,{method:'POST',headers:{'Content-Type':'application/json','Cache-Control':'no-cache'},body});}");
  html += F("async function postForm(url, body){const opts={method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'}}; if(body)opts.body=body; return fetch(url,opts);} ");
  html += F("async function toggleZone(z,on){if(_busy)return;_busy=true;try{await postJson('/valve/'+(on?'on/':'off/')+z,{t:Date.now()});setTimeout(()=>location.reload(),150);}catch(e){console.error(e);}finally{_busy=false;}}");

  html += F("const btnBack=document.getElementById('toggle-backlight-btn'); if(btnBack){btnBack.addEventListener('click',async()=>{if(_busy)return;_busy=true;try{await postJson('/toggleBacklight',{t:Date.now()});}catch(e){}finally{_busy=false;}});} ");
  html += F("const btnReboot=document.getElementById('rebootBtn'); if(btnReboot){btnReboot.addEventListener('click',async()=>{if(confirm('Reboot controller now?')){try{await postJson('/reboot',{t:Date.now()});}catch(e){}}});} ");
  html += F("const bClr=document.getElementById('btn-clear-delays'); if(bClr){bClr.addEventListener('click',async()=>{await postForm('/clear_delays','a=1'); setTimeout(()=>location.reload(),150);});}");
  html += F("const bP24=document.getElementById('btn-pause-24'); if(bP24){bP24.addEventListener('click',async()=>{await postForm('/pause','sec=86400'); setTimeout(()=>location.reload(),200);});}");
  html += F("const bP7=document.getElementById('btn-pause-7d'); if(bP7){bP7.addEventListener('click',async()=>{await postForm('/pause','sec='+(7*86400)); setTimeout(()=>location.reload(),200);});}");
  html += F("const bRes=document.getElementById('btn-resume'); if(bRes){bRes.addEventListener('click',async()=>{await postForm('/resume','x=1'); setTimeout(()=>location.reload(),150);});}");

  // Master pill → POST /master and optimistic flip
  html += F("const btnMaster=document.getElementById('btn-master');");
  html += F("if(btnMaster){btnMaster.addEventListener('click',async()=>{");
  html += F("  const cur=(btnMaster.getAttribute('aria-pressed')==='true'); const turnOn=!cur;");
  html += F("  try{await fetch('/master',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:turnOn?'on=1':''});");
  html += F("      btnMaster.setAttribute('aria-pressed',turnOn?'true':'false'); const st=document.getElementById('master-state'); if(st) st.textContent=turnOn?'On':'Off';");
  html += F("  }catch(e){console.error(e);} ");
  html += F("});}");

  html += F("function applyTheme(t){document.documentElement.setAttribute('data-theme',t==='dark'?'dark':'light');}");
  html += F("(function(){let saved=localStorage.getItem('theme');if(saved!=='light'&&saved!=='dark'){saved=(window.matchMedia&&window.matchMedia('(prefers-color-scheme: dark)').matches)?'dark':'light';localStorage.setItem('theme',saved);}applyTheme(saved);})();");
  html += F("const themeBtn=document.getElementById('themeBtn'); if(themeBtn){themeBtn.addEventListener('click',()=>{const cur=(document.documentElement.getAttribute('data-theme')==='dark')?'dark':'light';const nxt=(cur==='dark')?'light':'dark';applyTheme(nxt);localStorage.setItem('theme',nxt);});}");

  html += F("function toLocalHHMM(epoch){if(!epoch||epoch===0)return'--:--'; const d=new Date(epoch*1000); return pad(d.getHours())+':'+pad(d.getMinutes());}");
  html += F("async function refreshStatus(){try{const r=await fetch('/status');const st=await r.json();");
  html += F("if(typeof st.deviceEpoch==='number' && st.deviceEpoch>0 && _devEpoch===null){ startDeviceClock(st.deviceEpoch); }");
  html += F("const rb=document.getElementById('rainBadge');const wb=document.getElementById('windBadge');");
  html += F("if(rb){rb.className='badge '+(st.rainDelayActive?'b-bad':'b-ok');rb.innerHTML='🌧️ Rain: <b>'+(st.rainDelayActive?'Active':'Off')+'</b>';}");
  html += F("if(wb){wb.className='badge '+(st.windDelayActive?'b-warn':'b-ok');wb.innerHTML='💨 Wind: <b>'+(st.windDelayActive?'Active':'Off')+'</b>';}");
  html += F("const cause=document.getElementById('rainCauseBadge'); if(cause) cause.textContent=st.rainDelayCause||'Off';");
  html += F("const pct=st.tankPct||0; const tf=document.getElementById('tankFill'); const tl=document.getElementById('tankPctLabel');");
  html += F("if(tf) tf.style.width=Math.max(0,Math.min(100,pct))+'%'; if(tl) tl.textContent=pct+'%';");
  html += F("const src=document.getElementById('srcChip'); if(src) src.textContent=st.sourceMode||'';");
  html += F("const up=document.getElementById('upChip'); if(up) up.textContent=fmtUptime(st.uptimeSec||0);");
  html += F("const rssi=document.getElementById('rssiChip'); if(rssi) rssi.textContent=(st.rssi)+' dBm';");

  // NEW: 1h (now) + 24h (actual)
  html += F("const acc1h=document.getElementById('acc1h'); if(acc1h){ let v = (typeof st.rain1hNow==='number')?st.rain1hNow:NaN; if((isNaN(v)||v===0)&&typeof st.rain3hNow==='number'&&st.rain3hNow>0){ v = st.rain3hNow/3.0; } acc1h.textContent=isNaN(v)?'--':v.toFixed(1);}");
  html += F("const acc24=document.getElementById('acc24'); if(acc24){ const v=(typeof st.rain24hActual==='number')?st.rain24hActual:(typeof st.rain24h==='number'?st.rain24h:NaN); acc24.textContent=isNaN(v)?'--':v.toFixed(1);} ");

  html += F("if(Array.isArray(st.zones)){ st.zones.forEach((z,idx)=>{");
  html += F("const stateEl=document.getElementById('zone-'+idx+'-state'); const remEl=document.getElementById('zone-'+idx+'-rem'); const barEl=document.getElementById('zone-'+idx+'-bar');");
  html += F("if(stateEl){stateEl.className='badge '+(z.active?'b-ok':'');stateEl.innerHTML=z.active?'▶︎ Running':'⏹ Off';}");
  html += F("if(remEl){ if(z.active){ const r=z.remaining||0; const rm=Math.floor(r/60),rs=r%60; remEl.textContent=rm+'m '+(rs<10?'0':'')+rs+'s left'; } else remEl.textContent='—'; }");
  html += F("if(barEl){ let p=0; const total=z.totalSec||0; const rem=z.remaining||0; p=total>0?Math.max(0,Math.min(100,Math.round(100*(total-rem)/total))):0; barEl.style.width=p+'%'; }");
  html += F("}); }");

  // Weather stats
  html += F("const tmin=document.getElementById('tmin'); const tmax=document.getElementById('tmax'); const sunr=document.getElementById('sunr'); const suns=document.getElementById('suns'); const press=document.getElementById('press');");
  html += F("if(tmin) tmin.textContent=(st.tmin??0).toFixed(0);");
  html += F("if(tmax) tmax.textContent=(st.tmax??0).toFixed(0);");
  html += F("if(sunr) sunr.textContent = st.sunriseLocal || '--:--';");
  html += F("if(suns) suns.textContent = st.sunsetLocal  || '--:--';");
  html += F("if(press) press.textContent = (typeof st.pressure==='number' && st.pressure>0) ? st.pressure : '--';");

  // Master toggle: keep UI in sync with API state (if provided)
  html += F("const bm=document.getElementById('btn-master'); const ms=document.getElementById('master-state');");
  html += F("if(bm && ms && typeof st.systemMasterEnabled==='boolean'){ bm.setAttribute('aria-pressed', st.systemMasterEnabled?'true':'false'); ms.textContent = st.systemMasterEnabled?'On':'Off'; }");

  // Run mode label
  html += F("const rm=document.getElementById('runModeLabel'); if(rm && typeof st.runConcurrent==='boolean'){ rm.textContent = st.runConcurrent ? 'Concurrent' : 'Sequential'; }");

  // Next Water
  html += F("(function(){ const zEl=document.getElementById('nwZone'); const tEl=document.getElementById('nwTime'); const eEl=document.getElementById('nwETA'); const dEl=document.getElementById('nwDur');");
  html += F("const epoch=st.nextWaterEpoch|0; const zone=st.nextWaterZone; const name=st.nextWaterName || (Number.isInteger(zone)?('Z'+(zone+1)):'--'); const dur=st.nextWaterDurSec|0;");
  html += F("function fmtDur(s){ if(s<=0) return '--'; const h=Math.floor(s/3600), m=Math.floor((s%3600)/60), sec=s%60; return (h? (h+'h '):'') + (m? (m+'m '):'') + (h? '' : (sec+'s')); }");
  html += F("function fmtETA(delta){ if(delta<=0) return 'now'; const d=Math.floor(delta/86400); delta%=86400; const h=Math.floor(delta/3600); delta%=3600; const m=Math.floor(delta/60); if(d>0) return d+'d '+pad(h)+':'+pad(m); if(h>0) return h+'h '+m+'m'; return m+'m'; }");
  html += F("if(zEl) zEl.textContent = (zone>=0 && zone<255) ? (name) : '--'; if(tEl) tEl.textContent = toLocalHHMM(epoch); if(dEl) dEl.textContent = fmtDur(dur);");
  html += F("let nowEpoch = (typeof st.deviceEpoch==='number' && st.deviceEpoch>0 && _devEpoch!=null) ? _devEpoch : Math.floor(Date.now()/1000);");
  html += F("if(eEl) eEl.textContent = epoch ? fmtETA(epoch - nowEpoch) : '--'; })();");

  html += F("}catch(e){} } setInterval(refreshStatus,1200); refreshStatus();");

  // expose zones count to JS
  html += F("const ZC="); html += String(zonesCount); html += F(";");

  // Save All (collect all zone forms and POST once to /submit)
  html += F("async function saveAll(){");
  html += F("  const fd=new URLSearchParams();");
  html += F("  for(let z=0; z<ZC; z++){");
  html += F("    const get = n=>document.querySelector(`[name='${n}']`);");
  html += F("    const name = get('zoneName'+z); if(name) fd.append('zoneName'+z, name.value);");
  html += F("    const h1=get('startHour'+z), m1=get('startMin'+z); if(h1) fd.append('startHour'+z,h1.value); if(m1) fd.append('startMin'+z,m1.value);");
  html += F("    const h2=get('startHour2'+z), m2=get('startMin2'+z); if(h2) fd.append('startHour2'+z,h2.value); if(m2) fd.append('startMin2'+z,m2.value);");
  html += F("    const en2=get('enableStartTime2'+z); if(en2 && en2.checked) fd.append('enableStartTime2'+z,'on');");
  html += F("    const dm=get('durationMin'+z), ds=get('durationSec'+z); if(dm) fd.append('durationMin'+z,dm.value); if(ds) fd.append('durationSec'+z,ds.value);");
  html += F("    for(let d=0; d<7; d++){ const cb=get('day'+z+'_'+d); if(cb && cb.checked) fd.append('day'+z+'_'+d,'on'); }");
  html += F("  }");
  html += F("  try{ await fetch('/submit',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:fd.toString()}); location.reload(); }catch(e){ console.error(e); }");
  html += F("} ");
  html += F("document.getElementById('btn-save-all')?.addEventListener('click', saveAll);");

  html += F("</script></body></html>");

  server.send(200, "text/html", html);
}



// ---------- Setup Page ----------
void handleSetupPage() {
  loadConfig();
  String html; html.reserve(20000);

  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>Setup — ESP32 Irrigation</title>");
  html += F("<style>body{margin:0;font-family:Inter,system-ui,Segoe UI,Roboto,Arial,sans-serif;background:#0e1726;color:#e8eef6}");
  html += F(".wrap{max-width:820px;margin:32px auto;padding:0 12px}h1{margin:0 0 16px 0;font-size:1.6em}");
  html += F(".card{background:#111927;border:1px solid #1f2a44;border-radius:14px;box-shadow:0 8px 34px rgba(0,0,0,.35);padding:16px 14px;margin-bottom:14px}");
  html += F("label{display:inline-block;min-width:160px}");
  html += F("input[type=text],input[type=number]{width:100%;max-width:420px;background:#0b1220;color:#e8eef6;border:1px solid #233357;border-radius:10px;padding:8px 10px}");
  html += F(".row{display:flex;align-items:center;gap:10px;margin:10px 0;flex-wrap:wrap}.row small{color:#aab8d0}");
  html += F(".btn{background:#1976d2;color:#fff;border:none;border-radius:10px;padding:10px 14px;font-weight:600;cursor:pointer;box-shadow:0 6px 16px rgba(25,118,210,.25)}");
  html += F(".btn-alt{background:#263244;color:#e8eef6;border:none;border-radius:10px;padding:10px 14px}.grid{display:grid;grid-template-columns:1fr;gap:10px}");
  html += F("@media(min-width:680px){.grid{grid-template-columns:1fr 1fr}}.switchline{display:flex;gap:10px;align-items:center;flex-wrap:wrap}</style></head>");
  html += F("<body><div class='wrap'><h1>⚙️ Setup</h1><form action='/configure' method='POST'>");

  // Weather
  html += F("<div class='card'><h3Current Weather</h3>");
  html += F("<div class='row'><label>API Key</label><input type='text' name='apiKey' value='"); html += apiKey; html += F("'></div>");
  html += F("<div class='row'><label>City ID</label><input type='text' name='city' value='"); html += city; html += F("'><small>OpenWeather city id</small></div>");
  html += F("</div>");

  // Zones
  html += F("<div class='card'><h3>Zones</h3>");
  html += F("<div class='row switchline'><label>Zones</label>");
  html += F("<label><input type='radio' name='zonesMode' value='4' "); html += (zonesCount==4?"checked":""); html += F("> 4 Zone + (Mains/Tank)</label>");
  html += F("<label><input type='radio' name='zonesMode' value='6' "); html += (zonesCount==6?"checked":""); html += F("> 6 Zone </label></div>");

  // NEW: run mode
  html += F("<div class='row switchline'><label>Run Mode</label>");
  html += F("<label><input type='checkbox' name='runConcurrent' ");
  html += (runZonesConcurrent ? "checked" : "");
  html += F("> Run Zones Concurrently</label><small>Unchecked = one-at-a-time</small></div>");

  html += F("</div>");

  // Physical rain sensor
  html += F("<div class='card'><h3>Physical Rain Sensor</h3>");
  html += F("<div class='row switchline'><label>Disable OpenWeatherMap Rain</label><input type='checkbox' name='rainForecastDisabled' ");
  html += (!rainDelayFromForecastEnabled ? "checked" : ""); html += F("><small>Checked = ignore OpenWeather rain.</small></div>");
  html += F("<div class='row switchline'><label>Enable Rain Sensor</label><input type='checkbox' name='rainSensorEnabled' "); html += (rainSensorEnabled?"checked":""); html += F("></div>");
  html += F("<div class='row'><label>GPIO</label><input type='number' min='0' max='39' name='rainSensorPin' value='"); html += String(rainSensorPin); html += F("'><small>Use INPUT_PULLUP (e.g., 27)</small></div>");
  html += F("<div class='row switchline'><label>Invert</label><input type='checkbox' name='rainSensorInvert' "); html += (rainSensorInvert?"checked":""); html += F("><small>Enable if sensor board is NO</small></div>");
  html += F("</div>");

  // Delays & Pause + Cooldown + Threshold (Master deliberately NOT shown here)
  html += F("<div class='card'><h3>Delays & Pause</h3>");

  html += F("<div class='row switchline'><label>Rain Delay Enable</label>"
          "<input type='checkbox' name='rainDelay' ");
  html += (rainDelayEnabled ? "checked" : "");
  html += F("></div>");

  html += F("<div class='row switchline'><label>Wind Delay Enable</label>"
          "<input type='checkbox' name='windCancelEnabled' ");
  html += (windDelayEnabled ? "checked" : "");
  html += F("></div>");

  // Tank/Main row with two checkboxes
  html += F("<div class='row switchline'>"
          "<label style='min-width:160px'>Tank/Main</label>"
          "<div style='display:flex;gap:14px;align-items:center;flex-wrap:wrap'>"
          "<label><input type='checkbox' name='justUseTank' ");
  html += (justUseTank ? "checked" : "");
  html += F("> Only Use Tank</label>"
          "<label><input type='checkbox' name='justUseMains' ");
  html += (justUseMains ? "checked" : "");
  html += F("> Only Use Mains</label>"
          "<small>(4-zone only)</small>"
          "</div></div>");

  html += F("<div class='row switchline'><label>System Pause</label>"
          "<input type='checkbox' name='pauseEnable' ");
  html += (systemPaused ? "checked" : "");
  html += F("><small>Enable System Pause</small></div>");

  time_t nowEp = time(nullptr);
  uint32_t remain = (pauseUntilEpoch > nowEp && systemPaused) ? (pauseUntilEpoch - nowEp) : 0;
  uint32_t remainHours = remain / 3600;

  html += F("<div class='row'><label>Pause for (Hours)</label>"
          "<input type='number' min='0' max='720' name='pauseHours' value='");
  html += String(remainHours);
  html += F("' placeholder='e.g., 24'>"
          "<small>0 = Until Manually Resumed</small></div>");

  html += F("<div class='row'><label>Rain Cooldown (hours)</label>"
          "<input type='number' min='0' max='720' name='rainCooldownHours' value='");
  html += String(rainCooldownMin / 60);
  html += F("'><small>Wait after rain clears before running</small></div>");

  html += F("<div class='row'><label>Rain Threshold 24h (mm)</label>"
          "<input type='number' min='0' max='200' name='rainThreshold24h' value='");
  html += String(rainThreshold24h_mm);
  html += F("'><small>Delay if 24h total ≥ threshold</small></div>");

  html += F("<div class='row' style='gap:10px;flex-wrap:wrap'>"
          "<button class='btn' type='button' id='btn-pause-24'>Pause 24h</button>"
          "<button class='btn' type='button' id='btn-pause-7d'>Pause 7d</button>"
          "<button class='btn' type='button' id='btn-resume'>Resume</button>"
          "<button class='btn' type='button' id='btn-toggle-backlight'>Toggle LCD</button>"
          "<button class='btn' type='button' id='btn-clear-delays'>Clear Delays</button>"
          "</div>");

  html += F("<div class='row' style='gap:10px;margin-top:6px'>"
          "<button class='btn' type='submit'>Save</button>"
          "<button class='btn btn-alt' formaction='/' formmethod='GET'>Home</button>"
          "<button class='btn btn-alt' type='button' onclick=\"fetch('/clear_cooldown',{method:'POST'})\">Clear Cooldown</button>"
          "<button class='btn btn-alt' formaction='/configure' formmethod='POST' name='resumeNow' value='1'>Resume Now</button>"
          "</div>");

  html += F("</div>"); // end card

  // GPIO fallback pins
  html += F("<div class='card'><h3>GPIO Fallback (if I²C relays not found)</h3><div class='grid'>");
  for (uint8_t i=0;i<MAX_ZONES;i++){
    html += F("<div class='row'><label>Zone "); html += String(i+1);
    html += F(" Pin</label><input type='number' min='0' max='39' name='zonePin"); html += String(i);
    html += F("' value='"); html += String(zonePins[i]); html += F("'></div>");
  }
  html += F("<div class='row'><label>Main Pin</label><input type='number' min='0' max='39' name='mainsPin' value='"); html += String(mainsPin); html += F("'><small>4-zone fallback</small></div>");
  html += F("<div class='row'><label>Tank Pin</label><input type='number' min='0' max='39' name='tankPin' value='"); html += String(tankPin); html += F("'><small>4-zone fallback</small></div>");
  html += F("</div></div>");

  // Timezone
  html += F("<div class='card'><h3>Timezone</h3>");
  html += F("<div class='row switchline'><label>Mode</label>");
  html += F("<label><input type='radio' name='tzMode' value='0' "); html += (tzMode==TZ_POSIX?"checked":""); html += F("> POSIX</label>");
  html += F("<label><input type='radio' name='tzMode' value='1' "); html += (tzMode==TZ_IANA?"checked":""); html += F("> IANA</label>");
  html += F("<label><input type='radio' name='tzMode' value='2' "); html += (tzMode==TZ_FIXED?"checked":""); html += F("> Fixed Offset</label></div>");

  html += F("<div class='row'><label>POSIX TZ</label><input type='text' name='tzPosix' value='"); html += tzPosix; html += F("'><small>e.g. ACST-9:30ACDT-10:30,M10.1.0/2,M4.1.0/3</small></div>");
  html += F("<div class='row'><label>IANA Zone</label><input type='text' name='tzIANA' value='"); html += tzIANA; html += F("'><small>e.g. Australia/Adelaide</small></div>");
  html += F("<div class='row'><label>Fixed Offset (min)</label><input type='number' name='tzFixed' value='"); html += String(tzFixedOffsetMin); html += F("'><small>Minutes from UTC</small></div>");
  html += F("</div>");

  // MQTT
  html += F("<div class='card'><h3>MQTT (Home Assistant)</h3>");
  html += F("<div class='row switchline'><label>Enable MQTT</label><input type='checkbox' name='mqttEnabled' "); html += (mqttEnabled ? "checked" : ""); html += F("></div>");
  html += F("<div class='row'><label>Broker Host</label><input type='text' name='mqttBroker' value='"); html += mqttBroker; html += F("'></div>");
  html += F("<div class='row'><label>Port</label><input type='number' name='mqttPort' value='"); html += String(mqttPort); html += F("'></div>");
  html += F("<div class='row'><label>User</label><input type='text' name='mqttUser' value='"); html += mqttUser; html += F("'></div>");
  html += F("<div class='row'><label>Password</label><input type='text' name='mqttPass' value='"); html += mqttPass; html += F("'></div>");
  html += F("<div class='row'><label>Base Topic</label><input type='text' name='mqttBase' value='"); html += mqttBase; html += F("'><small>e.g. espirrigation</small></div>");
  html += F("</div>");

  html += F("</form>");

  // quick-actions
  html += F("<script>");
  html += F("async function post(path, body){try{await fetch(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});}catch(e){console.error(e)}}");
  html += F("const g=id=>document.getElementById(id);");
  html += F("g('btn-toggle-backlight')?.addEventListener('click',()=>post('/toggleBacklight','x=1'));");
  html += F("g('btn-pause-24')?.addEventListener('click',()=>post('/pause','sec=86400'));");
  html += F("g('btn-pause-7d')?.addEventListener('click',()=>post('/pause','sec='+(7*86400)));");
  html += F("g('btn-clear-delays')?.addEventListener('click',()=>post('/clear_delays','x=1'));");
  html += F("g('btn-resume')?.addEventListener('click',()=>post('/resume','x=1'));");
  html += F("</script>");

  html += F("</div></body></html>");

  server.send(200,"text/html",html);
}

// ---------- Schedule POST (per-zone card or full form) ----------
void handleSubmit() {
  // If onlyZone is present -> only update that zone from fields in this form
  if (server.hasArg("onlyZone")) {
    int z = server.arg("onlyZone").toInt();
    if (z >= 0 && z < (int)MAX_ZONES) {
      // Zone name
      if (server.hasArg("zoneName"+String(z))) {
        String nm=cleanName(server.arg("zoneName"+String(z)));
        if (nm.length()) zoneNames[z]=nm;
      }
      // Times / duration
      if (server.hasArg("startHour"+String(z)))  startHour[z]  = server.arg("startHour"+String(z)).toInt();
      if (server.hasArg("startMin"+String(z)))   startMin[z]   = server.arg("startMin"+String(z)).toInt();
      if (server.hasArg("startHour2"+String(z))) startHour2[z] = server.arg("startHour2"+String(z)).toInt();
      if (server.hasArg("startMin2"+String(z)))  startMin2[z]  = server.arg("startMin2"+String(z)).toInt();
      if (server.hasArg("durationMin"+String(z))) durationMin[z]=server.arg("durationMin"+String(z)).toInt();
      if (server.hasArg("durationSec"+String(z))) durationSec[z]=server.arg("durationSec"+String(z)).toInt();
      enableStartTime2[z] = server.hasArg("enableStartTime2"+String(z));
      // Days
      for (int d=0; d<7; d++) days[z][d] = server.hasArg("day"+String(z)+"_"+String(d));

      saveSchedule(); saveConfig(); updateCachedWeather();
      server.sendHeader("Location","/",true); server.send(302,"text/plain","");
      return;
    }
  }

  // Otherwise: legacy full update of all zones (expects all fields present)
  for (int z=0; z<(int)MAX_ZONES; z++) {
    for (int d=0; d<7; d++) days[z][d] = server.hasArg("day"+String(z)+"_"+String(d));
    if (server.hasArg("zoneName"+String(z))) {
      String nm=cleanName(server.arg("zoneName"+String(z)));
      if (nm.length()) zoneNames[z]=nm;
    }
    if (server.hasArg("startHour"+String(z)))  startHour[z]  = server.arg("startHour"+String(z)).toInt();
    if (server.hasArg("startMin"+String(z)))   startMin[z]   = server.arg("startMin"+String(z)).toInt();
    if (server.hasArg("startHour2"+String(z))) startHour2[z] = server.arg("startHour2"+String(z)).toInt();
    if (server.hasArg("startMin2"+String(z)))  startMin2[z]  = server.arg("startMin2"+String(z)).toInt();
    if (server.hasArg("durationMin"+String(z))) durationMin[z]=server.arg("durationMin"+String(z)).toInt();
    if (server.hasArg("durationSec"+String(z))) durationSec[z]=server.arg("durationSec"+String(z)).toInt();
    enableStartTime2[z] = server.hasArg("enableStartTime2"+String(z));
  }
  saveSchedule(); saveConfig(); updateCachedWeather();
  server.sendHeader("Location","/",true); server.send(302,"text/plain","");
}

// ---------- Event Log Page ----------
void handleLogPage() {
  File f = LittleFS.open("/events.csv","r");
  if (!f) { server.send(404,"text/plain","No event log"); return; }

  String html; html.reserve(8000);
  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>Event Log</title>");
  html += F("<style>body{font-family:Inter,system-ui,Segoe UI,Roboto,Arial,sans-serif;background:#0f1522;color:#e8eef6;margin:0}");
  html += F("header{background:#13213a;color:#fff;text-align:center;padding:20px 0 14px;font-size:1.4em}");
  html += F(".wrap{max-width:980px;margin:18px auto;padding:0 12px}");
  html += F("table{width:100%;border-collapse:collapse;background:#0b1220;border:1px solid #22314f;border-radius:12px;overflow:hidden}");
  html += F("th,td{border:1px solid #22314f;padding:8px 6px;font-size:.96em}th{background:#172540}a{color:#a9cbff}</style></head><body>");
  html += F("<header>📜 Irrigation Event Log</header><div class='wrap'>");
  html += F("<div style='margin:10px 0'><a class='btn' href='/' style='padding:8px 12px;background:#1e2d4c;border-radius:10px;text-decoration:none;color:#fff'>Home</a>");
  html += F("&nbsp; <a class='btn' href='/download/events.csv' style='padding:8px 12px;background:#1e2d4c;border-radius:10px;text-decoration:none;color:#fff'>Download CSV</a>");
  html += F("&nbsp; <form style='display:inline' method='POST' action='/clearevents'><button style='padding:8px 12px;background:#b93b3b;border:none;border-radius:10px;color:#fff;cursor:pointer'>Clear</button></form>");
  html += F("&nbsp; <form style='display:inline' method='POST' action='/stopall'><button style='padding:8px 12px;background:#a15517;border:none;border-radius:10px;color:#fff;cursor:pointer'>Stop All</button></form>");
  html += F("</div><table><tr><th>Time</th><th>Zone</th><th>Event</th><th>Source</th><th>RainDelay</th><th>Details</th></tr>");

  while (f.available()) {
    String line=f.readStringUntil('\n'); if (line.length()<5) continue;
    int i1=line.indexOf(','), i2=line.indexOf(',',i1+1), i3=line.indexOf(',',i2+1), i4=line.indexOf(',',i3+1);
    int i5=line.indexOf(',',i4+1), i6=line.indexOf(',',i5+1), i7=line.indexOf(',',i6+1), i8=line.indexOf(',',i7+1), i9=line.indexOf(',',i8+1);
    String ts=line.substring(0,i1), zone=line.substring(i1+1,i2), ev=line.substring(i2+1,i3), src=line.substring(i3+1,i4), rd=line.substring(i4+1,i5);
    String temp=(i6>i5)?line.substring(i5+1,i6):"", hum=(i7>i6)?line.substring(i6+1,i7):"", wind=(i8>i7)?line.substring(i7+1,i8):"";
    String cond=(i9>i8)?line.substring(i8+1,i9):"", city=(i9>=0)?line.substring(i9+1):"";
    String details = (temp.length()?("T="+temp+"°C, H="+hum+"%, W="+wind+"m/s, "+cond+" @ "+city):"n/a");
    html += F("<tr><td>"); html += ts; html += F("</td><td>"); html += zone; html += F("</td><td>"); html += ev; html += F("</td><td>"); html += src; html += F("</td><td>"); html += rd; html += F("</td><td>"); html += details; html += F("</td></tr>");
  }
  f.close();
  html += F("</table></div></body></html>");
  server.send(200,"text/html",html);
}

// ---------- Tank Calibration Page ----------
void handleTankCalibration() {
  int raw=analogRead(TANK_PIN);
  int pct=tankPercent();

  String html; html.reserve(2000);
  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>Tank Calibration</title>");
  html += F("<style>body{font-family:Inter,system-ui,Segoe UI,Roboto,Arial,sans-serif;background:#0f1522;color:#e8eef6;margin:0}");
  html += F(".wrap{max-width:520px;margin:30px auto;padding:0 12px}.card{background:#0b1220;border:1px solid #22314f;border-radius:14px;padding:18px 14px}");
  html += F(".btn{background:#1976d2;color:#fff;border:none;border-radius:10px;padding:10px 14px;font-weight:600;cursor:pointer}.row{display:flex;gap:10px;justify-content:space-between;margin-top:10px}");
  html += F("a{color:#a9cbff}</style></head><body><div class='wrap'><h2>🚰 Tank Calibration</h2><div class='card'>");

  html += F("<p>Raw: <b>"); html += String(raw); html += F("</b></p>");
  html += F("<p>Calibrated range: <b>"); html += String(tankEmptyRaw); html += F("</b> → <b>"); html += String(tankFullRaw); html += F("</b></p>");
  html += F("<p>Level: <b>"); html += String(pct); html += F("%</b></p>");
  html += F("<div class='row'><form method='POST' action='/setTankEmpty'><button class='btn' type='submit'>Set Empty</button></form>");
  html += F("<form method='POST' action='/setTankFull'><button class='btn' type='submit'>Set Full</button></form></div>");
  html += F("<p><a href='/'>Home</a> · <a href='/setup'>Setup</a></p></div></div>");
  html += F("<script>setTimeout(()=>location.reload(),2000);</script></body></html>");

  server.send(200,"text/html",html);
}

// ---------- Config & Schedule ----------
static String _safeReadLine(File& f) {
  if (!f.available()) return String("");
  String s = f.readStringUntil('\n'); s.trim(); return s;
}

void loadConfig() {
  File f = LittleFS.open("/config.txt","r");
  if (!f) return;

  String s;
  if ((s=_safeReadLine(f)).length()) apiKey = s;
  if ((s=_safeReadLine(f)).length()) city   = s;
  if ((s=_safeReadLine(f)).length()) tzOffsetHours = s.toFloat(); // legacy
  if ((s=_safeReadLine(f)).length()) rainDelayEnabled = (s.toInt()==1);
  if ((s=_safeReadLine(f)).length()) windSpeedThreshold = s.toFloat();
  if ((s=_safeReadLine(f)).length()) windDelayEnabled = (s.toInt()==1);
  if ((s=_safeReadLine(f)).length()) justUseTank  = (s.toInt()==1);
  if ((s=_safeReadLine(f)).length()) justUseMains = (s.toInt()==1);
  if ((s=_safeReadLine(f)).length()) tankEmptyRaw = s.toInt();
  if ((s=_safeReadLine(f)).length()) tankFullRaw  = s.toInt();
  if ((s=_safeReadLine(f)).length()) { uint8_t z=s.toInt(); zonesCount=(z==6?6:4); }
  if ((s=_safeReadLine(f)).length()) rainSensorEnabled = (s.toInt()==1);
  if ((s=_safeReadLine(f)).length()) rainSensorPin = s.toInt();
  if ((s=_safeReadLine(f)).length()) rainSensorInvert = (s.toInt()==1);
  if ((s=_safeReadLine(f)).length()) { int th=s.toInt(); if (th>=0 && th<=100) tankLowThresholdPct=th; }

  // GPIO pins
  for (int i=0;i<MAX_ZONES;i++) if ((s=_safeReadLine(f)).length()) zonePins[i]=s.toInt();
  if ((s=_safeReadLine(f)).length()) mainsPin=s.toInt();
  if ((s=_safeReadLine(f)).length()) tankPin =s.toInt();

  // Zone names
  for (int i=0;i<MAX_ZONES;i++) {
    if (!f.available()) break;
    String nm=_safeReadLine(f);
    if (nm.length()) zoneNames[i]=nm;
  }

  // trailing fields (older new ones)
  if (f.available()) { if ((s=_safeReadLine(f)).length()) rainDelayFromForecastEnabled = (s.toInt()==1); }
  if (f.available()) { if ((s=_safeReadLine(f)).length()) systemPaused = (s.toInt()==1); }
  if (f.available()) { if ((s=_safeReadLine(f)).length()) pauseUntilEpoch = (uint32_t)s.toInt(); }

  // timezone
  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) tzMode=(TZMode)sx.toInt(); }
  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) tzPosix=sx; }
  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) tzIANA=sx; }
  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) tzFixedOffsetMin=(int16_t)sx.toInt(); }

  // NEW trailing: Master / cooldown / threshold / MQTT
  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) systemMasterEnabled = (sx.toInt()==1); }
  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) rainCooldownMin = sx.toInt(); }
  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) rainThreshold24h_mm = sx.toInt(); }

  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) mqttEnabled = (sx.toInt()==1); }
  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) mqttBroker = sx; }
  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) mqttPort = (uint16_t)sx.toInt(); }
  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) mqttUser = sx; }
  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) mqttPass = sx; }
  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) mqttBase = sx; }

  // NEW: run mode (boolean) — tolerant trailing read
  if (f.available()) { String sx=_safeReadLine(f); if (sx.length()) runZonesConcurrent = (sx.toInt()==1); }

  f.close();
}

void saveConfig() {
  File f = LittleFS.open("/config.txt","w");
  if (!f) return;

  f.println(apiKey);
  f.println(city);
  f.println(tzOffsetHours,2);           // legacy, preserved
  f.println(rainDelayEnabled? "1":"0");
  f.println(windSpeedThreshold,1);
  f.println(windDelayEnabled? "1":"0");
  f.println(justUseTank?  "1":"0");
  f.println(justUseMains? "1":"0");
  f.println(tankEmptyRaw);
  f.println(tankFullRaw);
  f.println(zonesCount);
  f.println(rainSensorEnabled? "1":"0");
  f.println(rainSensorPin);
  f.println(rainSensorInvert? "1":"0");
  f.println(tankLowThresholdPct);

  for (int i=0;i<MAX_ZONES;i++) f.println(zonePins[i]);
  f.println(mainsPin);
  f.println(tankPin);

  for (int i=0;i<MAX_ZONES;i++) f.println(zoneNames[i]);

  // older new fields
  f.println(rainDelayFromForecastEnabled ? "1" : "0");
  f.println(systemPaused ? "1" : "0");
  f.println(pauseUntilEpoch);

  // timezone
  f.println((int)tzMode);
  f.println(tzPosix);
  f.println(tzIANA);
  f.println((int)tzFixedOffsetMin);

  // NEW trailing: Master / cooldown / threshold / MQTT
  f.println(systemMasterEnabled ? "1" : "0");
  f.println(rainCooldownMin);
  f.println(rainThreshold24h_mm);
  f.println(mqttEnabled ? "1" : "0");
  f.println(mqttBroker);
  f.println((int)mqttPort);
  f.println(mqttUser);
  f.println(mqttPass);
  f.println(mqttBase);

  // NEW: run mode at end (backward compatible)
  f.println(runZonesConcurrent ? "1" : "0");

  f.close();
}

void loadSchedule() {
  File f = LittleFS.open("/schedule.txt","r");
  if (!f) return;

  for (int i=0; i<MAX_ZONES; i++) {
    String line=f.readStringUntil('\n'); line.trim(); if (!line.length()) continue;

    int idx=0;
    auto next=[&](int& outIdx){
      int nidx=line.indexOf(',',idx); if (nidx<0) nidx=line.length();
      String sv=line.substring(idx,nidx); sv.trim(); outIdx=nidx;
      int v = sv.toInt(); idx=nidx+1; return v;
    };

    int tmp;
    startHour[i]   = next(tmp);
    startMin[i]    = next(tmp);
    startHour2[i]  = next(tmp);
    startMin2[i]   = next(tmp);
    durationMin[i] = next(tmp);
    durationSec[i] = next(tmp);
    enableStartTime2[i] = (next(tmp)==1);

    for (int d=0; d<7; d++) {
      int nidx=line.indexOf(',',idx); if (nidx<0) nidx=line.length();
      String sv=line.substring(idx,nidx); sv.trim();
      days[i][d] = (sv.toInt()==1);
      idx = (nidx<(int)line.length()) ? nidx+1 : nidx;
    }
  }
  f.close();
}

void saveSchedule() {
  File f = LittleFS.open("/schedule.txt","w");
  if (!f) return;
  for (int i=0; i<MAX_ZONES; i++) {
    f.print(startHour[i]);  f.print(',');
    f.print(startMin[i]);   f.print(',');
    f.print(startHour2[i]); f.print(',');
    f.print(startMin2[i]);  f.print(',');
    f.print(durationMin[i]);f.print(',');
    f.print(durationSec[i]);f.print(',');
    f.print(enableStartTime2[i] ? '1' : '0');
    for (int d=0; d<7; d++){ f.print(','); f.print(days[i][d] ? '1' : '0'); }
    f.println();
  }
  f.close();
}

// ---------- Configure (POST) ----------
void handleConfigure() {
  bool needRestart=false;
  if (server.hasArg("apiKey")) { apiKey = server.arg("apiKey"); needRestart=true; }
  if (server.hasArg("city"))   { city   = server.arg("city");   needRestart=true; }

  if (server.hasArg("zonesMode")) {
    int zm = server.arg("zonesMode").toInt();
    zonesCount = (zm==6)?6:4; needRestart=true;
  }
  if (server.hasArg("tankThresh")) {
    int th=server.arg("tankThresh").toInt();
    if (th>=0 && th<=100) tankLowThresholdPct = th;
  }

  rainDelayEnabled = server.hasArg("rainDelay");
  windDelayEnabled = server.hasArg("windCancelEnabled");
  justUseTank  = server.hasArg("justUseTank");
  justUseMains = server.hasArg("justUseMains");
  if (justUseTank && justUseMains){ justUseTank=false; justUseMains=true; }
  if (zonesCount==6){ justUseTank=false; justUseMains=false; }

  if (server.hasArg("windSpeedThreshold")) windSpeedThreshold = server.arg("windSpeedThreshold").toFloat();

  // Physical rain sensor
  rainSensorEnabled = server.hasArg("rainSensorEnabled");
  if (server.hasArg("rainSensorPin"))  rainSensorPin = server.arg("rainSensorPin").toInt();
  rainSensorInvert  = server.hasArg("rainSensorInvert");

  // Forecast rain toggle (saved)
  rainDelayFromForecastEnabled = !server.hasArg("rainForecastDisabled");

  // NEW: Run mode
  runZonesConcurrent = server.hasArg("runConcurrent");

  // Pause controls
  if (server.hasArg("resumeNow")) {
    systemPaused = false; pauseUntilEpoch = 0;
  } else {
    bool reqPause = server.hasArg("pauseEnable");
    int pauseHours = 0;
    if (server.hasArg("pauseHours")) {
      pauseHours = server.arg("pauseHours").toInt();
      if (pauseHours < 0) pauseHours = 0;
    }
    if (reqPause) {
      systemPaused = true;
      pauseUntilEpoch = (pauseHours == 0) ? 0 : (uint32_t)(time(nullptr) + (pauseHours * 3600));
    } else {
      systemPaused = false; pauseUntilEpoch = 0;
    }
  }

  // Master / cooldown / threshold
  systemMasterEnabled = server.hasArg("masterOn"); // (Master switch is controlled from dashboard; Setup doesn't render it)

  // New UI posts HOURS; still accept legacy MINUTES if present
  if (server.hasArg("rainCooldownHours")) {
    int h = server.arg("rainCooldownHours").toInt();
    if (h < 0) h = 0; if (h > 720) h = 720;
    rainCooldownMin = h * 60;                 // store internally in minutes
  } else if (server.hasArg("rainCooldownMin")) {
    int m = server.arg("rainCooldownMin").toInt();
    if (m < 0) m = 0; if (m > 43200) m = 43200; // 720h cap as minutes
    rainCooldownMin = m;
  }

  if (server.hasArg("rainThreshold24h")) {
    int mm = server.arg("rainThreshold24h").toInt();
    if (mm < 0) mm = 0; if (mm > 200) mm = 200;
    rainThreshold24h_mm = mm;
  }
  // Debug toggles (if you add in UI later)
  dbgForceRain = server.hasArg("dbgRain");
  dbgForceWind = server.hasArg("dbgWind");

  // Fallback pins
  for (int i=0;i<MAX_ZONES;i++) if (server.hasArg("zonePin"+String(i))) zonePins[i]=server.arg("zonePin"+String(i)).toInt();
  if (server.hasArg("mainsPin")) mainsPin = server.arg("mainsPin").toInt();
  if (server.hasArg("tankPin"))  tankPin  = server.arg("tankPin").toInt();

  // Timezone fields
  if (server.hasArg("tzMode")) {
    int m = server.arg("tzMode").toInt();
    tzMode = (m==1) ? TZ_IANA : (m==2) ? TZ_FIXED : TZ_POSIX;
  }
  if (server.hasArg("tzPosix")) { String s = server.arg("tzPosix"); s.trim(); if (s.length() > 0) tzPosix = s; }
  if (server.hasArg("tzIANA"))  { String s = server.arg("tzIANA");  s.trim(); if (s.length() > 0) tzIANA = s; }
  if (server.hasArg("tzFixed")) { tzFixedOffsetMin = (int16_t)server.arg("tzFixed").toInt(); }
  needRestart = true; // clean SNTP reapply

  // MQTT
  mqttEnabled = server.hasArg("mqttEnabled");
  if (server.hasArg("mqttBroker")) mqttBroker = server.arg("mqttBroker");
  if (server.hasArg("mqttPort"))   mqttPort   = (uint16_t)server.arg("mqttPort").toInt();
  if (server.hasArg("mqttUser"))   mqttUser   = server.arg("mqttUser");
  if (server.hasArg("mqttPass"))   mqttPass   = server.arg("mqttPass");
  if (server.hasArg("mqttBase"))   mqttBase   = server.arg("mqttBase");

  saveConfig(); loadConfig();

  String html = F(
    "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<meta http-equiv='refresh' content='2;url=/setup'/>"
    "<title>Saved</title><style>body{font-family:Inter,system-ui,Segoe UI,Roboto,Arial,sans-serif;text-align:center;padding:40px;color:#e8eef6;background:#0e1726}</style></head>"
    "<body><h2>✅ Settings Saved</h2><p>Returning to Setup…</p></body></html>"
  );
  server.send(200,"text/html",html);
  if (needRestart){ delay(1200); ESP.restart(); }
}

void handleClearEvents() {
  if (LittleFS.exists("/events.csv")) LittleFS.remove("/events.csv");
  server.sendHeader("Location","/events",true);
  server.send(302,"text/plain","");
}
