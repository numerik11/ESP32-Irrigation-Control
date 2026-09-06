//-- If using Arduino IDE, select Minimal SPIFFS (Large APPS with OTA).
//-- It provides two application slots for OTA plus a SPIFFS/LittleFS partition.

#ifndef ENABLE_DEBUG_ROUTES
  #define ENABLE_DEBUG_ROUTES 0   // set to 1 when you need them
#endif
#ifndef ENABLE_OTA
  #define ENABLE_OTA 1
#endif
#ifndef ENABLE_WIFI_LONG_RANGE
  #define ENABLE_WIFI_LONG_RANGE 0   // long-range mode improves weak links but slows HTTP browsing
#endif
#ifndef TFT_MISO
  #define TFT_MISO -1   // ST7789 typically doesn't use MISO
#endif
#ifndef ENABLE_TFT
  #define ENABLE_TFT 1
#endif
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#if ENABLE_OTA
  #include <Update.h>
#endif
#include <WiFiManager.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LittleFS.h>
#include <Ticker.h>
#include <PCF8574.h> // (Must use Library from Github)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Adafruit_ST7789.h> // (Adafruit ST7735 and ST7789 Library)
#include <Adafruit_NeoPixel.h>
#include <Adafruit_AHTX0.h>
#include <DHT.h>
#include <new>
#include <math.h>
#include "driver/gpio.h"
#include "esp_heap_caps.h"
extern "C" {
  #include "esp_log.h"
  #include "esp_system.h"
  #include "esp_wifi.h"
}
#include <time.h>
#include <ESPmDNS.h> 
#include <PubSubClient.h>   // MQTT

// ---------- Hardware ----------
static const char kFirmwareSignature[] __attribute__((used)) =
  "Original author: Beau Kaczmarek - https://github.com/numerik11/ESP32-Irrigation-Controller";
static const char kFirmwareVersion[] = "2.8";
static const char kFirmwareBuildDate[] = __DATE__ " " __TIME__;
static const char kUpdateReportUrl[] =
  "https://irrigation-update-counter.beaukacz86.workers.dev/v1/report";
static const char kUpdateReportStatePath[] = "/update_report.txt";
static const uint32_t UPDATE_REPORT_BOOT_DELAY_MS = 30000UL;
static const uint32_t UPDATE_REPORT_RETRY_MS = 15UL * 60UL * 1000UL;
static String updateReportEventId;
static bool updateReportPending = false;
static uint32_t updateReportNextAttemptMs = 0;
static const uint8_t MAX_ZONES = 16;
#if defined(CONFIG_IDF_TARGET_ESP32)
static const int I2C_SDA_DEFAULT = 21;
static const int I2C_SCL_DEFAULT = 22;
#else
static const int I2C_SDA_DEFAULT = 8;
static const int I2C_SCL_DEFAULT = 9;
#endif
static int i2cSdaPin = I2C_SDA_DEFAULT;
static int i2cSclPin = I2C_SCL_DEFAULT;
#ifndef STATUS_PIXEL_PIN
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    #define STATUS_PIXEL_PIN 48   // ESP32-S3 DevKitC-1 onboard WS2812
  #else
    #define STATUS_PIXEL_PIN -1   // disable by default on non-S3 targets
  #endif
#endif
static const uint8_t STATUS_PIXEL_COUNT = 1;

TwoWire I2Cbus = TwoWire(0);
PCF8574 pcfIn (&I2Cbus, 0x22, I2C_SDA_DEFAULT, I2C_SCL_DEFAULT);
PCF8574 pcfOut(&I2Cbus, 0x24, I2C_SDA_DEFAULT, I2C_SCL_DEFAULT);
Adafruit_NeoPixel statusPixel(STATUS_PIXEL_COUNT, STATUS_PIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_AHTX0 localAht;
DHT* localDht = nullptr;
bool statusPixelReady = false;
uint32_t statusPixelLastColor = 0;
extern uint32_t bootMillis;

// ---------- ST7789 (1.9") TFT ----------
// Common ST7789 sizes are 170x320 and 240x320.
// Kept runtime-configurable so the Setup page can persist the panel geometry.
static constexpr int16_t TFT_W_DEFAULT = 170;
static constexpr int16_t TFT_H_DEFAULT = 320;

// Choose pins that do NOT clash with your I2C (4/15) or other IO.
// Defaults (editable from Setup -> SPI (TFT)):
#if defined(CONFIG_IDF_TARGET_ESP32)
static const int TFT_SCLK_DEFAULT = 18;
static const int TFT_MOSI_DEFAULT = 23;
static const int TFT_CS_DEFAULT   = 5;
static const int TFT_DC_DEFAULT   = 2;
static const int TFT_RST_DEFAULT  = 4;
#else
static const int TFT_SCLK_DEFAULT = 41;  // SCL
static const int TFT_MOSI_DEFAULT = 42;  // SDA
static const int TFT_CS_DEFAULT   = 1;   // CS (avoid GPIO1 tank ADC)
static const int TFT_DC_DEFAULT   = 2;   // DC (avoid GPIO2 LED)
static const int TFT_RST_DEFAULT  = 21;  // RST (-1 allowed)
#endif
static const int TFT_BL_DEFAULT   = -1;  // or -1 if tied to 3V3

static int tftSclkPin = TFT_SCLK_DEFAULT;
static int tftMosiPin = TFT_MOSI_DEFAULT;
static int tftCsPin   = TFT_CS_DEFAULT;
static int tftDcPin   = TFT_DC_DEFAULT;
static int tftRstPin  = TFT_RST_DEFAULT;
static int tftBlPin   = TFT_BL_DEFAULT;
static int16_t tftPanelWidth  = TFT_W_DEFAULT;
static int16_t tftPanelHeight = TFT_H_DEFAULT;
static uint8_t tftRotation = 0; // ST7789 rotation: 0..3

Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, -1, -1, -1);

// ---------- OLED ----------
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Cbus, OLED_RESET);
bool displayEnabled = true;
bool tempUseFahrenheit = false;
bool displayUseTft = (ENABLE_TFT != 0); // default mode; can be changed in Setup
bool clockUse24Hour = true;

// ===================== UI helpers + palette (MUST be before RainScreen/HomeScreen) =====================
static inline uint16_t RGB(uint8_t r, uint8_t g, uint8_t b){
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Palette (RGB565)
static const uint16_t C_BG       = RGB(7, 10, 16);
static const uint16_t C_PANEL    = RGB(17, 23, 33);
static const uint16_t C_PANEL_2  = RGB(23, 32, 45);
static const uint16_t C_EDGE     = RGB(58, 73, 92);
static const uint16_t C_SHADOW   = RGB(3, 5, 8);
static const uint16_t C_HILITE   = RGB(83, 101, 124);
static const uint16_t C_TEXT     = RGB(235, 242, 250);
static const uint16_t C_MUTED    = RGB(150, 165, 182);
static const uint16_t C_ACCENT   = RGB(249, 196, 65);
static const uint16_t C_GOOD     = RGB(69, 196, 138);
static const uint16_t C_WARN     = RGB(246, 166, 67);
static const uint16_t C_BAD      = RGB(238, 96, 86);
static const uint16_t C_BLUE     = RGB(77, 164, 222);

static int cardRadiusFor(int w, int h) {
  int r = min(5, min(w, h) / 6);
  return max(1, r);
}

static void drawCard(int x,int y,int w,int h,uint16_t fill,uint16_t edge){
  if (w <= 2 || h <= 2) return;
  const int r = cardRadiusFor(w, h);
  tft.fillRoundRect(x, y, w, h, r, fill);
  tft.drawRoundRect(x, y, w, h, r, edge);
  tft.drawFastHLine(x + r, y + 1, max(0, w - 2 * r), C_HILITE);
  tft.drawFastHLine(x + r, y + h - 2, max(0, w - 2 * r), C_SHADOW);
}

static void drawMetricBox(int x, int y, int w, int h) {
  if (w <= 2 || h <= 2) return;
  const int r = cardRadiusFor(w, h);
  tft.fillRoundRect(x, y, w, h, r, C_PANEL_2);
  tft.drawRoundRect(x, y, w, h, r, C_EDGE);
  tft.drawFastHLine(x + r, y + 1, max(0, w - 2 * r), C_HILITE);
}

static void drawPressureTrendArrow(int x, int y, int trend, uint16_t bg) {
  tft.fillRect(x, y, 13, 13, bg);
  if (trend > 0) {
    tft.fillTriangle(x + 6, y, x + 1, y + 6, x + 11, y + 6, C_BAD);
    tft.fillRect(x + 5, y + 6, 3, 7, C_BAD);
  } else if (trend < 0) {
    tft.fillRect(x + 5, y, 3, 7, C_BLUE);
    tft.fillTriangle(x + 1, y + 6, x + 11, y + 6, x + 6, y + 12, C_BLUE);
  }
}

static void drawTopBar(const char* title, const char* pill, uint16_t pillColor){
  const int W = tft.width();
  tft.setTextWrap(false);
  tft.fillRect(0, 0, W, 34, C_PANEL_2);
  tft.fillRect(0, 0, 4, 34, C_ACCENT);
  tft.drawFastHLine(0, 34, W, C_EDGE);
  tft.drawFastHLine(4, 1, max(0, W - 4), C_HILITE);

  tft.setTextSize(2);
  tft.setTextColor(C_TEXT, C_PANEL_2);
  tft.setCursor(10, 8);
  tft.print(title);

  int pillW = 8 + (int)strlen(pill) * 12; // rough for textSize(2)
  int x = W - pillW - 8;
  tft.fillRoundRect(x, 7, pillW, 20, 5, pillColor);
  tft.drawRoundRect(x, 7, pillW, 20, 5, C_EDGE);
  tft.setTextColor(ST77XX_BLACK, pillColor);
  tft.setCursor(x + 6, 9);
  tft.print(pill);
}

static void fmtMMSS(char* out, size_t n, unsigned long sec){
  unsigned long m = sec / 60;
  unsigned long s = sec % 60;
  snprintf(out, n, "%02lum%02lus", m, s);
}

static void formatClockTime(const struct tm& t, char* out, size_t n) {
  if (clockUse24Hour) {
    snprintf(out, n, "%02d:%02d", t.tm_hour, t.tm_min);
    return;
  }

  int hour = t.tm_hour % 12;
  if (hour == 0) hour = 12;
  snprintf(out, n, "%d:%02d%c", hour, t.tm_min, (t.tm_hour < 12) ? 'a' : 'p');
}

static String formatRuntimeClock(unsigned long totalSec) {
  unsigned long hours = totalSec / 3600UL;
  unsigned long minutes = (totalSec / 60UL) % 60UL;
  unsigned long seconds = totalSec % 60UL;

  char buf[16];
  if (hours > 0) snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", hours, minutes, seconds);
  else snprintf(buf, sizeof(buf), "%lu:%02lu", minutes, seconds);
  return String(buf);
}

static const char* wifiSignalQuality(int rssi) {
  if (WiFi.status() != WL_CONNECTED) return "Disconnected";
  if (rssi >= -50) return "Excellent";
  if (rssi >= -60) return "Strong";
  if (rssi >= -67) return "Good";
  if (rssi >= -75) return "Fair";
  if (rssi >= -82) return "Weak";
  return "Very weak";
}

WiFiManager wifiManager;
WebServer server(80);

// PCF mapping
const uint8_t ALL_P = 6;
const uint8_t PCH[ALL_P] = { P0, P1, P2, P3, P4, P5 };
uint8_t mainsChannel = P4; // reserved for mains relay
uint8_t tankChannel  = P5; // reserved for tank relay
uint8_t zonesCount   = 4;  // 1..MAX_ZONES

// GPIO fallback (configurable polarity)
bool useGpioFallback = false;
bool gpioActiveLow   = false;
bool relayActiveHigh = true;
bool zoneGpioActiveLow[MAX_ZONES] = {false};
bool mainsGpioActiveLow = false;
bool tankGpioActiveLow  = false;
bool tankEnabled     = true;
int powerSupplyPin = -1;          // optional switched supply enable output (-1 = disabled)
bool powerSupplyActiveLow = false;

#if defined(CONFIG_IDF_TARGET_ESP32)
int zonePins[MAX_ZONES] = {
  15, 16, 17, 18, 4, 5,
  25, 26, -1, -1, -1, -1, -1, -1, -1, -1
};
int mainsPin = 32;
int tankPin  = 33;
int tankLevelPin = 34; // ADC1 input on classic ESP32 (works while Wi-Fi is active)
#else
int zonePins[MAX_ZONES] = {
  15, 16, 17, 18, 6, 7,
  10, 11, -1, -1, -1, -1, -1, -1, -1, -1
};
int mainsPin = 39;
int tankPin  = 40;
int tankLevelPin = 19; // ADC input (ESP32-S3: GPIO1..20 are ADC)
#endif

#ifndef STATUS_LED_PIN
  #define STATUS_LED_PIN -1
#endif
#ifndef STATUS_LED_ACTIVE_LOW
  #define STATUS_LED_ACTIVE_LOW 0
#endif
const int LED_PIN = STATUS_LED_PIN;
static const bool STATUS_LED_IS_ACTIVE_LOW = (STATUS_LED_ACTIVE_LOW != 0);

// Physical rain sensor
bool rainSensorEnabled = false;
bool rainSensorInvert  = false;
int  rainSensorPin     = 16;

// Physical manual control buttons (disabled by default)
int manualSelectPin = -1;   // cycles the target zone (INPUT_PULLUP, -1 = disabled)
int manualStartPin  = -1;   // toggles start/stop for the selected zone (INPUT_PULLUP, -1 = disabled)
uint8_t manualSelectedZone = 0;
uint32_t manualScreenUntilMs = 0;
const unsigned long MANUAL_BTN_DEBOUNCE_MS = 60;
const unsigned long MANUAL_BTN_REPEAT_GUARD_MS = 180;

struct ManualButtonState {
  int stableState = HIGH;
  int lastRawState = HIGH;
  uint32_t lastRawChangeMs = 0;
  uint32_t lastPressMs = 0;
};

static void resetManualButtonState(ManualButtonState& state, int pin);
static bool consumeManualButtonPress(int pin, ManualButtonState& state, uint32_t nowMs);

static ManualButtonState g_manualSelectBtn;
static ManualButtonState g_manualStartBtn;

// ---------- Config / State ----------
static unsigned long totalScheduledRuntimeSec = 0;
static unsigned long totalManualRuntimeSec = 0;

float  meteoLat = -35.1076f;
float  meteoLon = 138.5573f;
String meteoLocation = "Adelaide"; // Open-Meteo display label (optional)
String meteoModel = "best_match"; // Open-Meteo forecast model slug
enum LocalClimateSource : uint8_t {
  CLIMATE_OPEN_METEO = 0,
  CLIMATE_AHT20_I2C = 2,
  CLIMATE_DHT22_GPIO = 3
};
static LocalClimateSource parseClimateSource(const String& raw);
LocalClimateSource climateSource = CLIMATE_OPEN_METEO;
int dhtSensorPin = -1;
int localDhtActivePin = -1;
bool localAhtReady = false;
uint32_t localAhtLastAttemptMs = 0;
uint32_t localClimateLastReadMs = 0;
bool localClimateValid = false;
float localTempC = NAN;
float localHumidityPct = NAN;
String cachedWeatherData;
String lastWeatherError;
String lastForecastError;
int    lastWeatherHttpCode  = 0;
int    lastForecastHttpCode = 0;

// Weather cache / metrics
unsigned long lastWeatherUpdate = 0;
const unsigned long weatherUpdateInterval = 15UL * 60UL * 1000UL; // 15m

// Forecast cache / metrics
String cachedForecastData;
unsigned long lastForecastUpdate = 0;
const unsigned long forecastUpdateInterval = 60UL * 60UL * 1000UL; // 60m
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

// Master/Cooldown/Threshold
bool     systemMasterEnabled = true;     // Master On/Off
uint32_t rainCooldownUntilEpoch = 0;     // when > now => block starts
int      rainCooldownMin = 60;           // minutes to wait after rain clears
int      rainThreshold24h_mm = 5;        // forecast/actual 24h total triggers delay
bool     smartWateringEnabled = false;   // adjust scheduled/manual runtimes from temp/rain
float    smartCoolTempC = 18.0f;
int      smartCoolAdjustPct = -20;
float    smartHotTempC = 28.0f;
int      smartHotAdjustPct = 20;
float    smartVeryHotTempC = 34.0f;
int      smartVeryHotAdjustPct = 40;
float    smartActualRainSkipMm = 5.0f;
int      smartLightRainAdjustPct = -30;
float    smartForecastRainSkipMm = 5.0f;
bool     moistureProbeEnabled = false;
bool     moistureUseMeteo = false;
int      meteoMoisturePct = -1;
unsigned long meteoMoistureFetchedMs = 0;
unsigned long meteoMoistureAttemptMs = 0;
bool     meteoMoistureAttempted = false;
int      moisturePin = -1;
int      moistureDryRaw = 3000;
int      moistureWetRaw = 1200;
int      moistureSkipThresholdPct = 70;

// NEW Run mode: sequential (false) or concurrent (true)
bool runZonesConcurrent = false;

// Scheduling
bool enableStartTime2[MAX_ZONES] = {false};
// Preserve wind protection when loading older schedules.
bool zoneWindDelayEnabled[MAX_ZONES] = {
  true, true, true, true, true, true, true, true,
  true, true, true, true, true, true, true, true
};
bool days[MAX_ZONES][7] = {{false}};
bool zoneActive[MAX_ZONES] = {false};
bool zoneStartedManual[MAX_ZONES] = {false};
bool pendingStart[MAX_ZONES] = {false};
uint8_t lastStartSlot[MAX_ZONES] = {1}; // 1=primary, 2=secondary

bool     windActive = false;
bool     rainActive             = false;
bool     rainByWeatherActive    = false;
bool     rainBySensorActive     = false;
uint8_t  rainCooldownHours      = 24;      // or loaded from config

// Photoresistor (auto backlight)
bool photoAutoEnabled = false;
bool photoInvert = false;
int  photoPin = -1;
int  photoThreshold = 1500; // ADC raw (0-4095)


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
int duration2Min[MAX_ZONES] = {0};
int duration2Sec[MAX_ZONES] = {0};
int lastCheckedMinute[MAX_ZONES] = {
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1
};

int tankEmptyRaw = 100;
int tankFullRaw  = 900;
String zoneNames[MAX_ZONES] = {
  "Zone 1","Zone 2","Zone 3","Zone 4","Zone 5","Zone 6",
  "Zone 7","Zone 8","Zone 9","Zone 10","Zone 11","Zone 12",
  "Zone 13","Zone 14","Zone 15","Zone 16"
};

unsigned long zoneStartMs[MAX_ZONES] = {0};
unsigned long zoneRunTotalSec[MAX_ZONES] = {0}; // actual duration for current run
unsigned long lastScreenRefresh = 0;

const uint8_t expanderAddrs[] = { 0x22, 0x24 };
const uint8_t I2C_HEALTH_DEBOUNCE = 10;
uint8_t i2cFailCount = 0;

// Timing
static const uint32_t LOOP_SLEEP_MS    = 20;
static const uint32_t WIFI_CHECK_MS    = 10000;
static const uint32_t I2C_CHECK_MS     = 1000;
static const uint32_t TIME_QUERY_MS    = 1000;
static const uint32_t SCHEDULE_TICK_MS = 1000;
static const uint32_t TFT_CONNECTED_SCREEN_MS = 1200;
static uint32_t lastI2cCheck     = 0;
static uint32_t lastTimeQuery    = 0;
static uint32_t lastScheduleTick = 0;
static bool midnightDone = false;
static tm cachedTm = {};

// Uptime
uint32_t bootMillis = 0;
uint32_t bootCount = 0;

// ---------- NEW: Actual rainfall history (rolling 24h) + globals for 1h ----------
static float rainHist[24] = {0};   // last 24 hourly buckets (mm/hour)
static int   rainIdx = 0;          // points to most recent bucket
static time_t lastRainHistHour = 0;
static const char* RAIN_HISTORY_STATE_PATH = "/rain_state.txt";
static float last24hActualRain(); // forward
float rain1hNow = 0.0f;  // mm from current precipitation (Open-Meteo)
float rain3hNow = 0.0f;  // kept for compatibility

// Decoded current-weather snapshot (avoids repeated JSON parsing in hot paths)
float curTempC = NAN;
float curFeelsC = NAN;
float curPressureHpa = NAN;
float prevPressureHpa = NAN;
int   curPressureTrend = 0; // -1 falling, 0 steady/unknown, 1 rising
float curWindMs = NAN;
float curGustMs = NAN;
float curWindDirDeg = NAN;
int   curHumidityPct = -1;
int   curWeatherCode = -1;
int   curUtcOffsetSec = 0;
bool  curWeatherValid = false;
bool  g_configLoadedFromFs = false;

// ---------- NEW: guard to avoid fetching while serving HTTP ----------
volatile bool g_inHttp = false;
struct HttpScope {
  HttpScope()  { g_inHttp = true; }
  ~HttpScope() { g_inHttp = false; }
};

static void configureWifiForHttp() {
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
#if ENABLE_WIFI_LONG_RANGE
  WiFi.enableLongRange(true);
  const uint8_t protocols = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR;
#else
  WiFi.enableLongRange(false);
  const uint8_t protocols = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
#endif
  esp_err_t err = esp_wifi_set_protocol(WIFI_IF_STA, protocols);
  if (err != ESP_OK) {
    Serial.printf("[WiFi] set_protocol failed: %d\n", (int)err);
  }
}

#if defined(BOARD_HAS_PSRAM)
static constexpr size_t PSRAM_LARGE_ALLOC_THRESHOLD = 2048;

static void configurePsramForLargeBuffers() {
  if (!psramFound()) {
    Serial.println("[BOOT] PSRAM board selected, but no PSRAM was detected.");
    return;
  }
  heap_caps_malloc_extmem_enable(PSRAM_LARGE_ALLOC_THRESHOLD);
  Serial.printf("[BOOT] PSRAM ready: total=%u free=%u threshold=%u\n",
                (unsigned)ESP.getPsramSize(),
                (unsigned)ESP.getFreePsram(),
                (unsigned)PSRAM_LARGE_ALLOC_THRESHOLD);
}
#else
static void configurePsramForLargeBuffers() {}
#endif

// ---------- Prototypes ----------
void wifiCheck();
void loadConfig();
void saveConfig();
void loadSchedule();
void saveSchedule();
void updateCachedWeather();
void tickWeather();                 // NEW
static void loadRainHistoryState();
static void saveRainHistoryState();
static void clearRainHistoryState();
void HomeScreen();
void RainScreen();
void updateLCDForZone(int zone);
bool shouldStartZone(int zone);
bool hasDurationCompleted(int zone);
void turnOnZone(int zone);
void turnOffZone(int zone);
bool turnOnValveManual(int z, String* error = nullptr);
void turnOffValveManual(int z);
static void updateBootCount();
static bool saveBootCount(uint32_t count);
static void initUpdateReportState();
static void tickUpdateReport();
void handleRoot();
void handleSubmit();
void handleSetupPage();
void handleConfigure();
void handleLogPage();
void handleClearEvents();
void handleDiagnosticsPage();
void handleDiagnosticsJson();
void handleTankCalibration();
String fetchWeather();
String fetchWeatherHourlyForCurrent(const String& model, float lat, float lon, bool useForecastEndpoint);
bool buildCurrentFromHourlyPayload(const String& hourlyPayload, String& outPayload);
String fetchForecast(float lat, float lon);
bool checkWindRain();
void checkI2CHealth();
void initGpioFallback();
void initGpioPinsForZones();
bool initExpanders();
void toggleBacklight();
void logEvent(int zone, const char* eventType, const char* source, bool rainDelayed);
void printCurrentTime();
int    tankPercent();
bool   isTankLow();
int    moistureRaw();
int    moisturePercent();
bool   isSoilWetForSmartSkip();
String sourceModeText();
bool refreshCurrentWeatherSnapshotFromCache();
void initManualButtons();
void tickManualButtons();
void showManualSelection();
void drawManualSelection();
bool isValidAdcPin(int pin);
bool isValidGpioPin(int pin);
bool isValidPhotoPin(int pin);
void updateStatusPixel();
void initStatusLedPwm();
void updateStatusLedPwm();
static inline unsigned long durationForSlot(int z, int slot);
static float smartWateringFactor();
static unsigned long smartWateringDurationForSlot(int z, int slot);
time_t parseEventTimestamp(const String& ts);
void rebuildRuntimeCountersFromEvents();
void statusPixelSet(uint8_t r,uint8_t g,uint8_t b);
uint8_t statusPixelPulseLevel(uint16_t periodMs, uint8_t low, uint8_t high);
bool statusPixelWindowOn(uint16_t periodMs, uint16_t startMs, uint16_t widthMs);
uint8_t statusLedDutyLevel();
bool physicalRainNowRaw();
String rainDelayCauseText();
static inline bool isRainDelayBlockingNow();


// ===================== Timezone config =====================
enum TZMode : uint8_t { TZ_POSIX = 0, TZ_IANA = 1, TZ_FIXED = 2 };
TZMode tzMode = TZ_POSIX;
String tzPosix = "ACST-9:30ACDT,M10.1.0,M4.1.0/3"; // default
String tzIANA  = "Australia/Adelaide";
int16_t tzFixedOffsetMin = 570;

static const char* posixForIanaZone(const String& iana) {
  if (iana == "UTC" || iana == "Etc/UTC") return "UTC0";
  if (iana == "Australia/Adelaide") return "ACST-9:30ACDT,M10.1.0,M4.1.0/3";
  if (iana == "Australia/Brisbane") return "AEST-10";
  if (iana == "Australia/Broken_Hill") return "ACST-9:30ACDT,M10.1.0,M4.1.0/3";
  if (iana == "Australia/Darwin") return "ACST-9:30";
  if (iana == "Australia/Hobart") return "AEST-10AEDT,M10.1.0,M4.1.0/3";
  if (iana == "Australia/Lord_Howe") return "<+1030>-10:30<+11>-11,M10.1.0,M4.1.0";
  if (iana == "Australia/Melbourne") return "AEST-10AEDT,M10.1.0,M4.1.0/3";
  if (iana == "Australia/Perth") return "AWST-8";
  if (iana == "Australia/Sydney") return "AEST-10AEDT,M10.1.0,M4.1.0/3";
  if (iana == "Pacific/Auckland") return "NZST-12NZDT,M9.5.0,M4.1.0/3";
  if (iana == "Pacific/Chatham") return "<+1245>-12:45<+1345>,M9.5.0/2:45,M4.1.0/3:45";
  if (iana == "Pacific/Fiji") return "<+12>-12";
  if (iana == "Pacific/Guam") return "ChST-10";
  if (iana == "Pacific/Honolulu") return "HST10";
  if (iana == "Pacific/Noumea") return "<+11>-11";
  if (iana == "Pacific/Port_Moresby") return "<+10>-10";
  if (iana == "Asia/Bangkok") return "<+07>-7";
  if (iana == "Asia/Dubai") return "<+04>-4";
  if (iana == "Asia/Hong_Kong") return "HKT-8";
  if (iana == "Asia/Jakarta") return "WIB-7";
  if (iana == "Asia/Kolkata") return "IST-5:30";
  if (iana == "Asia/Manila") return "PST-8";
  if (iana == "Asia/Seoul") return "KST-9";
  if (iana == "Asia/Shanghai") return "CST-8";
  if (iana == "Asia/Singapore") return "<+08>-8";
  if (iana == "Asia/Tokyo") return "JST-9";
  if (iana == "Europe/Amsterdam") return "CET-1CEST,M3.5.0,M10.5.0/3";
  if (iana == "Europe/Berlin") return "CET-1CEST,M3.5.0,M10.5.0/3";
  if (iana == "Europe/London") return "GMT0BST,M3.5.0/1,M10.5.0";
  if (iana == "Europe/Madrid") return "CET-1CEST,M3.5.0,M10.5.0/3";
  if (iana == "Europe/Paris") return "CET-1CEST,M3.5.0,M10.5.0/3";
  if (iana == "Europe/Rome") return "CET-1CEST,M3.5.0,M10.5.0/3";
  if (iana == "America/Anchorage") return "AKST9AKDT,M3.2.0,M11.1.0";
  if (iana == "America/Chicago") return "CST6CDT,M3.2.0,M11.1.0";
  if (iana == "America/Denver") return "MST7MDT,M3.2.0,M11.1.0";
  if (iana == "America/Los_Angeles") return "PST8PDT,M3.2.0,M11.1.0";
  if (iana == "America/New_York") return "EST5EDT,M3.2.0,M11.1.0";
  if (iana == "America/Phoenix") return "MST7";
  if (iana == "America/Toronto") return "EST5EDT,M3.2.0,M11.1.0";
  if (iana == "America/Vancouver") return "PST8PDT,M3.2.0,M11.1.0";
  if (iana == "Africa/Johannesburg") return "SAST-2";
  return nullptr;
}

static void applyTimezoneAndSNTP() {
  const char* ntp1 = "pool.ntp.org";
  const char* ntp2 = "time.google.com";
  const char* ntp3 = "time.cloudflare.com";

  switch (tzMode) {
    case TZ_IANA: {
      const char* mapped = posixForIanaZone(tzIANA);
      configTzTime(mapped ? mapped : tzPosix.c_str(), ntp1, ntp2, ntp3);
      break;
    }
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
String otaPassword = "";

WiFiClient   _mqttNetCli;
PubSubClient _mqtt(_mqttNetCli);
uint32_t     _lastMqttPub = 0;
uint32_t     _lastMqttAttempt = 0;
uint32_t     _mqttReconnectDelayMs = 15000;
uint8_t      _mqttReconnectFailures = 0;
bool         _mqttDiscoveryPublished = false;
uint32_t     _lastMqttDiscoveryAttempt = 0;

static const uint32_t MQTT_RECONNECT_DELAY_MIN_MS = 15000;
static const uint32_t MQTT_RECONNECT_DELAY_MAX_MS = 300000;
static const uint32_t MQTT_DISCOVERY_RETRY_MS = 30000;
static const char MQTT_DISCOVERY_PREFIX[] = "homeassistant";

static void mqttAddHomeAssistantDevice(JsonDocument& d) {
  JsonObject device = d["device"].to<JsonObject>();
  JsonArray identifiers = device["identifiers"].to<JsonArray>();
  identifiers.add("espirrigation");
  device["name"] = "ESP Irrigation";
  device["manufacturer"] = "ESP32 Irrigation Controller";
  device["model"] = "ESP32 Irrigation Controller";
}

static void mqttBeginHomeAssistantEntity(JsonDocument& d,
                                         const String& name,
                                         const String& uniqueId,
                                         const String& stateTopic) {
  d.clear();
  d["name"] = name;
  d["unique_id"] = uniqueId;
  d["state_topic"] = stateTopic;
  mqttAddHomeAssistantDevice(d);
}

static bool mqttPublishHomeAssistantConfig(const char* component,
                                           const String& objectId,
                                           JsonDocument& d) {
  String topic = String(MQTT_DISCOVERY_PREFIX) + "/" + component + "/" + objectId + "/config";
  String payload;
  payload.reserve(640);
  serializeJson(d, payload);
  return _mqtt.publish(topic.c_str(), payload.c_str(), true);
}

static bool mqttPublishHomeAssistantDiscovery() {
  if (!mqttEnabled || !_mqtt.connected()) return false;

  const String stateTopic = mqttBase + "/status";
  bool allPublished = true;
  JsonDocument d;

  mqttBeginHomeAssistantEntity(d, "24h Rainfall", "espirrigation_rainfall_24h", stateTopic);
  d["value_template"] = "{{ value_json.rain24hActual }}";
  d["unit_of_measurement"] = "mm";
  d["device_class"] = "precipitation";
  d["state_class"] = "measurement";
  if (!mqttPublishHomeAssistantConfig("sensor", "espirrigation_rainfall_24h", d)) allPublished = false;

  mqttBeginHomeAssistantEntity(d, "Irrigation Cooldown", "espirrigation_cooldown", stateTopic);
  d["value_template"] = "{{ value_json.cooldownRemaining }}";
  d["unit_of_measurement"] = "s";
  d["device_class"] = "duration";
  d["state_class"] = "measurement";
  if (!mqttPublishHomeAssistantConfig("sensor", "espirrigation_cooldown", d)) allPublished = false;

  mqttBeginHomeAssistantEntity(d, "Irrigation Master", "espirrigation_master", stateTopic);
  d["value_template"] = "{{ 'ON' if value_json.masterOn else 'OFF' }}";
  d["payload_on"] = "ON";
  d["payload_off"] = "OFF";
  if (!mqttPublishHomeAssistantConfig("binary_sensor", "espirrigation_master", d)) allPublished = false;

  mqttBeginHomeAssistantEntity(d, "Irrigation Paused", "espirrigation_paused", stateTopic);
  d["value_template"] = "{{ 'ON' if value_json.paused else 'OFF' }}";
  d["payload_on"] = "ON";
  d["payload_off"] = "OFF";
  if (!mqttPublishHomeAssistantConfig("binary_sensor", "espirrigation_paused", d)) allPublished = false;

  mqttBeginHomeAssistantEntity(d, "Rain Active", "espirrigation_rain_active", stateTopic);
  d["value_template"] = "{{ 'ON' if value_json.rainActive else 'OFF' }}";
  d["payload_on"] = "ON";
  d["payload_off"] = "OFF";
  if (!mqttPublishHomeAssistantConfig("binary_sensor", "espirrigation_rain_active", d)) allPublished = false;

  mqttBeginHomeAssistantEntity(d, "Wind Active", "espirrigation_wind_active", stateTopic);
  d["value_template"] = "{{ 'ON' if value_json.windActive else 'OFF' }}";
  d["payload_on"] = "ON";
  d["payload_off"] = "OFF";
  if (!mqttPublishHomeAssistantConfig("binary_sensor", "espirrigation_wind_active", d)) allPublished = false;

  for (int i = 0; i < (int)zonesCount; ++i) {
    const String zoneNumber = String(i + 1);
    const String objectId = "espirrigation_zone_" + zoneNumber;
    String zoneLabel = zoneNames[i];
    zoneLabel.trim();
    if (!zoneLabel.length()) zoneLabel = "Zone " + zoneNumber;

    mqttBeginHomeAssistantEntity(d, "Irrigation " + zoneLabel, objectId, stateTopic);
    d["value_template"] = "{{ 'ON' if value_json.zones[" + String(i) + "].active else 'OFF' }}";
    d["command_topic"] = mqttBase + "/cmd/zone/" + String(i);
    d["payload_on"] = "on";
    d["payload_off"] = "off";
    d["state_on"] = "ON";
    d["state_off"] = "OFF";
    d["icon"] = "mdi:sprinkler";
    if (!mqttPublishHomeAssistantConfig("switch", objectId, d)) allPublished = false;
  }

  // Remove retained discovery entries for zones that are no longer configured.
  for (int i = (int)zonesCount; i < (int)MAX_ZONES; ++i) {
    const String objectId = "espirrigation_zone_" + String(i + 1);
    const String topic = String(MQTT_DISCOVERY_PREFIX) + "/switch/" + objectId + "/config";
    if (!_mqtt.publish(topic.c_str(), "", true)) allPublished = false;
  }

  Serial.printf("[MQTT] Home Assistant discovery %s (%u zones)\n",
                allPublished ? "published" : "incomplete", (unsigned)zonesCount);
  return allPublished;
}

static void mqttTryPublishHomeAssistantDiscovery(uint32_t now) {
  if (_mqttDiscoveryPublished) return;
  if (_lastMqttDiscoveryAttempt != 0 &&
      now - _lastMqttDiscoveryAttempt < MQTT_DISCOVERY_RETRY_MS) return;
  _lastMqttDiscoveryAttempt = now;
  _mqttDiscoveryPublished = mqttPublishHomeAssistantDiscovery();
}

void mqttSetup(){
  _mqttDiscoveryPublished = false;
  _lastMqttDiscoveryAttempt = 0;
  if (!mqttEnabled || mqttBroker.length()==0) return;
  _mqtt.setServer(mqttBroker.c_str(), mqttPort);
  _mqtt.setBufferSize(2048);
  _mqtt.setKeepAlive(30);
  _mqtt.setSocketTimeout(1);
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
  if (!mqttEnabled || mqttBroker.length()==0) return;
  if (WiFi.status() != WL_CONNECTED) return;
  const uint32_t now = millis();
  if (_mqtt.connected()) {
    _mqttReconnectFailures = 0;
    _mqttReconnectDelayMs = MQTT_RECONNECT_DELAY_MIN_MS;
    mqttTryPublishHomeAssistantDiscovery(now);
    return;
  }
  if (now - _lastMqttAttempt < _mqttReconnectDelayMs) return;
  _lastMqttAttempt = now;
  String cid = "espirrigation-" + WiFi.macAddress();
  bool ok = false;
  if (mqttUser.length()) {
    ok = _mqtt.connect(cid.c_str(), mqttUser.c_str(), mqttPass.c_str());
  } else {
    ok = _mqtt.connect(cid.c_str());
  }
  if (ok) {
    _mqttReconnectFailures = 0;
    _mqttReconnectDelayMs = MQTT_RECONNECT_DELAY_MIN_MS;
    _mqtt.subscribe( (mqttBase + "/cmd/#").c_str() );
    _mqttDiscoveryPublished = false;
    _lastMqttDiscoveryAttempt = 0;
    mqttTryPublishHomeAssistantDiscovery(now);
  } else {
    if (_mqttReconnectFailures < 8) _mqttReconnectFailures++;
    uint32_t nextDelay = MQTT_RECONNECT_DELAY_MIN_MS;
    for (uint8_t i = 0; i < _mqttReconnectFailures; ++i) {
      if (nextDelay >= MQTT_RECONNECT_DELAY_MAX_MS / 2) {
        nextDelay = MQTT_RECONNECT_DELAY_MAX_MS;
        break;
      }
      nextDelay *= 2;
    }
    _mqttReconnectDelayMs = nextDelay;
    Serial.printf("[MQTT] connect failed, rc=%d; retry in %lus\n",
                  _mqtt.state(), (unsigned long)(_mqttReconnectDelayMs / 1000UL));
  }
}

void mqttPublishStatus(){
  if (!mqttEnabled || !_mqtt.connected()) return;
  if (millis() - _lastMqttPub < 3000) return;
  _lastMqttPub = millis();

  JsonDocument d;
  d["masterOn"] = systemMasterEnabled;
  d["paused"]   = (systemPaused && (pauseUntilEpoch==0 || time(nullptr)<(time_t)pauseUntilEpoch));
  d["cooldownRemaining"] = (rainCooldownUntilEpoch>time(nullptr)? (rainCooldownUntilEpoch - time(nullptr)) : 0);
  d["rainActive"] = rainActive;
  d["rainDelayActive"] = isRainDelayBlockingNow();
  d["rainDelayCause"] = rainDelayCauseText();
  d["windActive"] = windActive;
  d["tankPct"]    = tankPercent();
  d["sourceMode"] = sourceModeText();
  d["rain24hActual"] = last24hActualRain();  // NEW
  d["runConcurrent"] = runZonesConcurrent;   // NEW
  d["smartWatering"] = smartWateringEnabled;
  d["smartFactor"] = smartWateringFactor();
  d["moistureEnabled"] = moistureProbeEnabled;
  d["moistureSource"] = moistureUseMeteo ? "meteo" : "probe";
  d["moisturePct"] = moisturePercent();
  d["moistureRaw"] = moistureRaw();
  d["moistureSkip"] = isSoilWetForSmartSkip();
  JsonArray arr = d["zones"].to<JsonArray>();
  for (int i=0;i<zonesCount;i++){
    JsonObject z = arr.add<JsonObject>();
    z["name"] = zoneNames[i];
    z["active"] = zoneActive[i];
  }
  String out;
  out.reserve(900);
  serializeJson(d, out);
  _mqtt.publish( (mqttBase + "/status").c_str(), out.c_str(), true);
}

static inline int i_min(int a, int b) { return (a < b) ? a : b; }

static inline bool isPausedNow() {
  time_t now = time(nullptr);
  return systemPaused && (pauseUntilEpoch == 0 || now < (time_t)pauseUntilEpoch);
}

static inline bool isCooldownActiveNow() {
  time_t now = time(nullptr);
  return (rainCooldownUntilEpoch && now < (time_t)rainCooldownUntilEpoch);
}

static inline bool isRainDelayBlockingNow() {
  return rainActive || isCooldownActiveNow();
}

static inline bool isBlockedNow(){
  if (!systemMasterEnabled) return true;
  if (isPausedNow()) return true;
  if (isCooldownActiveNow()) return true;
  return false;
}

static bool hasActiveManualZone() {
  for (int z = 0; z < (int)zonesCount; ++z) {
    if (zoneActive[z] && zoneStartedManual[z]) return true;
  }
  return false;
}

static unsigned long elapsedRunSec(int zone) {
  if (zone < 0 || zone >= (int)MAX_ZONES) return 0;
  if (zoneStartMs[zone] == 0) return 0;

  unsigned long elapsed = (millis() - zoneStartMs[zone]) / 1000UL;
  unsigned long total = zoneRunTotalSec[zone];
  if (total == 0) total = durationForSlot(zone, 1);
  if (total != 0 && elapsed > total) elapsed = total;
  return elapsed;
}

time_t parseEventTimestamp(const String& ts) {
  int year = 0, mon = 0, day = 0, hour = 0, min = 0, sec = 0;
  if (sscanf(ts.c_str(), "%d-%d-%d %d:%d:%d", &year, &mon, &day, &hour, &min, &sec) != 6) {
    return (time_t)-1;
  }

  struct tm t = {};
  t.tm_year = year - 1900;
  t.tm_mon  = mon - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min  = min;
  t.tm_sec  = sec;
  t.tm_isdst = -1;
  return mktime(&t);
}

void rebuildRuntimeCountersFromEvents() {
  totalScheduledRuntimeSec = 0;
  totalManualRuntimeSec = 0;

  File f = LittleFS.open("/events.csv", "r");
  if (!f) return;

  String activeZone[MAX_ZONES];
  time_t activeStart[MAX_ZONES] = {};
  bool activeManual[MAX_ZONES] = {};
  bool activeUsed[MAX_ZONES] = {};

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() < 5) continue;

    int i1 = line.indexOf(',');
    int i2 = line.indexOf(',', i1 + 1);
    int i3 = line.indexOf(',', i2 + 1);
    int i4 = line.indexOf(',', i3 + 1);
    if (i1 < 0 || i2 < 0 || i3 < 0 || i4 < 0) continue;

    String ts   = line.substring(0, i1);
    String zone = line.substring(i1 + 1, i2);
    String ev   = line.substring(i2 + 1, i3);
    String src  = line.substring(i3 + 1, i4);
    time_t epoch = parseEventTimestamp(ts);
    if (epoch == (time_t)-1) continue;

    int slot = -1;
    for (int i = 0; i < (int)MAX_ZONES; ++i) {
      if (activeUsed[i] && activeZone[i] == zone) {
        slot = i;
        break;
      }
    }

    if (ev == "START") {
      if (slot < 0) {
        for (int i = 0; i < (int)MAX_ZONES; ++i) {
          if (!activeUsed[i]) {
            slot = i;
            break;
          }
        }
      }
      if (slot < 0) continue;

      activeUsed[slot] = true;
      activeZone[slot] = zone;
      activeManual[slot] = (src == "MANUAL");
      activeStart[slot] = epoch;
      continue;
    }

    if (ev != "STOPPED" || slot < 0 || !activeUsed[slot]) continue;

    if (epoch >= activeStart[slot]) {
      unsigned long dur = (unsigned long)(epoch - activeStart[slot]);
      if (activeManual[slot] || src == "MANUAL") totalManualRuntimeSec += dur;
      else totalScheduledRuntimeSec += dur;
    }

    activeUsed[slot] = false;
    activeZone[slot] = "";
    activeManual[slot] = false;
    activeStart[slot] = 0;
  }

  f.close();
}

static bool windBlocksZone(int z) {
  return z >= 0 && z < (int)MAX_ZONES && windActive && zoneWindDelayEnabled[z];
}

static void stopAutoZonesForBlock() {
  for (int z = 0; z < (int)zonesCount; ++z) {
    if (zoneActive[z] && !zoneStartedManual[z] &&
        (!systemMasterEnabled || isBlockedNow() || rainActive || windBlocksZone(z))) turnOffZone(z);
  }
}

void updateStatusPixel() {
  if (!statusPixelReady) return;

  // Default: ready/idle = soft green breathe.
  uint8_t r = 0, g = statusPixelPulseLevel(1800, 4, 18), b = 0;

  // Is any zone currently active?
  bool anyZoneOn = false;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) { anyZoneOn = true; break; }
  }

  const uint32_t sinceBootMs = millis() - bootMillis;
  if (sinceBootMs < 3000U) {
    // Boot: cyan breathe.
    uint8_t level = statusPixelPulseLevel(1200, 6, 24);
    r = 0; g = level; b = level;
  } else if (WiFi.status() != WL_CONNECTED) {
    // Wi-Fi disconnected: violet double flash.
    bool on = statusPixelWindowOn(1100, 0, 120) || statusPixelWindowOn(1100, 180, 120);
    r = on ? 22 : 1;
    g = 0;
    b = on ? 14 : 0;
  } else if (anyZoneOn) {
    // Watering in progress: solid blue.
    r = 0; g = 0; b = 28;
  } else if (!systemMasterEnabled || isPausedNow()) {
    // Master off / pause: red slow pulse.
    r = statusPixelPulseLevel(1400, 2, 26);
    g = 0;
    b = 0;
  } else if (rainActive || windActive || isBlockedNow()) {
    // Weather / cooldown blocking: amber heartbeat.
    bool on = statusPixelWindowOn(1300, 0, 110) || statusPixelWindowOn(1300, 170, 110);
    r = on ? 28 : 2;
    g = on ? 14 : 1;
    b = 0;
  } else if (useGpioFallback) {
    // Fallback mode: teal pulse to distinguish degraded I/O from normal ready.
    uint8_t level = statusPixelPulseLevel(1500, 4, 22);
    r = 0;
    g = level;
    b = level / 3;
  }

  statusPixelSet(r,g,b);
}

// --- Cancel helper: cancel a (would-be) start instead of queueing ---
static inline void cancelStart(int z, const char* reason, bool dueToRain) {
  if (z >= 0 && z < (int)MAX_ZONES) pendingStart[z] = false; // ensure not queued
  logEvent(z, "CANCELLED", reason, dueToRain);               // record why it was cancelled
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

static String cleanMeteoModel(String s) {
  s.trim();
  s.toLowerCase();
  String out; out.reserve(s.length());
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-') {
      out += c;
    }
  }
  if (!out.length() || out == "auto" || out == "forecast" || out == "best") out = "best_match";
  // Compatibility for older configs that stored provider/endpoint names.
  if (out == "gfs" || out == "noaa") out = "gfs_seamless";
  else if (out == "icon" || out == "dwd-icon" || out == "dwd_icon") out = "icon_seamless";
  else if (out == "ecmwf") out = "ecmwf_ifs025";
  else if (out == "meteofrance" || out == "meteo-france" || out == "meteo_france") out = "meteofrance_seamless";
  else if (out == "jma") out = "jma_seamless";
  else if (out == "cma") out = "cma_grapes_global";
  else if (out == "gem") out = "gem_seamless";
  else if (out == "bom") out = "bom_access_global";
  else if (out == "ukmo") out = "ukmo_seamless";
  return out;
}

static bool isKnownMeteoModel(const String& s) {
  return (s == "best_match" ||
          s == "gfs_seamless" ||
          s == "icon_seamless" || s == "icon_global" || s == "icon_eu" ||
          s == "ecmwf_ifs025" ||
          s == "meteofrance_seamless" ||
          s == "jma_seamless" ||
          s == "cma_grapes_global" ||
          s == "gem_seamless" ||
          s == "bom_access_global" || s == "ukmo_seamless");
}

static bool isMeteoErrorPayload(const String& payload) {
  if (payload.indexOf("\"error\"") == -1) return false;
  JsonDocument js;
  if (deserializeJson(js, payload) != DeserializationError::Ok) return false;
  return (js["error"] | false) == true;
}

static String meteoErrorReason(const String& payload) {
  JsonDocument js;
  if (deserializeJson(js, payload) != DeserializationError::Ok) return "";
  if ((js["error"] | false) != true) return "";
  return js["reason"] | "";
}

static String meteoBaseUrl(const String& model, bool useForecastEndpoint) {
  (void)model;
  (void)useForecastEndpoint;
  return "https://api.open-meteo.com/v1/forecast";
}

static String httpGetMeteo(const String& url, int& code, uint16_t timeoutMs) {
  WiFiClientSecure secure;   // <-- This one should be created first
  HTTPClient http;           // <-- This one second
  secure.setInsecure();
  http.setTimeout(timeoutMs);
  http.begin(secure, url);
  code = http.GET();
  String payload = (code > 0) ? http.getString() : "";
  http.end();
  return payload;
}

static uint16_t meteoHttpTimeoutMs() {
  const int rssi = WiFi.RSSI();
  return (WiFi.status() == WL_CONNECTED && rssi >= -75) ? 6000 : 3000;
}

static inline bool isValidLatLon(float lat, float lon) {
  return isfinite(lat) && isfinite(lon) && lat >= -90.0f && lat <= 90.0f && lon >= -180.0f && lon <= 180.0f;
}

static bool parseLatLon(const String& s, float& lat, float& lon) {
  String t = s; t.trim();
  if (!t.length()) return false;
  int sep = t.indexOf(',');
  if (sep < 0) sep = t.indexOf(' ');
  if (sep < 0) return false;
  String a = t.substring(0, sep);
  String b = t.substring(sep + 1);
  a.trim(); b.trim();
  if (!a.length() || !b.length()) return false;
  float la = a.toFloat();
  float lo = b.toFloat();
  if (!isValidLatLon(la, lo)) return false;
  lat = la; lon = lo;
  return true;
}

static String meteoLocationLabel() {
  if (meteoLocation.length()) return meteoLocation;
  if (isValidLatLon(meteoLat, meteoLon)) {
    return String(meteoLat, 4) + "," + String(meteoLon, 4);
  }
  return String("-");
}

static const char* meteoCodeToMain(int code) {
  if (code == 0) return "Clear";
  if (code >= 1 && code <= 3) return "Cloudy";
  if (code == 45 || code == 48) return "Fog";
  if (code >= 51 && code <= 57) return "Drizzle";
  if (code >= 61 && code <= 67) return "Rain";
  if (code >= 71 && code <= 77) return "Snow";
  if (code >= 80 && code <= 82) return "Showers";
  if (code >= 85 && code <= 86) return "Snow";
  if (code >= 95) return "Thunder";
  return "Unknown";
}

static const char* meteoCodeToDesc(int code) {
  switch (code) {
    case 0:  return "Clear sky";
    case 1:  return "Mainly clear";
    case 2:  return "Partly cloudy";
    case 3:  return "Overcast";
    case 45: return "Fog";
    case 48: return "Rime fog";
    case 51: return "Light drizzle";
    case 53: return "Moderate drizzle";
    case 55: return "Dense drizzle";
    case 56: return "Freezing drizzle";
    case 57: return "Dense freezing drizzle";
    case 61: return "Slight rain";
    case 63: return "Moderate rain";
    case 65: return "Heavy rain";
    case 66: return "Freezing rain";
    case 67: return "Heavy freezing rain";
    case 71: return "Slight snow";
    case 73: return "Moderate snow";
    case 75: return "Heavy snow";
    case 77: return "Snow grains";
    case 80: return "Rain showers";
    case 81: return "Heavy showers";
    case 82: return "Violent showers";
    case 85: return "Snow showers";
    case 86: return "Heavy snow showers";
    case 95: return "Thunderstorm";
    case 96: return "Thunderstorm hail";
    case 99: return "Thunderstorm heavy hail";
    default: return "Unknown";
  }
}

static inline bool meteoCodeIsWet(int code) {
  return (code >= 51 && code <= 67) || (code >= 71 && code <= 77) ||
         (code >= 80 && code <= 86) || (code >= 95);
}

static float normalizeDegrees360(float deg) {
  if (!isfinite(deg)) return NAN;
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

static const char* meteoWindDirectionToCompass(float deg) {
  static const char* DIRS[16] = {
    "N", "NNE", "NE", "ENE",
    "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW",
    "W", "WNW", "NW", "NNW"
  };
  float norm = normalizeDegrees360(deg);
  if (!isfinite(norm)) return "";
  int idx = (int)floor((norm + 11.25f) / 22.5f) & 15;
  return DIRS[idx];
}

static String formatWindDirection(float deg) {
  float norm = normalizeDegrees360(deg);
  if (!isfinite(norm)) return String("--");
  char buf[24];
  snprintf(buf, sizeof(buf), "%s (%d deg)", meteoWindDirectionToCompass(norm), (int)lroundf(norm));
  return String(buf);
}

static void drawWindCompass(int cx, int cy, int r, float deg, uint16_t bg) {
  tft.fillRect(cx - r - 3, cy - r - 8, r * 2 + 7, r * 2 + 14, bg);
  tft.drawCircle(cx, cy, r, C_EDGE);
  tft.drawFastVLine(cx, cy - r + 2, r * 2 - 3, C_EDGE);
  tft.drawFastHLine(cx - r + 2, cy, r * 2 - 3, C_EDGE);

  tft.setTextSize(1);
  tft.setTextColor(C_MUTED);
  tft.setCursor(cx - 3, cy - r - 7);
  tft.print("N");

  float norm = normalizeDegrees360(deg);
  if (!isfinite(norm)) {
    tft.setTextColor(C_MUTED);
    tft.setCursor(cx - 3, cy - 3);
    tft.print("-");
    return;
  }

  float rad = norm * PI / 180.0f;
  int tipX = cx + (int)lroundf(sinf(rad) * (r - 2));
  int tipY = cy - (int)lroundf(cosf(rad) * (r - 2));
  int tailX = cx - (int)lroundf(sinf(rad) * 4.0f);
  int tailY = cy + (int)lroundf(cosf(rad) * 4.0f);
  tft.drawLine(tailX, tailY, tipX, tipY, C_ACCENT);
  tft.fillCircle(tipX, tipY, 2, C_ACCENT);

  const char* dir = meteoWindDirectionToCompass(norm);
  tft.setTextColor(C_TEXT);
  int16_t bx, by; uint16_t bw, bh;
  tft.getTextBounds(dir, 0, 0, &bx, &by, &bw, &bh);
  tft.setCursor(cx - (int)bw / 2, cy + r + 2);
  tft.print(dir);
}

static void drawWeatherCloud(int x, int y, int s, uint16_t color) {
  const int cy = y + s * 3 / 5;
  tft.fillCircle(x + s * 3 / 10, cy, max(2, s / 5), color);
  tft.fillCircle(x + s / 2, y + s * 9 / 20, max(3, s / 4), color);
  tft.fillCircle(x + s * 7 / 10, cy, max(2, s / 5), color);
  tft.fillRoundRect(x + s / 5, cy - s / 7, s * 3 / 5, s / 3, max(2, s / 8), color);
}

static void drawWeatherRain(int x, int y, int s, uint16_t color, bool heavy) {
  const int top = y + s * 7 / 10;
  const int len = max(4, s / 5);
  for (int i = 0; i < (heavy ? 4 : 3); ++i) {
    int dx = x + s / 5 + i * s / 5;
    tft.drawLine(dx, top + (i & 1), dx - s / 12, top + len + (i & 1), color);
  }
}

static void drawWeatherSnow(int x, int y, int s, uint16_t color) {
  const int top = y + s * 7 / 10;
  for (int i = 0; i < 3; ++i) {
    int cx = x + s / 4 + i * s / 4;
    int cy = top + ((i & 1) ? s / 8 : 0);
    tft.drawFastHLine(cx - 2, cy, 5, color);
    tft.drawFastVLine(cx, cy - 2, 5, color);
  }
}

static void drawWeatherIcon(int x, int y, int s, int code, uint16_t bg) {
  if (s < 18) s = 18;
  tft.fillRect(x, y, s, s, bg);

  const uint16_t sun = C_ACCENT;
  const uint16_t cloud = RGB(178, 194, 220);
  const uint16_t rain = RGB(72, 170, 255);
  const uint16_t snow = RGB(225, 245, 255);
  const uint16_t fog = RGB(150, 165, 190);
  const uint16_t storm = RGB(255, 215, 80);

  if (code < 0) {
    tft.drawCircle(x + s / 2, y + s / 2, s / 3, C_EDGE);
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(x + s / 2 - 3, y + s / 2 - 3);
    tft.print("?");
    return;
  }

  if (code == 0) {
    int cx = x + s / 2, cy = y + s / 2, r = s / 5;
    tft.fillCircle(cx, cy, r, sun);
    tft.drawFastHLine(x + 2, cy, s - 4, sun);
    tft.drawFastVLine(cx, y + 2, s - 4, sun);
    tft.drawLine(x + s / 5, y + s / 5, x + s * 4 / 5, y + s * 4 / 5, sun);
    tft.drawLine(x + s * 4 / 5, y + s / 5, x + s / 5, y + s * 4 / 5, sun);
  } else if (code == 1 || code == 2) {
    tft.fillCircle(x + s / 3, y + s / 3, s / 5, sun);
    drawWeatherCloud(x + s / 8, y + s / 5, s * 4 / 5, cloud);
  } else if (code == 3) {
    drawWeatherCloud(x + s / 10, y + s / 5, s * 4 / 5, cloud);
    drawWeatherCloud(x + s / 5, y + s / 8, s * 7 / 10, RGB(135, 152, 180));
  } else if (code == 45 || code == 48) {
    drawWeatherCloud(x + s / 10, y + s / 8, s * 4 / 5, cloud);
    for (int i = 0; i < 3; ++i) {
      int yy = y + s * 13 / 20 + i * s / 8;
      tft.drawFastHLine(x + s / 8, yy, s * 3 / 4, fog);
    }
  } else if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82)) {
    drawWeatherCloud(x + s / 10, y + s / 8, s * 4 / 5, cloud);
    drawWeatherRain(x + s / 10, y + s / 8, s * 4 / 5, rain, code >= 63 || code >= 81);
  } else if ((code >= 71 && code <= 77) || (code >= 85 && code <= 86)) {
    drawWeatherCloud(x + s / 10, y + s / 8, s * 4 / 5, cloud);
    drawWeatherSnow(x + s / 10, y + s / 8, s * 4 / 5, snow);
  } else if (code >= 95) {
    drawWeatherCloud(x + s / 10, y + s / 8, s * 4 / 5, cloud);
    tft.fillTriangle(x + s / 2, y + s * 11 / 20,
                     x + s * 2 / 5, y + s * 7 / 10,
                     x + s * 3 / 5, y + s * 7 / 10, storm);
    tft.fillTriangle(x + s * 3 / 5, y + s * 7 / 10,
                     x + s * 9 / 20, y + s * 19 / 20,
                     x + s * 7 / 10, y + s * 7 / 10, storm);
  } else {
    tft.drawCircle(x + s / 2, y + s / 2, s / 3, C_EDGE);
    tft.drawFastHLine(x + s / 3, y + s / 2, s / 3, C_MUTED);
  }
}

static time_t parseLocalIsoTime(const char* s) {
  if (!s) return 0;
  if (strlen(s) < 16) return 0;
  struct tm tmv = {};
  tmv.tm_year = atoi(s) - 1900;
  tmv.tm_mon  = atoi(s + 5) - 1;
  tmv.tm_mday = atoi(s + 8);
  tmv.tm_hour = atoi(s + 11);
  tmv.tm_min  = atoi(s + 14);
  tmv.tm_sec  = 0;
  tmv.tm_isdst = -1;
  time_t t = mktime(&tmv);
  return (t < 0) ? 0 : t;
}

static int readTankRaw() {
  if (!isValidAdcPin(tankLevelPin)) return -1;
  const int samples = 16;
  uint32_t total = 0;
  for (int i = 0; i < samples; ++i) {
    total += analogRead(tankLevelPin);
    delayMicroseconds(200);
  }
  return (int)(total / samples);
}

int tankPercent() {
  if (tankLevelPin == -1) return 0;
  if (!isValidAdcPin(tankLevelPin)) {
    static bool warned = false;
    if (!warned) {
      #if defined(CONFIG_IDF_TARGET_ESP32)
      Serial.printf("[TANK] Invalid ADC pin %d. Set tankLevelPin to GPIO32-39 for ESP32.\n", tankLevelPin);
      #else
      Serial.printf("[TANK] Invalid ADC pin %d. Set tankLevelPin to GPIO1-20 for ESP32-S3.\n", tankLevelPin);
      #endif
      warned = true;
    }
    return 0;
  }
  int raw = readTankRaw();
  if (raw < 0) return 0;
  int pct = map(raw, tankEmptyRaw, tankFullRaw, 0, 100);
  return constrain(pct, 0, 100);
}

bool isTankLow() {
  if (!tankEnabled) return true; // tank disabled => treat as low to force mains
  return tankPercent() <= tankLowThresholdPct;
}

int moistureRaw() {
  if (!moistureProbeEnabled || moistureUseMeteo) return -1;
  if (!isValidAdcPin(moisturePin)) return -1;
  const int N = 8;
  uint32_t acc = 0;
  for (int i = 0; i < N; i++) { acc += analogRead(moisturePin); delayMicroseconds(200); }
  return (int)(acc / N);
}

int moisturePercent() {
  if (!moistureProbeEnabled) return -1;
  if (moistureUseMeteo) {
    if (millis() - meteoMoistureFetchedMs >= 2UL * 60UL * 60UL * 1000UL) return -1;
    return meteoMoisturePct;
  }
  const int raw = moistureRaw();
  if (raw < 0) return -1;
  if (moistureDryRaw == moistureWetRaw) return -1;
  int pct = map(raw, moistureDryRaw, moistureWetRaw, 0, 100);
  return constrain(pct, 0, 100);
}

bool isSoilWetForSmartSkip() {
  if (!moistureProbeEnabled) return false;
  const int pct = moisturePercent();
  return (pct >= 0 && pct >= moistureSkipThresholdPct);
}

String sourceModeText() {
  if (!tankEnabled) return "Tank Disabled";
  if (justUseTank)  return "Force: Tank";
  if (justUseMains) return "Force: Mains";
  return isTankLow() ? "Auto: Mains" : "Auto: Tank";
}

static inline void chooseWaterSource(const char*& src, bool& mainsOn, bool& tankOn) {
  mainsOn = false; tankOn = false;
  if (!tankEnabled) { src = "Tank Disabled"; mainsOn = true; return; }
  if (justUseTank)  { src = "Tank";  tankOn  = true; return; }
  if (justUseMains) { src = "Mains"; mainsOn = true; return; }
  if (isTankLow())  { src = "Mains"; mainsOn = true; return; }
  src = "Tank"; tankOn = true;
}

bool physicalRainNowRaw() {
  if (!rainSensorEnabled) return false;
  if (rainSensorPin < 0 || rainSensorPin > 39) {
    static bool warned = false;
    if (!warned) {
      Serial.printf("[RAIN] Invalid rainSensorPin=%d; ignoring sensor.\n", rainSensorPin);
      warned = true;
    }
    return false;
  }
  pinMode(rainSensorPin, INPUT_PULLUP);
  int v = digitalRead(rainSensorPin); // LOW=dry, HIGH=wet (NC default)
  bool wet = (v == HIGH);
  if (rainSensorInvert) wet = !wet;
  return wet;
}

String rainDelayCauseText() {
  if (!rainDelayEnabled) return "Disabled";

  time_t now = time(nullptr);

  // Not currently raining, check for active cooldown with countdown
  if (!rainActive) {
    if (rainCooldownUntilEpoch && now < (time_t)rainCooldownUntilEpoch) {
      uint32_t rem = (uint32_t)(rainCooldownUntilEpoch - now); // seconds left
      // Round up to minutes so "59s" still shows as "1m"
      uint32_t mins = (rem + 59U) / 60U;

      char buf[40];

      if (mins >= 60U) {
        uint32_t h = mins / 60U;
        uint32_t m = mins % 60U;
        if (m > 0U) {
          // Example: "After-Rain Delay 1h 05m"
          snprintf(buf, sizeof(buf), "After-Rain Delay %luh %lum",
                   (unsigned long)h, (unsigned long)m);
        } else {
          // Example: "After-Rain Delay 2h"
          snprintf(buf, sizeof(buf), "After-Rain Delay %luh",
                   (unsigned long)h);
        }
      } else {
        // Under 1 hour: "After-Rain Delay 23m"
        snprintf(buf, sizeof(buf), "After-Rain Delay %lum",
                 (unsigned long)mins);
      }

      return String(buf);
    }

    if (!systemMasterEnabled) return "Master Off";
    if (isPausedNow())        return "Paused";
    if (windActive)           return "Windy";
    return "None";
  }

  // Currently in a rain-delay state
  if (rainByWeatherActive && rainBySensorActive) return "Both";
  if (rainByWeatherActive) return "Raining";
  if (rainBySensorActive)  return "Sensor Wet";
  return "Active";
}

static const char* kHost = "espirrigation";

static void mdnsStart() {
  MDNS.end(); // in case it was running
  if (MDNS.begin(kHost)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("arduino", "tcp", 3232); // keep OTA discoverable after restart
    Serial.println("[mDNS] started: http://espirrigation.local/");
  } else {
    Serial.println("[mDNS] begin() failed");
  }
}

// ---------- GPIO fallback helpers ----------
inline bool isValidOutputPin(int pin);
static bool anyZoneActive();

inline int gpioLevelForPolarity(bool on, bool activeLow) {
  if (activeLow) {
    // Active-LOW: LOW = ON, HIGH = OFF
    return on ? LOW : HIGH;
  } else {
    // Active-HIGH: HIGH = ON, LOW = OFF
    return on ? HIGH : LOW;
  }
}

inline int gpioLevel(bool on) {
  return gpioLevelForPolarity(on, gpioActiveLow);
}

inline void gpioInitOutput(int pin, bool activeLow) {
  if (!isValidOutputPin(pin)) return;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, gpioLevelForPolarity(false, activeLow));  // ensure OFF
}

inline void gpioZoneWrite(int z, bool on) {
  if (z < 0 || z >= (int)MAX_ZONES) return;
  int pin = zonePins[z];
  if (!isValidOutputPin(pin)) return;
  digitalWrite(pin, gpioLevelForPolarity(on, zoneGpioActiveLow[z]));
}

inline void gpioSourceWrite(bool mainsOn, bool tankOn) {
  bool mainsOk = isValidOutputPin(mainsPin);
  bool tankOk  = isValidOutputPin(tankPin);
  if (mainsOk) digitalWrite(mainsPin, gpioLevelForPolarity(mainsOn, mainsGpioActiveLow));
  if (tankOk)  digitalWrite(tankPin,  gpioLevelForPolarity(tankOn, tankGpioActiveLow));
}

inline void gpioPowerSupplyWrite(bool on) {
  if (!isValidOutputPin(powerSupplyPin)) return;
  digitalWrite(powerSupplyPin, gpioLevelForPolarity(on, powerSupplyActiveLow));
}

inline void setWaterSourceRelays(bool mainsOn, bool tankOn) {
  if (mainsOn || tankOn) gpioPowerSupplyWrite(true);

  // Prefer PCF outputs for the classic wiring; otherwise drive the configured GPIO pins.
  if (!useGpioFallback) {
    // PCF8574: active-LOW
    pcfOut.digitalWrite(mainsChannel, mainsOn ? LOW : HIGH);
    pcfOut.digitalWrite(tankChannel,  tankOn  ? LOW : HIGH);
  } else {
    gpioSourceWrite(mainsOn, tankOn);
  }

  if (!mainsOn && !tankOn && !anyZoneActive()) gpioPowerSupplyWrite(false);
}

inline bool useExpanderForZone(int z) {
  if (useGpioFallback) return false;
  // Reserve PCF channels P4/P5 for mains/tank; only zones 0-3 use the expander
  return (z >= 0 && z < 4);
}

// Duration helper: slot=1 (primary) or 2 (secondary) with fallback to primary
static inline unsigned long durationForSlot(int z, int slot) {
  if (z < 0 || z >= (int)MAX_ZONES) return 0;
  if (slot == 2 && enableStartTime2[z]) {
    unsigned long m = (duration2Min[z] >= 0) ? (unsigned long)duration2Min[z] : 0;
    unsigned long s = (duration2Sec[z] >= 0) ? (unsigned long)duration2Sec[z] : 0;
    unsigned long tot = m * 60UL + s;
    if (tot > 0) return tot;
  }
  unsigned long m = (durationMin[z] >= 0) ? (unsigned long)durationMin[z] : 0;
  unsigned long s = (durationSec[z] >= 0) ? (unsigned long)durationSec[z] : 0;
  return m * 60UL + s;
}

static float smartWateringReferenceTempC() {
  if (isfinite(curTempC)) return curTempC;
  if (isfinite(todayMax_C)) return todayMax_C;
  return NAN;
}

static float temperatureForDisplay(float tempC) {
  if (!isfinite(tempC) || !tempUseFahrenheit) return tempC;
  return tempC * 9.0f / 5.0f + 32.0f;
}

static char temperatureUnitChar() {
  return tempUseFahrenheit ? 'F' : 'C';
}

static float smartWateringFactor() {
  if (!smartWateringEnabled) return 1.0f;

  const float actualRain24h = last24hActualRain();
  const float forecastRain24h = isfinite(rainNext24h_mm) ? rainNext24h_mm : 0.0f;

  if (actualRain24h > smartActualRainSkipMm ||
      forecastRain24h > smartForecastRainSkipMm) return 0.0f;
  if (isSoilWetForSmartSkip()) return 0.0f;

  float factor = 1.0f;
  const float tempC = smartWateringReferenceTempC();
  if (isfinite(tempC)) {
    if (tempC < smartCoolTempC) factor *= (100.0f + smartCoolAdjustPct) / 100.0f;
    else if (tempC >= smartVeryHotTempC) factor *= (100.0f + smartVeryHotAdjustPct) / 100.0f;
    else if (tempC >= smartHotTempC) factor *= (100.0f + smartHotAdjustPct) / 100.0f;
  }

  if (actualRain24h > 0.0f && actualRain24h <= smartActualRainSkipMm) {
    factor *= (100.0f + smartLightRainAdjustPct) / 100.0f;
  }
  return (factor > 0.0f) ? factor : 0.0f;
}

static unsigned long smartWateringDurationForSlot(int z, int slot) {
  unsigned long base = durationForSlot(z, slot);
  if (!smartWateringEnabled || base == 0) return base;

  const float factor = smartWateringFactor();
  if (factor <= 0.0f) return 0;

  unsigned long adjusted = (unsigned long)lroundf((float)base * factor);
  return (adjusted > 0) ? adjusted : 1;
}

inline bool isValidAdcPin(int pin) {
  #if defined(CONFIG_IDF_TARGET_ESP32)
  // Classic ESP32: use ADC1 only so readings still work while Wi-Fi is active.
  return (pin >= 32 && pin <= 39);
  #else
  // ESP32-S3 ADC pins: GPIO1..20 (ADC1: 1-10, ADC2: 11-20)
  return (pin >= 1 && pin <= 20);
  #endif
}

inline bool isValidPhotoPin(int pin) {
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
  return (pin >= 1 && pin <= 40) && isValidGpioPin(pin);
  #else
  return isValidAdcPin(pin);
  #endif
}

inline bool isValidGpioPin(int pin) {
  if (pin < 0) return false;
  if (!GPIO_IS_VALID_GPIO((gpio_num_t)pin)) return false;
  #if defined(CONFIG_IDF_TARGET_ESP32)
  if (pin >= 6 && pin <= 11) return false; // SPI flash/PSRAM pins on classic ESP32
  #endif
  return true;
}

inline bool isValidOutputPin(int pin) {
  if (pin < 0) return false;
  if (!GPIO_IS_VALID_OUTPUT_GPIO((gpio_num_t)pin)) return false;
  #if defined(CONFIG_IDF_TARGET_ESP32)
  if (pin >= 6 && pin <= 11) return false; // SPI flash/PSRAM pins on classic ESP32
  #endif
  return true;
}

inline bool isUnsafeTftPin(int pin) {
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
  // ESP32-S3:
  // - 0/3/45/46 are strapping pins
  // - 26..32 are typically used for flash/PSRAM
  // - 33..37 are also reserved on octal PSRAM modules
  // GPIO19/20 are USB by default, but still usable as GPIO if USB is not needed.
  return (pin == 0 || pin == 3 || pin == 45 || pin == 46 ||
          (pin >= 26 && pin <= 32) || (pin >= 33 && pin <= 37));
  #else
  return (pin == 0 || pin == 19 || pin == 20 || pin == 45 || pin == 46 ||
          (pin >= 9 && pin <= 14) || (pin >= 35 && pin <= 38));
  #endif
}

inline bool isValidTftSignalPin(int pin) {
  return isValidOutputPin(pin) && !isUnsafeTftPin(pin);
}

inline bool isValidOptionalTftPin(int pin) {
  return (pin == -1) || isValidTftSignalPin(pin);
}

inline bool isValidTftDimension(int v) {
  return (v >= 120 && v <= 400);
}

static bool tryParseStrictInt(const String& raw, int &out) {
  String s = raw;
  s.trim();
  if (!s.length()) return false;
  char* end = nullptr;
  long v = strtol(s.c_str(), &end, 10);
  if (end == s.c_str() || *end != '\0') return false;
  out = (int)v;
  return true;
}

static bool parseRequiredIntArg(const String& raw, int &out, String &err, const __FlashStringHelper* label) {
  String s = raw;
  s.trim();
  if (!s.length()) {
    err += String(label) + " is blank\n";
    return false;
  }
  if (!tryParseStrictInt(s, out)) {
    err += String(label) + " is not a valid integer: " + s + "\n";
    return false;
  }
  return true;
}

static void warnPinConflict(const char* aName, int aPin, const char* bName, int bPin) {
  if (aPin < 0 || bPin < 0) return;
  if (aPin != bPin) return;
  Serial.printf("[PIN] Conflict: %s and %s both on GPIO%d\n", aName, bName, aPin);
}

static void sanitizePinConfig() {
  auto clampGpio = [](int &p) {
    if (p != -1 && !isValidGpioPin(p)) p = -1;
  };
  auto clampOutputGpio = [](int &p) {
    if (p != -1 && !isValidOutputPin(p)) p = -1;
  };

  // Zone + source GPIOs
  for (uint8_t i = 0; i < MAX_ZONES; i++) {
    clampOutputGpio(zonePins[i]);
  }
  clampOutputGpio(mainsPin);
  clampOutputGpio(tankPin);
  clampOutputGpio(powerSupplyPin);

  // Optional inputs
  clampGpio(rainSensorPin);
  clampGpio(manualSelectPin);
  clampGpio(manualStartPin);
  clampOutputGpio(dhtSensorPin);

  // ADC-only pins
  #if defined(CONFIG_IDF_TARGET_ESP32)
  if (tankLevelPin != -1 && !isValidAdcPin(tankLevelPin)) tankLevelPin = 34; // safe default on classic ESP32
  #else
  if (tankLevelPin != -1 && !isValidAdcPin(tankLevelPin)) tankLevelPin = 1;  // safe default on ESP32-S3
  #endif
  if (moisturePin != -1 && !isValidAdcPin(moisturePin)) moisturePin = -1;
  if (!isValidPhotoPin(photoPin)) photoPin = -1;

  // TFT pins: fall back to safe defaults if invalid/unsafe
  if (!isValidTftSignalPin(tftSclkPin)) tftSclkPin = TFT_SCLK_DEFAULT;
  if (!isValidTftSignalPin(tftMosiPin)) tftMosiPin = TFT_MOSI_DEFAULT;
  if (!isValidTftSignalPin(tftCsPin))   tftCsPin   = TFT_CS_DEFAULT;
  if (!isValidTftSignalPin(tftDcPin))   tftDcPin   = TFT_DC_DEFAULT;

  // Optional TFT pins
  if (!isValidOptionalTftPin(tftRstPin)) tftRstPin = TFT_RST_DEFAULT;
  if (!isValidOptionalTftPin(tftBlPin))  tftBlPin  = TFT_BL_DEFAULT;

  if (!isValidTftDimension(tftPanelWidth))  tftPanelWidth  = TFT_W_DEFAULT;
  if (!isValidTftDimension(tftPanelHeight)) tftPanelHeight = TFT_H_DEFAULT;

  // I2C pins
  if (!isValidGpioPin(i2cSdaPin)) i2cSdaPin = I2C_SDA_DEFAULT;
  if (!isValidGpioPin(i2cSclPin)) i2cSclPin = I2C_SCL_DEFAULT;
}

static void validatePinMap() {
  warnPinConflict("DHT22", dhtSensorPin, "I2C_SDA", i2cSdaPin);
  warnPinConflict("DHT22", dhtSensorPin, "I2C_SCL", i2cSclPin);
  warnPinConflict("DHT22", dhtSensorPin, "RainSensor", rainSensorPin);
  warnPinConflict("DHT22", dhtSensorPin, "TankLevel", tankLevelPin);
  warnPinConflict("DHT22", dhtSensorPin, "Moisture", moisturePin);
  warnPinConflict("DHT22", dhtSensorPin, "ManualSelect", manualSelectPin);
  warnPinConflict("DHT22", dhtSensorPin, "ManualStart", manualStartPin);
  warnPinConflict("DHT22", dhtSensorPin, "StatusPixel", STATUS_PIXEL_PIN);

  // I2C vs sensors
  warnPinConflict("I2C_SDA", i2cSdaPin, "RainSensor", rainSensorPin);
  warnPinConflict("I2C_SCL", i2cSclPin, "RainSensor", rainSensorPin);
  warnPinConflict("I2C_SDA", i2cSdaPin, "TankLevel", tankLevelPin);
  warnPinConflict("I2C_SCL", i2cSclPin, "TankLevel", tankLevelPin);

  // TFT vs board IO
  if (displayEnabled && displayUseTft) {
    warnPinConflict("DHT22", dhtSensorPin, "TFT_SCLK", tftSclkPin);
    warnPinConflict("DHT22", dhtSensorPin, "TFT_MOSI", tftMosiPin);
    warnPinConflict("DHT22", dhtSensorPin, "TFT_CS", tftCsPin);
    warnPinConflict("DHT22", dhtSensorPin, "TFT_DC", tftDcPin);
    warnPinConflict("DHT22", dhtSensorPin, "TFT_RST", tftRstPin);
    warnPinConflict("DHT22", dhtSensorPin, "TFT_BL", tftBlPin);
    warnPinConflict("TFT_CS",  tftCsPin,  "TankLevel", tankLevelPin);
    warnPinConflict("TFT_DC",  tftDcPin,  "LED",       LED_PIN);
    warnPinConflict("TFT_CS",  tftCsPin,  "LED",       LED_PIN);
    warnPinConflict("TFT_DC",  tftDcPin,  "TankLevel", tankLevelPin);
    warnPinConflict("TFT_SCLK",tftSclkPin,"RainSensor",rainSensorPin);
    warnPinConflict("TFT_MOSI",tftMosiPin,"RainSensor",rainSensorPin);
  }
  warnPinConflict("PowerSupply", powerSupplyPin, "I2C_SDA", i2cSdaPin);
  warnPinConflict("PowerSupply", powerSupplyPin, "I2C_SCL", i2cSclPin);
  warnPinConflict("PowerSupply", powerSupplyPin, "RainSensor", rainSensorPin);
  warnPinConflict("PowerSupply", powerSupplyPin, "TankLevel", tankLevelPin);
  if (displayEnabled && displayUseTft) {
    warnPinConflict("PowerSupply", powerSupplyPin, "TFT_SCLK", tftSclkPin);
    warnPinConflict("PowerSupply", powerSupplyPin, "TFT_MOSI", tftMosiPin);
    warnPinConflict("PowerSupply", powerSupplyPin, "TFT_CS", tftCsPin);
    warnPinConflict("PowerSupply", powerSupplyPin, "TFT_DC", tftDcPin);
    warnPinConflict("PowerSupply", powerSupplyPin, "TFT_RST", tftRstPin);
    warnPinConflict("PowerSupply", powerSupplyPin, "TFT_BL", tftBlPin);
  }
  warnPinConflict("PowerSupply", powerSupplyPin, "CityRelay", mainsPin);
  warnPinConflict("PowerSupply", powerSupplyPin, "TankRelay", tankPin);
  warnPinConflict("DHT22", dhtSensorPin, "PowerSupply", powerSupplyPin);
  warnPinConflict("DHT22", dhtSensorPin, "CityRelay", mainsPin);
  warnPinConflict("DHT22", dhtSensorPin, "TankRelay", tankPin);
  for (uint8_t i = 0; i < zonesCount && i < MAX_ZONES; ++i) {
    if (dhtSensorPin >= 0 && dhtSensorPin == zonePins[i]) {
      Serial.printf("[PIN] Conflict: DHT22 and Zone%u both on GPIO%d\n", i + 1, dhtSensorPin);
    }
    char zoneName[8];
    snprintf(zoneName, sizeof(zoneName), "Zone%u", (unsigned)(i + 1));
    warnPinConflict("PowerSupply", powerSupplyPin, zoneName, zonePins[i]);
  }

  // TFT self-collisions
  if (displayEnabled && displayUseTft) {
    warnPinConflict("TFT_SCLK",tftSclkPin,"TFT_MOSI", tftMosiPin);
    warnPinConflict("TFT_CS",  tftCsPin,  "TFT_DC",   tftDcPin);
    warnPinConflict("TFT_CS",  tftCsPin,  "TFT_RST",  tftRstPin);
    warnPinConflict("TFT_CS",  tftCsPin,  "TFT_BL",   tftBlPin);
  }
}

void statusPixelSet(uint8_t r,uint8_t g,uint8_t b) {
  if (!statusPixelReady) return;
  uint32_t color = statusPixel.Color(r,g,b);
  if (color == statusPixelLastColor) return;
  statusPixelLastColor = color;
  statusPixel.setPixelColor(0, color);
  statusPixel.show();
}

uint8_t statusPixelPulseLevel(uint16_t periodMs, uint8_t low, uint8_t high) {
  if (high <= low || periodMs < 2) return high;
  const uint32_t halfPeriod = periodMs / 2U;
  if (halfPeriod == 0) return high;
  const uint32_t phase = millis() % periodMs;
  const uint32_t ramp = (phase < halfPeriod) ? phase : (periodMs - phase);
  const uint32_t span = (uint32_t)(high - low);
  return (uint8_t)(low + ((span * ramp) / halfPeriod));
}

bool statusPixelWindowOn(uint16_t periodMs, uint16_t startMs, uint16_t widthMs) {
  if (periodMs == 0 || widthMs == 0 || startMs >= periodMs) return false;
  const uint32_t phase = millis() % periodMs;
  const uint32_t endMs = (uint32_t)startMs + (uint32_t)widthMs;
  if (endMs <= periodMs) return phase >= startMs && phase < endMs;
  return phase >= startMs || phase < (endMs - periodMs);
}

uint8_t statusLedDutyLevel() {
  uint8_t duty = statusPixelPulseLevel(1800, 10, 42);

  bool anyZoneOn = false;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) { anyZoneOn = true; break; }
  }

  const uint32_t sinceBootMs = millis() - bootMillis;
  if (sinceBootMs < 3000U) {
    duty = statusPixelPulseLevel(1200, 14, 80);
  } else if (WiFi.status() != WL_CONNECTED) {
    const bool on = statusPixelWindowOn(1100, 0, 120) || statusPixelWindowOn(1100, 180, 120);
    duty = on ? 220 : 0;
  } else if (anyZoneOn) {
    duty = 255;
  } else if (!systemMasterEnabled || isPausedNow()) {
    duty = statusPixelPulseLevel(1400, 6, 170);
  } else if (rainActive || windActive || isBlockedNow()) {
    const bool on = statusPixelWindowOn(1300, 0, 110) || statusPixelWindowOn(1300, 170, 110);
    duty = on ? 230 : 8;
  } else if (useGpioFallback) {
    duty = statusPixelPulseLevel(1500, 12, 95);
  }

  return duty;
}

#if __has_include("soc/soc_caps.h")
  #include "soc/soc_caps.h"
#endif

static const char* climateSourceValue() {
  switch (climateSource) {
    case CLIMATE_AHT20_I2C: return "aht20";
    case CLIMATE_DHT22_GPIO: return "dht22";
    case CLIMATE_OPEN_METEO:
    default: return "meteo";
  }
}

static const char* climateSourceLabel() {
  switch (climateSource) {
    case CLIMATE_AHT20_I2C: return "AHT20/AHT21 I2C";
    case CLIMATE_DHT22_GPIO: return "DHT22/AM2302 GPIO";
    case CLIMATE_OPEN_METEO:
    default: return "Open-Meteo";
  }
}

static LocalClimateSource parseClimateSource(const String& raw) {
  String s = raw;
  s.trim();
  s.toLowerCase();
  if (s == "2" || s == "aht" || s == "aht20" || s == "aht21") return CLIMATE_AHT20_I2C;
  if (s == "3" || s == "dht" || s == "dht22" || s == "am2302") return CLIMATE_DHT22_GPIO;
  return CLIMATE_OPEN_METEO;
}

static bool initLocalAhtSensor() {
  if (localAhtReady) return true;
  const uint32_t nowMs = millis();
  if (localAhtLastAttemptMs && nowMs - localAhtLastAttemptMs < 30000UL) return false;
  localAhtLastAttemptMs = nowMs;
  localAhtReady = localAht.begin(&I2Cbus);
  return localAhtReady;
}

static void resetLocalDhtSensor() {
  delete localDht;
  localDht = nullptr;
  localDhtActivePin = -1;
}

static bool initLocalDhtSensor() {
  if (!isValidOutputPin(dhtSensorPin)) return false;
  if (localDht && localDhtActivePin == dhtSensorPin) return true;

  resetLocalDhtSensor();
  localDht = new DHT((uint8_t)dhtSensorPin, DHT22);
  if (!localDht) return false;
  localDhtActivePin = dhtSensorPin;
  localDht->begin();
  return true;
}

static bool readLocalClimate(float& outTempC, float& outHumidityPct) {
  outTempC = NAN;
  outHumidityPct = NAN;

  if (climateSource == CLIMATE_AHT20_I2C) {
    if (!initLocalAhtSensor()) return false;
    sensors_event_t humidity;
    sensors_event_t temp;
    if (!localAht.getEvent(&humidity, &temp)) return false;
    outTempC = temp.temperature;
    outHumidityPct = humidity.relative_humidity;
    return isfinite(outTempC) || isfinite(outHumidityPct);
  }

  if (climateSource == CLIMATE_DHT22_GPIO) {
    if (!initLocalDhtSensor()) return false;
    outHumidityPct = localDht->readHumidity();
    outTempC = localDht->readTemperature(false);
    return isfinite(outTempC) || isfinite(outHumidityPct);
  }

  return false;
}

static void applyLocalClimateOverride(bool force = false) {
  if (climateSource == CLIMATE_OPEN_METEO) {
    localClimateValid = false;
    localTempC = NAN;
    localHumidityPct = NAN;
    return;
  }

  const uint32_t nowMs = millis();
  if (!force && localClimateLastReadMs && nowMs - localClimateLastReadMs < 5000UL) {
    return;
  }
  localClimateLastReadMs = nowMs;

  float t = NAN;
  float h = NAN;
  localClimateValid = readLocalClimate(t, h);
  localTempC = t;
  localHumidityPct = h;

  if (localClimateValid && isfinite(t)) {
    curTempC = t;
    curFeelsC = t;
    curWeatherValid = true;
  }
  if (localClimateValid && isfinite(h)) {
    curHumidityPct = constrain((int)lroundf(h), 0, 100);
    curWeatherValid = true;
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

static bool g_tftInvert = false;
static bool g_tftBlOn   = true;
static bool g_tftDisplayOn = true;
static bool g_tftPwmReady = false;
static uint8_t g_tftBrightness = 125; // 0-255 duty when ON
static const int TFT_PWM_CH = 7;      // LEDC channel for TFT BL
static bool g_statusLedPwmReady = false;
static uint8_t g_statusLedLastDuty = 255;
static const int STATUS_LED_PWM_CH = 6;
static bool g_forceHomeReset = false; // force full HomeScreen repaint
static bool g_forceRainReset = false; // force full RainScreen repaint
static bool g_forceManualReset = false; // force full Manual screen repaint
static bool g_forceRunReset = false; // force full Running screen repaint
static const uint32_t DELAY_SCREEN_SCROLL_MS = 7000;

static bool anyZoneActive() {
  for (int i = 0; i < (int)zonesCount && i < (int)MAX_ZONES; ++i) {
    if (zoneActive[i]) return true;
  }
  return false;
}

// ---------- LEDC PWM compatibility (ESP32 Arduino core 2.x vs 3.x) ----------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static bool ledcAttachCompat(int pin, uint32_t freq, uint8_t resBits, int /*channel*/) {
  return ledcAttach(pin, freq, resBits);
}
static void ledcWriteCompat(int pin, uint32_t duty, int /*channel*/) {
  ledcWrite(pin, duty);
}
#else
static bool ledcAttachCompat(int pin, uint32_t freq, uint8_t resBits, int channel) {
  double actual = ledcSetup(channel, freq, resBits);
  if (actual <= 0) return false;
  ledcAttachPin(pin, channel);
  return true;
}
static void ledcWriteCompat(int /*pin*/, uint32_t duty, int channel) {
  ledcWrite(channel, duty);
}
#endif

static inline uint8_t statusLedPhysicalDuty(uint8_t logicalDuty) {
  return STATUS_LED_IS_ACTIVE_LOW ? (uint8_t)(255U - logicalDuty) : logicalDuty;
}

void initStatusLedPwm() {
  g_statusLedPwmReady = false;
  g_statusLedLastDuty = 255;
  if (LED_PIN < 0) return;
  if (!isValidGpioPin(LED_PIN)) {
    Serial.printf("[LED] Invalid LED_PIN=%d; PWM status LED disabled.\n", LED_PIN);
    return;
  }
  const uint32_t freq = 5000;
  const uint8_t resBits = 8;
  if (!ledcAttachCompat(LED_PIN, freq, resBits, STATUS_LED_PWM_CH)) {
    Serial.println("[LED] LEDC setup failed; PWM status LED disabled.");
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, STATUS_LED_IS_ACTIVE_LOW ? HIGH : LOW);
    return;
  }
  g_statusLedPwmReady = true;
  ledcWriteCompat(LED_PIN, statusLedPhysicalDuty(0), STATUS_LED_PWM_CH);
}

void updateStatusLedPwm() {
  if (!g_statusLedPwmReady) return;
  const uint8_t logicalDuty = statusLedDutyLevel();
  if (logicalDuty == g_statusLedLastDuty) return;
  g_statusLedLastDuty = logicalDuty;
  ledcWriteCompat(LED_PIN, statusLedPhysicalDuty(logicalDuty), STATUS_LED_PWM_CH);
}

static void tftInitBacklightPwm() {
  if (!displayEnabled) return;
  if (!displayUseTft) return;
  g_tftPwmReady = false;
  if (tftBlPin < 0) return;
  if (!isValidGpioPin(tftBlPin)) {
    Serial.printf("[TFT] Invalid tftBlPin=%d; disabling backlight pin.\n", tftBlPin);
    tftBlPin = -1;
    return;
  }
  const uint32_t freq = 5000;
  const uint8_t resBits = 8;
  if (!ledcAttachCompat(tftBlPin, freq, resBits, TFT_PWM_CH)) {
    Serial.println("[TFT] LEDC setup failed; PWM backlight disabled.");
    return;
  }
  g_tftPwmReady = true;
  ledcWriteCompat(tftBlPin, g_tftBlOn ? g_tftBrightness : 0, TFT_PWM_CH);
}

static inline void tftDisplay(bool on){
  if (!displayEnabled) return;
  if (!displayUseTft) return;
  if (g_tftDisplayOn == on) return;
  if (on) {
    // Sleep OUT then Display ON (common ST7789/ILI9341 sequence).
    tft.sendCommand(0x11);
    delay(120);
    tft.sendCommand(0x29);
    delay(20);
  } else {
    // Display OFF before Sleep IN to reduce panel power.
    tft.sendCommand(0x28);
    delay(10);
    tft.sendCommand(0x10);
    delay(120);
  }
  g_tftDisplayOn = on;
}

static inline void tftBacklight(bool on){
  if (!displayEnabled) return;
  if (!displayUseTft) return;
  if (tftBlPin >= 0) {
    if (!isValidGpioPin(tftBlPin)) {
      static bool warned = false;
      if (!warned) {
        Serial.printf("[TFT] Invalid tftBlPin=%d; disabling backlight pin.\n", tftBlPin);
        warned = true;
      }
      tftBlPin = -1;
      // Continue and use panel sleep/wake commands.
    }
    if (tftBlPin >= 0) {
      if (g_tftPwmReady) {
        ledcWriteCompat(tftBlPin, on ? g_tftBrightness : 0, TFT_PWM_CH);
      } else {
        pinMode(tftBlPin, OUTPUT);
        digitalWrite(tftBlPin, on ? HIGH : LOW);   // most modules: HIGH = on
      }
      g_tftBlOn = on;
    }
  }
  tftDisplay(on);
}

static void tftSetBrightness(uint8_t pct){ // pct: 0-100
  if (!displayEnabled) return;
  if (!displayUseTft) return;
  if (pct > 100) pct = 100;
  uint8_t duty = map(pct, 0, 100, 0, 255);
  g_tftBrightness = duty;
  if (tftBlPin >= 0) {
    if (g_tftPwmReady) {
      ledcWriteCompat(tftBlPin, g_tftBlOn ? duty : 0, TFT_PWM_CH);
    } else {
      // if no PWM, fall back to on/off at threshold
      digitalWrite(tftBlPin, (pct > 0) ? HIGH : LOW);
    }
  }
}

static void markTftScreensDirty() {
  g_forceHomeReset = true;
  g_forceRainReset = true;
  g_forceManualReset = true;
  g_forceRunReset = true;
  lastScreenRefresh = 0;
}

static void tftTextMode(uint8_t size, uint16_t fg, uint16_t bg = C_BG) {
  tft.setTextWrap(false);
  tft.setTextSize(size);
  tft.setTextColor(fg, bg);
}

static void tftPrintCentered(const String& text, int y, uint8_t size, uint16_t color) {
  int16_t x1, y1;
  uint16_t w, h;
  tftTextMode(size, color);
  tft.getTextBounds(text.c_str(), 0, 0, &x1, &y1, &w, &h);
  int x = (tft.width() - (int)w) / 2;
  if (x < 0) x = 0;
  tft.setCursor(x, y);
  tft.print(text);
}

static bool initTftDisplay() {
  if (!displayEnabled || !displayUseTft) return false;

  if (!isValidTftSignalPin(tftSclkPin) ||
      !isValidTftSignalPin(tftMosiPin) ||
      !isValidTftSignalPin(tftCsPin) ||
      !isValidTftSignalPin(tftDcPin) ||
      !isValidOptionalTftPin(tftRstPin) ||
      !isValidOptionalTftPin(tftBlPin) ||
      !isValidTftDimension(tftPanelWidth) ||
      !isValidTftDimension(tftPanelHeight)) {
    Serial.println("[TFT] Invalid TFT configuration; display disabled.");
    displayEnabled = false;
    displayUseTft = false;
    return false;
  }

  tft = Adafruit_ST7789(&SPI, tftCsPin, tftDcPin, tftRstPin);
  SPI.begin(tftSclkPin, -1, tftMosiPin, tftCsPin);
  tftInitBacklightPwm();
  tft.init(tftPanelWidth, tftPanelHeight);
  tft.setRotation(tftRotation);
  tft.setTextWrap(false);
  tft.fillScreen(C_BG);
  tftBacklight(true);
  markTftScreensDirty();
  return true;
}

static void showTftBootSplash() {
  if (!displayEnabled || !displayUseTft) return;
  const int H = tft.height();
  const int titleSize = (tft.width() >= 300) ? 3 : 2;
  tft.fillScreen(C_BG);
  tftPrintCentered("Irrigation", max(8, H / 8), titleSize, C_ACCENT);
  tftPrintCentered("AP: ESPIrrigationAP", max(44, H / 3), 1, C_TEXT);
  tftPrintCentered("http://192.168.4.1", max(62, H / 3 + 18), 1, C_MUTED);
}

static void showTftConnectedScreen() {
  if (!displayEnabled || !displayUseTft) return;
  const int H = tft.height();
  tft.fillScreen(C_BG);
  tftPrintCentered("Connected", max(8, H / 10), (tft.width() >= 300) ? 3 : 2, C_ACCENT);
  tftPrintCentered("IP " + WiFi.localIP().toString(), max(42, H / 3), 1, C_TEXT);
  tftPrintCentered("espirrigation.local", max(60, H / 3 + 18), 1, C_MUTED);
  String signal = "RSSI " + String(WiFi.RSSI()) + " dBm";
  tftPrintCentered(signal, max(78, H / 3 + 36), 1, C_MUTED);
  markTftScreensDirty();
}

static void tickAutoBacklight(){
  if (!displayEnabled) return;
  if (!displayUseTft) return;
  if (!photoAutoEnabled) return;
  if (!isValidPhotoPin(photoPin)) return;

  static uint32_t lastMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastMs < 1000) return;
  lastMs = nowMs;

  const int N = 4;
  uint32_t acc = 0;
  for (int i = 0; i < N; i++) { acc += analogRead(photoPin); delayMicroseconds(200); }
  int raw = (int)(acc / N);

  const int hysteresis = 50;
  int low = photoThreshold - hysteresis;
  int high = photoThreshold + hysteresis;
  if (low < 0) low = 0;
  if (high > 4095) high = 4095;

  static bool autoOn = true;
  static uint32_t darkSinceMs = 0;
  const uint32_t DARK_HOLD_MS = 2000; // keep screen on for 10s after it goes dark

  if (!photoInvert) {
    if (raw < low) {
      if (darkSinceMs == 0) darkSinceMs = nowMs;
      if ((nowMs - darkSinceMs) >= DARK_HOLD_MS) autoOn = false;
    } else {
      darkSinceMs = 0;
      if (raw > high) autoOn = true;
    }
  } else {
    if (raw > high) {
      if (darkSinceMs == 0) darkSinceMs = nowMs;
      if ((nowMs - darkSinceMs) >= DARK_HOLD_MS) autoOn = false;
    } else {
      darkSinceMs = 0;
      if (raw < low) autoOn = true;
    }
  }

  bool curOn = (tftBlPin >= 0) ? g_tftBlOn : g_tftDisplayOn;
  if (autoOn != curOn) {
    tftBacklight(autoOn);
  }
}

static const char* resetReasonText(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "power_on";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "interrupt_watchdog";
    case ESP_RST_TASK_WDT:  return "task_watchdog";
    case ESP_RST_WDT:       return "other_watchdog";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "sdio";
    default:                return "unknown";
  }
}

void handleDiagnosticsJson() {
  HttpScope _scope;

  JsonDocument doc;
  const uint32_t nowMs = millis();
  const uint32_t uptimeSec = (nowMs - bootMillis) / 1000UL;
  const time_t nowEpoch = time(nullptr);

  doc["device"] = kHost;
  doc["signature"] = kFirmwareSignature;
  doc["firmwareVersion"] = kFirmwareVersion;
  doc["firmwareBuild"] = kFirmwareBuildDate;
  doc["uptimeSec"] = uptimeSec;
  doc["uptime"] = formatRuntimeClock(uptimeSec);
  doc["epoch"] = (uint32_t)nowEpoch;
  doc["sdk"] = ESP.getSdkVersion();
  doc["chipModel"] = ESP.getChipModel();
  doc["chipRevision"] = ESP.getChipRevision();
  doc["cpuMHz"] = ESP.getCpuFreqMHz();
  doc["flashSize"] = ESP.getFlashChipSize();
  doc["sketchSize"] = ESP.getSketchSize();
  doc["freeSketchSpace"] = ESP.getFreeSketchSpace();
  doc["resetReason"] = resetReasonText(esp_reset_reason());
  doc["resetReasonCode"] = (int)esp_reset_reason();
  doc["bootCount"] = bootCount;

  JsonObject heap = doc["heap"].to<JsonObject>();
  heap["free"] = ESP.getFreeHeap();
  heap["minFree"] = ESP.getMinFreeHeap();
  heap["maxAlloc"] = ESP.getMaxAllocHeap();
  heap["psramSize"] = ESP.getPsramSize();
  heap["psramFree"] = ESP.getFreePsram();

  JsonObject fs = doc["littleFs"].to<JsonObject>();
  fs["total"] = LittleFS.totalBytes();
  fs["used"] = LittleFS.usedBytes();
  fs["configExists"] = LittleFS.exists("/config.txt");
  fs["configLoaded"] = g_configLoadedFromFs;
  fs["scheduleExists"] = LittleFS.exists("/schedule.txt");
  fs["eventsExists"] = LittleFS.exists("/events.csv");

  JsonObject wifi = doc["wifi"].to<JsonObject>();
  wifi["connected"] = (WiFi.status() == WL_CONNECTED);
  wifi["ssid"] = WiFi.SSID();
  wifi["ip"] = WiFi.localIP().toString();
  wifi["rssi"] = WiFi.RSSI();
  wifi["mac"] = WiFi.macAddress();
  wifi["hostname"] = WiFi.getHostname();

  JsonObject mqtt = doc["mqtt"].to<JsonObject>();
  mqtt["enabled"] = mqttEnabled;
  mqtt["connected"] = _mqtt.connected();
  mqtt["broker"] = mqttBroker;
  mqtt["port"] = mqttPort;
  mqtt["baseTopic"] = mqttBase;
  mqtt["lastState"] = _mqtt.state();
  mqtt["reconnectDelaySec"] = _mqttReconnectDelayMs / 1000UL;
  mqtt["reconnectFailures"] = _mqttReconnectFailures;

  JsonObject io = doc["io"].to<JsonObject>();
  io["i2cSda"] = i2cSdaPin;
  io["i2cScl"] = i2cSclPin;
  io["dht22Pin"] = dhtSensorPin;
  io["gpioFallback"] = useGpioFallback;
  io["i2cFailCount"] = i2cFailCount;
  JsonArray devices = io["i2cDevices"].to<JsonArray>();
  for (uint8_t addr = 1; addr < 127; ++addr) {
    I2Cbus.beginTransmission(addr);
    if (I2Cbus.endTransmission() == 0) {
      char buf[6];
      snprintf(buf, sizeof(buf), "0x%02X", addr);
      devices.add(buf);
    }
  }

  JsonObject display = doc["display"].to<JsonObject>();
  display["type"] = !displayEnabled ? "none" : (displayUseTft ? "tft" : "oled");
  display["tftWidth"] = tftPanelWidth;
  display["tftHeight"] = tftPanelHeight;
  display["tftRotation"] = tftRotation;
  display["backlightPin"] = tftBlPin;
  display["backlightOn"] = g_tftBlOn;
  display["displayOn"] = g_tftDisplayOn;
  display["pwm"] = g_tftPwmReady;
  display["brightnessPct"] = (int)lroundf((float)g_tftBrightness * 100.0f / 255.0f);

  JsonObject pins = doc["pins"].to<JsonObject>();
  JsonObject busPins = pins["buses"].to<JsonObject>();
  busPins["i2cSda"] = i2cSdaPin;
  busPins["i2cScl"] = i2cSclPin;
  busPins["tftSclk"] = tftSclkPin;
  busPins["tftMosi"] = tftMosiPin;
  busPins["tftCs"] = tftCsPin;
  busPins["tftDc"] = tftDcPin;
  busPins["tftRst"] = tftRstPin;
  busPins["tftBacklight"] = tftBlPin;
  busPins["statusPixel"] = STATUS_PIXEL_PIN;
  JsonObject relayPins = pins["relays"].to<JsonObject>();
  relayPins["gpioFallback"] = useGpioFallback;
  relayPins["powerSupply"] = powerSupplyPin;
  relayPins["powerSupplyActiveLow"] = powerSupplyActiveLow;
  relayPins["mainsGpio"] = mainsPin;
  relayPins["mainsActiveLow"] = mainsGpioActiveLow;
  relayPins["mainsPcfChannel"] = mainsChannel;
  relayPins["tankGpio"] = tankPin;
  relayPins["tankActiveLow"] = tankGpioActiveLow;
  relayPins["tankPcfChannel"] = tankChannel;
  JsonArray zonePinArray = pins["zones"].to<JsonArray>();
  for (int i = 0; i < MAX_ZONES; ++i) {
    JsonObject z = zonePinArray.add<JsonObject>();
    z["zone"] = i + 1;
    z["name"] = zoneNames[i];
    z["enabled"] = i < (int)zonesCount;
    z["gpio"] = zonePins[i];
    z["activeLow"] = zoneGpioActiveLow[i];
    if (i < ALL_P) z["pcfChannel"] = PCH[i];
    else z["pcfChannel"] = nullptr;
  }
  JsonObject sensorPins = pins["sensors"].to<JsonObject>();
  sensorPins["tankLevel"] = tankLevelPin;
  sensorPins["rainSensor"] = rainSensorPin;
  sensorPins["rainSensorEnabled"] = rainSensorEnabled;
  sensorPins["rainSensorInvert"] = rainSensorInvert;
  sensorPins["moisture"] = moisturePin;
  sensorPins["moistureEnabled"] = moistureProbeEnabled;
  sensorPins["photo"] = photoPin;
  sensorPins["photoAutoEnabled"] = photoAutoEnabled;
  sensorPins["photoInvert"] = photoInvert;
  JsonObject buttonPins = pins["buttons"].to<JsonObject>();
  buttonPins["manualSelect"] = manualSelectPin;
  buttonPins["manualStart"] = manualStartPin;

  JsonObject water = doc["water"].to<JsonObject>();
  water["zonesCount"] = zonesCount;
  water["sourceMode"] = sourceModeText();
  water["tankEnabled"] = tankEnabled;
  water["tankPct"] = tankPercent();
  water["tankLow"] = isTankLow();
  water["moistureEnabled"] = moistureProbeEnabled;
  water["moistureSource"] = moistureUseMeteo ? "meteo" : "probe";
  water["moisturePct"] = moisturePercent();
  water["moistureRaw"] = moistureRaw();
  water["moistureSkip"] = isSoilWetForSmartSkip();
  water["masterOn"] = systemMasterEnabled;
  water["paused"] = isPausedNow();
  water["rainDelayActive"] = isRainDelayBlockingNow();
  water["rainActive"] = rainActive;
  water["rainCooldownActive"] = isCooldownActiveNow();
  water["windDelayActive"] = windActive;
  water["rainDelayCause"] = rainDelayCauseText();

  JsonObject weather = doc["weather"].to<JsonObject>();
  weather["location"] = meteoLocationLabel();
  weather["model"] = meteoModel;
  weather["climateSource"] = climateSourceValue();
  weather["climateSourceLabel"] = climateSourceLabel();
  weather["dht22Pin"] = dhtSensorPin;
  weather["localClimateValid"] = localClimateValid;
  if (isfinite(localTempC)) weather["localTempC"] = localTempC;
  else                      weather["localTempC"] = nullptr;
  if (isfinite(localHumidityPct)) weather["localHumidity"] = localHumidityPct;
  else                            weather["localHumidity"] = nullptr;
  weather["currentValid"] = curWeatherValid;
  weather["lastCurrentHttp"] = lastWeatherHttpCode;
  weather["lastForecastHttp"] = lastForecastHttpCode;
  weather["lastCurrentAgeSec"] = lastWeatherUpdate ? (nowMs - lastWeatherUpdate) / 1000UL : 0;
  weather["lastForecastAgeSec"] = lastForecastUpdate ? (nowMs - lastForecastUpdate) / 1000UL : 0;
  weather["httpTimeoutMs"] = meteoHttpTimeoutMs();
  if (lastWeatherError.length()) weather["lastCurrentError"] = lastWeatherError;
  if (lastForecastError.length()) weather["lastForecastError"] = lastForecastError;

  String out;
  out.reserve(5200);
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleDiagnosticsPage() {
  HttpScope _scope;

  const uint32_t nowMs = millis();
  const uint32_t uptimeSec = (nowMs - bootMillis) / 1000UL;
  const uint32_t fsTotal = LittleFS.totalBytes();
  const uint32_t fsUsed = LittleFS.usedBytes();
  const int rssi = WiFi.RSSI();
  const String rssiLabel = wifiSignalQuality(rssi);
  const int activeZones = []() {
    int n = 0;
    for (int i = 0; i < (int)zonesCount; ++i) if (zoneActive[i]) ++n;
    return n;
  }();

  String i2cList;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    I2Cbus.beginTransmission(addr);
    if (I2Cbus.endTransmission() == 0) {
      char buf[8];
      snprintf(buf, sizeof(buf), "%s0x%02X", i2cList.length() ? " " : "", addr);
      i2cList += buf;
    }
  }
  if (!i2cList.length()) i2cList = "none";

  String html;
  html.reserve(12000);
  auto pinText = [](int pin) -> String {
    return pin < 0 ? String("disabled") : String("GPIO") + String(pin);
  };
  auto activeText = [](bool activeLow) -> const char* {
    return activeLow ? "active LOW" : "active HIGH";
  };
  auto appendPinRow = [&](const String& assignment, const String& pin, const String& notes) {
    html += F("<tr><td>");
    html += assignment;
    html += F("</td><td><code>");
    html += pin;
    html += F("</code></td><td>");
    html += notes;
    html += F("</td></tr>");
  };
  html += F("<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<!-- ");
  html += kFirmwareSignature;
  html += F(" -->");
  html += F("<title>ESP32 Diagnostics</title><style>");
  html += F(":root{color-scheme:light dark;--bg:#f4f7fb;--panel:#fff;--ink:#172033;--muted:#68758a;--line:#dce4ef;--brand:#2563eb;--brand-dark:#1e3a8a;--ok:#2563eb;--warn:#b45309;--bad:#b91c1c}");
  html += F("@media(prefers-color-scheme:dark){:root{--bg:#0b1220;--panel:#111c2e;--ink:#e7eef9;--muted:#9aa9bd;--line:#293a54;--brand:#60a5fa;--brand-dark:#1d4ed8}}");
  html += F("*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font-family:'Segoe UI',Arial,sans-serif;line-height:1.45;-webkit-font-smoothing:antialiased}.wrap{max-width:1180px;margin:0 auto;padding:20px}");
  html += F(".nav{display:flex;justify-content:space-between;gap:16px;align-items:center;flex-wrap:wrap;margin:-20px -20px 24px;padding:16px 20px;background:var(--brand-dark);color:#fff;box-shadow:0 6px 18px rgba(15,23,42,.18)}.brand{font-weight:800;font-size:1.3rem}.links{display:flex;gap:8px;flex-wrap:wrap}");
  html += F("a,.btn{display:inline-flex;align-items:center;justify-content:center;border:1px solid var(--line);border-radius:7px;padding:8px 12px;color:inherit;text-decoration:none;background:var(--panel);font-weight:700;cursor:pointer;transition:filter .15s,transform .08s}a:hover,.btn:hover{filter:brightness(1.08)}a:active,.btn:active{transform:translateY(1px)}.links a{border-color:rgba(255,255,255,.28);background:rgba(255,255,255,.1);color:#fff;font-size:.9rem}.links a:last-child{background:#fff;color:var(--brand-dark)}");
  html += F(".grid{display:grid;gap:14px}.diag-grid{grid-template-columns:repeat(12,minmax(0,1fr));align-items:stretch;grid-auto-flow:row dense}.card{min-width:0;background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:16px;box-shadow:0 4px 14px rgba(15,23,42,.055)}.status-card{grid-column:span 4}.weather-card{grid-column:span 8}.pin-card{grid-column:span 6;overflow:hidden}.card-wide{grid-column:1/-1}");
  html += F("h1,h2{margin:0}h1{font-size:1.55rem}h2{font-size:1rem}.card-head{display:flex;align-items:center;justify-content:space-between;gap:12px;min-height:34px;margin:-2px 0 4px;padding-bottom:10px;border-bottom:1px solid var(--line)}.card-tag{flex:0 0 auto;color:var(--muted);font-size:.7rem;font-weight:750;text-transform:uppercase;letter-spacing:.08em}.k{color:var(--muted);font-size:.76rem;text-transform:uppercase;letter-spacing:.09em;font-weight:750}.v{font-size:1.35rem;font-weight:800;overflow-wrap:anywhere}.card .k+.v{margin-top:4px}");
  html += F("table{width:100%;border-collapse:collapse;font-size:.88rem}.status-card table,.weather-card table{table-layout:fixed}td,th{padding:8px 0;border-top:1px solid var(--line);vertical-align:top}tbody tr:first-child td{border-top:0}th{text-align:left;color:var(--muted);font-size:.68rem;text-transform:uppercase;letter-spacing:.08em}td:first-child{width:44%;color:var(--muted);padding-right:14px}td:last-child{text-align:right;font-weight:700;overflow-wrap:anywhere}.pin-table th:nth-child(2),.pin-table td:nth-child(2){width:104px;text-align:left}.pin-table th:nth-child(3),.pin-table td:nth-child(3){padding-left:16px}.pin-table td:first-child{width:auto;color:var(--ink);font-weight:700}.pin-table td:last-child{text-align:left;font-weight:500;color:var(--muted)}.pin-table td:nth-child(2){font-weight:800;white-space:nowrap}.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}code{display:inline-block;max-width:100%;border:1px solid var(--line);border-radius:5px;padding:2px 6px;background:var(--bg);font-family:Consolas,monospace;font-size:.82rem;color:var(--ink);white-space:normal;overflow-wrap:anywhere}");
  html += F("@media(max-width:820px){.status-card,.weather-card{grid-column:span 6}.hardware-card{grid-column:1/-1}.pin-card{grid-column:1/-1}}@media(max-width:640px){.wrap{padding:12px}.nav{margin:-12px -12px 16px;padding:14px 12px;align-items:flex-start}.links{width:100%;display:grid;grid-template-columns:repeat(4,1fr);gap:6px}.links a{padding:7px 5px;font-size:.8rem;white-space:nowrap}.diag-grid{grid-template-columns:1fr;gap:10px}.status-card,.weather-card,.pin-card,.card-wide,.hardware-card{grid-column:1}.card{padding:14px}h1{font-size:1.35rem}.card-head{margin-top:0}.pin-table thead{display:none}.pin-table tr{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:2px 10px;padding:9px 0;border-top:1px solid var(--line)}.pin-table tbody tr:first-child{border-top:0}.pin-table td{display:block;width:auto!important;padding:0!important;border:0}.pin-table td:nth-child(2){text-align:right}.pin-table td:nth-child(3){grid-column:1/-1;font-size:.82rem}.pin-table td:last-child{text-align:left}td,th{padding:8px 0}}</style></head><body><main class='wrap'>");
  html += F("<div class='nav'><div><div class='k'>Controller Health</div><h1>Diagnostics</h1></div><div class='links'><a href='/'>Home</a><a href='/setup'>Setup</a><a href='/events'>Events</a><a href='/diagnostics.json'>JSON</a></div></div>");
  html += F("<style>.metric-grid{grid-template-columns:repeat(5,minmax(0,1fr));align-items:stretch}.metric-card{min-height:104px;display:flex;flex-direction:column;justify-content:space-between;overflow:hidden}.metric-card .v{font-size:1.14rem}.metric-card .k{font-size:.68rem}.metric-value-row{display:flex;align-items:center;gap:9px}.btn-small{padding:3px 7px;border-radius:5px;font-size:.68rem;line-height:1.2;color:var(--muted)}.metric-card:before{content:'';display:block;width:42px;height:3px;border-radius:3px;background:var(--brand);margin:-3px 0 12px}.metric-card:nth-child(2):before{background:var(--warn)}.metric-card:nth-child(3):before{background:var(--ok)}.metric-card:nth-child(4):before{background:#2563eb}.metric-card:nth-child(5):before{background:#0891b2}.section-title{grid-column:1/-1;display:flex;align-items:center;gap:12px;margin:18px 0 0;color:var(--ink);font-size:.82rem;text-transform:uppercase;letter-spacing:.1em;font-weight:800}.section-title:after{content:'';height:1px;flex:1;background:var(--line)}.status{display:inline-flex;align-items:center;gap:7px}.status:before{content:'';flex:0 0 auto;width:8px;height:8px;border-radius:50%;background:currentColor}.grid+.grid{margin-top:14px}@media(max-width:980px){.metric-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.metric-grid .metric-card:first-child{grid-column:1/-1}}@media(max-width:480px){.metric-grid{grid-template-columns:1fr}.metric-grid .metric-card:first-child{grid-column:1}.metric-card{min-height:88px}.metric-card .v{font-size:1.08rem}}</style>");

  html += F("<section class='grid metric-grid'>");
  html += F("<div class='card metric-card'><div class='k'>Uptime</div><div class='v'>"); html += formatRuntimeClock(uptimeSec); html += F("</div></div>");
  html += F("<div class='card metric-card'><div class='k'>Boot Count</div><div class='metric-value-row'><div id='diagBootCount' class='v'>"); html += String(bootCount); html += F("</div><button id='resetBootCount' class='btn btn-small' type='button' title='Set boot count to zero'>Reset</button></div><div class='k'>Reset: "); html += resetReasonText(esp_reset_reason()); html += F("</div></div>");
  html += F("<div class='card metric-card'><div class='k'>WiFi</div><div id='diagWifiIp' class='v status ");
  html += (WiFi.status() == WL_CONNECTED ? "ok" : "bad"); html += F("'>");
  html += (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("Disconnected")); html += F("</div><div class='k' id='diagWifiRssi'>RSSI ");
  html += String(rssi); html += F(" dBm - "); html += rssiLabel; html += F("</div></div>");
  html += F("<div class='card metric-card'><div class='k'>Heap Free</div><div class='v'>"); html += String(ESP.getFreeHeap()); html += F(" B</div><div class='k'>Min ");
  html += String(ESP.getMinFreeHeap()); html += F(" B</div></div>");
  html += F("<div class='card metric-card'><div class='k'>LittleFS</div><div class='v'>"); html += String(fsUsed); html += F(" / "); html += String(fsTotal); html += F(" B</div></div>");
  html += F("</section>");

  html += F("<section class='grid diag-grid'>");
  html += F("<div class='section-title'>System Status</div>");
  html += F("<div class='card status-card'><div class='card-head'><h2>Runtime</h2><span class='card-tag'>Controller</span></div><table>");
  html += F("<tr><td>Device</td><td>"); html += kHost; html += F("</td></tr>");
  html += F("<tr><td>Firmware</td><td>v"); html += kFirmwareVersion; html += F("</td></tr>");
  html += F("<tr><td>Build</td><td>"); html += kFirmwareBuildDate; html += F("</td></tr>");
  html += F("<tr><td>Reset reason</td><td>"); html += resetReasonText(esp_reset_reason()); html += F("</td></tr>");
  html += F("<tr><td>Reset code</td><td>"); html += String((int)esp_reset_reason()); html += F("</td></tr>");
  html += F("<tr><td>Boot count</td><td id='diagBootCountTable'>"); html += String(bootCount); html += F("</td></tr>");
  html += F("<tr><td>SDK</td><td>"); html += ESP.getSdkVersion(); html += F("</td></tr>");
  html += F("<tr><td>CPU</td><td>"); html += String(ESP.getCpuFreqMHz()); html += F(" MHz</td></tr>");
  html += F("<tr><td>Sketch</td><td>"); html += String(ESP.getSketchSize()); html += F(" B</td></tr>");
  html += F("</table></div>");

  html += F("<div class='card status-card'><div class='card-head'><h2>Network</h2><span class='card-tag'>Connectivity</span></div><table>");
  html += F("<tr><td>SSID</td><td>"); html += WiFi.SSID(); html += F("</td></tr>");
  html += F("<tr><td>Hostname</td><td>"); html += WiFi.getHostname(); html += F("</td></tr>");
  html += F("<tr><td>WiFi signal</td><td id='diagNetworkWifi'>"); html += String(rssi); html += F(" dBm - "); html += rssiLabel; html += F("</td></tr>");
  html += F("<tr><td>MQTT</td><td class='"); html += (_mqtt.connected() ? "ok" : (mqttEnabled ? "warn" : "")); html += F("'>");
  html += (mqttEnabled ? (_mqtt.connected() ? "connected" : "offline") : "disabled"); html += F("</td></tr>");
  html += F("<tr><td>Base topic</td><td>"); html += mqttBase; html += F("</td></tr>");
  html += F("</table></div>");

  html += F("<div class='card status-card hardware-card'><div class='card-head'><h2>Hardware</h2><span class='card-tag'>Devices</span></div><table>");
  html += F("<tr><td>I2C pins</td><td>"); html += String(i2cSdaPin); html += F(" / "); html += String(i2cSclPin); html += F("</td></tr>");
  html += F("<tr><td>I2C devices</td><td><code>"); html += i2cList; html += F("</code></td></tr>");
  html += F("<tr><td>GPIO fallback</td><td>"); html += (useGpioFallback ? "yes" : "no"); html += F("</td></tr>");
  html += F("<tr><td>Display</td><td>"); html += (!displayEnabled ? "Disabled" : (displayUseTft ? "TFT" : "OLED")); html += F("</td></tr>");
  html += F("</table></div>");

  html += F("<div class='card status-card irrigation-card'><div class='card-head'><h2>Irrigation</h2><span class='card-tag'>Live state</span></div><table>");
  html += F("<tr><td>Zones</td><td>"); html += String(activeZones); html += F(" active / "); html += String(zonesCount); html += F("</td></tr>");
  html += F("<tr><td>Source</td><td>"); html += sourceModeText(); html += F("</td></tr>");
  html += F("<tr><td>Tank</td><td>"); html += String(tankPercent()); html += F("%</td></tr>");
  html += F("<tr><td>Delay</td><td>"); html += rainDelayCauseText(); html += F("</td></tr>");
  html += F("</table></div>");

  html += F("<div class='card weather-card'><div class='card-head'><h2>Weather</h2><span class='card-tag'>Service</span></div><table>");
  html += F("<tr><td>Temp/Humidity Source</td><td>"); html += climateSourceLabel(); html += F("</td></tr>");
  html += F("<tr><td>DHT22 GPIO</td><td>"); html += pinText(dhtSensorPin); html += F("</td></tr>");
  html += F("<tr><td>Local Sensor</td><td>");
  if (climateSource == CLIMATE_OPEN_METEO) html += F("not selected");
  else html += (localClimateValid ? "ready" : "unavailable");
  html += F("</td></tr>");
  html += F("<tr><td>Location</td><td>"); html += meteoLocationLabel(); html += F("</td></tr>");
  html += F("<tr><td>Model</td><td>"); html += meteoModel; html += F("</td></tr>");
  html += F("<tr><td>Current HTTP</td><td>"); html += String(lastWeatherHttpCode); html += F("</td></tr>");
  html += F("<tr><td>Forecast HTTP</td><td>"); html += String(lastForecastHttpCode); html += F("</td></tr>");
  html += F("<tr><td>HTTP timeout</td><td>"); html += String(meteoHttpTimeoutMs()); html += F(" ms</td></tr>");
  html += F("</table></div>");

  html += F("<div class='section-title'>Pin Assignments</div>");

  html += F("<div class='card pin-card'><div class='card-head'><h2>Display & Buses</h2><span class='card-tag'>9 assignments</span></div><table class='pin-table'><thead><tr><th>Assignment</th><th>Pin</th><th>Notes</th></tr></thead><tbody>");
  appendPinRow(F("I2C SDA"), pinText(i2cSdaPin), F("OLED and PCF8574 bus"));
  appendPinRow(F("I2C SCL"), pinText(i2cSclPin), F("OLED and PCF8574 bus"));
  appendPinRow(F("TFT SCLK"), pinText(tftSclkPin), displayUseTft ? F("ST7789 SPI clock") : F("Configured, TFT not selected"));
  appendPinRow(F("TFT MOSI"), pinText(tftMosiPin), displayUseTft ? F("ST7789 SPI data") : F("Configured, TFT not selected"));
  appendPinRow(F("TFT CS"), pinText(tftCsPin), F("ST7789 chip select"));
  appendPinRow(F("TFT DC"), pinText(tftDcPin), F("ST7789 data/command"));
  appendPinRow(F("TFT RST"), pinText(tftRstPin), F("-1 means tied to reset or unused"));
  appendPinRow(F("TFT Backlight"), pinText(tftBlPin), g_tftPwmReady ? F("PWM brightness control") : F("-1 means tied on or disabled"));
  appendPinRow(F("Status Pixel"), pinText(STATUS_PIXEL_PIN), statusPixelReady ? F("WS2812 status LED ready") : F("Not initialized"));
  html += F("</tbody></table></div>");

  html += F("<div class='card pin-card'><div class='card-head'><h2>Sensors & Inputs</h2><span class='card-tag'>7 assignments</span></div><table class='pin-table'><thead><tr><th>Assignment</th><th>Pin</th><th>Notes</th></tr></thead><tbody>");
  appendPinRow(F("DHT22 / AM2302"), pinText(dhtSensorPin), climateSource == CLIMATE_DHT22_GPIO ? F("selected climate source") : F("not selected"));
  appendPinRow(F("Tank Level ADC"), pinText(tankLevelPin), F("Tank percentage sensor"));
  appendPinRow(F("Rain Sensor"), pinText(rainSensorPin), String(rainSensorEnabled ? "enabled" : "disabled") + (rainSensorInvert ? F(", inverted") : F(", normal polarity")));
  appendPinRow(F("Moisture Probe ADC"), pinText(moisturePin), String(moistureProbeEnabled ? "enabled" : "disabled"));
  appendPinRow(F("Photoresistor ADC"), pinText(photoPin), String(photoAutoEnabled ? "auto backlight enabled" : "auto backlight disabled") + (photoInvert ? F(", inverted") : F(", normal polarity")));
  appendPinRow(F("Manual Select Button"), pinText(manualSelectPin), F("INPUT_PULLUP, -1 disabled"));
  appendPinRow(F("Manual Start Button"), pinText(manualStartPin), F("INPUT_PULLUP, -1 disabled"));
  html += F("</tbody></table></div>");

  html += F("<div class='card card-wide pin-card'><div class='card-head'><h2>Relays & Zones</h2><span class='card-tag'>Outputs</span></div><table class='pin-table'><thead><tr><th>Assignment</th><th>Pin</th><th>Notes</th></tr></thead><tbody>");
  appendPinRow(F("Power Supply Enable"), pinText(powerSupplyPin), String(activeText(powerSupplyActiveLow)));
  appendPinRow(F("City Water Relay"), pinText(mainsPin), String(activeText(mainsGpioActiveLow)) + F(", PCF P") + String(mainsChannel));
  appendPinRow(F("Tank Relay"), pinText(tankPin), String(activeText(tankGpioActiveLow)) + F(", PCF P") + String(tankChannel));
  for (int z = 0; z < MAX_ZONES; ++z) {
    String name = String("Zone ") + String(z + 1);
    if (zoneNames[z].length()) {
      name += F(" - ");
      name += zoneNames[z];
    }
    String notes = (z < (int)zonesCount) ? F("enabled") : F("not in active zone count");
    notes += F(", ");
    notes += activeText(zoneGpioActiveLow[z]);
    if (z < ALL_P) {
      notes += F(", PCF P");
      notes += String(PCH[z]);
    } else {
      notes += F(", GPIO only");
    }
    appendPinRow(name, pinText(zonePins[z]), notes);
  }
  html += F("</tbody></table></div>");
  html += F("</section></main><script>");
  html += F("function wifiQuality(v,c){if(!c)return'Disconnected';if(typeof v!=='number')return'Connected';if(v>=-50)return'Excellent';if(v>=-60)return'Strong';if(v>=-67)return'Good';if(v>=-75)return'Fair';if(v>=-82)return'Weak';return'Very weak';}");
  html += F("async function refreshWifiStatus(){try{const r=await fetch('/status',{cache:'no-store'});const s=await r.json();const connected=!!s.wifiConnected;const rssi=(typeof s.rssi==='number')?s.rssi:null;const quality=wifiQuality(rssi,connected);const signal=(rssi===null?'--':rssi)+' dBm - '+quality;const ip=document.getElementById('diagWifiIp');const rr=document.getElementById('diagWifiRssi');const nw=document.getElementById('diagNetworkWifi');if(ip){ip.textContent=connected?(s.wifiIp||'Connected'):'Disconnected';ip.className='v status '+(connected?'ok':'bad');}if(rr)rr.textContent='RSSI '+signal;if(nw)nw.textContent=signal;}catch(e){}}");
  html += F("document.getElementById('resetBootCount')?.addEventListener('click',async function(){this.disabled=true;try{const r=await fetch('/reset_boot_count',{method:'POST'});if(!r.ok)throw new Error(await r.text());document.getElementById('diagBootCount').textContent='0';document.getElementById('diagBootCountTable').textContent='0';}catch(e){alert('Unable to reset boot count');}finally{this.disabled=false;}});");
  html += F("setInterval(refreshWifiStatus,60000);refreshWifiStatus();");
  html += F("</script></body></html>");

  server.send(200, "text/html", html);
}

#if ENABLE_OTA
static bool otaUploadAuthorized = false;
static bool otaUploadStarted = false;
static bool otaUploadComplete = false;
static bool otaUploadFailed = false;
static String otaUploadError;

static bool otaHttpAuthenticate() {
  if (!otaPassword.length()) {
    server.send(503, "text/plain", "Browser OTA is disabled. Set an OTA password in Setup first.");
    return false;
  }
  if (!server.authenticate("admin", otaPassword.c_str())) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

static void handleOtaUpdatePage() {
  HttpScope _scope;
  if (!otaHttpAuthenticate()) return;

  String html;
  html.reserve(8500);
  html += F("<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>ESP Irrigation Firmware Update</title><style>:root{color-scheme:light dark;--bg:#eef4fb;--panel:#fff;--ink:#172033;--muted:#66758a;--line:#d8e3ef;--blue:#2563eb;--bad:#b91c1c}@media(prefers-color-scheme:dark){:root{--bg:#0b1220;--panel:#111c2e;--ink:#e7eef9;--muted:#9aa9bd;--line:#293a54;--blue:#60a5fa;--bad:#f87171}}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.45 'Segoe UI',Arial,sans-serif}.wrap{max-width:680px;margin:0 auto;padding:24px}.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:22px;box-shadow:0 10px 28px rgba(15,23,42,.1)}h1{margin:0 0 8px;font-size:1.55rem}p{color:var(--muted)}.meta{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:18px 0}.meta div{border:1px solid var(--line);border-radius:8px;padding:10px}.meta span{display:block;color:var(--muted);font-size:.78rem;text-transform:uppercase;letter-spacing:.06em}input{display:block;width:100%;padding:12px;border:1px solid var(--line);border-radius:8px;background:var(--panel);color:var(--ink);margin:12px 0}button,.link{display:inline-block;border:0;border-radius:8px;padding:11px 15px;background:var(--blue);color:#fff;font-weight:700;text-decoration:none;cursor:pointer}.link{background:transparent;color:var(--blue);border:1px solid var(--line);margin-left:6px}progress{width:100%;height:18px;margin-top:18px}#status{min-height:24px;font-weight:700}.warn{color:var(--bad);font-weight:700;font-size:.9rem}@media(max-width:540px){.wrap{padding:14px}.meta{grid-template-columns:1fr}}</style></head><body><main class='wrap'><section class='card'>");
  html += F("<h1>Firmware update</h1><p>Upload the application <code>.bin</code> produced with <b>Minimal SPIFFS (Large APPS with OTA)</b>.</p><div class='meta'><div><span>Installed version</span><b>");
  html += kFirmwareVersion;
  html += F("</b></div><div><span>Available OTA slot</span><b>");
  html += String(ESP.getFreeSketchSpace());
  html += F(" bytes</b></div></div><p class='warn'>Keep power connected. Active valves will be stopped before flash writing starts.</p><form id='uploadForm' method='POST' action='/update' enctype='multipart/form-data'><input id='firmware' type='file' name='firmware' accept='.bin,application/octet-stream' required><button id='uploadButton' type='submit'>Install firmware</button><a class='link' href='/setup'>Back to setup</a><progress id='progress' max='100' value='0' hidden></progress><p id='status'></p></form><form id='urlForm'><label for='firmwareUrl'>GitHub firmware URL</label><input id='firmwareUrl' type='url' placeholder='https://raw.githubusercontent.com/owner/repo/main/firmware.bin' required><p>Use a public GitHub file or raw link to the application .bin for your board. For release assets, download the file and use the upload above.</p><button id='urlButton' type='submit'>Install from GitHub</button></form></section></main><script>");
  html += F("const OTA_MAX_BYTES="); html += String(ESP.getFreeSketchSpace()); html += F(";");
  html += F("const form=document.getElementById('uploadForm'),file=document.getElementById('firmware'),button=document.getElementById('uploadButton'),progress=document.getElementById('progress'),status=document.getElementById('status'),urlForm=document.getElementById('urlForm'),urlButton=document.getElementById('urlButton');");
  html += F("function busy(on){button.disabled=on;urlButton.disabled=on;}");
  html += F("function uploadFirmware(data){progress.hidden=false;progress.removeAttribute('value');status.textContent='Uploading...';const xhr=new XMLHttpRequest();xhr.open('POST','/update');xhr.upload.onprogress=e=>{if(e.lengthComputable)progress.value=Math.round(e.loaded*100/e.total);};xhr.onload=()=>{status.textContent=xhr.responseText||((xhr.status===200)?'Update installed. Restarting...':'Update failed.');if(xhr.status!==200)busy(false);};xhr.onerror=()=>{status.textContent='Connection lost during upload. Check the controller before retrying.';busy(false);};xhr.send(data);}");
  html += F("form.addEventListener('submit',e=>{e.preventDefault();if(!file.files.length)return;if(!confirm('Install this firmware and restart the controller?'))return;busy(true);uploadFirmware(new FormData(form));});");
  html += F("function githubFirmwareUrl(value){const u=new URL(value);if(u.protocol!=='https:'||u.username||u.password||u.port)throw Error('Use a public HTTPS GitHub file URL.');if(u.hostname==='github.com'){const parts=u.pathname.split('/');if(parts[3]!=='blob'&&parts[3]!=='raw')throw Error('Use a GitHub file or raw link. Download release assets and use file upload.');parts.splice(3,1);u.hostname='raw.githubusercontent.com';u.pathname=parts.join('/');}if(u.hostname!=='raw.githubusercontent.com')throw Error('Use a github.com file link or raw.githubusercontent.com URL.');if(!u.pathname.toLowerCase().endsWith('.bin'))throw Error('The URL must point to an application .bin file.');u.search='';u.hash='';return u.href;}");
  html += F("urlForm.addEventListener('submit',async e=>{e.preventDefault();try{const url=githubFirmwareUrl(document.getElementById('firmwareUrl').value.trim());if(!confirm('Download this firmware, install it and restart the controller?'))return;busy(true);progress.hidden=false;progress.removeAttribute('value');status.textContent='Downloading from GitHub...';const response=await fetch(url,{credentials:'omit',cache:'no-store',signal:AbortSignal.timeout(120000)});if(!response.ok)throw Error('GitHub download failed (HTTP '+response.status+').');const blob=await response.blob();if(blob.size===0||blob.size>OTA_MAX_BYTES)throw Error('Firmware is empty or exceeds the available OTA slot.');const header=new Uint8Array(await blob.slice(0,1).arrayBuffer());if(header[0]!==0xe9)throw Error('The download is not an ESP application image.');const data=new FormData();data.append('firmware',blob,'firmware.bin');uploadFirmware(data);}catch(error){status.textContent=(error.message||'Download failed.')+' You can also download the .bin and upload it above.';progress.hidden=true;busy(false);}});");
  html += F("</script></body></html>");
  server.send(200, "text/html", html);
}

static void handleOtaUploadData() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    otaUploadAuthorized = otaHttpAuthenticate();
    otaUploadStarted = false;
    otaUploadComplete = false;
    otaUploadFailed = false;
    otaUploadError = "";
    if (!otaUploadAuthorized) return;

    String filename = upload.filename;
    filename.toLowerCase();
    if (!filename.endsWith(".bin")) {
      otaUploadFailed = true;
      otaUploadError = "Only an application .bin file can be installed.";
      return;
    }

    g_inHttp = true;
    for (int z = 0; z < (int)MAX_ZONES; ++z) {
      pendingStart[z] = false;
      if (zoneActive[z]) turnOffZone(z);
    }

    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
      Update.printError(Serial);
      otaUploadFailed = true;
      otaUploadError = "No OTA slot is available or the partition is too small.";
      g_inHttp = false;
      return;
    }
    otaUploadStarted = true;
    Serial.printf("[OTA HTTP] Receiving %s\n", upload.filename.c_str());
  } else if (!otaUploadAuthorized || otaUploadFailed) {
    return;
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
      otaUploadFailed = true;
      otaUploadError = "Flash write failed. The existing firmware is still selected.";
      Update.abort();
      g_inHttp = false;
    } else {
      yield();
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) {
      Update.printError(Serial);
      otaUploadFailed = true;
      otaUploadError = "Firmware validation failed. The existing firmware is still selected.";
    } else {
      otaUploadComplete = true;
      Serial.printf("[OTA HTTP] Installed %u bytes\n", (unsigned)upload.totalSize);
    }
    g_inHttp = false;
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    otaUploadFailed = true;
    otaUploadError = "Upload was aborted. The existing firmware is still selected.";
    g_inHttp = false;
  }
}

static void handleOtaUploadResult() {
  if (!otaHttpAuthenticate()) {
    g_inHttp = false;
    return;
  }

  const bool success = otaUploadAuthorized && otaUploadStarted &&
                       otaUploadComplete && !otaUploadFailed && !Update.hasError();
  server.sendHeader("Connection", "close");
  if (!success) {
    const String message = otaUploadError.length()
      ? otaUploadError
      : String("No valid firmware file was received.");
    server.send(400, "text/plain", message);
    otaUploadAuthorized = false;
    otaUploadStarted = false;
    otaUploadComplete = false;
    g_inHttp = false;
    return;
  }

  server.send(200, "text/plain", "Update installed successfully. Controller restarting...");
  delay(750);
  ESP.restart();
}
#endif

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  Serial.printf("[BOOT] %s %s\n", __DATE__, __TIME__);

  // Clamp noisy logs (IDF)
  esp_log_level_set("*", ESP_LOG_WARN);
  esp_log_level_set("i2c", ESP_LOG_NONE);
  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  esp_log_level_set("i2c_master", ESP_LOG_NONE);

  // Route larger heap allocations to PSRAM before long-lived page/cache buffers reserve.
  configurePsramForLargeBuffers();

  // I2C bus
  I2Cbus.begin(i2cSdaPin, i2cSclPin, 100000);
  I2Cbus.setTimeOut(20);

  bootMillis = millis();

  // Pre-size hot Strings to reduce heap churn over long runtimes.
  meteoLocation.reserve(64);
  meteoModel.reserve(24);
  cachedWeatherData.reserve(4096);
  cachedForecastData.reserve(16384);
  lastWeatherError.reserve(96);
  lastForecastError.reserve(96);

  // LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed; formatting...");
    if (!(LittleFS.format() && LittleFS.begin())) {
      Serial.println("LittleFS unavailable; halt.");
      while (true) delay(1000);
    }
  }
  updateBootCount();
  initUpdateReportState();
  Serial.printf("[BOOT] count=%lu reset=%s (%d)\n",
                (unsigned long)bootCount,
                resetReasonText(esp_reset_reason()),
                (int)esp_reset_reason());

  // Config + schedule
  loadConfig();
  loadRainHistoryState();
  sanitizePinConfig();
  validatePinMap();
  if (!LittleFS.exists("/schedule.txt")) saveSchedule();
  loadSchedule();
  initManualButtons();

  mainsChannel = P4; 
  tankChannel  = P5;

  // PCF8574 expanders (or GPIO fallback)
  bool expandersOk = initExpanders();
  if (!expandersOk) {
    Serial.println("PCF8574 relays not found; GPIO fallback.");
    initGpioFallback();
    useGpioFallback = true;
  } else {
    useGpioFallback = false;
    checkI2CHealth();
  }
  initGpioPinsForZones();

  initStatusLedPwm();

  // Status pixel (WS2812)
  if (STATUS_PIXEL_PIN >= 0) {
    statusPixel.begin();
    statusPixel.setBrightness(32);
    statusPixel.clear();
    statusPixel.show();
    statusPixelLastColor = 0;
    statusPixelReady = true;
    statusPixelSet(0, 0, 20); // boot = blue
  } else {
    statusPixelReady = false;
    statusPixelLastColor = 0;
  }

  applyLocalClimateOverride(true);

  // ---------- Display init ----------
  if (!displayEnabled) {
    Serial.println("[DISPLAY] disabled");
  } else if (displayUseTft) {
    if (initTftDisplay()) {
      showTftBootSplash();
    }
  } else {
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
  }

  // Hostname + WiFi events
  WiFi.setHostname(kHost);
  WiFi.persistent(true);            // keep credentials in NVS across resets
  WiFi.setAutoReconnect(true);      // let core retry silently after power cycles

  // WiFiManager connect
  wifiManager.setWiFiAutoReconnect(true); // ESP32S3 sometimes drops without this
  wifiManager.setConnectRetries(6);       // more chances before giving up
  wifiManager.setConnectTimeout(20);      // seconds per connect attempt
  wifiManager.setSaveConnect(true);       // reconnect immediately after saving
  wifiManager.setTimeout(180);
  wifiManager.setConfigPortalTimeout(0);      // keep portal open until user configures
  wifiManager.setBreakAfterConfig(true);      // exit once credentials are saved
  WiFi.mode(WIFI_STA);                        // ensure STA mode for S2/S3
  if (!wifiManager.autoConnect("ESPIrrigationAP")) {
    Serial.println("WiFi connect failed -> opening config portal (ESPIrrigationAP).");
    wifiManager.startConfigPortal("ESPIrrigationAP");  // block here until configured
  }

  // Improve HTTP responsiveness
  configureWifiForHttp();

  if (!displayEnabled) {
    // Headless mode: web UI and controller logic stay active.
  } else if (displayUseTft) {
    showTftConnectedScreen();
    delay(TFT_CONNECTED_SCREEN_MS);
  } else {
    display.clearDisplay();
    display.setTextSize(2); display.setCursor(0, 0);  display.print("Connected!");
    display.setTextSize(1); display.setCursor(0, 20); display.print(WiFi.localIP().toString());
    display.setCursor(0, 32); display.print("espirrigation.local");
    display.display();
    delay(2500);
  }

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
  rebuildRuntimeCountersFromEvents();

  // OTA
  #if ENABLE_OTA
    ArduinoOTA.setHostname(kHost);
    if (otaPassword.length()) ArduinoOTA.setPassword(otaPassword.c_str());
    ArduinoOTA.begin();
  #endif

  mdnsStart();

  // -------- Routes --------
  Serial.println("[BOOT] registering routes");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/submit", HTTP_POST, handleSubmit);

  server.on("/setup", HTTP_GET, handleSetupPage);
  server.on("/configure", HTTP_POST, handleConfigure);

  server.on("/events", HTTP_GET, handleLogPage);
  server.on("/clearevents", HTTP_POST, handleClearEvents);

  server.on("/tank", HTTP_GET, handleTankCalibration);
  server.on("/diagnostics", HTTP_GET, handleDiagnosticsPage);
  server.on("/diagnostics.json", HTTP_GET, handleDiagnosticsJson);
  #if ENABLE_OTA
    server.on("/update", HTTP_GET, handleOtaUpdatePage);
    server.on("/update", HTTP_POST, handleOtaUploadResult, handleOtaUploadData);
  #endif
  server.on("/reset_boot_count", HTTP_POST, [](){
    HttpScope _scope;
    if (!saveBootCount(0)) {
      server.send(500, "text/plain", "Unable to save boot count");
      return;
    }
    bootCount = 0;
    server.send(200, "text/plain", "OK");
  });

  // /status JSON
  server.on("/status", HTTP_GET, [](){
    HttpScope _scope; // NEW: avoid heavy work while serving
    JsonDocument doc;

  const bool cooldownActiveNow = isCooldownActiveNow();
  const bool rainDelayBlockingNow = (rainActive || cooldownActiveNow);

  doc["rainDelayActive"] = rainDelayBlockingNow;
  doc["rainActive"]      = rainActive;
  doc["rainCooldownActive"] = cooldownActiveNow;
  doc["windDelayActive"] = windActive;
  doc["rainDelayCause"]  = rainDelayCauseText();
  doc["zonesCount"]      = zonesCount;
  doc["tankPct"]         = tankPercent();
  doc["tankRaw"]         = readTankRaw();
  doc["tankEmptyRaw"]    = tankEmptyRaw;
  doc["tankFullRaw"]     = tankFullRaw;
  doc["sourceMode"]      = sourceModeText();
  doc["wifiConnected"]   = (WiFi.status() == WL_CONNECTED);
  doc["wifiIp"]          = WiFi.localIP().toString();
  doc["rssi"]            = WiFi.RSSI();
  doc["firmwareVersion"] = kFirmwareVersion;
  doc["firmwareBuild"]   = kFirmwareBuildDate;
  doc["uptimeSec"]       = (millis() - bootMillis) / 1000;
  doc["totalScheduledRuntimeSec"] = totalScheduledRuntimeSec;
  doc["totalManualRuntimeSec"]    = totalManualRuntimeSec;
  doc["heapFree"]        = ESP.getFreeHeap();
  doc["heapMin"]         = ESP.getMinFreeHeap();
  doc["heapMaxAlloc"]    = ESP.getMaxAllocHeap();
  const int tftPct = (int)lroundf((float)g_tftBrightness * 100.0f / 255.0f);
  doc["tftBlPin"]         = tftBlPin;
  doc["tftPwm"]           = g_tftPwmReady;
  doc["tftBlOn"]          = g_tftBlOn;
  doc["tftDisplayOn"]     = g_tftDisplayOn;
  doc["tftBrightnessPct"] = tftPct;
  doc["tftWidth"]         = tftPanelWidth;
  doc["tftHeight"]        = tftPanelHeight;
    doc["displayType"]      = !displayEnabled ? "none" : (displayUseTft ? "tft" : "oled");
    doc["tempUnit"]         = tempUseFahrenheit ? "F" : "C";
    doc["climateSource"]    = climateSourceValue();
    doc["climateSourceLabel"] = climateSourceLabel();
    doc["dht22Pin"]          = dhtSensorPin;
    doc["localClimateValid"] = localClimateValid;
    if (isfinite(localTempC)) doc["localTempC"] = localTempC;
    else                      doc["localTempC"] = nullptr;
    if (isfinite(localHumidityPct)) doc["localHumidity"] = localHumidityPct;
    else                           doc["localHumidity"] = nullptr;

    // Current rain (actuals)
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

    char sunriseLocal[6] = "--:--";
    char sunsetLocal[6] = "--:--";
    if (todaySunrise > 0) {
      struct tm tsr;
      localtime_r(&todaySunrise, &tsr);
      strftime(sunriseLocal, sizeof(sunriseLocal), "%H:%M", &tsr);
    }
    if (todaySunset > 0) {
      struct tm tss;
      localtime_r(&todaySunset, &tss);
      strftime(sunsetLocal, sizeof(sunsetLocal), "%H:%M", &tss);
    }

    doc["deviceEpoch"]  = (uint32_t)nowEpoch;
    doc["utcOffsetMin"] = deltaMin;
    doc["isDST"]        = (ltm.tm_isdst > 0);
    doc["tzAbbrev"]     = (ltm.tm_isdst>0) ? "DST" : "STD";
    doc["sunriseLocal"] = sunriseLocal;
    doc["sunsetLocal"]  = sunsetLocal;
    doc["weatherHttp"]  = lastWeatherHttpCode;
    doc["forecastHttp"] = lastForecastHttpCode;
    if (lastWeatherError.length())  doc["weatherError"]  = lastWeatherError;
    if (lastForecastError.length()) doc["forecastError"] = lastForecastError;

    // Feature gates
    doc["masterOn"]          = systemMasterEnabled;
    doc["cooldownUntil"]     = rainCooldownUntilEpoch;
    doc["cooldownRemaining"] = (rainCooldownUntilEpoch>nowEpoch) ? (rainCooldownUntilEpoch - nowEpoch) : 0;
    doc["rainThresh24h"]     = rainThreshold24h_mm;
    doc["rainCooldownMin"]   = rainCooldownMin;
    doc["rainCooldownHours"] = rainCooldownMin / 60;
    doc["runConcurrent"] = runZonesConcurrent;
    doc["smartWatering"] = smartWateringEnabled;
    doc["smartFactor"] = smartWateringFactor();
    doc["moistureEnabled"] = moistureProbeEnabled;
    doc["moistureSource"] = moistureUseMeteo ? "meteo" : "probe";
    doc["moisturePct"] = moisturePercent();
    doc["moistureRaw"] = moistureRaw();
    doc["moistureSkip"] = isSoilWetForSmartSkip();

    // Zones snapshot
    JsonArray zones = doc["zones"].to<JsonArray>();
    for (int i=0; i<zonesCount; i++){
      JsonObject z = zones.add<JsonObject>();
      z["active"] = zoneActive[i];
      z["name"]   = zoneNames[i];
      unsigned long rem = 0;
      if (zoneActive[i]) {
        unsigned long elapsed = (millis() - zoneStartMs[i]) / 1000;
        unsigned long total   = zoneRunTotalSec[i];
        if (total == 0) total = durationForSlot(i,1);
        rem = (elapsed < total ? total - elapsed : 0);
      }
      z["remaining"] = rem;
      unsigned long total = zoneRunTotalSec[i];
      if (total == 0) total = durationForSlot(i,1);
      z["totalSec"]  = total;
    }

    // Current weather pass-through from decoded snapshot.
    doc["temp"]       = isfinite(curTempC) ? curTempC : 0.0f;
    doc["feels_like"] = isfinite(curFeelsC) ? curFeelsC : 0.0f;
    doc["humidity"]   = (curHumidityPct >= 0) ? curHumidityPct : 0;
    doc["pressure"]   = isfinite(curPressureHpa) ? curPressureHpa : 0.0f;
    doc["pressureTrend"] = curPressureTrend;
    doc["wind"]       = isfinite(curWindMs) ? curWindMs : 0.0f;
    doc["gustNow"]    = isfinite(curGustMs) ? curGustMs : 0.0f;
    doc["windDirText"]= formatWindDirection(curWindDirDeg);
    if (isfinite(curWindDirDeg)) doc["windDirDeg"] = normalizeDegrees360(curWindDirDeg);
    else                         doc["windDirDeg"] = nullptr;
    doc["condMain"]   = (curWeatherCode >= 0) ? meteoCodeToMain(curWeatherCode) : "";
    doc["condDesc"]   = (curWeatherCode >= 0) ? meteoCodeToDesc(curWeatherCode) : "";
    doc["icon"]       = "";
    doc["owmTzSec"]   = curUtcOffsetSec;
    // Always expose location for UI
    doc["cityName"] = meteoLocationLabel();
    doc["meteoModel"] = meteoModel;
    if (isValidLatLon(meteoLat, meteoLon)) {
      doc["lat"] = meteoLat;
      doc["lon"] = meteoLon;
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

    String out;
    out.reserve(2200);
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  // Time API
  server.on("/api/time", HTTP_GET, [](){
    HttpScope _scope;
    time_t nowEpoch = time(nullptr);
    struct tm lt; localtime_r(&nowEpoch, &lt);
    struct tm gt; gmtime_r(&nowEpoch,  &gt);
    JsonDocument d;
    d["epoch"] = (uint32_t)nowEpoch;
    char buf[32];
    strftime(buf,sizeof(buf),"%Y-%m-%d %H:%M:%S",&lt); d["local"] = buf;
    strftime(buf,sizeof(buf),"%Y-%m-%d %H:%M:%S",&gt); d["utc"]   = buf;
    d["isDST"] = (lt.tm_isdst>0);
    d["tz"]    = (lt.tm_isdst>0) ? "DST" : "STD";
    String out;
    out.reserve(180);
    serializeJson(d, out);
    server.send(200,"application/json",out);
  });

  // Tank calibration endpoints
  server.on("/setTankEmpty", HTTP_POST, []() {
    HttpScope _scope;
    if (isValidAdcPin(tankLevelPin)) {
      tankEmptyRaw = readTankRaw();
      saveConfig();
      server.sendHeader("Location", "/tank", true);
      server.send(302, "text/plain", "");
    } else {
      server.send(400, "text/plain", "Invalid ADC pin for tank sensor");
    }
  });
  server.on("/setTankFull", HTTP_POST, []() {
    HttpScope _scope;
    if (isValidAdcPin(tankLevelPin)) {
      tankFullRaw = readTankRaw();
      saveConfig();
      server.sendHeader("Location", "/tank", true);
      server.send(302, "text/plain", "");
    } else {
      server.send(400, "text/plain", "Invalid ADC pin for tank sensor");
    }
  });

  // Manual control per zone
  for (int i=0; i<MAX_ZONES; i++){
    server.on(String("/valve/on/")+i,  HTTP_POST, [i](){
      HttpScope _s;
      String error;
      if (turnOnValveManual(i, &error)) server.send(200, "text/plain", "OK");
      else server.send(409, "text/plain", error.length() ? error : "Manual start failed");
    });
    server.on(String("/valve/off/")+i, HTTP_POST, [i](){ HttpScope _s; turnOffValveManual(i); server.send(200,"text/plain","OK"); });
  }
  server.on("/stopall", HTTP_POST, [](){
    HttpScope _scope;
    for (int z=0; z<MAX_ZONES; ++z) if (zoneActive[z]) turnOffZone(z);
    server.send(200,"text/plain","OK");
  });
  server.on("/toggleBacklight", HTTP_POST, [](){
    HttpScope _scope;
    toggleBacklight();
    server.send(200,"text/plain","Display toggled");
  });

  // I2C tools
#if ENABLE_DEBUG_ROUTES
  server.on("/i2c-test", HTTP_GET, [](){
    HttpScope _scope;
    if (useGpioFallback) { server.send(500,"text/plain","Fallback active"); return; }
    for (uint8_t ch : PCH) { pcfOut.digitalWrite(ch, LOW); delay(100); pcfOut.digitalWrite(ch, HIGH); delay(60); }
    server.send(200,"text/plain","PCF8574 pulse OK");
  });
  server.on("/whereami", HTTP_GET, [](){
    HttpScope _scope;
    String out;
    out.reserve(96);
    out += "ip=";   out += WiFi.localIP().toString(); out += "\n";
    out += "ssid="; out += WiFi.SSID();               out += "\n";
    out += "mac=";  out += WiFi.macAddress();
    server.send(200,"text/plain",out);
  });
#endif
  
  // Downloads / Admin
  server.on("/download/config.txt", HTTP_GET, [](){
    HttpScope _scope;
#if ENABLE_OTA
    // config.txt contains the MQTT and OTA credentials. Once an OTA password
    // exists, require it before allowing the file to leave the controller.
    if (otaPassword.length() && !otaHttpAuthenticate()) return;
#endif
    if (LittleFS.exists("/config.txt")){ File f=LittleFS.open("/config.txt","r"); server.streamFile(f,"text/plain"); f.close(); }
    else server.send(404,"text/plain","missing");
  });
  server.on("/download/schedule.txt", HTTP_GET, [](){
    HttpScope _scope;
    if (LittleFS.exists("/schedule.txt")){ File f=LittleFS.open("/schedule.txt","r"); server.streamFile(f,"text/plain"); f.close(); }
    else server.send(404,"text/plain","missing");
  });
  server.on("/download/events.csv", HTTP_GET, [](){
    HttpScope _scope;
    if (LittleFS.exists("/events.csv")){ File f=LittleFS.open("/events.csv","r"); server.streamFile(f,"text/csv"); f.close(); }
    else server.send(404,"text/plain","No event log");
  });
  server.on("/tft_selftest", HTTP_GET, [](){
    HttpScope _scope;
    if (!displayEnabled || !displayUseTft) {
      server.send(400, "text/plain", displayEnabled ? "Display mode is OLED" : "Display is disabled");
      return;
    }
    String msg;
    bool pinsOk = true;
    auto chk = [&](const char* name, int pin, bool allowNeg){
      if (pin == -1 && allowNeg) return;
      if ((allowNeg && !isValidOptionalTftPin(pin)) || (!allowNeg && !isValidTftSignalPin(pin))) {
        pinsOk = false;
        msg += String(name) + " invalid/unsafe: " + String(pin) + "\n";
      }
    };
    chk("TFT_SCLK", tftSclkPin, false);
    chk("TFT_MOSI", tftMosiPin, false);
    chk("TFT_CS",   tftCsPin,   false);
    chk("TFT_DC",   tftDcPin,   false);
    chk("TFT_RST",  tftRstPin,  true);
    chk("TFT_BL",   tftBlPin,   true);

    if (!pinsOk) {
      server.send(500, "text/plain", msg.length() ? msg : "Invalid TFT pins");
      return;
    }

    const bool oldBlOn = g_tftBlOn;
    const bool oldDisp = g_tftDisplayOn;
    const uint8_t oldDuty = g_tftBrightness;

    tftInitBacklightPwm();

    tft.fillScreen(ST77XX_RED);   delay(120);
    tft.fillScreen(ST77XX_GREEN); delay(120);
    tft.fillScreen(ST77XX_BLUE);  delay(120);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.print("TFT SELF-TEST");
    tft.setTextSize(1);
    tft.setCursor(10, 36);
    tft.print("SCLK "); tft.print(tftSclkPin);
    tft.print(" MOSI "); tft.print(tftMosiPin);
    tft.print(" CS "); tft.print(tftCsPin);
    tft.setCursor(10, 52);
    tft.print("DC "); tft.print(tftDcPin);
    tft.print(" RST "); tft.print(tftRstPin);
    tft.print(" BL "); tft.print(tftBlPin);

    if (tftBlPin >= 0) {
      if (g_tftPwmReady) {
        tftSetBrightness(100); delay(120);
        tftSetBrightness(20);  delay(120);
        tftSetBrightness(80);  delay(120);
      } else {
        tftBacklight(false); delay(150);
        tftBacklight(true);  delay(150);
      }
    }

    if (tftBlPin >= 0) {
      const int oldPct = (int)lroundf((float)oldDuty * 100.0f / 255.0f);
      tftSetBrightness((uint8_t)oldPct);
      tftBacklight(oldBlOn);
    } else {
      tftDisplay(oldDisp);
    }
    markTftScreensDirty();

    msg = "TFT self-test OK\n";
    msg += "PWM: "; msg += (g_tftPwmReady ? "yes" : "no");
    msg += "\nBacklight: ";
    msg += (tftBlPin >= 0 ? (g_tftBlOn ? "on" : "off") : (g_tftDisplayOn ? "display-on" : "display-off"));
    server.send(200, "text/plain", msg);
  });
  server.on("/tft_brightness", HTTP_POST, [](){
    HttpScope _scope;
    if (!displayEnabled || !displayUseTft) { server.send(400, "text/plain", displayEnabled ? "Display mode is OLED" : "Display is disabled"); return; }
    if (tftBlPin < 0) { server.send(500,"text/plain","No backlight pin"); return; }
    int lvl = server.hasArg("level") ? server.arg("level").toInt() : 100;
    if (lvl < 0) lvl = 0; if (lvl > 100) lvl = 100;
    tftSetBrightness((uint8_t)lvl);
    server.send(200,"text/plain","OK");
  });
  server.on("/reboot", HTTP_POST, [](){
    HttpScope _scope;
    server.send(200,"text/plain","restarting"); delay(200); ESP.restart();
  }); 
  // Pause/Resume/Delays/Forecast toggle
  server.on("/clear_delays", HTTP_POST, [](){
    HttpScope _scope;
    for (int z=0; z<(int)zonesCount; ++z) pendingStart[z] = false;
    for (int z=0; z<(int)zonesCount; ++z) if (zoneActive[z]) turnOffZone(z);
    for (int z=0; z<(int)zonesCount; ++z) lastCheckedMinute[z] = -1;
    server.send(200,"text/plain","OK");
  });
  server.on("/pause", HTTP_POST, [](){
    HttpScope _scope;
    time_t nowEp = time(nullptr);
    String secStr = server.arg("sec");
    uint32_t sec = secStr.length()? secStr.toInt() : (24u*3600u);
    systemPaused = true;
    pauseUntilEpoch = sec ? (nowEp + sec) : 0;
    saveConfig();
    server.send(200,"text/plain","OK");
  });
  server.on("/resume", HTTP_POST, [](){
    HttpScope _scope;
    systemPaused = false; pauseUntilEpoch = 0; saveConfig();
    server.send(200,"text/plain","OK");
  });
  server.on("/set_rain_forecast", HTTP_POST, [](){
    HttpScope _scope;
    rainDelayFromForecastEnabled = server.hasArg("on");
    saveConfig();
    server.send(200,"text/plain", rainDelayFromForecastEnabled ? "on" : "off");
  });

  // Master and cooldown
  server.on("/master", HTTP_POST, [](){
    HttpScope _scope;
    systemMasterEnabled = server.hasArg("on");  // Dashboard only (not in Setup)
    saveConfig();
    server.send(200,"text/plain", systemMasterEnabled ? "on" : "off");
  });
  server.on("/clear_cooldown", HTTP_POST, [](){
    HttpScope _scope;
    rainCooldownUntilEpoch = 0;
    server.send(200,"text/plain","OK");
  });

  Serial.println("[BOOT] starting web server");
  server.begin();
  Serial.println("[BOOT] web server started");

  // MQTT
  Serial.println("[BOOT] mqtt setup");
  mqttSetup();
  _lastMqttAttempt = millis();
  Serial.println("[BOOT] setup complete");
}

// ---------- Loop ----------
void loop() {
  static bool firstLoopLogged = false;
  static uint32_t lastWifiCheck = 0;

  if (!firstLoopLogged) {
    firstLoopLogged = true;
    Serial.println("[BOOT] first loop");
  }

  uint32_t now = millis();
  #if ENABLE_OTA
  ArduinoOTA.handle();
  #endif

  server.handleClient();  // serve UI first to keep it responsive

  if (WiFi.status() != WL_CONNECTED && now - lastWifiCheck >= WIFI_CHECK_MS) {
    lastWifiCheck = now;
    wifiCheck();
    now = millis();
  }

  tickUpdateReport();

  checkWindRain();
  mqttEnsureConnected();
  if (mqttEnabled) _mqtt.loop();
  mqttPublishStatus(); 
  tickWeather(); // Timed weather fetch after servicing HTTP to avoid blocking the UI
  tickManualButtons();
  tickAutoBacklight();

  // Track transitions to clear/refresh screens
  static bool lastWasDelayScreen = false;
  static bool delayScreenShowingHome = false;
  static uint32_t delayScreenPageSince = 0;

  bool manualActive = (manualScreenUntilMs != 0 && (int32_t)(manualScreenUntilMs - now) > 0);
  if (!manualActive && manualScreenUntilMs != 0) {
    manualScreenUntilMs = 0;
    g_forceHomeReset = true;
    lastScreenRefresh = 0;
  }

  const bool hardBlock = (!systemMasterEnabled || isPausedNow());
  const bool cooldownActive = isCooldownActiveNow();
  const bool manualDelayBypassActive = hasActiveManualZone();
  bool windBypassRunning = false;
  for (int z = 0; z < (int)zonesCount; ++z) {
    if (zoneActive[z] && !zoneWindDelayEnabled[z]) windBypassRunning = true;
  }
  const bool delayScreenActive = !manualActive &&
                                 !manualDelayBypassActive &&
                                 (hardBlock || cooldownActive || rainActive || (windActive && !windBypassRunning));

  if (hardBlock || cooldownActive || rainActive || windActive) {
    stopAutoZonesForBlock();
  }

  if (delayScreenActive) {
    if (!lastWasDelayScreen) {
      g_forceRainReset = true;
      g_forceHomeReset = true;
      delayScreenShowingHome = false;
      delayScreenPageSince = now;
      lastScreenRefresh = 0;
    }

    const bool scrollRainWindScreens = (rainActive || windActive || !systemMasterEnabled);
    if (scrollRainWindScreens) {
      if (now - delayScreenPageSince >= DELAY_SCREEN_SCROLL_MS) {
        delayScreenPageSince = now;
        delayScreenShowingHome = !delayScreenShowingHome;
        if (delayScreenShowingHome) g_forceHomeReset = true;
        else                        g_forceRainReset = true;
        lastScreenRefresh = 0;
      }
    } else if (delayScreenShowingHome) {
      delayScreenShowingHome = false;
      delayScreenPageSince = now;
      g_forceRainReset = true;
      lastScreenRefresh = 0;
    }

    if (now - lastScreenRefresh >= 1000) {
      lastScreenRefresh = now;
      if (delayScreenShowingHome) HomeScreen();
      else                        RainScreen();
      lastWasDelayScreen = true;
    }
  } else if (lastWasDelayScreen) {
    // Only clear once delay has actually ended
    lastWasDelayScreen = false;
    if (!displayEnabled) {
      // No display to clear.
    } else if (displayUseTft) {
      tft.fillScreen(C_BG);
    } else {
      display.clearDisplay();
      display.display();
    }
    lastScreenRefresh = 0; // force immediate HomeScreen paint
    g_forceHomeReset = true;
  }

  if (now - lastTimeQuery >= TIME_QUERY_MS) {
    lastTimeQuery = now;
    time_t nowTime = time(nullptr);
    localtime_r(&nowTime, &cachedTm);
  }

  if (cachedTm.tm_hour == 0 && cachedTm.tm_min == 0) {
    if (!midnightDone) {
      memset(lastCheckedMinute, -1, sizeof(lastCheckedMinute));
      todayMin_C = NAN;
      todayMax_C = NAN;
      midnightDone = true;
    }
  } else midnightDone = false;

  if (!useGpioFallback && (now - lastI2cCheck >= I2C_CHECK_MS)) {
    lastI2cCheck = now; checkI2CHealth();
  }

  // Status LEDs (non-blocking, light refresh)
  static uint32_t lastPixelUpdate = 0;
  if (statusPixelReady && now - lastPixelUpdate >= 100) {
    lastPixelUpdate = now;
    updateStatusPixel();
  }
  static uint32_t lastStatusLedUpdate = 0;
  if (g_statusLedPwmReady && now - lastStatusLedUpdate >= 50) {
    lastStatusLedUpdate = now;
    updateStatusLedPwm();
  }

  bool anyActive=false;
  for (int z=0; z<(int)zonesCount; z++) if (zoneActive[z]) { anyActive=true; break; }

  // -------- SCHEDULER (supports sequential or concurrent) --------
  if (now - lastScheduleTick >= SCHEDULE_TICK_MS) {
    lastScheduleTick = now;

    for (int z=0; z<(int)zonesCount; z++) {
      if (zoneActive[z] && hasDurationCompleted(z)) turnOffZone(z);
    }

    anyActive = false;
    for (int z=0; z<(int)zonesCount; z++) {
      if (zoneActive[z]) { anyActive = true; break; }
    }

    if (!isBlockedNow()) {
      for (int z=0; z<(int)zonesCount; z++) {
        if (shouldStartZone(z)) {
          // Cancel when blocked/rain; queue during wind so it can run later.
          if (isBlockedNow()) { cancelStart(z, "BLOCKED", false); continue; }
          if (rainActive)     { cancelStart(z, "RAIN",    true ); continue; }
          if (windBlocksZone(z)) { pendingStart[z] = true; logEvent(z,"QUEUED","WIND",false); continue; }

          if (!runZonesConcurrent) {
            // sequential: only start if nothing is running; else queue (still allowed)
            if (anyActive) { pendingStart[z] = true; logEvent(z,"QUEUED","ACTIVE RUN",false); }
            else { turnOnZone(z); anyActive = true; }
          } else {
            // concurrent: start regardless of other active zones
            turnOnZone(z); anyActive = true;
          }
        }
      }

      if (!runZonesConcurrent) {
        // sequential: drain one queued zone if nothing currently running
        if (!anyActive && !rainActive) {
          for (int z=0; z<(int)zonesCount; z++) if (pendingStart[z] && !windBlocksZone(z)) { pendingStart[z]=false; turnOnZone(z); break; }
        }
      } else if (!rainActive) {
        // concurrent: start any queued zones once delays clear
        for (int z=0; z<(int)zonesCount; z++) {
          if (pendingStart[z] && !windBlocksZone(z)) { pendingStart[z]=false; turnOnZone(z); anyActive = true; }
        }
      }
    }
  }

  if (!delayScreenActive && now - lastScreenRefresh >= 1000) {
    lastScreenRefresh = now;
    if (manualActive) {
      drawManualSelection();
    } else {
      int activeZone = -1;
      for (int z = 0; z < (int)zonesCount; ++z) {
        if (zoneActive[z]) { activeZone = z; break; }
      }
      if (activeZone >= 0) updateLCDForZone(activeZone);
      else HomeScreen();
    }
  }
  delay(LOOP_SLEEP_MS);
}

// ---------- Connectivity / I2C health ----------
void wifiCheck() {
  if (WiFi.status()!=WL_CONNECTED) {
    Serial.println("Reconnecting WiFi...");
    WiFi.disconnect(false, false);   // do NOT clear saved credentials
    WiFi.reconnect();

    // wait up to ~8s for reconnect using stored creds (no portal)
    unsigned long t0 = millis();
    while (WiFi.status()!=WL_CONNECTED && (millis() - t0) < 8000UL) {
      delay(250);
    }

    if (WiFi.status()==WL_CONNECTED) {
      Serial.println("Reconnected.");
      configureWifiForHttp();
    } else {
      Serial.println("Reconnection failed (kept creds, not opening portal).");
    }
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
      Serial.println("I2C unstable ? GPIO fallback");
      useGpioFallback = true; initGpioFallback();
    }
  } else i2cFailCount = 0;
}

void initGpioFallback() {
  useGpioFallback = true;

  for (uint8_t i = 0; i < MAX_ZONES; i++) {
    gpioInitOutput(zonePins[i], zoneGpioActiveLow[i]);
  }

  gpioInitOutput(mainsPin, mainsGpioActiveLow);
  gpioInitOutput(tankPin, tankGpioActiveLow);
  gpioInitOutput(powerSupplyPin, powerSupplyActiveLow);
}

void initGpioPinsForZones() {
  // Initialise GPIO outputs for any zones not handled by the PCF expanders.
  for (uint8_t i = 0; i < zonesCount && i < MAX_ZONES; i++) {
    if (useExpanderForZone(i)) continue;
    gpioInitOutput(zonePins[i], zoneGpioActiveLow[i]);
  }

  // Prepare mains/tank GPIO pins when using fallback or when running more than 4 zones
  // (so PCF pins remain dedicated to zone control).
  if (useGpioFallback || zonesCount > 4) {
    gpioInitOutput(mainsPin, mainsGpioActiveLow);
    gpioInitOutput(tankPin, tankGpioActiveLow);
  }

  gpioInitOutput(powerSupplyPin, powerSupplyActiveLow);
}

// ---------- Manual hardware buttons (select + start/stop) ----------
static void resetManualButtonState(ManualButtonState& state, int pin) {
  int level = HIGH;
  if (pin >= 0) level = digitalRead(pin);
  state.stableState = level;
  state.lastRawState = level;
  state.lastRawChangeMs = millis();
  state.lastPressMs = 0;
}

static bool consumeManualButtonPress(int pin, ManualButtonState& state, uint32_t nowMs) {
  if (pin < 0) return false;

  const int raw = digitalRead(pin);
  if (raw != state.lastRawState) {
    state.lastRawState = raw;
    state.lastRawChangeMs = nowMs;
  }
  if ((uint32_t)(nowMs - state.lastRawChangeMs) < MANUAL_BTN_DEBOUNCE_MS) return false;
  if (raw == state.stableState) return false;

  state.stableState = raw;
  if (raw != LOW) return false;
  if ((uint32_t)(nowMs - state.lastPressMs) < MANUAL_BTN_REPEAT_GUARD_MS) return false;

  state.lastPressMs = nowMs;
  return true;
}

static int firstActiveZoneIndex() {
  for (int i = 0; i < (int)zonesCount; ++i) {
    if (zoneActive[i]) return i;
  }
  return -1;
}

void showManualSelection() {
  manualScreenUntilMs = millis() + 15000UL;
  g_forceManualReset = true;
  drawManualSelection();
}

void drawManualSelection() {
  if (!displayEnabled) return;
  const int selectedZone = manualSelectedZone % zonesCount;
  const bool selectedActive = zoneActive[selectedZone];
  const char* selectedZoneName = zoneNames[selectedZone].c_str();

  if (!displayUseTft) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("Manual");
    display.setCursor(0, 24);
    display.print(selectedZoneName);
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("State: ");
    display.print(selectedActive ? "ON" : "OFF");
    display.display();
    return;
  }

  const int W = tft.width();
  const int H = tft.height();
  static bool init = false;
  static int lastW = -1;
  static int lastH = -1;
  static int lastZone = -1;
  static int lastStatusId = -1;
  static int lastActiveZone = -2;
  static bool lastQueued = false;
  static bool lastPinReady = false;
  static bool lastSelectedActive = false;

  const bool layoutChanged = g_forceManualReset || !init || lastW != W || lastH != H;
  const int activeZone = firstActiveZoneIndex();
  const bool selectedQueued = pendingStart[selectedZone];
  const bool pinReady = useExpanderForZone(selectedZone) ||
                        isValidOutputPin(zonePins[selectedZone]);

  int statusId = 0;
  const char* statusText = "READY";
  uint16_t statusColor = C_GOOD;
  char detailLine[64] = "Press Start to toggle";

  if (!pinReady) {
    statusId = 1;
    statusText = "NO PIN";
    statusColor = C_BAD;
    snprintf(detailLine, sizeof(detailLine), "Assign an output pin for %s", selectedZoneName);
  } else if (selectedActive) {
    statusId = 2;
    statusText = "ACTIVE";
    statusColor = C_GOOD;
    snprintf(detailLine, sizeof(detailLine), "Running now");
  } else if (selectedQueued) {
    statusId = 3;
    statusText = "QUEUED";
    statusColor = C_WARN;
    snprintf(detailLine, sizeof(detailLine), "Waiting for wind or active run");
  } else if (!systemMasterEnabled) {
    statusId = 4;
    statusText = "MANUAL";
    statusColor = C_ACCENT;
    snprintf(detailLine, sizeof(detailLine), "Master off: manual control ready");
  } else if (isBlockedNow()) {
    statusId = 4;
    statusText = "BLOCKED";
    statusColor = C_BAD;
    if (isPausedNow()) snprintf(detailLine, sizeof(detailLine), "System pause is active");
    else snprintf(detailLine, sizeof(detailLine), "After-Rain Delay is active");
  } else if (rainActive) {
    statusId = 5;
    statusText = "RAIN";
    statusColor = C_BAD;
    snprintf(detailLine, sizeof(detailLine), "Rain delay is active");
  } else if (windBlocksZone(selectedZone)) {
    statusId = 7;
    statusText = "WIND";
    statusColor = C_WARN;
    snprintf(detailLine, sizeof(detailLine), "Start will queue until wind clears");
  } else if (!runZonesConcurrent && activeZone >= 0 && activeZone != selectedZone) {
    statusId = 6;
    statusText = "BUSY";
    statusColor = C_WARN;
    snprintf(detailLine, sizeof(detailLine), "%s is already running", zoneNames[activeZone].c_str());
  }

  if (layoutChanged) {
    tft.fillScreen(C_BG);
    drawTopBar("Manual", "BTN", C_ACCENT);

    int cardW = W - 24;
    int cardH = 108;
    int cardX = 12;
    int cardY = (H - cardH) / 2 - 10;
    if (cardY < 44) cardY = 44;
    drawCard(cardX, cardY, cardW, cardH, C_PANEL, C_EDGE);
    tft.drawFastHLine(cardX + 1, cardY + 58, cardW - 2, C_EDGE);
    tft.drawFastHLine(cardX + 1, cardY + 84, cardW - 2, C_EDGE);

    init = true;
    lastW = W;
    lastH = H;
    lastZone = -1;
    lastStatusId = -1;
    lastActiveZone = -2;
    lastQueued = false;
    lastPinReady = false;
    lastSelectedActive = false;
    g_forceManualReset = false;
  }

  if (layoutChanged || lastZone != selectedZone || lastStatusId != statusId ||
      lastActiveZone != activeZone || lastQueued != selectedQueued || lastPinReady != pinReady ||
      lastSelectedActive != selectedActive) {
    int cardW = W - 24;
    int cardH = 108;
    int cardX = 12;
    int cardY = (H - cardH) / 2 - 10;
    if (cardY < 44) cardY = 44;

    int pillW = 10 + (int)strlen(statusText) * 6;
    int pillX = cardX + cardW - pillW - 12;
    int titleX = cardX + 12;
    int maxTitleW = pillX - titleX - 8;
    int16_t titleX1 = 0, titleY1 = 0;
    uint16_t titleW = 0, titleH = 0;
    uint8_t titleSize = 2;
    tft.setTextSize(titleSize);
    tft.getTextBounds(selectedZoneName, 0, 0, &titleX1, &titleY1, &titleW, &titleH);
    if ((int)titleW > maxTitleW) {
      titleSize = 1;
      tft.setTextSize(titleSize);
      tft.getTextBounds(selectedZoneName, 0, 0, &titleX1, &titleY1, &titleW, &titleH);
    }

    tft.fillRect(cardX + 10, cardY + 10, cardW - 20, cardH - 20, C_PANEL);
    tft.setTextColor(C_TEXT);
    tft.setTextSize(titleSize);
    tft.setCursor(titleX, cardY + (titleSize == 2 ? 18 : 22));
    tft.print(selectedZoneName);

    tft.fillRect(pillX, cardY + 16, pillW, 18, statusColor);
    tft.drawRect(pillX, cardY + 16, pillW, 18, C_EDGE);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_BLACK);
    tft.setCursor(pillX + 5, cardY + 21);
    tft.print(statusText);

    tft.setTextColor(C_MUTED);
    tft.setCursor(cardX + 12, cardY + 66);
    tft.print(detailLine);

    const char* stateText = selectedActive ? "ON" : "OFF";
    uint16_t stateColor = selectedActive ? C_GOOD : C_BAD;
    int statePillW = 24;
    int statePillX = cardX + 12;
    int statePillY = cardY + 88;
    tft.fillRect(statePillX, statePillY, statePillW, 14, stateColor);
    tft.drawRect(statePillX, statePillY, statePillW, 14, C_EDGE);
    tft.setTextColor(ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setCursor(statePillX + 5, statePillY + 4);
    tft.print(stateText);
    tft.setTextColor(C_TEXT);
    tft.setCursor(cardX + 42, cardY + 92);
    tft.print("Selected zone is ");
    tft.print(stateText);
    tft.setTextColor(C_MUTED);
    tft.setCursor(cardX + 168, cardY + 92);
    tft.print("| Start toggles");

    lastZone = selectedZone;
    lastStatusId = statusId;
    lastActiveZone = activeZone;
    lastQueued = selectedQueued;
    lastPinReady = pinReady;
    lastSelectedActive = selectedActive;
  }
}

void initManualButtons() {
  if (manualSelectedZone >= zonesCount) manualSelectedZone = 0;

  if (manualSelectPin >= 0 && manualSelectPin <= 39) {
    pinMode(manualSelectPin, INPUT_PULLUP);
    resetManualButtonState(g_manualSelectBtn, manualSelectPin);
  } else {
    manualSelectPin = -1;
    resetManualButtonState(g_manualSelectBtn, -1);
  }

  if (manualStartPin >= 0 && manualStartPin <= 39) {
    pinMode(manualStartPin, INPUT_PULLUP);
    resetManualButtonState(g_manualStartBtn, manualStartPin);
  } else {
    manualStartPin = -1;
    resetManualButtonState(g_manualStartBtn, -1);
  }
}

void tickManualButtons() {
  const uint32_t nowMs = millis();
  if (zonesCount == 0) return;
  manualSelectedZone = manualSelectedZone % zonesCount;

  if (consumeManualButtonPress(manualSelectPin, g_manualSelectBtn, nowMs)) {
    manualSelectedZone = (manualSelectedZone + 1) % zonesCount;
    Serial.printf("[BTN] Manual select -> Z%d\n", manualSelectedZone + 1);
    showManualSelection();
  }

  if (consumeManualButtonPress(manualStartPin, g_manualStartBtn, nowMs)) {
    uint8_t z = manualSelectedZone % zonesCount;
    if (zoneActive[z]) {
      turnOffValveManual(z);
    } else {
      turnOnValveManual(z);
    }
  }
}

// ---------- Weather / Forecast ----------
int meteoSoilPercent(float fraction) {
  if (!isfinite(fraction) || fraction < 0.0f || fraction > 1.0f) return -1;
  return (int)lroundf(fraction * 100.0f);
}

// Use automatic model selection independently of the weather model: not all
// weather models provide the 0-1 cm layer. The first hour is the current hour.
void updateMeteoMoisture() {
  if (!moistureProbeEnabled || !moistureUseMeteo) return;
  const unsigned long nowMs = millis();
  if (meteoMoistureAttempted && nowMs - meteoMoistureAttemptMs < 15UL * 60UL * 1000UL) return;
  meteoMoistureAttempted = true;
  meteoMoistureAttemptMs = nowMs;
  meteoMoisturePct = -1;
  if (!isValidLatLon(meteoLat, meteoLon)) return;
  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(meteoLat, 6) +
    "&longitude=" + String(meteoLon, 6) +
    "&hourly=soil_moisture_0_to_1cm&forecast_hours=1&timeformat=unixtime&timezone=GMT";
  int code = 0;
  String payload = httpGetMeteo(url, code, meteoHttpTimeoutMs());
  if (code != 200) return;
  JsonDocument data;
  if (deserializeJson(data, payload) != DeserializationError::Ok) return;
  JsonVariant value = data["hourly"]["soil_moisture_0_to_1cm"][0];
  if (!value.is<float>()) return;
  const float fraction = value.as<float>();
  meteoMoisturePct = meteoSoilPercent(fraction);
  meteoMoistureFetchedMs = millis();
}

String fetchWeather() {
  if (!isValidLatLon(meteoLat, meteoLon)) return "";
  String model = cleanMeteoModel(meteoModel);
  lastWeatherError = "";
  lastWeatherHttpCode = 0;

  auto buildUrl = [&](bool useSelectedModel) -> String {
    String url = meteoBaseUrl(model, useSelectedModel);
    url += "?latitude=" + String(meteoLat,6) + "&longitude=" + String(meteoLon,6);
    url += "&current=temperature_2m,relative_humidity_2m,apparent_temperature,pressure_msl,surface_pressure,"
           "wind_speed_10m,wind_gusts_10m,wind_direction_10m,precipitation,weather_code";
    if (useSelectedModel && model != "best_match") url += "&models=" + model;
    url += "&temperature_unit=celsius&wind_speed_unit=ms&precipitation_unit=mm&pressure_unit=hPa&timezone=auto";
    return url;
  };

  for (int pass = 0; pass < 2; ++pass) {
    bool useSelectedModel = (pass == 0);
    String url = buildUrl(useSelectedModel);
    int code = 0;
    String payload = httpGetMeteo(url, code, meteoHttpTimeoutMs());
    lastWeatherHttpCode = code;

    if (code == 200 && payload.length() && !isMeteoErrorPayload(payload)) {
      if (payload.indexOf("\"current\"") != -1) return payload;
      lastWeatherError = "No current in response";
    } else {
      String reason = meteoErrorReason(payload);
      if (reason.length()) lastWeatherError = reason;
      else if (code != 200) lastWeatherError = "HTTP " + String(code);
      else lastWeatherError = "Empty payload";
    }
  }

  // Fallback: derive current from hourly
  String hourlyPayload = fetchWeatherHourlyForCurrent(model, meteoLat, meteoLon, true);
  if (hourlyPayload.length()) {
    String out;
    if (buildCurrentFromHourlyPayload(hourlyPayload, out)) return out;
  }
  String hourlyPayloadForecast = fetchWeatherHourlyForCurrent(model, meteoLat, meteoLon, false);
  if (hourlyPayloadForecast.length()) {
    String out;
    if (buildCurrentFromHourlyPayload(hourlyPayloadForecast, out)) return out;
  }
  return "";
}

String fetchWeatherHourlyForCurrent(const String& model, float lat, float lon, bool useSelectedModel) {
  if (!isValidLatLon(lat, lon)) return "";
  String url = meteoBaseUrl(model, useSelectedModel);
  url += "?latitude=" + String(lat,6) + "&longitude=" + String(lon,6);
  url += "&hourly=temperature_2m,relative_humidity_2m,apparent_temperature,pressure_msl,surface_pressure,"
         "wind_speed_10m,wind_gusts_10m,wind_direction_10m,precipitation,weather_code";
  if (useSelectedModel && model != "best_match") url += "&models=" + model;
  url += "&forecast_hours=48&timeformat=unixtime&temperature_unit=celsius&wind_speed_unit=ms"
         "&precipitation_unit=mm&pressure_unit=hPa&timezone=auto";
  int code = 0;
  String payload = httpGetMeteo(url, code, meteoHttpTimeoutMs());
  if (code != 200 || isMeteoErrorPayload(payload)) return "";
  return payload;
}

bool buildCurrentFromHourlyPayload(const String& hourlyPayload, String& outPayload) {
  if (isMeteoErrorPayload(hourlyPayload)) return false;
  JsonDocument js;
  if (deserializeJson(js, hourlyPayload) != DeserializationError::Ok) {
    return false;
  }
  JsonObject hourly = js["hourly"].as<JsonObject>();
  JsonArray timeArr = hourly["time"].as<JsonArray>();
  if (!timeArr.size()) return false;

  time_t now = time(nullptr);
  int bestIdx = 0;
  long bestDiff = LONG_MAX;
  for (int i = 0; i < (int)timeArr.size(); ++i) {
    long t = timeArr[i].as<long>();
    long d = labs(t - (long)now);
    if (d < bestDiff) {
      bestDiff = d;
      bestIdx = i;
    }
  }

  auto pick = [&](const char* key) -> float {
    JsonArray arr = hourly[key].as<JsonArray>();
    if (!arr.size() || bestIdx >= (int)arr.size()) return NAN;
    return arr[bestIdx].as<float>();
  };

  JsonDocument out;
  out["utc_offset_seconds"] = js["utc_offset_seconds"] | 0;
  JsonObject cur = out["current"].to<JsonObject>();
  cur["time"] = timeArr[bestIdx] | 0;
  cur["temperature_2m"]       = pick("temperature_2m");
  cur["relative_humidity_2m"] = pick("relative_humidity_2m");
  cur["apparent_temperature"] = pick("apparent_temperature");
  cur["pressure_msl"]         = pick("pressure_msl");
  cur["surface_pressure"]     = pick("surface_pressure");
  cur["wind_speed_10m"]       = pick("wind_speed_10m");
  cur["wind_gusts_10m"]       = pick("wind_gusts_10m");
  cur["wind_direction_10m"]   = pick("wind_direction_10m");
  cur["precipitation"]        = pick("precipitation");
  float wcode = pick("weather_code");
  cur["weather_code"]         = isfinite(wcode) ? (int)wcode : -1;

  outPayload = "";
  serializeJson(out, outPayload);
  return outPayload.length() > 0;
}

String fetchForecast(float lat, float lon) {
  if (!isValidLatLon(lat, lon)) return "";
  String model = cleanMeteoModel(meteoModel);
  lastForecastError = "";
  lastForecastHttpCode = 0;

  auto buildUrl = [&](bool useSelectedModel) -> String {
    String url = meteoBaseUrl(model, useSelectedModel);
    url += "?latitude=" + String(lat,6) + "&longitude=" + String(lon,6);
    url += "&hourly=precipitation,precipitation_probability,wind_gusts_10m"
           "&daily=temperature_2m_min,temperature_2m_max,sunrise,sunset"
           "&forecast_hours=24&forecast_days=2";
    if (useSelectedModel && model != "best_match") url += "&models=" + model;
    url += "&temperature_unit=celsius&wind_speed_unit=ms&precipitation_unit=mm&pressure_unit=hPa&timezone=auto";
    return url;
  };

  for (int pass = 0; pass < 2; ++pass) {
    bool useSelectedModel = (pass == 0);
    String url = buildUrl(useSelectedModel);
    int code = 0;
    String payload = httpGetMeteo(url, code, meteoHttpTimeoutMs());
    lastForecastHttpCode = code;
    if (code == 200 && payload.length() && !isMeteoErrorPayload(payload)) {
      return payload;
    }
    String reason = meteoErrorReason(payload);
    if (reason.length()) lastForecastError = reason;
    else if (code != 200) lastForecastError = "HTTP " + String(code);
    else lastForecastError = "Empty payload";
  }
  return "";
}

// ---------- NEW helpers for rain history ----------
// Rolling 24h rain history (ACTUAL ONLY, no forecast).
// Record once per elapsed hour instead of relying on a narrow HH:00 window.
static void loadRainHistoryState() {
  if (!LittleFS.exists(RAIN_HISTORY_STATE_PATH)) return;

  File f = LittleFS.open(RAIN_HISTORY_STATE_PATH, "r");
  if (!f) return;

  String magic = f.readStringUntil('\n');
  magic.trim();
  if (magic != "RAINHIST1") {
    f.close();
    clearRainHistoryState();
    return;
  }

  int idx = f.readStringUntil('\n').toInt();
  time_t hour = (time_t)strtoull(f.readStringUntil('\n').c_str(), nullptr, 10);
  if (idx < 0 || idx >= 24 || hour < 0) {
    f.close();
    clearRainHistoryState();
    return;
  }

  float loaded[24] = {0};
  for (int i = 0; i < 24 && f.available(); ++i) {
    loaded[i] = f.readStringUntil('\n').toFloat();
    if (!isfinite(loaded[i]) || loaded[i] < 0.0f) loaded[i] = 0.0f;
  }
  f.close();

  rainIdx = idx;
  lastRainHistHour = hour;
  for (int i = 0; i < 24; ++i) rainHist[i] = loaded[i];
  lastRainAmount = last24hActualRain();
}

static void saveRainHistoryState() {
  File f = LittleFS.open(RAIN_HISTORY_STATE_PATH, "w");
  if (!f) {
    Serial.println("[RAIN] Unable to persist rain history");
    return;
  }

  f.println("RAINHIST1");
  f.println(rainIdx);
  char hourBuf[24];
  snprintf(hourBuf, sizeof(hourBuf), "%llu", (unsigned long long)lastRainHistHour);
  f.println(hourBuf);
  for (int i = 0; i < 24; ++i) {
    float v = rainHist[i];
    if (!isfinite(v) || v < 0.0f) v = 0.0f;
    f.println(v, 3);
  }
  f.close();
}

static void clearRainHistoryState() {
  for (int i = 0; i < 24; ++i) rainHist[i] = 0.0f;
  rainIdx = 0;
  lastRainHistHour = 0;
  lastRainAmount = 0.0f;
  if (LittleFS.exists(RAIN_HISTORY_STATE_PATH)) {
    LittleFS.remove(RAIN_HISTORY_STATE_PATH);
  }
}

static void tickActualRainHistory() {
  time_t now = time(nullptr);
  if (now <= 0) return;

  const time_t thisHour = now - (now % 3600);
  float v = rain1hNow;

  // Sanity: NaN / negative => 0
  if (!isfinite(v) || v < 0.0f) {
    v = 0.0f;
  }

  // Seed the current hour immediately so reboots or loop jitter do not
  // require hitting a tiny HH:00 window before history starts moving again.
  if (lastRainHistHour == 0) {
    rainIdx = (rainIdx + 1) % 24;
    rainHist[rainIdx] = v;
    lastRainHistHour = thisHour;
    saveRainHistoryState();
    return;
  }

  if (thisHour <= lastRainHistHour) {
    if (fabsf(rainHist[rainIdx] - v) >= 0.01f) {
      rainHist[rainIdx] = v;
      saveRainHistoryState();
    }
    return;
  }

  int hoursElapsed = (int)((thisHour - lastRainHistHour) / 3600);
  if (hoursElapsed > 24) hoursElapsed = 24;

  // Advance one slot per elapsed hour to preserve a true rolling 24h window.
  // If several hours were skipped, fill the gap with zeros and store the
  // latest observed rain value in the newest hour slot.
  for (int h = 1; h <= hoursElapsed; ++h) {
    rainIdx = (rainIdx + 1) % 24;
    rainHist[rainIdx] = (h == hoursElapsed) ? v : 0.0f;
  }

  lastRainHistHour = thisHour;
  saveRainHistoryState();
}

// Sum rolling 24h actual rainfall from the ring buffer
static float last24hActualRain() {
  float total = 0.0f;
  for (int i = 0; i < 24; ++i) {
    float v = rainHist[i];
    if (!isfinite(v) || v < 0.0f) {
      v = 0.0f;
    }
    total += v;
  }
  return total;
}

bool refreshCurrentWeatherSnapshotFromCache() {
  const float oldPressure = curPressureHpa;
  curTempC = NAN;
  curFeelsC = NAN;
  curPressureHpa = NAN;
  curPressureTrend = 0;
  curWindMs = NAN;
  curGustMs = NAN;
  curWindDirDeg = NAN;
  curHumidityPct = -1;
  curWeatherCode = -1;
  curUtcOffsetSec = 0;
  curWeatherValid = false;
  rain1hNow = 0.0f;
  rain3hNow = 0.0f;

  if (!cachedWeatherData.length()) return false;

  JsonDocument js;
  if (deserializeJson(js, cachedWeatherData) != DeserializationError::Ok) {
    return false;
  }

  JsonObject cur = js["current"].as<JsonObject>();
  curTempC = cur["temperature_2m"] | NAN;
  curFeelsC = cur["apparent_temperature"] | NAN;
  curHumidityPct = cur["relative_humidity_2m"] | -1;
  float pmsl = cur["pressure_msl"] | NAN;
  float psfc = cur["surface_pressure"] | NAN;
  curPressureHpa = isfinite(pmsl) ? pmsl : psfc;
  if (isfinite(curPressureHpa) && isfinite(oldPressure)) {
    const float delta = curPressureHpa - oldPressure;
    curPressureTrend = (delta > 0.2f) ? 1 : ((delta < -0.2f) ? -1 : 0);
    prevPressureHpa = oldPressure;
  } else {
    curPressureTrend = 0;
    prevPressureHpa = NAN;
  }
  curWindMs = cur["wind_speed_10m"] | NAN;
  curGustMs = cur["wind_gusts_10m"] | NAN;
  curWindDirDeg = cur["wind_direction_10m"] | NAN;
  curWeatherCode = cur["weather_code"] | -1;
  curUtcOffsetSec = js["utc_offset_seconds"] | 0;

  float r1 = cur["precipitation"] | 0.0f;
  if (!isfinite(r1) || r1 < 0.0f) r1 = 0.0f;
  rain1hNow = r1;
  rain3hNow = 0.0f;

  curWeatherValid = true;
  return true;
}



void updateCachedWeather() {
  // NEW: never start fetches while handling HTTP requests
  if (g_inHttp) return;
  // On erased flash, do not make HTTPS weather requests before the user has
  // saved setup once. This keeps first WiFi provisioning focused on bringing
  // up the local UI and avoids early heap-heavy TLS work after WiFiManager.
  if (!g_configLoadedFromFs) return;

  updateMeteoMoisture();

  unsigned long nowms = millis();
  bool needCur = (cachedWeatherData == "" ||
                  (nowms - lastWeatherUpdate >= weatherUpdateInterval));
  bool haveCoord = isValidLatLon(meteoLat, meteoLon);
  bool weatherChanged = false;

  if (needCur) {
    String fresh = fetchWeather();
    if (fresh.length() > 0) {
      weatherChanged = (fresh != cachedWeatherData);
      cachedWeatherData = fresh;
      lastWeatherUpdate = nowms;
    }
  }

  // Decode once when weather payload changes (or first run).
  static bool snapshotReady = false;
  if (!snapshotReady || weatherChanged || !curWeatherValid) {
    refreshCurrentWeatherSnapshotFromCache();
    snapshotReady = true;
  }
  applyLocalClimateOverride();

  // ---- Forecast fetch / parse ----
  if (haveCoord && (cachedForecastData == "" || (nowms - lastForecastUpdate >= forecastUpdateInterval))) {
    String freshFc = fetchForecast(meteoLat, meteoLon);
    if (freshFc.length() > 0) {
      cachedForecastData = freshFc;
      lastForecastUpdate = nowms;

      rainNext12h_mm = 0; 
      rainNext24h_mm = 0; 
      popNext12h_pct = 0; 
      nextRainIn_h   = -1;
      maxGust24h_ms  = 0; 
      todaySunrise   = 0;   
      todaySunset    = 0;

      JsonDocument fc;
      if (deserializeJson(fc, cachedForecastData) == DeserializationError::Ok) {
        JsonObject daily = fc["daily"].as<JsonObject>();
        JsonArray sunr = daily["sunrise"].as<JsonArray>();
        JsonArray suns = daily["sunset"].as<JsonArray>();
        JsonArray tmin = daily["temperature_2m_min"].as<JsonArray>();
        JsonArray tmax = daily["temperature_2m_max"].as<JsonArray>();

        if (sunr.size() > 0) todaySunrise = parseLocalIsoTime(sunr[0] | "");
        if (suns.size() > 0) todaySunset  = parseLocalIsoTime(suns[0] | "");
        if (tmin.size() > 0) todayMin_C   = tmin[0] | todayMin_C;
        if (tmax.size() > 0) todayMax_C   = tmax[0] | todayMax_C;

        JsonObject hourly = fc["hourly"].as<JsonObject>();
        JsonArray prec = hourly["precipitation"].as<JsonArray>();
        JsonArray pop  = hourly["precipitation_probability"].as<JsonArray>();
        JsonArray gust = hourly["wind_gusts_10m"].as<JsonArray>();

        int hrs = max((int)prec.size(), max((int)pop.size(), (int)gust.size()));
        int L24 = min(24, hrs);
        int L12 = min(12, L24);
        for (int i = 0; i < L24; i++) {
          float r = (prec.size() > i) ? prec[i].as<float>() : 0.0f;
          if (!isfinite(r) || r < 0.0f) r = 0.0f;
          if (i < L12) {
            rainNext12h_mm += r;
            int p = (pop.size() > i) ? pop[i].as<int>() : 0;
            if (p > popNext12h_pct) popNext12h_pct = p;
          }
          rainNext24h_mm += r;
          if (nextRainIn_h < 0) {
            int p = (pop.size() > i) ? pop[i].as<int>() : 0;
            if (r > 0.1f || p >= 50) nextRainIn_h = i;
          }
          float g = (gust.size() > i) ? gust[i].as<float>() : 0.0f;
          if (g > maxGust24h_ms) maxGust24h_ms = g;
        }
      }
      if (isnan(rainNext12h_mm) || rainNext12h_mm < 0) rainNext12h_mm = 0.0f;
      if (isnan(rainNext24h_mm) || rainNext24h_mm < 0) rainNext24h_mm = 0.0f;
      if (isnan(maxGust24h_ms)  || maxGust24h_ms  < 0) maxGust24h_ms  = 0.0f;
    }
  }

  // Fallback min/max from current weather snapshot
  if (isfinite(curTempC)) {
    if (!isfinite(todayMin_C) || curTempC < todayMin_C) todayMin_C = curTempC;
    if (!isfinite(todayMax_C) || curTempC > todayMax_C) todayMax_C = curTempC;
  }

  // Roll hourly history
  tickActualRainHistory();
}

// NEW: called only from loop()
void tickWeather(){
  static uint32_t lastTick = 0;
  uint32_t nowMs = millis();
  const uint32_t WEATHER_TICK_MS = 10000UL; // every 10s is plenty

  if (nowMs - lastTick < WEATHER_TICK_MS) {
    return;
  }
  lastTick = nowMs;

  updateCachedWeather();
}

// ---------- Rain/Wind logic with cooldown & threshold ----------
bool checkWindRain() {
  // Throttle to at most once per second
  static uint32_t lastCheckMs = 0;
  static bool     lastResult  = false;
  // Track only the kind of "rain" that is allowed to start cooldown
  static bool     prevCoolRain = false;

  uint32_t nowMs = millis();
  if (nowMs - lastCheckMs < 1000UL) {
    // Reuse last computed state (rainActive/windActive already set)
    return lastResult;
  }
  lastCheckMs = nowMs;

  bool newWeatherRainActual = false;   // raw "is it raining now" from Open-Meteo
  bool newSensorRainActual  = false;   // raw sensor state
  bool newWindActual        = false;   // raw wind above threshold

  // --- 1) Use decoded current-weather snapshot (no JSON parse in fast loop) ---
  if (curWeatherValid) {
    float rainNow = rain1hNow;
    if (!isfinite(rainNow) || rainNow < 0.0f) rainNow = 0.0f;
    const float MAX_RAIN1H_FOR_LOGIC = 20.0f;  // mm
    if (rainNow > MAX_RAIN1H_FOR_LOGIC) rainNow = MAX_RAIN1H_FOR_LOGIC;

    if (rainNow > 0.0f) {
      newWeatherRainActual = true;
    } else if (curWeatherCode >= 0 && meteoCodeIsWet(curWeatherCode)) {
      newWeatherRainActual = true;
    }

    if (windSpeedThreshold > 0.0f && isfinite(curWindMs)) {
      newWindActual = (curWindMs >= windSpeedThreshold);
    }
  }

  // --- 2) Physical rain sensor (raw state) ---
  newSensorRainActual = physicalRainNowRaw();

  // --- 3) Accumulated rainfall (24h ACTUAL ONLY) ---
  float actual24 = last24hActualRain();
  bool aboveThreshold = false;
  if (rainThreshold24h_mm > 0) {
    if (actual24 >= rainThreshold24h_mm) {
      aboveThreshold = true;
    }
  }

  // --- 4) Apply feature gates and build "effective" flags ---
  bool effectiveInstantRain =
      rainDelayEnabled &&
      rainDelayFromForecastEnabled &&
      newWeatherRainActual;

  // 24h threshold rain: only when actual 24h rainfall >= user threshold.
  bool effectiveThresholdRain =
      rainDelayEnabled &&
      rainDelayFromForecastEnabled &&
      (rainThreshold24h_mm > 0) &&
      aboveThreshold;

  // Physical sensor rain: used for both blocking + cooldown.
  bool effectiveSensorRain =
      rainDelayEnabled &&
      rainSensorEnabled &&
      newSensorRainActual;

  // Public flags / state
  rainByWeatherActive = effectiveInstantRain || effectiveThresholdRain;
  rainBySensorActive  = effectiveSensorRain;
  rainActive          = (rainByWeatherActive || rainBySensorActive);

  windActive = (windDelayEnabled && newWindActual);

  // --- 5) Cooldown logic (ONLY threshold/sensor-based rain) ---
  // Define which rain sources are allowed to start cooldown when they clear:
  //  - 24h threshold actual rainfall
  //  - Physical rain sensor
  bool coolRainNow = (effectiveThresholdRain || effectiveSensorRain);

  time_t now = time(nullptr);

  // (a) If we *previously* had threshold/sensor rain, and now it's clear,
  //     start cooldown window.
  if (prevCoolRain && !coolRainNow) {
    if (rainDelayEnabled && rainCooldownHours > 0) {
      rainCooldownUntilEpoch =
          (uint32_t)now + (uint32_t)rainCooldownHours * 3600UL;
    } else {
      rainCooldownUntilEpoch = 0;
    }
  }

  // (b) While threshold/sensor rain is active, there is no cooldown.
  if (coolRainNow) {
    rainCooldownUntilEpoch = 0;
  }

  // (c) When cooldown expires, clear it.
  if (!coolRainNow &&
      rainCooldownUntilEpoch > 0 &&
      (uint32_t)now >= rainCooldownUntilEpoch) {
    rainCooldownUntilEpoch = 0;
  }

  // Track last 24h amount for UI (RainScreen "Last: x.xx mm")
  lastRainAmount = actual24;

  // Remember for next edge detection
  prevCoolRain = coolRainNow;

  // Final combined state that the rest of the code cares about
  lastResult = (rainActive || windActive);
  return lastResult;
}

// ---------- Event log ----------
static bool isRunLogEvent(const char* eventType) {
  return eventType && (strcmp(eventType, "START") == 0 || strcmp(eventType, "STOPPED") == 0);
}

void logEvent(int zone, const char* eventType, const char* source, bool rainDelayed) {
  if (!isRunLogEvent(eventType)) return;

  // Track runtime for scheduled/manual runs on STOPPED events.
  if (strcmp(eventType, "STOPPED") == 0) {
    bool isManual = false;
    if (zone >= 0 && zone < MAX_ZONES) {
      isManual = zoneStartedManual[zone];
    }
    unsigned long dur = elapsedRunSec(zone);
    if (isManual) totalManualRuntimeSec += dur;
    else totalScheduledRuntimeSec += dur;
  }

  updateCachedWeather(); // safe early-out if g_inHttp==true, keeps details recent enough
  float temp = curTempC;
  float wind = curWindMs;
  int hum = (curHumidityPct >= 0) ? curHumidityPct : 0;
  String cond = (curWeatherCode >= 0) ? meteoCodeToMain(curWeatherCode) : "?";
  String cname = meteoLocationLabel();

  File f = LittleFS.open("/events.csv","a");
  if (!f) return;

  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  char ts[32];
  sprintf(ts,"%04d-%02d-%02d %02d:%02d:%02d", t->tm_year+1900,t->tm_mon+1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);

  String line; line.reserve(280);
  line += ts; line += ","; line += (zone>=0?zoneNames[zone]:String("n/a")); line += ",";
  line += eventType; line += ","; line += source; line += ",";
  line += (rainDelayed?"Active":"Off"); line += ",";
  line += String(temp,1); line += ","; line += String(hum); line += ",";
  line += String(wind,1); line += ","; line += cond; line += ","; line += cname;
  if (smartWateringEnabled) {
    const int smartPct = (int)lroundf(smartWateringFactor() * 100.0f);
    line += F("; Factor ");
    line += String(smartPct);
    line += F("%");
    line += F("; Moisture ");
    if (!moistureProbeEnabled) {
      line += F("Disabled");
    } else {
      const int moisturePct = moisturePercent();
      if (moisturePct < 0) line += F("No valid reading");
      else {
        line += String(moisturePct);
        line += F("% wet");
      }
    }
  }
  line += "\n";

  f.print(line); f.close();
}

// ---------- OLED UI ----------
void toggleBacklight(){
  if (!displayEnabled) return;
  if (displayEnabled && !displayUseTft) {
    static bool inverted=false;
    inverted=!inverted;
    display.invertDisplay(inverted);
    return;
  }
  static bool on = true;
  on = !on;
  tftBacklight(on);
}

void updateLCDForZone(int zone) {
  if (!displayEnabled) return;
  static unsigned long lastUpdate = 0; unsigned long now = millis();
  const unsigned long minUiInterval = displayUseTft ? 180UL : 1000UL;
  if (!g_forceRunReset && (now - lastUpdate < minUiInterval)) return;
  lastUpdate = now;

  unsigned long elapsed = (now - zoneStartMs[zone]) / 1000;
  unsigned long total = zoneRunTotalSec[zone];
  if (total == 0) total = durationForSlot(zone, 1);
  unsigned long rem = (elapsed < total ? total - elapsed : 0);
  int pct = (total > 0) ? (int)((elapsed * 100UL) / total) : 0;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;

  char remBuf[12];
  char totalBuf[12];
  char elapsedBuf[12];
  unsigned long rm = rem / 60;
  unsigned long rs = rem % 60;
  unsigned long em = elapsed / 60;
  unsigned long es = elapsed % 60;
  unsigned long tm = total / 60;
  unsigned long ts = total % 60;
  snprintf(remBuf, sizeof(remBuf), "%02lu:%02lu", rm, rs);
  snprintf(elapsedBuf, sizeof(elapsedBuf), "%02lu:%02lu", em, es);
  snprintf(totalBuf, sizeof(totalBuf), "%02lu:%02lu", tm, ts);

  const char* src = "None";
  bool mainsOn = false, tankOn = false;
  chooseWaterSource(src, mainsOn, tankOn);
  int activeCount = 0;
  for (int i = 0; i < (int)zonesCount; ++i) if (zoneActive[i]) activeCount++;
  int tankPct = constrain(tankPercent(), 0, 100);

  static bool runInit = false;
  static bool lastModeTft = false;
  static int lastRunZone = -1;
  static int lastRunW = -1;
  static int lastRunH = -1;
  static unsigned long lastRunTotal = 0;
  static int lastRunTank = -1;
  static int lastRunActiveCount = -1;
  static char lastRunSource[12] = "";
  static char lastRunRemBuf[12] = "";
  static char lastRunElapsedBuf[12] = "";
  static char lastRunTotalBuf[12] = "";
  static int lastRunPct = -1;
  static int lastRunBarFill = -1;
  const int dispW = displayUseTft ? tft.width() : SCREEN_WIDTH;
  const int dispH = displayUseTft ? tft.height() : SCREEN_HEIGHT;
  const bool layoutChanged = g_forceRunReset || !runInit || lastModeTft != displayUseTft ||
                             lastRunZone != zone || lastRunW != dispW || lastRunH != dispH ||
                             lastRunTotal != total;
  const bool metaChanged = layoutChanged || lastRunTank != tankPct ||
                           lastRunActiveCount != activeCount || strcmp(lastRunSource, src) != 0;

  if (!displayUseTft) {
    if (layoutChanged) {
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.fillRect(0, 0, SCREEN_WIDTH, 11, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setTextSize(1);
      display.setCursor(2, 2);
      display.print("RUNNING");
      display.setCursor(88, 2);
      display.print("Z");
      display.print(zone + 1);

      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(1);
      String zoneLabel = zoneNames[zone];
      if (zoneLabel.length() > 18) zoneLabel = zoneLabel.substring(0, 18);
      display.setCursor(0, 36);
      display.print(zoneLabel);

      int barX = 0;
      int barY = 47;
      int barW = SCREEN_WIDTH;
      int barH = 8;
      display.drawRect(barX, barY, barW, barH, SSD1306_WHITE);
    }

    display.fillRect(0, 14, SCREEN_WIDTH, 18, SSD1306_BLACK);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    int timerX = 18;
    display.setCursor(timerX, 16);
    display.print(remBuf);

    int barX = 0;
    int barY = 47;
    int barW = SCREEN_WIDTH;
    int barH = 8;
    display.fillRect(barX + 1, barY + 1, barW - 2, barH - 2, SSD1306_BLACK);
    int fillW = ((barW - 2) * pct) / 100;
    if (fillW > 0) display.fillRect(barX + 1, barY + 1, fillW, barH - 2, SSD1306_WHITE);

    display.fillRect(0, 56, SCREEN_WIDTH, 8, SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(0, 57);
    display.print(elapsedBuf);
    display.print("/");
    display.print(totalBuf);
    display.setCursor(78, 57);
    display.print(src);
    display.display();
    runInit = true;
    lastModeTft = false;
    lastRunZone = zone;
    lastRunW = dispW;
    lastRunH = dispH;
    lastRunTotal = total;
    lastRunTank = tankPct;
    lastRunActiveCount = activeCount;
    strncpy(lastRunSource, src, sizeof(lastRunSource));
    lastRunSource[sizeof(lastRunSource) - 1] = '\0';
    g_forceRunReset = false;
    return;
  }

  const int W = tft.width();
  const int H = tft.height();

  const int pad = 10;
  const int headerH = 32;

  int16_t bx, by;
  uint16_t bw, bh;
  int cardX = pad;
  int cardY = headerH + 8;
  int cardW = W - pad * 2;
  int cardH = H - cardY - 10;

  tft.setTextSize((W >= 300 && H >= 220) ? 5 : 4);
  tft.setTextColor(C_ACCENT);
  int16_t remX1, remY1;
  uint16_t remW, remH;
  tft.getTextBounds(remBuf, 0, 0, &remX1, &remY1, &remW, &remH);
  int timerY = cardY + 42;
  int timerX = cardX + (cardW - (int)remW) / 2;

  int barX = cardX + 12;
  int barY = timerY + (int)remH + 12;
  int barW = cardW - 24;
  int barH = 16;

  if (layoutChanged) {
    tft.fillScreen(C_BG);
    lastRunRemBuf[0] = '\0';
    lastRunElapsedBuf[0] = '\0';
    lastRunTotalBuf[0] = '\0';
    lastRunPct = -1;
    lastRunBarFill = -1;

    tft.fillRect(0, 0, W, headerH, C_PANEL_2);
    tft.fillRect(0, 0, 4, headerH, C_ACCENT);
    tft.drawFastHLine(0, headerH, W, C_EDGE);
    tft.drawFastHLine(4, 1, max(0, W - 4), C_HILITE);
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED, C_PANEL_2);
    tft.setCursor(pad + 2, 6);
    tft.print("IRRIGATION");
    tft.setTextColor(C_GOOD);
    tft.setCursor(pad + 2, 18);
    tft.print("RUNNING");

    char zonePill[10];
    snprintf(zonePill, sizeof(zonePill), "ZONE %d", zone + 1);
    tft.getTextBounds(zonePill, 0, 0, &bx, &by, &bw, &bh);
    int pillW = (int)bw + 14;
    int pillX = W - pillW - pad;
    tft.fillRoundRect(pillX, 8, pillW, 18, 5, C_GOOD);
    tft.drawRoundRect(pillX, 8, pillW, 18, 5, C_EDGE);
    tft.setTextColor(ST77XX_BLACK, C_GOOD);
    tft.setCursor(pillX + 7, 13);
    tft.print(zonePill);

    drawCard(cardX, cardY, cardW, cardH, C_PANEL, C_EDGE);

    String zoneLabel = zoneNames[zone];
    uint8_t nameSize = 2;
    int16_t x1, y1;
    uint16_t w, h;
    tft.setTextSize(nameSize);
    tft.getTextBounds(zoneLabel.c_str(), 0, 0, &x1, &y1, &w, &h);
    if ((int)w > cardW - 24) {
      nameSize = 1;
      tft.setTextSize(nameSize);
      tft.getTextBounds(zoneLabel.c_str(), 0, 0, &x1, &y1, &w, &h);
    }
    tft.fillRect(cardX + 8, cardY + 8, cardW - 16, 24, C_PANEL);
    tft.setTextColor(C_TEXT);
    int nameX = cardX + (cardW - (int)w) / 2;
    if (nameX < cardX + 8) nameX = cardX + 8;
    tft.setCursor(nameX, cardY + 12);
    tft.print(zoneLabel);

    tft.drawRoundRect(barX, barY, barW, barH, 5, C_EDGE);
  }

  int fillW = ((barW - 4) * pct) / 100;

  char pctBuf[8];
  snprintf(pctBuf, sizeof(pctBuf), "%d%%", pct);

  if (layoutChanged || strcmp(remBuf, lastRunRemBuf) != 0) {
    tft.fillRect(cardX + 8, timerY - 2, cardW - 16, (int)remH + 6, C_PANEL);
    tft.setTextSize((W >= 300 && H >= 220) ? 5 : 4);
    tft.setTextColor(C_ACCENT);
    tft.getTextBounds(remBuf, 0, 0, &remX1, &remY1, &remW, &remH);
    timerX = cardX + (cardW - (int)remW) / 2;
    tft.setCursor(timerX, timerY);
    tft.print(remBuf);
  }

  if (layoutChanged || lastRunBarFill < 0) {
    tft.fillRect(barX + 2, barY + 2, barW - 4, barH - 4, C_PANEL);
    if (fillW > 0) tft.fillRoundRect(barX + 2, barY + 2, fillW, barH - 4, 4, C_GOOD);
  } else if (fillW > lastRunBarFill) {
    tft.fillRect(barX + 2 + lastRunBarFill, barY + 2, fillW - lastRunBarFill, barH - 4, C_GOOD);
  } else if (fillW < lastRunBarFill) {
    tft.fillRect(barX + 2 + fillW, barY + 2, lastRunBarFill - fillW, barH - 4, C_PANEL);
  }

  if (layoutChanged || strcmp(elapsedBuf, lastRunElapsedBuf) != 0 ||
      strcmp(totalBuf, lastRunTotalBuf) != 0 || pct != lastRunPct) {
    tft.fillRect(barX, barY + barH + 4, barW, 12, C_PANEL);
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(barX, barY + barH + 6);
    tft.print(elapsedBuf);
    tft.print(" / ");
    tft.print(totalBuf);
    tft.getTextBounds(pctBuf, 0, 0, &bx, &by, &bw, &bh);
    tft.setCursor(barX + barW - (int)bw, barY + barH + 6);
    tft.setTextColor(C_TEXT);
    tft.print(pctBuf);
  }

  int metaY = barY + barH + 24;
  int metaH = cardY + cardH - metaY - 10;
  if (metaChanged && metaH >= 30) {
    int boxGap = 6;
    int boxW = (cardW - 24 - boxGap * 2) / 3;
    int boxH = min(42, metaH);
    int boxX = cardX + 12;
    auto drawMini = [&](int x, const char* label, const char* value, uint16_t valueColor) {
      drawMetricBox(x, metaY, boxW, boxH);
      tft.setTextSize(1);
      tft.setTextColor(C_MUTED, C_PANEL_2);
      tft.setCursor(x + 5, metaY + 5);
      tft.print(label);
      tft.setTextColor(valueColor, C_PANEL_2);
      tft.setCursor(x + 5, metaY + 20);
      tft.print(value);
    };

    char tankBuf[12];
    snprintf(tankBuf, sizeof(tankBuf), "%d%%", tankPct);
    char activeBuf[12];
    snprintf(activeBuf, sizeof(activeBuf), "%d/%d", activeCount, (int)zonesCount);
    drawMini(boxX, "SOURCE", src, C_TEXT);
    drawMini(boxX + boxW + boxGap, "TANK", tankBuf, tankPct <= (int)tankLowThresholdPct ? C_BAD : C_GOOD);
    drawMini(boxX + (boxW + boxGap) * 2, "ACTIVE", activeBuf, C_GOOD);
  }

  tft.setTextWrap(false);
  runInit = true;
  lastModeTft = true;
  lastRunZone = zone;
  lastRunW = dispW;
  lastRunH = dispH;
  lastRunTotal = total;
  lastRunTank = tankPct;
  lastRunActiveCount = activeCount;
  strncpy(lastRunSource, src, sizeof(lastRunSource));
  lastRunSource[sizeof(lastRunSource) - 1] = '\0';
  strncpy(lastRunRemBuf, remBuf, sizeof(lastRunRemBuf));
  lastRunRemBuf[sizeof(lastRunRemBuf) - 1] = '\0';
  strncpy(lastRunElapsedBuf, elapsedBuf, sizeof(lastRunElapsedBuf));
  lastRunElapsedBuf[sizeof(lastRunElapsedBuf) - 1] = '\0';
  strncpy(lastRunTotalBuf, totalBuf, sizeof(lastRunTotalBuf));
  lastRunTotalBuf[sizeof(lastRunTotalBuf) - 1] = '\0';
  lastRunPct = pct;
  lastRunBarFill = fillW;
  g_forceRunReset = false;
}

void RainScreen(){
  if (!displayEnabled) return;
  if (displayEnabled && !displayUseTft) {
    display.clearDisplay();
    display.setTextSize(2); display.setCursor(0,0); display.print(isPausedNow()? "System Pause" : "Rain/Wind");
    display.setTextSize(1);
    display.setCursor(0,20); display.printf("Last: %.2f mm", lastRainAmount);
    display.setCursor(0,32); display.print("Cause: "); display.print(rainDelayCauseText());
    int delayed=0; for (int i=0;i<(int)zonesCount;i++) if (pendingStart[i]) delayed++;
    display.setCursor(0,46); display.printf("Queued: %d", delayed);
    display.display();
    return;
  }

  const int W = tft.width();
  const int H = tft.height();
  const int pad = 8;
  const int topY = 38;
  const int rainH = 42;
  const int rainY = topY;
  const int statusY = rainY + rainH + 6;
  int causeH = H - statusY - 8;
  if (causeH < 96) causeH = 96;

  static bool init = false;
  static int lastW = -1;
  static int lastH = -1;
  static int lastStatusId = -1;
  static char lastR1[12] = "";
  static char lastR24[12] = "";
  static char lastWind[16] = "";
  static char lastCause[40] = "";
  static char lastCooldown[24] = "";
  static char lastPauseMaster[32] = "";
  static int lastQueued = -1;
  static int lastRunning = -1;
  static int lastRssi = 9999;
  static bool lastMqtt = false;

  if (g_forceRainReset) {
    init = false;
    g_forceRainReset = false;
  }

  const bool paused = isPausedNow();
  const bool masterOff = !systemMasterEnabled;
  const bool cooldownActive = isCooldownActiveNow();
  const bool delay = (rainActive || windActive || cooldownActive);
  const int statusId = masterOff ? 3 : (paused ? 2 : (delay ? 1 : 0));

  bool layoutChanged = (!init || lastW != W || lastH != H);
  if (layoutChanged) {
    tft.fillScreen(C_BG);
    lastW = W;
    lastH = H;
    init = true;
  }

  if (layoutChanged || statusId != lastStatusId) {
    const char* pill = "READY";
    uint16_t pillColor = C_GOOD;
    if (masterOff) { pill = "MASTER OFF"; pillColor = C_BAD; }
    else if (paused) { pill = "PAUSED"; pillColor = C_WARN; }
    else if (delay) { pill = "DELAY"; pillColor = C_WARN; }
    drawTopBar("RAIN / WIND", pill, pillColor);
    lastStatusId = statusId;
  }

  int y = topY;
  if (layoutChanged) {
    drawCard(pad, y, W - 2*pad, rainH, C_PANEL, C_EDGE);
    const int colW = (W - 2 * pad) / 3;
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(pad + 4, y + 4); tft.print("1h");
    tft.setCursor(pad + colW + 4, y + 4); tft.print("24h");
    tft.setCursor(pad + 2 * colW + 4, y + 4); tft.print("Wind");

    y += rainH + 6;
    drawCard(pad, y, W - 2*pad, causeH, C_PANEL, C_EDGE);
    tft.setTextSize(2);
    tft.setTextColor(C_MUTED);
    tft.setCursor(pad + 6, y + 4);
    tft.print("Status");
  }

  const int colW = (W - 2 * pad) / 3;
  const int text2H = 18;
  const int text1H = 10;

  float windNow = curWindMs;

  char r1buf[12];
  char r24buf[12];
  char windbuf[16];

  if (isfinite(rain1hNow)) snprintf(r1buf, sizeof(r1buf), "%.1fmm", rain1hNow);
  else strncpy(r1buf, "--", sizeof(r1buf));

  if (isfinite(lastRainAmount)) snprintf(r24buf, sizeof(r24buf), "%.1fmm", lastRainAmount);
  else strncpy(r24buf, "--", sizeof(r24buf));

  if (isfinite(windNow)) snprintf(windbuf, sizeof(windbuf), "%.1f/%.1f", windNow, windSpeedThreshold);
  else snprintf(windbuf, sizeof(windbuf), "--/%.1f", windSpeedThreshold);

  r1buf[sizeof(r1buf) - 1] = '\0';
  r24buf[sizeof(r24buf) - 1] = '\0';
  windbuf[sizeof(windbuf) - 1] = '\0';

  auto updateColValue = [&](int idx, const char* val, char* last, size_t lastLen) {
    if (!layoutChanged && strcmp(val, last) == 0) return;
    int x = pad + idx * colW + 4;
    int yv = rainY + 18;
    int w = colW - 8;
    tft.fillRect(x, yv, w, text2H, C_PANEL);
    tft.setTextSize(2);
    tft.setTextColor(C_TEXT);
    tft.setCursor(x, yv);
    tft.print(val);
    strncpy(last, val, lastLen);
    last[lastLen - 1] = '\0';
  };

  updateColValue(0, r1buf, lastR1, sizeof(lastR1));
  updateColValue(1, r24buf, lastR24, sizeof(lastR24));
  updateColValue(2, windbuf, lastWind, sizeof(lastWind));

  String causeS = rainDelayCauseText();
  const time_t now = time(nullptr);
  char causeBuf[40];
  if (causeS.length()) {
    strncpy(causeBuf, causeS.c_str(), sizeof(causeBuf));
  } else {
    strncpy(causeBuf, "--", sizeof(causeBuf));
  }
  causeBuf[sizeof(causeBuf) - 1] = '\0';

  if (layoutChanged || strcmp(causeBuf, lastCause) != 0) {
    int x = pad + 6;
    int yv = statusY + 20;
    int w = W - 2 * pad - 12;
    tft.fillRect(x, yv, w, text2H, C_PANEL);
    uint16_t causeColor = C_TEXT;
    if (masterOff) causeColor = C_BAD;
    else if (paused || delay || cooldownActive) causeColor = C_WARN;
    else causeColor = C_GOOD;
    tft.setTextSize(strlen(causeBuf) > 14 ? 1 : 2);
    tft.setTextColor(causeColor);
    tft.setCursor(x, yv);
    tft.print(causeBuf);
    strncpy(lastCause, causeBuf, sizeof(lastCause));
    lastCause[sizeof(lastCause) - 1] = '\0';
  }

  char cooldownBuf[24];
  if (cooldownActive) {
    uint32_t rem = (uint32_t)(rainCooldownUntilEpoch - now);
    uint32_t mins = (rem + 59U) / 60U;
    if (mins >= 60U) {
      uint32_t h = mins / 60U;
      uint32_t m = mins % 60U;
      if (m > 0U) snprintf(cooldownBuf, sizeof(cooldownBuf), "%luh %lum", (unsigned long)h, (unsigned long)m);
      else snprintf(cooldownBuf, sizeof(cooldownBuf), "%luh", (unsigned long)h);
    } else {
      snprintf(cooldownBuf, sizeof(cooldownBuf), "%lum", (unsigned long)mins);
    }
  } else {
    strncpy(cooldownBuf, "None", sizeof(cooldownBuf));
  }
  cooldownBuf[sizeof(cooldownBuf) - 1] = '\0';

  if (layoutChanged || strcmp(cooldownBuf, lastCooldown) != 0) {
    int x = pad + 6;
    int yv = statusY + 38;
    int w = W - 2 * pad - 12;
    tft.fillRect(x, yv, w, text1H, C_PANEL);
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(x, yv);
    tft.print("After-Rain Delay: ");
    tft.setTextColor(C_TEXT);
    tft.print(cooldownBuf);
    strncpy(lastCooldown, cooldownBuf, sizeof(lastCooldown));
    lastCooldown[sizeof(lastCooldown) - 1] = '\0';
  }

  char pmBuf[32];
  snprintf(pmBuf, sizeof(pmBuf), "Pause %s | Master %s", paused ? "Yes" : "No", systemMasterEnabled ? "On" : "Off");
  if (layoutChanged || strcmp(pmBuf, lastPauseMaster) != 0) {
    int x = pad + 6;
    int yv = statusY + 50;
    int w = W - 2 * pad - 12;
    tft.fillRect(x, yv, w, text1H, C_PANEL);
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(x, yv);
    tft.print("Pause ");
    tft.setTextColor(C_TEXT);
    tft.print(paused ? "Yes" : "No");
    tft.setTextColor(C_MUTED);
    tft.print(" | Master ");
    tft.setTextColor(C_TEXT);
    tft.print(systemMasterEnabled ? "On" : "Off");
    strncpy(lastPauseMaster, pmBuf, sizeof(lastPauseMaster));
    lastPauseMaster[sizeof(lastPauseMaster) - 1] = '\0';
  }

  int queued = 0;
  int running = 0;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (pendingStart[i]) queued++;
    if (zoneActive[i]) running++;
  }

  if (layoutChanged || running != lastRunning || queued != lastQueued) {
    int x = pad + 6;
    int yv = statusY + 62;
    int w = W - 2 * pad - 12;
    tft.fillRect(x, yv, w, text1H, C_PANEL);
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(x, yv);
    tft.print("Run ");
    tft.setTextColor(C_TEXT);
    tft.print(running);
    tft.setTextColor(C_MUTED);
    tft.print(" | Q ");
    tft.setTextColor(C_TEXT);
    tft.print(queued);
    lastRunning = running;
    lastQueued = queued;
  }

  int rssi = WiFi.RSSI();
  bool mqttOk = _mqtt.connected();
  bool rssiChanged = (abs(rssi - lastRssi) >= 4);
  if (layoutChanged || rssiChanged || mqttOk != lastMqtt) {
    int x = pad + 6;
    int yv = statusY + 74;
    int w = W - 2 * pad - 12;
    tft.fillRect(x, yv, w, text1H, C_PANEL);
    tft.setTextSize(1);
    uint16_t rssiColor = (rssi > -60) ? C_GOOD : (rssi > -75 ? C_WARN : C_BAD);
    tft.setTextColor(C_MUTED);
    tft.setCursor(x, yv);
    tft.print("WiFi ");
    tft.setTextColor(rssiColor);
    tft.print(rssi);
    tft.setTextColor(C_MUTED);
    tft.print("dBm | MQTT ");
    tft.setTextColor(mqttOk ? C_GOOD : C_BAD);
    tft.print(mqttOk ? "up" : "down");
    lastRssi = rssi;
    lastMqtt = mqttOk;
  }
}

void HomeScreen() {
  if (!displayEnabled) return;
  if (displayEnabled && !displayUseTft) {
    const int OW = display.width();
    const int pctOled = constrain(tankPercent(), 0, 100);
    const bool masterOff = !systemMasterEnabled;
    const bool paused = isPausedNow();
    const bool delayed = (isRainDelayBlockingNow() || windActive);
    const char* statusStr = masterOff ? "MASTER OFF" : (paused ? "PAUSED" : (delayed ? "DELAY" : "READY"));

    int running = 0;
    int queued = 0;
    for (int i = 0; i < (int)zonesCount; ++i) {
      if (zoneActive[i]) running++;
      if (pendingStart[i]) queued++;
    }

    char timeBuf[8] = "--:--";
    char dateBuf[8] = "--/--";
    time_t nowOled = time(nullptr);
    struct tm tOled;
    if (localtime_r(&nowOled, &tOled)) {
      formatClockTime(tOled, timeBuf, sizeof(timeBuf));
      snprintf(dateBuf, sizeof(dateBuf), "%02d/%02d", tOled.tm_mday, tOled.tm_mon + 1);
    }

    char envBuf[32];
    const float displayTemp = temperatureForDisplay(curTempC);
    const char tempUnit = temperatureUnitChar();
    const char* condOled = (curWeatherCode >= 0) ? meteoCodeToMain(curWeatherCode) : "--";
    if (isnan(curTempC) && curHumidityPct < 0) {
      snprintf(envBuf, sizeof(envBuf), "--%c --%% %s", tempUnit, condOled);
    } else if (curHumidityPct < 0) {
      snprintf(envBuf, sizeof(envBuf), "%.0f%c --%% %s", displayTemp, tempUnit, condOled);
    } else if (isnan(curTempC)) {
      snprintf(envBuf, sizeof(envBuf), "--%c %d%% %s", tempUnit, curHumidityPct, condOled);
    } else {
      snprintf(envBuf, sizeof(envBuf), "%.0f%c %d%% %s", displayTemp, tempUnit, curHumidityPct, condOled);
    }

    char nextBuf[28] = "Next none";
    NextWaterInfo nwOled = computeNextWatering();
    if (nwOled.zone >= 0) {
      struct tm tnw;
      localtime_r(&nwOled.epoch, &tnw);
      long diff = (long)difftime(nwOled.epoch, nowOled);
      if (diff < 0) diff = 0;
      int mins = (int)(diff / 60L);
      if (mins >= 60) {
        char nextTime[8];
        formatClockTime(tnw, nextTime, sizeof(nextTime));
        snprintf(nextBuf, sizeof(nextBuf), "Next Z%d %s %dh%02d",
                 nwOled.zone + 1, nextTime, mins / 60, mins % 60);
      } else {
        char nextTime[8];
        formatClockTime(tnw, nextTime, sizeof(nextTime));
        snprintf(nextBuf, sizeof(nextBuf), "Next Z%d %s %dm",
                 nwOled.zone + 1, nextTime, mins);
      }
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.fillRect(0, 0, OW, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(3, 2);
    display.print(statusStr);
    display.setCursor(OW - (clockUse24Hour ? 66 : 78), 2);
    display.print(timeBuf);
    display.print(" ");
    display.print(dateBuf);

    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 16);
    display.print(envBuf);
    display.setCursor(0, 28);
    display.print("Tank ");
    display.print(pctOled);
    display.print("%");
    display.drawRect(62, 29, OW - 64, 7, SSD1306_WHITE);
    int barFill = ((OW - 66) * pctOled) / 100;
    if (barFill > 0) display.fillRect(63, 30, barFill, 5, SSD1306_WHITE);

    display.drawFastHLine(0, 40, OW, SSD1306_WHITE);
    display.setCursor(0, 44);
    display.print(nextBuf);
    display.setCursor(0, 56);
    display.print("Run ");
    display.print(running);
    display.print(" Q ");
    display.print(queued);
    display.print(" MQTT ");
    display.print(_mqtt.connected() ? "up" : "down");

    display.display();
    return;
  }

  float temp = temperatureForDisplay(curTempC);
  int   hum  = curHumidityPct;
  float windNow = curWindMs;
  int   pct  = tankPercent();
  const int moisturePctHome = moisturePercent();
  const bool moistureSkipHome = isSoilWetForSmartSkip();

  time_t now = time(nullptr);
  struct tm* tmv = localtime(&now);
  uint32_t nowMs = millis();

  const bool delayed = (isRainDelayBlockingNow() || windActive || isPausedNow() || !systemMasterEnabled);

  const int W = tft.width();
  const int H = tft.height();

  const bool landscape = (H < W);

  const int curMinute = tmv ? tmv->tm_min : -1;

  // Legacy fallback metrics retained for the older branch below.
  const int pad = 6;
  const int gap = 6;
  const int topY = 44;

  if (landscape) {
    static bool modernInit = false;
    static int lastW = -1;
    static int lastH = -1;
    static int lastMinute = -1;
    static int lastStatus = -1;
    static int lastTemp = -1000;
    static int lastHum = -2;
    static int lastPct = -1;
    static int lastCondCode = -999;
    static int lastNextZone = -2;
    static time_t lastNextEpoch = 0;
    static int lastRain1h = -1;
    static int lastRain24h = -1;
    static int lastGust = -1;
    static int lastFeels = -1000;
    static int lastPressure = -1;
    static int lastPressureTrend = 99;
    static int lastCompassWindDir = -1;
    static int lastCompassWindTenths = 10000;
    static int lastCompassGust = -1;
    NextWaterInfo nw = computeNextWatering();
    int tempRounded = isnan(temp) ? -1000 : (int)lroundf(temp);
    float feels = temperatureForDisplay(curFeelsC);
    int feelsRounded = isnan(feels) ? -1000 : (int)lroundf(feels);
    int pressureRounded = isfinite(curPressureHpa) ? (int)lroundf(curPressureHpa) : -1;
    int pctClamped = constrain(pct, 0, 100);
    int windTenths = isfinite(windNow) ? (int)lroundf(windNow * 10.0f) : 10000;
    int windDirRounded = isfinite(curWindDirDeg) ? (int)lroundf(normalizeDegrees360(curWindDirDeg)) : -1;
    bool fullRedraw = g_forceHomeReset || !modernInit || lastW != W || lastH != H;

    // Additional weather data
    int rain1hVal = isfinite(rain1hNow) ? (int)lroundf(rain1hNow * 10.0f) : -1;
    int rain24hVal = isfinite(rainNext24h_mm) ? (int)lroundf(rainNext24h_mm * 10.0f) : -1;
    int gustVal = isfinite(curGustMs) ? (int)lroundf(curGustMs * 10) : -1;

    // Header height for layout calculations (declared here for broader scope)
    int headerH = 24;

    if (fullRedraw) {
      tft.fillScreen(C_BG);
      modernInit = true;
      lastW = W;
      lastH = H;
      lastMinute = -1;
      lastStatus = -1;
      lastTemp = -1000;
      lastHum = -2;
      lastPct = -1;
      lastCondCode = -999;
      lastNextZone = -2;
      lastNextEpoch = 0;
      lastRain1h = -1;
      lastRain24h = -1;
      lastGust = -1;
      lastFeels = -1000;
      lastPressure = -1;
      lastPressureTrend = 99;
      lastCompassWindDir = -1;
      lastCompassWindTenths = 10000;
      lastCompassGust = -1;
      g_forceHomeReset = false;

      // ========== MODERN HEADER ==========
      tft.fillRect(0, 0, W, headerH + 1, C_PANEL_2);
      tft.fillRect(0, 0, 4, headerH, C_ACCENT);
      tft.drawFastHLine(0, headerH, W, C_EDGE);
      tft.drawFastHLine(4, 1, max(0, W - 4), C_HILITE);

      // ========== MODERN CARD LAYOUT ==========
      int cardY = headerH + 6;
      int cardH = H - cardY - 6;
      
      // Left column - Time & Next Water
      int leftW = 96;
      drawCard(6, cardY, leftW, cardH, C_PANEL, C_EDGE);
      
      // Center - Weather & Tank
      int centerX = leftW + 12;
      int centerW = 88;
      int centerGap = 6;
      int weatherH = (cardH * 2) / 5;
      int minTankH = 64;
      if (cardH - centerGap - weatherH < minTankH) weatherH = cardH - centerGap - minTankH;
      if (weatherH < 44) weatherH = max(24, cardH / 2 - 3);
      int tankCardY = cardY + weatherH + centerGap;
      int tankCardH = cardY + cardH - tankCardY;
      drawCard(centerX, cardY, centerW, weatherH, C_PANEL, C_EDGE);
      drawCard(centerX, tankCardY, centerW, tankCardH, C_PANEL, C_EDGE);
      
      // Right - detailed weather
      int rightX = centerX + centerW + 10;
      int rightW = W - rightX - 6;
      drawCard(rightX, cardY, rightW, cardH, C_PANEL, C_EDGE);

      // Labels
      tft.setTextSize(1);
      tft.setTextColor(C_MUTED);
      tft.setCursor(12, cardY + 6);
      tft.print("CLOCK");
      tft.drawFastHLine(12, cardY + 54, leftW - 12, C_EDGE);
      tft.setCursor(12, cardY + 58);
      tft.print("NEXT WATER");
      
      tft.setCursor(centerX + 6, cardY + 6);
      tft.print("WEATHER");
      tft.setCursor(centerX + 6, tankCardY + 6);
      tft.print("TANK LEVEL");
      
      tft.setCursor(rightX + 6, cardY + 6);
      tft.print("WEATHER DETAILS");
    }

    int cardY = headerH + 6;
    int cardH = H - cardY - 6;
    int leftW = 96;
    int centerX = leftW + 12;
    int centerW = 88;
    int centerGap = 6;
    int weatherH = (cardH * 2) / 5;
    int minTankH = 64;
    if (cardH - centerGap - weatherH < minTankH) weatherH = cardH - centerGap - minTankH;
    if (weatherH < 44) weatherH = max(24, cardH / 2 - 3);
    int tankCardY = cardY + weatherH + centerGap;
    int tankCardH = cardY + cardH - tankCardY;
    int rightX = centerX + centerW + 10;
    int rightW = W - rightX - 6;
    int status = !systemMasterEnabled ? 3 : (isPausedNow() ? 2 : ((isRainDelayBlockingNow() || windActive) ? 1 : 0));

    // ========== STATUS PILL (top right) ==========
    if (fullRedraw || curMinute != lastMinute || status != lastStatus) {
      tft.fillRect(0, 0, W, headerH, C_PANEL_2);
      tft.fillRect(0, 0, 4, headerH, C_ACCENT);
      tft.drawFastHLine(0, headerH, W, C_EDGE);
      tft.drawFastHLine(4, 1, max(0, W - 4), C_HILITE);

      tft.setTextSize(1);
      tft.setTextColor(C_TEXT, C_PANEL_2);
      tft.setCursor(8, 8);
      tft.print("IRRIGATION");
      tft.setTextColor(C_MUTED, C_PANEL_2);
      tft.print(" ");
      tft.print(zonesCount);
      tft.print(" Zone");

      const char* statusText = "READY";
      uint16_t statusColor = C_GOOD;
      if (!systemMasterEnabled) { statusText = "MASTER OFF"; statusColor = C_BAD; }
      else if (isPausedNow()) { statusText = "PAUSED"; statusColor = C_WARN; }
      else if (isRainDelayBlockingNow() || windActive) { statusText = "DELAY"; statusColor = C_WARN; }
      int16_t bx, by; uint16_t bw, bh;
      tft.setTextSize(1);
      tft.getTextBounds(statusText, 0, 0, &bx, &by, &bw, &bh);
      int pillW = (int)bw + 14;
      int pillX = W - pillW - 8;
      tft.fillRoundRect(pillX, 3, pillW, 18, 5, statusColor);
      tft.drawRoundRect(pillX, 3, pillW, 18, 5, C_EDGE);
      tft.setTextColor(ST77XX_BLACK, statusColor);
      tft.setCursor(pillX + 7, 8);
      tft.print(statusText);
      lastStatus = status;
    }

    // ========== TIME DISPLAY ==========
    char tbuf[8] = "--:--";
    char dbuf[16] = "--/--";
    char dayBuf[6] = "---";
    if (tmv) {
      formatClockTime(*tmv, tbuf, sizeof(tbuf));
      snprintf(dbuf, sizeof(dbuf), "%02d/%02d", tmv->tm_mday, tmv->tm_mon + 1);
      static const char* days[] = {"SUN","MON","TUE","WED","THU","FRI","SAT"};
      if (tmv->tm_wday >= 0 && tmv->tm_wday <= 6) strncpy(dayBuf, days[tmv->tm_wday], 5);
    }
    if (fullRedraw || curMinute != lastMinute) {
      tft.fillRect(12, cardY + 16, leftW - 12, 34, C_PANEL);
      tft.setTextSize(2);
      tft.setTextColor(C_ACCENT);
      tft.setCursor(14, cardY + 22);
      tft.print(tbuf);
      tft.setTextSize(1);
      tft.setTextColor(C_TEXT);
      tft.setCursor(12, cardY + 42);
      tft.print(dayBuf);
      tft.print(" ");
      tft.print(dbuf);
      lastMinute = curMinute;
    }

    // ========== NEXT WATER ==========
    if (fullRedraw || nw.zone != lastNextZone || nw.epoch != lastNextEpoch) {
      tft.fillRect(12, cardY + 70, leftW - 12, 34, C_PANEL);
      tft.setTextSize(1);
      tft.setTextColor(C_TEXT);
      if (nw.zone >= 0) {
        struct tm tnw;
        localtime_r(&nw.epoch, &tnw);
        tft.setCursor(16, cardY + 72);
        tft.print("Z");
        tft.print(nw.zone + 1);
        char nbuf[8];
        formatClockTime(tnw, nbuf, sizeof(nbuf));
        tft.print(" ");
        tft.print(nbuf);
        long diff = (long)difftime(nw.epoch, now);
        if (diff < 0) diff = 0;
        int mins = (int)(diff / 60L);
        tft.setTextSize(1);
        tft.setTextColor(C_MUTED);
        tft.setCursor(16, cardY + 88);
        if (mins >= 60) { tft.print(mins / 60); tft.print("h "); tft.print(mins % 60); tft.print("m"); }
        else { tft.print("in "); tft.print(mins); tft.print("m"); }
      } else {
        tft.setCursor(16, cardY + 72);
        tft.print("None");
        tft.setTextSize(1);
        tft.setTextColor(C_MUTED);
        tft.setCursor(16, cardY + 88);
        tft.print("No schedule");
      }
      lastNextZone = nw.zone;
      lastNextEpoch = nw.epoch;
    }

    // ========== WEATHER CARD ==========
    if (fullRedraw || tempRounded != lastTemp || hum != lastHum || curWeatherCode != lastCondCode) {
      tft.fillRect(centerX + 4, cardY + 20, centerW - 8, max(20, weatherH - 24), C_PANEL);
      
      // Temperature and humidity
      tft.setTextSize(2);
      tft.setTextColor(C_TEXT);
      tft.setCursor(centerX + 8, cardY + 22);
      if (isnan(temp)) tft.print("--");
      else { tft.print(temp, 0); tft.print(temperatureUnitChar()); }
      
      tft.setTextSize(1);
      tft.setTextColor(C_MUTED);
      tft.setCursor(centerX + 58, cardY + 11);
      if (hum >= 0) { tft.print(hum); tft.print("%"); }
      else tft.print("--%");
      
      const int iconSize = min(28, max(20, weatherH - 28));
      const int iconX = centerX + centerW - iconSize - 8;
      const int iconY = cardY + max(22, weatherH - iconSize - 19);
      drawWeatherIcon(iconX, iconY, iconSize, curWeatherCode, C_PANEL);

      // Condition
      tft.setCursor(centerX + 8, cardY + 42);
      const char* cond = (curWeatherCode >= 0) ? meteoCodeToMain(curWeatherCode) : "--";
      tft.setTextColor(C_TEXT);
      tft.print(cond);
      
      lastTemp = tempRounded;
      lastHum = hum;
      lastCondCode = curWeatherCode;
    }

    // ========== TANK CARD ==========
    if (fullRedraw || pctClamped != lastPct) {
      int tankY = tankCardY + 20;
      tft.fillRect(centerX + 4, tankY, centerW - 8, max(20, tankCardH - 24), C_PANEL);
      
      // Tank percentage
      tft.setTextSize(2);
      tft.setTextColor(pctClamped <= (int)tankLowThresholdPct ? C_BAD : C_GOOD);
      tft.setCursor(centerX + 8, tankY + 6);
      tft.print(pctClamped);
      tft.setTextSize(1);
      tft.print("%");
      
      // Tank bar
      int barX = centerX + 8;
      int barY = tankY + 28;
      int barW = centerW - 16;
      tft.drawRect(barX, barY, barW, 8, C_EDGE);
      tft.fillRect(barX + 1, barY + 1, barW - 2, 6, C_PANEL);
      int fillW = (barW - 2) * pctClamped / 100;
      if (fillW > 0) tft.fillRect(barX + 1, barY + 1, fillW, 6, pctClamped <= (int)tankLowThresholdPct ? C_BAD : C_GOOD);
      
      // Tank status text
      tft.setTextSize(1);
      tft.setTextColor(C_MUTED);
      if (tankCardH >= 72) {
        tft.setCursor(centerX + 8, tankY + 44);
        if (pctClamped <= (int)tankLowThresholdPct) tft.print("LOW!");
        else if (pctClamped >= 80) tft.print("Full");
        else tft.print("Normal");
      }
      
      lastPct = pctClamped;
    }

    // ========== WEATHER DETAILS CARD ==========
    if (fullRedraw || feelsRounded != lastFeels || pressureRounded != lastPressure ||
        curPressureTrend != lastPressureTrend ||
        rain1hVal != lastRain1h || rain24hVal != lastRain24h ||
        windDirRounded != lastCompassWindDir || windTenths != lastCompassWindTenths ||
        gustVal != lastCompassGust) {
      tft.fillRect(rightX + 4, cardY + 20, rightW - 8, max(94, cardH - 24), C_PANEL);

      tft.setTextSize(1);
      tft.setTextColor(C_MUTED);
      tft.setCursor(rightX + 8, cardY + 26);
      tft.print("Feels ");
      tft.setTextColor(C_TEXT);
      if (feelsRounded == -1000) tft.print("--");
      else tft.print(feelsRounded);
      tft.print(temperatureUnitChar());

      tft.setTextColor(C_MUTED);
      tft.setCursor(rightX + 8, cardY + 42);
      tft.print("Press ");
      tft.setTextColor(C_TEXT);
      if (pressureRounded < 0) tft.print("--");
      else tft.print(pressureRounded);
      tft.print("hPa");
      drawPressureTrendArrow(min(rightX + rightW - 16, rightX + 92), cardY + 40,
                             pressureRounded < 0 ? 0 : curPressureTrend, C_PANEL);

      tft.setTextColor(C_MUTED);
      tft.setCursor(rightX + 8, cardY + 58);
      tft.print("Rain 1h ");
      tft.setTextColor(rain1hVal > 0 ? C_WARN : C_TEXT);
      if (rain1hVal < 0) tft.print("--");
      else tft.print(rain1hVal / 10.0f, 1);
      tft.print(" mm");

      tft.setTextColor(C_MUTED);
      tft.setCursor(rightX + 8, cardY + 74);
      tft.print("Next 24h ");
      tft.setTextColor(rain24hVal > 0 ? C_WARN : C_TEXT);
      if (rain24hVal < 0) tft.print("--");
      else tft.print(rain24hVal / 10.0f, 1);
      tft.print(" mm");

      int compassX = rightX + rightW - 24;
      int compassY = cardY + cardH - 28;
      drawWindCompass(compassX, compassY, 14, curWindDirDeg, C_PANEL);
      tft.setTextSize(1);
      tft.setTextColor(C_MUTED);
      tft.setCursor(rightX + 8, compassY - 8);
      tft.print("Wind ");
      if (isfinite(windNow)) tft.print(windNow, 1);
      else tft.print("--");
      if (gustVal >= 0) {
        tft.setCursor(rightX + 8, compassY + 5);
        tft.print("Gust ");
        tft.print(gustVal / 10.0f, 1);
      }

      lastFeels = feelsRounded;
      lastPressure = pressureRounded;
      lastPressureTrend = curPressureTrend;
      lastRain1h = rain1hVal;
      lastRain24h = rain24hVal;
      lastCompassWindDir = windDirRounded;
      lastCompassWindTenths = windTenths;
      lastCompassGust = gustVal;
    }

    return;
  }

  if (!landscape) {
    static bool portraitInit = false;
    static int lastW = -1;
    static int lastH = -1;
    static int lastMinute = -1;
    static int lastStatus = -1;
    static int lastTemp = -1000;
    static int lastHum = -2;
    static int lastPct = -1;
    static int lastWindTenths = 10000;
    static int lastWindDir = -1;
    static int lastCondCode = -999;
    static int lastNextZone = -2;
    static time_t lastNextEpoch = 0;
    static int lastSystemStatus = -1;
    static int lastMoisturePct = -999;
    static int lastMoistureSkip = -1;
    static int lastFeels = -1000;
    static int lastPressure = -1;
    static int lastPressureTrend = 99;
    static int lastRain1h = -1;
    static int lastRain24h = -1;
    static int lastGust = -1;

    NextWaterInfo nw = computeNextWatering();
    int tempRounded = isnan(temp) ? -1000 : (int)lroundf(temp);
    float feels = temperatureForDisplay(curFeelsC);
    int feelsRounded = isnan(feels) ? -1000 : (int)lroundf(feels);
    int pressureRounded = isfinite(curPressureHpa) ? (int)lroundf(curPressureHpa) : -1;
    int rain1hVal = isfinite(rain1hNow) ? (int)lroundf(rain1hNow * 10.0f) : -1;
    int rain24hVal = isfinite(rainNext24h_mm) ? (int)lroundf(rainNext24h_mm * 10.0f) : -1;
    int gustVal = isfinite(curGustMs) ? (int)lroundf(curGustMs * 10.0f) : -1;
    int pctClamped = constrain(pct, 0, 100);
    int windTenths = isfinite(windNow) ? (int)lroundf(windNow * 10.0f) : 10000;
    int windDirRounded = isfinite(curWindDirDeg) ? (int)lroundf(normalizeDegrees360(curWindDirDeg)) : -1;
    int status = !systemMasterEnabled ? 3 : (isPausedNow() ? 2 : ((isRainDelayBlockingNow() || windActive) ? 1 : 0));
    bool fullRedraw = g_forceHomeReset || !portraitInit || lastW != W || lastH != H;
    const int side = 8;
    const int cardW = W - 2 * side;
    const int clockY = 8;
    const int clockH = 70;
    const int tankY = 86;
    const int tankH = 74;
    const int nextY = 168;
    const int nextH = 42;
    const int envY = 218;
    const int envH = max(48, H - envY - 8);

    if (fullRedraw) {
      tft.fillScreen(C_BG);
      portraitInit = true;
      lastW = W;
      lastH = H;
      lastMinute = -1;
      lastStatus = -1;
      lastTemp = -1000;
      lastHum = -2;
      lastPct = -1;
      lastWindTenths = 10000;
      lastWindDir = -1;
      lastCondCode = -999;
      lastNextZone = -2;
      lastNextEpoch = 0;
      lastSystemStatus = -1;
      lastMoisturePct = -999;
      lastMoistureSkip = -1;
      lastFeels = -1000;
      lastPressure = -1;
      lastPressureTrend = 99;
      lastRain1h = -1;
      lastRain24h = -1;
      lastGust = -1;
      g_forceHomeReset = false;

      drawCard(side, clockY, cardW, clockH, C_PANEL, C_EDGE);
      drawCard(side, tankY, cardW, tankH, C_PANEL, C_EDGE);
      drawCard(side, nextY, cardW, nextH, C_PANEL, C_EDGE);
      drawCard(side, envY, cardW, envH, C_PANEL, C_EDGE);
      tft.setTextSize(1);
      tft.setTextColor(C_MUTED);
      tft.setCursor(16, nextY + 6);
      tft.print("NEXT WATER");
      tft.setCursor(16, envY + 6);
      tft.print("LOCAL WEATHER");
    }

    char tbuf[8] = "--:--";
    char dbuf[16] = "--/--";
    if (tmv) {
      formatClockTime(*tmv, tbuf, sizeof(tbuf));
      snprintf(dbuf, sizeof(dbuf), "%02d/%02d/%02d", tmv->tm_mday, tmv->tm_mon + 1, (tmv->tm_year + 1900) % 100);
    }
    if (fullRedraw || curMinute != lastMinute || status != lastStatus) {
      tft.fillRect(16, clockY + 10, W - 32, clockH - 16, C_PANEL);
      tft.setTextSize(3);
      tft.setTextColor(C_ACCENT);
      int16_t tx1, ty1; uint16_t tw, th;
      tft.getTextBounds(tbuf, 0, 0, &tx1, &ty1, &tw, &th);
      tft.setCursor((W - (int)tw) / 2, clockY + 18);
      tft.print(tbuf);
      tft.setTextSize(1);
      tft.setTextColor(C_TEXT);
      tft.setCursor(18, clockY + 52);
      tft.print(dbuf);
      const char* statusText = (status == 3) ? "MASTER OFF" : (status == 2 ? "PAUSED" : (status == 1 ? "DELAY" : "READY"));
      uint16_t statusColor = (status == 0) ? C_GOOD : (status == 3 ? C_BAD : C_WARN);
      tft.setTextSize(1);
      int16_t bx, by; uint16_t bw, bh;
      tft.getTextBounds(statusText, 0, 0, &bx, &by, &bw, &bh);
      int pillW = (int)bw + 14;
      int pillX = W - pillW - 16;
      tft.fillRoundRect(pillX, clockY + 47, pillW, 18, 5, statusColor);
      tft.setTextColor(ST77XX_BLACK);
      tft.setCursor(pillX + 7, clockY + 52);
      tft.print(statusText);
      lastMinute = curMinute;
      lastStatus = status;
    }

    if (fullRedraw || nw.zone != lastNextZone || nw.epoch != lastNextEpoch) {
      tft.fillRect(16, nextY + 17, W - 32, nextH - 20, C_PANEL);
      tft.setTextSize(1);
      tft.setTextColor(C_TEXT);
      tft.setCursor(16, nextY + 21);
      if (nw.zone >= 0) {
        struct tm tnw;
        localtime_r(&nw.epoch, &tnw);
        char nbuf[8];
        formatClockTime(tnw, nbuf, sizeof(nbuf));
        tft.print("Z");
        tft.print(nw.zone + 1);
        tft.print(" ");
        tft.print(nbuf);
      } else {
        tft.print("None");
      }
      lastNextZone = nw.zone;
      lastNextEpoch = nw.epoch;
    }

    if (fullRedraw || pctClamped != lastPct || status != lastSystemStatus ||
        moisturePctHome != lastMoisturePct || (moistureSkipHome ? 1 : 0) != lastMoistureSkip) {
      const char* waterSrc = "";
      bool mainsOn = false;
      bool tankOn = false;
      chooseWaterSource(waterSrc, mainsOn, tankOn);
      const bool lowTank = pctClamped <= (int)tankLowThresholdPct;
      const uint16_t tankColor = lowTank ? C_BAD : (pctClamped < 35 ? C_WARN : C_GOOD);
      tft.fillRect(16, tankY + 5, W - 32, tankH - 11, C_PANEL);
      tft.setTextSize(1);
      tft.setTextColor(C_TEXT);
      tft.setCursor(16, tankY + 7);
      tft.print("TANK LEVEL");
      if (moistureProbeEnabled) {
        tft.setTextColor(moistureSkipHome ? C_WARN : C_MUTED);
        tft.setCursor(91, tankY + 7);
        tft.print("Soil ");
        if (moisturePctHome >= 0) {
          tft.print(moisturePctHome);
          tft.print("%");
        } else {
          tft.print("--");
        }
      }
      const int barX = 18;
      const int barY = tankY + 25;
      const int barW = W - 36;
      tft.drawRoundRect(barX, barY, barW, 12, 4, C_EDGE);
      tft.fillRect(barX + 2, barY + 2, barW - 4, 8, RGB(8, 12, 20));
      const int fillW = ((barW - 4) * pctClamped) / 100;
      if (fillW > 0) tft.fillRect(barX + 2, barY + 2, fillW, 8, tankColor);

      char pctBuf[8];
      snprintf(pctBuf, sizeof(pctBuf), "%d", pctClamped);
      tft.setTextSize(2);
      tft.setTextColor(tankColor);
      tft.setCursor(18, tankY + 44);
      tft.print(pctBuf);
      tft.setTextSize(1);
      tft.print("%");

      tft.setTextSize(1);
      tft.setTextColor(lowTank ? C_BAD : C_MUTED);
      tft.setCursor(58, tankY + 47);
      if (!tankEnabled) tft.print("DISABLED");
      else if (lowTank) tft.print("LOW - MAINS");
      else if (pctClamped >= 80) tft.print("FULL");
      else tft.print("NORMAL");

      tft.setTextColor(C_MUTED);
      tft.setCursor(58, tankY + 59);
      tft.print("Source ");
      tft.setTextColor(tankOn ? C_GOOD : (mainsOn ? C_WARN : C_TEXT));
      tft.print(waterSrc && waterSrc[0] ? waterSrc : "--");
      lastPct = pctClamped;
      lastSystemStatus = status;
      lastMoisturePct = moisturePctHome;
      lastMoistureSkip = moistureSkipHome ? 1 : 0;
    }

    if (fullRedraw || tempRounded != lastTemp || hum != lastHum ||
        windTenths != lastWindTenths || windDirRounded != lastWindDir || curWeatherCode != lastCondCode ||
        feelsRounded != lastFeels || pressureRounded != lastPressure || curPressureTrend != lastPressureTrend ||
        rain1hVal != lastRain1h ||
        rain24hVal != lastRain24h || gustVal != lastGust) {
      tft.fillRect(16, envY + 18, W - 32, envH - 24, C_PANEL);
      const int iconSize = min(38, max(26, envH - 66));
      drawWeatherIcon(W - iconSize - 18, envY + 8, iconSize, curWeatherCode, C_PANEL);

      tft.setTextSize(2);
      tft.setTextColor(C_TEXT);
      tft.setCursor(16, envY + 20);
      if (isnan(temp)) { tft.print("--"); tft.print(temperatureUnitChar()); }
      else { tft.print(temp, 0); tft.print(temperatureUnitChar()); }
      tft.setTextColor(C_MUTED);
      tft.print("  ");
      if (hum < 0) tft.print("--%");
      else { tft.print(hum); tft.print("%"); }
      tft.setTextSize(1);
      tft.setTextColor(C_MUTED);
      tft.setCursor(16, envY + 42);
      tft.print((curWeatherCode >= 0) ? meteoCodeToMain(curWeatherCode) : "--");
      tft.setTextSize(1);
      tft.setTextColor(C_MUTED);
      tft.setCursor(16, envY + 56);
      tft.print("Wind ");
      tft.setTextColor(C_TEXT);
      tft.print(isfinite(curWindDirDeg) ? meteoWindDirectionToCompass(curWindDirDeg) : "--");
      tft.print(" ");
      if (isfinite(windNow)) tft.print(windNow, 1);
      else tft.print("--");
      if (gustVal >= 0) { tft.setTextColor(C_MUTED); tft.print(" G"); tft.print(gustVal / 10.0f, 1); }

      tft.setTextColor(C_MUTED);
      tft.setCursor(16, envY + 68);
      tft.print("Feels ");
      tft.setTextColor(C_TEXT);
      if (feelsRounded == -1000) tft.print("--"); else tft.print(feelsRounded);
      tft.print(temperatureUnitChar());
      tft.setTextColor(C_MUTED);
      tft.print("  ");
      if (pressureRounded < 0) tft.print("-- hPa");
      else { tft.print(pressureRounded); tft.print(" hPa"); }
      drawPressureTrendArrow(W - 28, envY + 66, pressureRounded < 0 ? 0 : curPressureTrend, C_PANEL);

      tft.setTextColor(C_MUTED);
      tft.setCursor(16, envY + 80);
      tft.print("Rain ");
      if (rain1hVal < 0) tft.print("--"); else tft.print(rain1hVal / 10.0f, 1);
      tft.print(" | 24h ");
      if (rain24hVal < 0) tft.print("--"); else tft.print(rain24hVal / 10.0f, 1);
      tft.print("mm");
      lastTemp = tempRounded;
      lastHum = hum;
      lastWindTenths = windTenths;
      lastWindDir = windDirRounded;
      lastCondCode = curWeatherCode;
      lastFeels = feelsRounded;
      lastPressure = pressureRounded;
      lastPressureTrend = curPressureTrend;
      lastRain1h = rain1hVal;
      lastRain24h = rain24hVal;
      lastGust = gustVal;
    }

    return;
  }

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

  const bool match1 = (hr == startHour[zone]  && mn == startMin[zone]);
  const bool match2 = (enableStartTime2[zone] && hr == startHour2[zone] && mn == startMin2[zone]);

  if (match1 || match2) {
    lastCheckedMinute[zone] = mn;
    lastStartSlot[zone] = match2 ? 2 : 1;
    unsigned long base = durationForSlot(zone, lastStartSlot[zone]);
    if (base == 0) return false;
    unsigned long adjusted = smartWateringDurationForSlot(zone, lastStartSlot[zone]);
    if (adjusted == 0) {
      logEvent(zone, "CANCELLED", "SMART", true);
      return false;
    }
    return true;
  }
  return false;
}

bool hasDurationCompleted(int zone) {
  unsigned long elapsed=(millis()-zoneStartMs[zone])/1000;
  unsigned long total = zoneRunTotalSec[zone];
  if (total == 0) total = durationForSlot(zone, 1);
  return (elapsed >= total);
}

void turnOnZone(int z) {
  if (z < 0 || z >= (int)zonesCount || z >= (int)MAX_ZONES) return;

  checkWindRain();
  Serial.printf("[VALVE] Request ON Z%d rain=%d wind=%d blocked=%d\n",
                z+1, rainActive?1:0, windActive?1:0, isBlockedNow()?1:0);

  if (isBlockedNow())  { cancelStart(z, "BLOCKED", false); return; }
  if (rainActive)      { cancelStart(z, "RAIN",    true ); return; }
  if (windBlocksZone(z)) { pendingStart[z] = true; logEvent(z, "QUEUED", "WIND", false); return; }

  const bool usePcf = useExpanderForZone(z);
  if (!usePcf) {
    int pin = zonePins[z];
    if (!isValidOutputPin(pin)) {
      Serial.printf("[VALVE] Z%d has no GPIO pin assigned; skipping\n", z+1);
      cancelStart(z, "NO_PIN", false);
      return;
    }
  }

  bool anyOn = false;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) { anyOn = true; break; }
  }
  if (!runZonesConcurrent && anyOn) {
    pendingStart[z] = true;
    logEvent(z, "QUEUED", "ACTIVE RUN", false);
    return;
  }

  unsigned long total = smartWateringDurationForSlot(z, lastStartSlot[z]);
  if (total == 0) total = smartWateringDurationForSlot(z, 1);
  if (total == 0) { cancelStart(z, "SMART", true); return; }
  zoneStartMs[z] = millis();
  zoneActive[z] = true;
  zoneStartedManual[z] = false;
  zoneRunTotalSec[z] = total;
  const char* src = "None";
  bool mainsOn=false, tankOn=false;
  chooseWaterSource(src, mainsOn, tankOn);
  gpioPowerSupplyWrite(true);

  if (!usePcf) {
  // Use config for active level
  gpioZoneWrite(z, true);   // ON

  setWaterSourceRelays(mainsOn, tankOn);
} else {
    // PCF8574 path unchanged (still active-LOW via expander)
    pcfOut.digitalWrite(PCH[z], LOW); // active-LOW
    setWaterSourceRelays(mainsOn, tankOn);
  }

  logEvent(z, "START", src, false);

  if (displayEnabled && !displayUseTft) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(2, 0);
    display.print(zoneNames[z]);
    display.print(" ON");
    display.display();
    delay(350);
  }

  // Force a clean redraw immediately for TFT (no ON splash delay).
  if (displayEnabled && displayUseTft) {
    g_forceHomeReset = true;
    g_forceRunReset = true;
    tft.fillScreen(C_BG);
  }
  HomeScreen();
}

void turnOffZone(int z) {
  if (z < 0 || z >= (int)zonesCount || z >= (int)MAX_ZONES) return;
  Serial.printf("[VALVE] Request OFF Z%d\n", z+1);
  const char* src = "None";
  bool mainsOn=false, tankOn=false;
  chooseWaterSource(src, mainsOn, tankOn);

  bool wasDelayed = rainActive || windBlocksZone(z) || isPausedNow() ||
                    !systemMasterEnabled ||
                    (rainCooldownUntilEpoch > time(nullptr));
  logEvent(z, "STOPPED", zoneStartedManual[z] ? "MANUAL" : src, wasDelayed);

  const bool usePcf = useExpanderForZone(z);

  if (!usePcf) {
  // OFF everything in fallback according to config
  gpioZoneWrite(z, false);       // OFF
} else {
    // PCF8574 path unchanged
    pcfOut.digitalWrite(PCH[z], HIGH);
  }

  zoneActive[z] = false;
  zoneStartedManual[z] = false;
  zoneRunTotalSec[z] = 0;

  bool anyStillOn = anyZoneActive();
  if (anyStillOn) {
    setWaterSourceRelays(mainsOn, tankOn);
  } else {
    setWaterSourceRelays(false, false);
  }

  if (displayEnabled && !displayUseTft) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(4, 0);
    display.print(zoneNames[z]);
    display.print(" OFF");
    display.display();
    delay(350);
  }
  if (displayEnabled && displayUseTft) {
    g_forceHomeReset = true;
    g_forceRunReset = true;
    if (!anyStillOn) tft.fillScreen(C_BG);
  }

  // Ensure the display returns to Home after showing the OFF banner
  HomeScreen();
}


bool turnOnValveManual(int z, String* error) {
  if (z < 0 || z >= (int)zonesCount || z >= (int)MAX_ZONES) {
    if (error) *error = "Invalid zone";
    return false;
  }
  if (zoneActive[z]) return true;

  const bool usePcf = useExpanderForZone(z);
  if (!usePcf) {
    int pin = zonePins[z];
    if (!isValidOutputPin(pin)) {
      Serial.printf("[VALVE] Z%d has no GPIO pin assigned; skipping manual start\n", z+1);
      cancelStart(z, "NO_PIN", false);
      showManualSelection();
      if (error) *error = "Zone has no valid output GPIO";
      return false;
    }
  }

  // Respect run mode overlap rules
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) {
      if (!runZonesConcurrent) {
        manualSelectedZone = i;
        Serial.printf("[VALVE] Manual start blocked: Z%d is already running\n", i + 1);
        showManualSelection();
        if (error) *error = "Another zone is already running";
        return false;
      }
      break; // concurrent: allow additional zone
    }
  }

  lastStartSlot[z] = 1;
  const unsigned long baseDuration = durationForSlot(z, 1);
  if (baseDuration == 0) {
    Serial.printf("[VALVE] Manual start blocked: Z%d duration is 0\n", z + 1);
    cancelStart(z, "NO_DURATION", false);
    showManualSelection();
    if (error) *error = "Zone duration is 0. Save a non-zero duration before manual start.";
    return false;
  }
  unsigned long total = smartWateringDurationForSlot(z, 1);
  if (total == 0) {
    cancelStart(z, "SMART", true);
    showManualSelection();
    if (error) *error = "Smart Watering reduced the zone duration to 0 seconds";
    return false;
  }
  zoneStartMs[z] = millis();
  zoneActive[z] = true;
  zoneStartedManual[z] = true;
  zoneRunTotalSec[z] = total;
  const char* src = "None";
  bool mainsOn=false, tankOn=false;
  chooseWaterSource(src, mainsOn, tankOn);
  gpioPowerSupplyWrite(true);

  if (!usePcf) {
  gpioZoneWrite(z, true);   // ON

  setWaterSourceRelays(mainsOn, tankOn);
} else {
    // PCF8574 path unchanged
    pcfOut.digitalWrite(PCH[z], LOW);
    setWaterSourceRelays(mainsOn, tankOn);
  }

  logEvent(z, "START", "MANUAL", false);
  manualScreenUntilMs = 0;  // leave zone-select overlay immediately
  g_forceManualReset = true;
  g_forceRunReset = true;
  lastScreenRefresh = 0;
  if (displayEnabled && displayUseTft) {
    tft.fillScreen(C_BG);
    updateLCDForZone(z);
  } else {
    HomeScreen();
  }
  return true;
}

void turnOffValveManual(int z) {
  if (z < 0 || z >= (int)zonesCount || z >= (int)MAX_ZONES) return;
  if (!zoneActive[z]) return;

  const bool usePcf = useExpanderForZone(z);
  const char* src="None";
  bool mainsOn=false, tankOn=false;
  chooseWaterSource(src, mainsOn, tankOn);

  // Turn this zone OFF
  if (!usePcf) {
    // Use configured polarity for GPIO fallback
    gpioZoneWrite(z, false);          // OFF
  } else {
    // PCF8574 path: keep active-LOW semantics (HIGH = OFF)
    pcfOut.digitalWrite(PCH[z], HIGH);
  }

  logEvent(z, "STOPPED", "MANUAL", false);

  zoneActive[z] = false;
  zoneStartedManual[z] = false;
  zoneRunTotalSec[z] = 0;

  bool anyStillOn = false;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) {
      anyStillOn = true;
      break;
    }
  }

  if (anyStillOn) {
    setWaterSourceRelays(mainsOn, tankOn);
  } else {
    setWaterSourceRelays(false, false);   // mains OFF, tank OFF
  }

  // Manual OFF path should also force a clean visual transition.
  const bool manualOverlayActive =
    (manualScreenUntilMs != 0 && (int32_t)(manualScreenUntilMs - millis()) > 0);

  if (displayEnabled && displayUseTft) {
    tft.fillScreen(C_BG);
    g_forceRunReset = true;
    if (!manualOverlayActive) g_forceHomeReset = true;
  } else if (displayEnabled) {
    display.clearDisplay();
    display.display();
  }

  lastScreenRefresh = 0; // repaint immediately on next loop cycle
  if (manualOverlayActive) drawManualSelection();
  else HomeScreen();
}

// ---------- Next Water (queue-first) ----------
static NextWaterInfo computeNextWatering() {
  NextWaterInfo best{0, -1, 0};

  for (int z=0; z<(int)zonesCount; ++z) {
    if (pendingStart[z]) {
      best.epoch = time(nullptr);
      best.zone  = z;
      best.durSec = (uint32_t)smartWateringDurationForSlot(z, lastStartSlot[z]);
      return best;
    }
  }

  time_t now = time(nullptr);
  struct tm base; localtime_r(&now, &base);

  auto consider = [&](int z, int hr, int mn, bool enabled, int slot) {
    if (!enabled) return;
    unsigned long tot = smartWateringDurationForSlot(z, slot);
    if (tot == 0) return;

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
          best.durSec = (uint32_t)tot;
        }
        return;
      }
      t += 24*60*60;
    }
  };

  for (int z=0; z<(int)zonesCount; ++z) {
    consider(z, startHour[z],  startMin[z],  true, 1);
    consider(z, startHour2[z], startMin2[z], enableStartTime2[z], 2);
  }
  return best;
}

// Main Page
void handleRoot() {
  HttpScope _scope;  // NEW: mark that we're in an HTTP handler so no blocking fetches

  // --- Precompute state / snapshots ---
  checkWindRain();

  time_t now = time(nullptr);
  struct tm* ti = localtime(&now);
  char timeStr[9], dateStr[11], heroDateStr[40];
  if (ti) {
    strftime(timeStr, sizeof(timeStr), "%H:%M:%S", ti);
    strftime(dateStr, sizeof(dateStr), "%d/%m/%Y", ti);
    strftime(heroDateStr, sizeof(heroDateStr), "%A, %d %B %Y", ti);
  } else {
    snprintf(timeStr, sizeof(timeStr), "--:--:--");
    snprintf(dateStr, sizeof(dateStr), "--/--/----");
    snprintf(heroDateStr, sizeof(heroDateStr), "Date unavailable");
  }

  // Keep this - it respects the g_inHttp guard
  updateCachedWeather();

  // Safe reads from decoded snapshot
  float temp = temperatureForDisplay(curTempC);
  float hum = (curHumidityPct >= 0) ? (float)curHumidityPct : NAN;
  float ws = curWindMs;
  float feels = temperatureForDisplay(curFeelsC);
  String windDir = formatWindDirection(curWindDirDeg);
  float windDirDeg = normalizeDegrees360(curWindDirDeg);
  int wcode = curWeatherCode;
  String cond = (wcode >= 0) ? String(meteoCodeToDesc(wcode)) : String("-");
  if (cond == "") cond = "-";
  String cityName = meteoLocationLabel();
  if (cityName == "") cityName = "-";

  const int    tankPct   = tankPercent();
  const int    rssi      = WiFi.RSSI();
  const String rssiLabel = wifiSignalQuality(rssi);
  const String causeText = rainDelayCauseText();
  const bool   pausedNow = isPausedNow();
  int activeZoneCount = 0;
  for (int i = 0; i < (int)zonesCount; ++i) {
    if (zoneActive[i]) activeZoneCount++;
  }
  NextWaterInfo nextWater = computeNextWatering();
  char nextWaterTime[6] = "--:--";
  char nextWaterDay[20] = "--";
  String nextWaterLabel = String("--");
  const bool rainDelayBlockingNow = isRainDelayBlockingNow();
  String nextWaterSub = rainDelayBlockingNow ? String("Waiting for rain delay to clear") : String("No queued run");
  if (nextWater.epoch > 0 && nextWater.zone >= 0 && nextWater.zone < (int)zonesCount) {
    struct tm nextTm;
    localtime_r(&nextWater.epoch, &nextTm);
    strftime(nextWaterTime, sizeof(nextWaterTime), "%H:%M", &nextTm);
    if (ti) {
      struct tm todayTm = *ti;
      todayTm.tm_hour = 0; todayTm.tm_min = 0; todayTm.tm_sec = 0;
      struct tm nextDayTm = nextTm;
      nextDayTm.tm_hour = 0; nextDayTm.tm_min = 0; nextDayTm.tm_sec = 0;
      time_t todayEpoch = mktime(&todayTm);
      time_t targetEpoch = mktime(&nextDayTm);
      long dayDiff = (todayEpoch > 0 && targetEpoch > 0) ? (long)((targetEpoch - todayEpoch) / 86400L) : 9999L;
      if (dayDiff == 0) {
        strncpy(nextWaterDay, "Today", sizeof(nextWaterDay));
        nextWaterDay[sizeof(nextWaterDay) - 1] = '\0';
      } else if (dayDiff == 1) {
        strncpy(nextWaterDay, "Tomorrow", sizeof(nextWaterDay));
        nextWaterDay[sizeof(nextWaterDay) - 1] = '\0';
      } else {
        strftime(nextWaterDay, sizeof(nextWaterDay), "%a %d %b", &nextTm);
      }
    } else {
      strftime(nextWaterDay, sizeof(nextWaterDay), "%a %d %b", &nextTm);
    }
    nextWaterLabel = String(nextWaterDay) + String(" ") + String(nextWaterTime);
    nextWaterSub = zoneNames[nextWater.zone];
    if (nextWater.durSec > 0) {
      char durBuf[16];
      fmtMMSS(durBuf, sizeof(durBuf), nextWater.durSec);
      nextWaterSub += " - ";
      nextWaterSub += durBuf;
    }
  }
  if (!systemMasterEnabled) {
    nextWaterLabel = String("Master off");
    nextWaterSub = String("Automation blocked");
  }
  String heroSystemValue = pausedNow ? String("Paused")
                          : (systemMasterEnabled ? String("Master On") : String("Master Off"));
  String heroSystemSub = pausedNow ? String("Schedules are temporarily suspended")
                        : (systemMasterEnabled
                           ? (activeZoneCount > 0
                              ? String(activeZoneCount) + String(activeZoneCount == 1 ? " zone running" : " zones running")
                              : String("Automation ready"))
                           : String("Automation blocked"));
  String heroWeatherValue = isnan(temp) ? String("--") : String(temp, 1) + " " + temperatureUnitChar();

  // --- HTML ---
  // Each streamed stylesheet segment stays below this preallocated capacity.
  String html; html.reserve(7000);
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  auto flush = [&](){
    if (html.length()) {
      server.sendContent(html);
      html = "";
    }
  };
  html += F("<!doctype html><html lang='en' data-theme='light'><head>");
  html += F("<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<meta name='theme-color' content='#1e40af'><meta name='color-scheme' content='light dark'>");
  html += F("<title>ESP32 Irrigation</title>");
  html += F("<style>");
  html += F(".center{max-width:1280px;margin:0 auto}");
  html += F(":root[data-theme='light']{--bg:#edf3ef;--bg2:#f8fbf8;--glass:rgba(255,255,255,.58);--glass-brd:rgba(122,149,140,.32);--panel:#ffffff;--line:#d3dfd9;");
  html += F("--card:#ffffff;--ink:#14232b;--muted:#5f736f;--primary:#2563eb;--primary-2:#1e40af;--ok:#2563eb;--warn:#c97a1a;--bad:#d9485f;");
  html += F("--chip:#edf4ff;--chip-brd:#c9daf5;--ring:#dcebe5;--ring2:#b4d4cb;--shadow:0 18px 40px rgba(20,47,45,.14)}");
  html += F(":root[data-theme='dark']{--bg:#08111f;--bg2:#0d1d21;--glass:rgba(12,28,31,.62);--glass-brd:rgba(84,123,118,.3);--panel:#0f1e22;--line:#214147;");
  html += F("--card:#0f1e22;--ink:#e6f0ec;--muted:#9ab4ad;--primary:#38bdf8;--primary-2:#0284c7;--ok:#60a5fa;--warn:#f0ae4d;--bad:#ff6b7d;");
  html += F("--chip:#10233d;--chip-brd:#24444a;--ring:#123036;--ring2:#1d5152;--shadow:0 18px 40px rgba(0,0,0,.42)}");
  html += F("*{box-sizing:border-box}html{scroll-behavior:smooth}html,body{margin:0;padding:0;");
  html += F("background:radial-gradient(1200px 600px at 10% -5%,var(--bg2),transparent),");
  html += F("radial-gradient(1200px 700px at 100% 0%,var(--ring),transparent),");
  html += F("radial-gradient(900px 500px at -10% 80%,var(--ring2),transparent),var(--bg);");
  html += F("color:var(--ink);font-family:'Trebuchet MS','Candara','Segoe UI',sans-serif;line-height:1.35}");
  html += F(":root{--gap:16px;--pad:18px;--pad-lg:22px;--radius:20px;--radius-sm:16px;}");
  html += F("a{text-decoration:none;color:inherit}");

  // Top nav - made a bit tighter on mobiles
  html += F(".nav{position:sticky;top:0;z-index:10;padding:10px 12px 12px;");
  html += F("background:linear-gradient(180deg,rgba(0,0,0,.25),transparent),var(--primary-2);");
  html += F("box-shadow:0 16px 36px rgba(0,0,0,.25)}");
  html += F(".nav .in{max-width:1280px;margin:0 auto;display:flex;align-items:center;justify-content:space-between;gap:12px;color:#fff;flex-wrap:wrap}");
  html += F(".brand{display:flex;align-items:center;gap:8px;font-weight:800;letter-spacing:.2px;font-size:1.12rem}");
  html += F(".brand-copy{display:flex;flex-direction:column;gap:2px}");
  html += F(".brand-title{text-transform:uppercase;letter-spacing:.9px;font-size:1rem;line-height:1}");
  html += F(".brand-sub{font-size:.74rem;font-weight:650;color:rgba(255,255,255,.8);letter-spacing:.08em;text-transform:uppercase}");
  html += F(".dot{width:12px;height:12px;border-radius:999px;background:#93c5fd;box-shadow:0 0 14px #60a5fa}");
  html += F(".nav .meta{display:flex;gap:8px;flex-wrap:wrap;align-items:center;font-weight:650;font-size:.88rem}");
  html += F(".pill{background:rgba(255,255,255,.16);border:1px solid rgba(255,255,255,.28);border-radius:999px;padding:7px 12px}");
  html += F(".btn-ghost{background:rgba(255,255,255,.14);border:1px solid rgba(255,255,255,.35);color:#fff;");
  html += F("border-radius:10px;padding:8px 14px;font-weight:700;cursor:pointer;font-size:.92rem}");
  html += F(".hero-shell{position:relative;overflow:hidden;padding:clamp(18px,3vw,30px);margin:18px 0 18px}");
  html += F(".hero-shell::before,.hero-shell::after{content:'';position:absolute;border-radius:999px;pointer-events:none;filter:blur(8px)}");
  html += F(".hero-shell::before{width:220px;height:220px;right:-50px;top:-40px;background:radial-gradient(circle,rgba(233,173,73,.2),transparent 68%)}");
  html += F(".hero-shell::after{width:260px;height:260px;left:-90px;bottom:-100px;background:radial-gradient(circle,rgba(56,189,248,.2),transparent 68%)}");
  html += F(".hero-grid{display:grid;grid-template-columns:minmax(0,1.15fr) minmax(320px,.85fr);gap:18px;align-items:stretch}");
  html += F(".hero-copy,.hero-mini-grid{position:relative;z-index:1}");
  html += F(".hero-copy{display:flex;flex-direction:column;justify-content:center;gap:14px}");
  html += F(".hero-kicker{text-transform:uppercase;letter-spacing:.22em;font-size:.72rem;font-weight:800;color:var(--primary)}");
  html += F(".hero-title{margin:0;font-size:clamp(1.95rem,4vw,3.2rem);line-height:1.02;max-width:11ch}");
  html += F(".hero-text{margin:0;max-width:60ch;color:var(--muted);font-size:1rem}");
  html += F(".hero-action-stack{display:grid;gap:8px}.hero-date{color:var(--muted);font-size:.9rem;font-weight:700;font-variant-numeric:tabular-nums}");
  html += F(".hero-meta{display:flex;gap:8px;flex-wrap:wrap;align-items:center}.hero-meta-item{display:inline-flex;align-items:center;gap:6px;padding:7px 10px;border:1px solid var(--chip-brd);border-radius:7px;background:var(--chip);color:var(--ink);font-size:.84rem;font-weight:750}.hero-meta-k{color:var(--muted);font-size:.7rem;text-transform:uppercase;letter-spacing:.08em;font-weight:850}.hero-meta-v{font-variant-numeric:tabular-nums}");
  flush();
  html += F(".hero-actions{display:flex;gap:10px;flex-wrap:wrap}");
  html += F(".dash-nav{display:flex;gap:10px;flex-wrap:wrap;align-items:center;padding:10px 12px;margin:-2px 0 18px;position:sticky;top:74px;z-index:9}");
  html += F(".dash-nav a{display:inline-flex;align-items:center;justify-content:center;padding:9px 14px;border-radius:999px;border:1px solid var(--chip-brd);background:rgba(255,255,255,.34);font-size:.84rem;font-weight:800;letter-spacing:.04em;text-transform:uppercase;color:var(--ink);transition:transform .08s ease,background .12s ease,border-color .12s ease,box-shadow .12s ease}");
  html += F(".dash-nav a:hover{transform:translateY(-1px);background:rgba(37,99,235,.12);border-color:rgba(37,99,235,.28);box-shadow:0 10px 22px rgba(30,64,175,.12)}");
  html += F(".hero-mini-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}");
  html += F(".hero-mini{min-height:132px;padding:16px;border-radius:18px;border:1px solid var(--glass-brd);");
  html += F("background:linear-gradient(180deg,rgba(255,255,255,.58),rgba(255,255,255,.14));box-shadow:0 14px 30px rgba(19,33,68,.12);");
  html += F("display:flex;flex-direction:column;justify-content:space-between;backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px)}");
  html += F(".hero-mini.hero-mini-strong{background:linear-gradient(135deg,rgba(37,99,235,.2),rgba(96,165,250,.1))}");
  html += F(".hero-mini-label{text-transform:uppercase;letter-spacing:.16em;font-size:.74rem;font-weight:800;color:var(--muted)}");
  html += F(".hero-mini-value{font-size:1.72rem;font-weight:800;line-height:1.02;font-variant-numeric:tabular-nums}");
  html += F(".hero-mini-value.weather-value{display:inline-flex;align-items:center;gap:8px}");
  html += F(".hero-mini-sub{color:var(--muted);font-size:.92rem}");
  html += F(".section-head{display:flex;align-items:flex-end;justify-content:space-between;gap:14px;max-width:1280px;margin:0 auto 14px;padding:0 2px}");
  html += F(".section-kicker{text-transform:uppercase;letter-spacing:.18em;font-size:.72rem;font-weight:800;color:var(--primary)}");
  html += F(".section-head h2{margin:4px 0 0;font-size:1.35rem;line-height:1.08}");
  html += F(".section-note{margin:0;max-width:42ch;color:var(--muted);font-size:.92rem;text-align:right}");
  // Keep the streamed response buffer small. If this String grows until an
  // allocation fails, every later CSS append is silently lost on the ESP32.
  flush();

  // Cards and grids
  html += F(".wrap{max-width:1280px;margin:20px auto;padding:0 16px}");
  html += F(".glass{background:var(--glass);backdrop-filter:blur(12px);-webkit-backdrop-filter:blur(12px);border:1px solid var(--glass-brd);border-radius:var(--radius);box-shadow:var(--shadow)}");
  html += F(".section{padding:var(--pad)}");
  html += F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:var(--gap)}");
  html += F(".card{background:var(--card);border:1px solid var(--glass-brd);border-radius:var(--radius);box-shadow:var(--shadow);padding:var(--pad)}");
  html += F(".card h3{margin:0 0 10px 0;font-size:1.15rem;color:var(--ink);font-weight:850;letter-spacing:.35px;");
  html += F("padding-bottom:6px;border-bottom:2px solid var(--primary);display:flex;align-items:center;gap:8px}");
  html += F(".card h4{margin:6px 0 6px 0;font-size:.95rem;color:var(--muted);font-weight:700;letter-spacing:.2px}");
  html += F(".chip{display:inline-flex;align-items:center;gap:6px;background:var(--chip);border:1px solid var(--chip-brd);border-radius:999px;padding:8px 14px;font-weight:650;white-space:nowrap;font-size:.95rem;color:var(--ink)}");
  html += F(".big{font-weight:800;font-size:1.3rem;font-variant-numeric:tabular-nums}.hint{color:var(--muted);font-size:.9rem;margin-top:4px}.sub{color:var(--muted);font-size:.88rem}");
  html += F(".meter{position:relative;height:18px;border-radius:999px;background:linear-gradient(180deg,rgba(0,0,0,.12),transparent);border:1px solid var(--glass-brd);overflow:hidden;margin-top:6px}");
  html += F(".fill{position:absolute;inset:0 0 0 0;width:0%;height:100%;background:linear-gradient(90deg,#38bdf8,#60a5fa,#2563eb);");
  html += F("box-shadow:0 0 30px rgba(37,99,235,.28) inset;transition:width .4s ease}");
  html += F(".badge{display:inline-flex;align-items:center;gap:6px;padding:8px 13px;border-radius:999px;border:1px solid var(--glass-brd);font-size:.9rem}");
  html += F(".b-ok{background:rgba(37,99,235,.12);border-color:rgba(37,99,235,.3)}");
  html += F(".b-warn{background:rgba(245,158,11,.12);border-color:rgba(245,158,11,.35)}");
  html += F(".b-bad{background:rgba(239,68,68,.12);border-color:rgba(239,68,68,.38)}");
  html += F(".summary-grid{align-items:stretch}");
  html += F(".summary-card{display:flex;flex-direction:column;gap:14px;min-height:178px;background:linear-gradient(180deg,var(--card),rgba(255,255,255,.42))}");
  html += F(".summary-card h3{margin-bottom:0}");
  html += F(".summary-link{display:flex;flex-direction:column;justify-content:center;gap:8px;min-height:120px;padding:16px;border-radius:18px;");
  html += F("background:linear-gradient(135deg,var(--chip),rgba(255,255,255,.72));border:1px solid var(--chip-brd)}");
  html += F(".summary-k{font-size:.75rem;letter-spacing:.16em;text-transform:uppercase;color:var(--muted);font-weight:800}");
  html += F(".summary-value{font-size:clamp(1.45rem,2.5vw,2.15rem);line-height:1.02;font-weight:800;letter-spacing:-.03em;font-variant-numeric:tabular-nums}");
  html += F(".summary-support{color:var(--muted);font-size:.92rem}");
  html += F(".summary-row{display:flex;align-items:flex-end;justify-content:space-between;gap:12px;flex-wrap:wrap}");
  html += F(".summary-meta{display:flex;gap:8px;flex-wrap:wrap}");
  html += F(".summary-meta.status-pills{display:grid;grid-template-columns:repeat(2,minmax(0,1fr))}");
  html += F(".summary-meta.status-pills .badge{width:100%;justify-content:center}");
  flush();
  html += F(".mini-chip{display:inline-flex;align-items:center;gap:6px;padding:7px 11px;border-radius:999px;border:1px solid var(--chip-brd);background:var(--chip);font-size:.82rem;font-weight:700;color:var(--ink)}");
  html += F(".summary-metric-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:10px}");
  html += F(".summary-metric-grid.metric-pair{grid-template-columns:repeat(2,minmax(0,1fr))}");
  html += F(".summary-subhead{font-size:.76rem;letter-spacing:.16em;text-transform:uppercase;color:var(--muted);font-weight:800;margin-top:2px}");
  html += F(".metric-tile{display:flex;flex-direction:column;justify-content:space-between;gap:8px;min-height:92px;padding:12px 13px;border-radius:16px;");
  html += F("border:1px solid var(--chip-brd);background:linear-gradient(180deg,rgba(255,255,255,.55),rgba(255,255,255,.15))}");
  html += F(".metric-tile.metric-wide{grid-column:span 2}");
  html += F(".metric-split{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}");
  html += F(".metric-split-item{display:flex;align-items:flex-end;justify-content:center;min-height:44px}");
  html += F(".metric-k{font-size:.74rem;letter-spacing:.14em;text-transform:uppercase;color:var(--muted);font-weight:800}");
  html += F(".metric-v{font-size:1.08rem;font-weight:800;color:var(--ink)}");
  html += F(".metric-v.big-metric{font-size:1.45rem;line-height:1.05;font-variant-numeric:tabular-nums}");
  html += F(".metric-v .metric-unit{font-size:.88rem;font-weight:700;color:var(--muted);margin-left:4px}");
  html += F(".pressure-value{display:inline-flex;align-items:center;gap:5px}.pressure-trend{display:none;min-width:1ch;font-size:1.12em;font-weight:950;line-height:1}.pressure-trend.up,.pressure-trend.down{display:inline-block}.pressure-trend.up{color:var(--bad)}.pressure-trend.down{color:var(--primary-2)}");
  html += F(".condition-tile{gap:10px}.condition-wrap{display:flex;align-items:center;gap:12px;min-width:0}.condition-wrap .metric-v{min-width:0;overflow-wrap:anywhere}.weather-icon{position:relative;flex:0 0 54px;width:54px;height:54px;border-radius:8px;border:1px solid var(--chip-brd);background:linear-gradient(180deg,var(--panel),var(--chip));overflow:hidden}");
  html += F(".weather-icon .wi-main,.weather-icon .wi-extra{position:absolute;display:block}.wi-clear .wi-main{left:17px;top:17px;width:20px;height:20px;border-radius:50%;background:#f7c948;box-shadow:0 0 0 5px rgba(247,201,72,.22),0 0 18px rgba(247,201,72,.45)}.wi-clear .wi-extra{left:26px;top:5px;width:2px;height:44px;background:#f7c948;transform:rotate(45deg)}.wi-clear .wi-extra::before{content:'';position:absolute;left:0;top:0;width:2px;height:44px;background:#f7c948;transform:rotate(90deg)}");
  html += F(".wi-cloud .wi-main,.wi-partly .wi-main,.wi-overcast .wi-main,.wi-drizzle .wi-main,.wi-showers .wi-main,.wi-rain .wi-main,.wi-freezing .wi-main,.wi-snow .wi-main,.wi-storm .wi-main{left:12px;top:24px;width:31px;height:15px;border-radius:999px;background:#9aa7b5;box-shadow:-5px 2px 0 #9aa7b5,7px -6px 0 3px #9aa7b5,-8px -4px 0 2px #9aa7b5}.wi-partly .wi-extra{left:8px;top:9px;width:19px;height:19px;border-radius:50%;background:#f7c948;box-shadow:0 0 0 4px rgba(247,201,72,.2);z-index:0}.wi-overcast .wi-extra{left:7px;top:17px;width:28px;height:14px;border-radius:999px;background:#7b8794;box-shadow:13px 2px 0 #7b8794}.wi-drizzle .wi-extra{left:17px;top:41px;width:2px;height:5px;border-radius:999px;background:#38bdf8;box-shadow:7px 0 #38bdf8,14px 0 #38bdf8,21px 0 #38bdf8}.wi-showers .wi-extra{left:15px;top:39px;width:4px;height:12px;border-radius:999px;background:#2563eb;box-shadow:10px 2px #2563eb,20px 0 #2563eb;transform:skewX(-18deg)}");
  html += F(".wi-rain .wi-extra{left:17px;top:40px;width:3px;height:9px;border-radius:999px;background:#3b82f6;box-shadow:9px 0 #3b82f6,18px 0 #3b82f6;transform:skewX(-18deg)}.wi-freezing .wi-extra{left:19px;top:37px;width:16px;height:16px;border:2px solid #93c5fd;border-radius:3px;transform:rotate(45deg);background:rgba(147,197,253,.12)}.wi-snow .wi-extra{left:17px;top:40px;width:4px;height:4px;border-radius:50%;background:#dbeafe;box-shadow:9px 0 #dbeafe,18px 0 #dbeafe,4px 8px #dbeafe,14px 8px #dbeafe}.wi-storm .wi-extra{left:25px;top:36px;width:9px;height:16px;background:#f59e0b;clip-path:polygon(42% 0,100% 0,62% 45%,100% 45%,25% 100%,43% 55%,0 55%)}.wi-fog .wi-main{left:10px;top:18px;width:34px;height:3px;border-radius:999px;background:#94a3b8;box-shadow:0 8px #94a3b8,0 16px #94a3b8}.wi-fog .wi-extra{left:17px;top:42px;width:21px;height:3px;border-radius:999px;background:#94a3b8}.wi-unknown .wi-main{left:17px;top:10px;font-size:28px;line-height:34px;font-weight:900;color:var(--muted)}.wi-unknown .wi-main::before{content:'?'}");
  html += F(".wind-dir-tile{gap:10px}.wind-compass-wrap{display:flex;align-items:center;gap:12px;min-width:0}.wind-compass{--dir:0deg;position:relative;flex:0 0 66px;width:66px;height:66px;border-radius:50%;border:1px solid var(--chip-brd);background:radial-gradient(circle,var(--panel) 0 48%,var(--chip) 49% 100%);box-shadow:inset 0 0 0 1px rgba(255,255,255,.18)}");
  html += F(".wind-compass::before,.wind-compass::after{content:'';position:absolute;background:var(--line);opacity:.8}.wind-compass::before{left:50%;top:8px;bottom:8px;width:1px}.wind-compass::after{top:50%;left:8px;right:8px;height:1px}.wind-compass .cardinal{position:absolute;font-size:.55rem;font-weight:900;color:var(--muted);line-height:1}.wind-compass .n{top:4px;left:50%;transform:translateX(-50%)}.wind-compass .e{right:5px;top:50%;transform:translateY(-50%)}.wind-compass .s{bottom:4px;left:50%;transform:translateX(-50%)}.wind-compass .w{left:5px;top:50%;transform:translateY(-50%)}");
  html += F(".wind-needle{position:absolute;left:50%;top:50%;z-index:2;width:4px;height:24px;border-radius:999px;background:linear-gradient(180deg,var(--bad),var(--primary));transform-origin:50% 100%;transform:translate(-50%,-100%) rotate(var(--dir));box-shadow:0 2px 8px rgba(0,0,0,.22)}.wind-needle::before{content:'';position:absolute;left:50%;top:-6px;transform:translateX(-50%);border-left:6px solid transparent;border-right:6px solid transparent;border-bottom:9px solid var(--bad)}.wind-center{position:absolute;left:50%;top:50%;z-index:3;width:8px;height:8px;border-radius:50%;background:var(--ink);transform:translate(-50%,-50%)}.wind-compass.is-empty .wind-needle{display:none}.wind-dir-text{min-width:0;overflow-wrap:anywhere}");
  flush();
  html += F(".summary-note{padding:12px 14px;border-radius:16px;border:1px solid rgba(37,99,235,.2);background:linear-gradient(180deg,rgba(37,99,235,.1),rgba(37,99,235,.03));color:var(--muted);font-size:.9rem}");
  html += F(".summary-note.is-disabled{border-color:var(--line);background:rgba(148,163,184,.12);color:rgba(100,116,139,.9);filter:grayscale(1);opacity:.72}");
  html += F(".toolbar{display:flex;gap:var(--gap);flex-wrap:wrap;margin:14px 0 0}");
  html += F(".section-block,.summary-shell,.sched-shell,.zones-shell,.controls-shell{scroll-margin-top:140px}");
  html += F(".btn{background:linear-gradient(180deg,var(--primary),var(--primary-2));color:#fff;border:1px solid rgba(0,0,0,.08);border-radius:13px;padding:11px 16px;font-weight:800;cursor:pointer;");
  html += F("box-shadow:0 8px 20px rgba(0,0,0,.22);font-size:1.02rem}");
  html += F(".btn-secondary{background:transparent;color:var(--ink);border:1px solid var(--line);box-shadow:none}");
  html += F(".btn:disabled{background:#7f8aa1;cursor:not-allowed;box-shadow:none}.btn-danger{background:linear-gradient(180deg,#ef4444,#b91c1c);border-color:rgba(185,28,28,.6)}");
  html += F(".btn,.btn-ghost,.pill{transition:transform .06s ease,box-shadow .06s ease,filter .06s ease}");
  html += F(".btn,.btn-ghost,.pill{touch-action:manipulation}");
  html += F(".btn:active:not(:disabled),.btn-ghost:active,.pill:active{transform:translateY(1px);box-shadow:inset 0 2px 6px rgba(0,0,0,.25)}");
  html += F(".btn,.btn-ghost,.pill{position:relative;overflow:hidden}");
  html += F(".ripple{position:absolute;border-radius:999px;transform:scale(0);background:rgba(255,255,255,.35);animation:ripple .5s ease-out;pointer-events:none}");
  html += F("@keyframes ripple{to{transform:scale(3.2);opacity:0;}}");
  html += F(".zone-overview{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:var(--gap);margin-bottom:18px}");
  html += F(".zone-summary-card{padding:18px;border:1px solid var(--line);border-radius:18px;background:linear-gradient(160deg,var(--panel),rgba(56,189,248,.08));display:flex;flex-direction:column;gap:8px;min-height:136px}");
  html += F(".zone-summary-k{font-size:.74rem;letter-spacing:.16em;text-transform:uppercase;color:var(--muted);font-weight:800}");
  html += F(".zone-summary-v{font-size:1.3rem;line-height:1.06;font-weight:850;color:var(--ink)}");
  html += F(".zone-summary-sub{font-size:.95rem;color:var(--ink);font-weight:650}");
  html += F(".zone-summary-meta{margin-top:auto;font-size:.84rem;color:var(--muted)}");
  html += F(".zone-list-head{display:flex;align-items:flex-end;justify-content:space-between;gap:12px;flex-wrap:wrap;margin-bottom:10px}");
  html += F(".zone-list-head h3{margin:0}");
  html += F(".zone-list-note{font-size:.9rem;color:var(--muted)}");
  html += F(".zone-list{display:flex;flex-direction:column;gap:10px}");
  html += F(".zone-row{display:grid;grid-template-columns:minmax(0,1.6fr) auto auto;gap:12px;align-items:center;padding:14px 16px;border:1px solid var(--line);border-radius:18px;background:linear-gradient(180deg,var(--panel),rgba(255,255,255,.04));transition:border-color .14s ease,box-shadow .14s ease,transform .14s ease}");
  html += F(".zone-row.is-active{border-color:rgba(37,99,235,.42);box-shadow:0 14px 30px rgba(37,99,235,.12)}");
  html += F(".zone-row-main{display:flex;align-items:center;gap:10px;min-width:0}");
  html += F(".zone-row-copy{min-width:0}");
  html += F(".zone-row-name{font-size:1rem;font-weight:780;color:var(--ink);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}");
  html += F(".zone-row-sub{font-size:.84rem;color:var(--muted);margin-top:2px}");
  html += F(".zone-index{width:26px;height:26px;border-radius:999px;display:flex;align-items:center;justify-content:center;");
  html += F("font-size:.82rem;background:var(--chip);border:1px solid var(--chip-brd);flex-shrink:0}");
  html += F(".zone-dot{width:13px;height:13px;border-radius:999px;background:var(--line);border:1px solid var(--glass-brd);box-shadow:0 0 0 2px rgba(0,0,0,.06) inset}");
  html += F(".zone-dot.on{background:var(--ok);box-shadow:0 0 10px rgba(37,99,235,.55)}");
  html += F(".zone-row-status{display:flex;align-items:center;justify-content:flex-end;gap:10px;flex-wrap:wrap}");
  html += F(".zone-row-rem{font-size:.96rem;font-weight:760;color:var(--ink);font-variant-numeric:tabular-nums;min-width:86px;text-align:right}");
  html += F(".zone-row-actions{margin:0;display:flex;gap:8px;justify-content:flex-end;flex-wrap:wrap}");
  html += F(".zone-row-actions .btn{padding:9px 14px;font-size:.92rem}");
  html += F("body{-webkit-font-smoothing:antialiased;text-rendering:optimizeLegibility}");
  html += F(".card{transition:transform .12s ease,box-shadow .12s ease}");
  html += F(".card:hover{transform:translateY(-2px);box-shadow:0 16px 34px rgba(0,0,0,.18)}");
  html += F(".btn:hover{filter:brightness(1.05)}");
  html += F(".chip:hover,.pill:hover{filter:brightness(1.02)}");
  html += F(".btn:focus-visible,.pill:focus-visible,.chip:focus-visible{outline:2px solid var(--primary);outline-offset:2px}");
  html += F("@media (prefers-reduced-motion: reduce){*{animation:none!important;transition:none!important}}");

  // Schedules styles (collapsible, mobile-friendly)
  html += F(".sched{margin-top:var(--gap)}");
  html += F(".sched-shell{padding:0;overflow:hidden}");
  html += F(".sched-top{display:flex;align-items:flex-end;justify-content:space-between;gap:16px;flex-wrap:wrap;padding:18px 18px 14px;border-bottom:1px solid var(--line);background:linear-gradient(180deg,rgba(255,255,255,.05),transparent)}");
  html += F(".sched-top-copy{display:flex;flex-direction:column;gap:6px;max-width:58ch}");
  html += F(".sched-top-copy h3{margin:0;font-size:1.2rem}");
  html += F(".sched-top-copy p{margin:0;color:var(--muted);font-size:.92rem}");
  html += F(".sched-tools{display:flex;gap:10px;flex-wrap:wrap;align-items:center}");
  flush();
  html += F(".sched-ctr{--schedW:360px;--gap:var(--gap);max-width:100%;margin:0 auto}");
  html += F(".sched-body{padding:18px}");
  html += F(".sched-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,420px),1fr));gap:var(--gap)}");
  html += F(".sched-card{min-width:0;background:var(--panel);border:1px solid var(--line);border-radius:var(--radius-sm);padding:var(--pad)}");
  html += F(".sched-card h4{margin:0 0 10px 0;font-size:1.05rem;font-weight:800;color:var(--ink);display:flex;align-items:center;gap:8px}");
  html += F(".sched-badge{background:var(--chip);border:1px solid var(--chip-brd);border-radius:999px;padding:4px 9px;font-size:.8rem;font-weight:700;color:var(--muted)}");
  html += F(".rowx{display:grid;grid-template-columns:96px minmax(0,1fr);gap:10px;align-items:center;margin:10px 0}");
  html += F(".rowx label{min-width:0;font-size:.82rem;color:var(--muted);font-weight:700;letter-spacing:.4px;text-transform:uppercase}");
  html += F(".field{display:flex;align-items:center;gap:8px;flex-wrap:wrap;min-width:0}");
  html += F(".field.inline .in{width:72px}");
  html += F(".field .sep{color:var(--muted);font-weight:700}");
  html += F(".field .unit{color:var(--muted);font-size:.85rem}");
  html += F(".time-spin{display:inline-grid;grid-template-columns:auto 10px auto auto;align-items:center;gap:8px;max-width:100%;padding:8px;border:1px solid var(--line);border-radius:12px;background:rgba(255,255,255,.04)}");
  html += F(".time-part{display:grid;grid-template-columns:auto 34px;grid-template-rows:23px 23px;align-items:stretch;justify-items:stretch;gap:3px}");
  html += F(".time-btn{appearance:none;-webkit-appearance:none;border:1px solid var(--chip-brd);background:var(--chip);color:var(--ink);border-radius:8px;cursor:pointer;font-weight:900;line-height:1;touch-action:manipulation;user-select:none;-webkit-user-select:none}");
  html += F(".time-arrow{width:34px;height:23px;padding:0;font-size:.7rem;border-radius:7px}.time-val{grid-row:1/3;width:58px;height:49px;font-size:1.08rem;font-variant-numeric:tabular-nums}.time-ampm{align-self:center;width:62px;height:49px;font-size:.95rem}");
  html += F(".time-val.is-editing{outline:2px solid var(--primary);outline-offset:2px;background:rgba(37,99,235,.16)}.time-colon{color:var(--muted);font-weight:900;font-size:1.25rem}.time-btn:hover{filter:brightness(1.05)}.time-btn:active{transform:translateY(1px)}.time-btn:focus-visible{outline:2px solid var(--primary);outline-offset:2px}");
  html += F(".duration-spin{grid-template-columns:auto auto auto auto}.duration-spin .time-val{width:68px}.duration-unit{color:var(--muted);font-size:.78rem;font-weight:800;text-transform:uppercase}");
  html += F(".toggle-inline{display:inline-flex;align-items:center;gap:6px;font-size:.85rem;color:var(--muted);padding:7px 11px;border-radius:999px;border:1px solid var(--chip-brd);background:transparent}");
  html += F(".sched-card input[type=checkbox]{appearance:none;-webkit-appearance:none;width:18px;height:18px;margin:0;flex:0 0 18px;display:inline-grid;place-content:center;cursor:pointer;");
  html += F("border:1.6px solid rgba(230,240,236,.78);border-radius:6px;background:transparent;box-shadow:none;transition:border-color .12s ease,box-shadow .12s ease,transform .06s ease}");
  html += F(".sched-card input[type=checkbox]::before{content:'';display:block;width:5px;height:9px;border-right:2px solid #fff;border-bottom:2px solid #fff;transform:rotate(45deg) scale(0);margin-top:-1px;transition:transform .12s ease}");
  html += F(".sched-card input[type=checkbox]:checked{border-color:#ffffff;background:transparent;box-shadow:none}");
  html += F(".sched-card input[type=checkbox]:checked::before{transform:rotate(45deg) scale(1)}");
  html += F(".sched-card input[type=checkbox]:focus-visible{outline:none;border-color:#ffffff;box-shadow:0 0 0 3px rgba(37,99,235,.18)}");
  html += F(".sched-card input[type=checkbox]:active{transform:translateY(1px)}");
  html += F(".in{border:1px solid var(--line);border-radius:10px;padding:8px 10px;background:transparent;color:var(--ink);font-size:.9rem}");
  flush();
  html += F(".days-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;width:100%}");
  html += F(".day{position:relative;display:block;min-width:0}");
  html += F(".day input{position:absolute;inset:0;width:100%;height:100%;margin:0;opacity:0;cursor:pointer;z-index:2}");
  html += F(".day span{display:flex;align-items:center;justify-content:center;min-height:44px;padding:10px 16px;border-radius:999px;border:1px solid var(--chip-brd);");
  html += F("background:linear-gradient(180deg,rgba(255,255,255,.06),rgba(255,255,255,.02));font-size:.78rem;font-weight:800;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);transition:border-color .12s ease,background .12s ease,box-shadow .12s ease,transform .06s ease}");
  html += F(".day input:checked + span{border-color:rgba(37,99,235,.46);background:linear-gradient(180deg,rgba(37,99,235,.18),rgba(37,99,235,.05));color:var(--ink);box-shadow:0 10px 20px rgba(30,64,175,.12)}");
  html += F(".day input:focus-visible + span{outline:2px solid var(--primary);outline-offset:2px}");
  html += F(".day input:active + span{transform:translateY(1px)}");
  html += F("@media(max-width:720px){.nav .in{flex-direction:column;align-items:flex-start}.zones{grid-template-columns:1fr}.sched-grid{grid-template-columns:1fr}.rowx{grid-template-columns:1fr}.rowx label{margin-bottom:4px}.days-grid{grid-template-columns:repeat(3,minmax(0,1fr))}.day span{min-height:42px}.time-spin{width:100%;grid-template-columns:auto 10px auto auto;justify-content:flex-start}.time-arrow{width:38px;height:25px}.time-val{width:62px;height:53px}.time-ampm{height:53px}}");
  html += F("@media(max-width:460px){.days-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.time-spin{grid-template-columns:auto 8px auto}.time-ampm{grid-column:1/-1;width:100%}.duration-spin{grid-template-columns:auto auto}.duration-unit{margin-right:6px}}");
  html += F(".collapse{cursor:pointer;user-select:none;display:flex;align-items:center;justify-content:space-between;font-size:1.05rem}");
  html += F(".collapse .arr{font-size:1rem;opacity:.8;margin-left:6px}");
  html += F(".sched-title{display:flex;flex-direction:column;gap:2px}");
  html += F(".sched-sub{font-size:.88rem;color:var(--muted);font-weight:600}");

  // Desktop enhancements
  html += F("@media(min-width:1024px){");
  html += F("body{font-size:16px;}");
  html += F(".wrap{margin:20px auto;padding:0 20px;}");
  html += F(".glass.section{padding:var(--pad-lg);}");
  html += F(".summary-grid{grid-template-columns:repeat(4,minmax(0,1fr));}");
  html += F(".zones{grid-template-columns:repeat(3,minmax(300px,1fr));}");
  html += F(".card{padding:var(--pad-lg);}");
  html += F(".card h3{font-size:1.12rem;}");
  html += F("}");
  flush();
  html += F("html,body{font-family:'Sora','Avenir Next','Trebuchet MS',sans-serif}");
  html += F("body{position:relative;min-height:100vh}");
  html += F("body::before,body::after{content:'';position:fixed;z-index:-1;pointer-events:none;filter:blur(28px);opacity:.42}");
  html += F("body::before{width:340px;height:340px;right:-120px;top:56px;background:radial-gradient(circle,#0284c7,transparent 68%);animation:floatbg 14s ease-in-out infinite}");
  html += F("body::after{width:290px;height:290px;left:-110px;bottom:36px;background:radial-gradient(circle,#3b82f6,transparent 66%);animation:floatbg 16s ease-in-out infinite reverse}");
  html += F("@keyframes floatbg{0%,100%{transform:translateY(0)}50%{transform:translateY(-14px)}}");
  html += F(".nav{border-bottom:1px solid rgba(255,255,255,.2);backdrop-filter:blur(7px);-webkit-backdrop-filter:blur(7px)}");
  html += F(".pill,#btn-master{font-weight:700}");
  html += F("#clock{font-family:'JetBrains Mono','Consolas',monospace;font-size:.84rem}");
  html += F(".card{position:relative;overflow:hidden}");
  html += F(".card::before{content:'';position:absolute;left:0;right:0;top:0;height:3px;background:linear-gradient(90deg,var(--primary),#60a5fa);opacity:.85}");
  html += F(".summary-grid .card{animation:rise .45s ease both}");
  html += F(".summary-grid .card:nth-child(2){animation-delay:.05s}.summary-grid .card:nth-child(3){animation-delay:.1s}.summary-grid .card:nth-child(4){animation-delay:.15s}");
  html += F("@keyframes rise{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:translateY(0)}}");
  html += F(".btn{font-weight:760;letter-spacing:.15px}");
  html += F(".chip b,.badge b{font-weight:800}");
  html += F("html[data-theme='dark'] .dash-nav a{background:transparent;border-color:var(--chip-brd)}");
  html += F("html[data-theme='dark'] .hero-mini{background:linear-gradient(180deg,rgba(15,30,34,.96),rgba(15,30,34,.9));box-shadow:0 14px 30px rgba(0,0,0,.22)}");
  html += F("html[data-theme='dark'] .hero-mini.hero-mini-strong{background:linear-gradient(135deg,rgba(37,99,235,.18),rgba(16,39,43,.94))}");
  html += F("html[data-theme='dark'] .summary-card{background:var(--card)}");
  html += F("html[data-theme='dark'] .summary-link{background:var(--chip)}");
  flush();
  html += F("html[data-theme='dark'] .metric-tile{background:#102327}");
  html += F("html[data-theme='dark'] .sched-top{background:transparent}");
  html += F("html[data-theme='dark'] .action-card{background:var(--card)}");
  html += F("html[data-theme='light'] .sched-card input[type=checkbox]{border-color:#8aa59c;background:rgba(255,255,255,.78);box-shadow:inset 0 1px 0 rgba(255,255,255,.75)}");
  html += F("html[data-theme='light'] .sched-card input[type=checkbox]::before{border-right-color:#1e40af;border-bottom-color:#1e40af}");
  html += F("html[data-theme='light'] .sched-card input[type=checkbox]:checked{border-color:#2563eb;background:#ffffff;box-shadow:0 0 0 3px rgba(37,99,235,.12)}");
  html += F("html[data-theme='light'] .sched-card input[type=checkbox]:focus-visible{border-color:#2563eb;box-shadow:0 0 0 3px rgba(37,99,235,.14)}");
  html += F(".summary-grid .weather-card,.summary-grid .next-card{grid-column:span 2}");
  html += F(".action-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:var(--gap);margin:12px auto 24px}");
  html += F(".action-card{min-height:180px;display:flex;flex-direction:column;justify-content:space-between;background:linear-gradient(180deg,var(--card),rgba(255,255,255,.32))}");
  html += F(".action-copy p{margin:8px 0 0;color:var(--muted)}");
  html += F("#clock,.hero-mini-value{font-variant-numeric:tabular-nums}");
  html += F("@media(max-width:980px){.hero-grid{grid-template-columns:1fr}.section-head{flex-direction:column;align-items:flex-start}.section-note{text-align:left}.summary-grid .weather-card,.summary-grid .next-card{grid-column:auto}.metric-tile.metric-wide{grid-column:auto}.dash-nav{top:68px}}");
  html += F(":root{--radius:8px;--radius-sm:8px;--shadow:0 10px 28px rgba(15,23,42,.12)}");
  html += F(":root[data-theme='light']{--bg:#f3f7fc;--bg2:#fbfdff;--glass:rgba(255,255,255,.78);--glass-brd:#d8e3dd;--card:#ffffff;--line:#dbe6f3;--muted:#60726e;--ring:#e6eefb;--ring2:#f3e8cf}");
  html += F(":root[data-theme='dark']{--bg:#0a1114;--bg2:#111d21;--glass:rgba(16,29,33,.78);--glass-brd:#264148;--card:#101d21;--line:#263f45;--chip:#13282d;--chip-brd:#2b4a51;--shadow:0 12px 30px rgba(0,0,0,.34)}");
  html += F(":root[data-theme='light']{--bg:#f3f7fc;--bg2:#fbfdff;--glass-brd:#d7e2f0;--line:#dbe6f3;--muted:#5f6f84;--primary:#2563eb;--primary-2:#1e40af;--chip:#edf4ff;--chip-brd:#c9daf5;--ring:#e6eefb;--ring2:#dbeafe}");
  html += F(":root[data-theme='dark']{--bg:#08111f;--bg2:#0d1b2f;--glass:rgba(13,27,47,.8);--glass-brd:#24405f;--card:#0f1d31;--line:#263f60;--primary:#38bdf8;--primary-2:#2563eb;--chip:#10233d;--chip-brd:#294b73}");
  html += F("html,body{background:linear-gradient(180deg,var(--bg2),var(--bg));letter-spacing:0}");
  html += F("body::before,body::after,.hero-shell::before,.hero-shell::after{display:none}");
  html += F(".nav{background:linear-gradient(180deg,#1e40af,#172554);box-shadow:0 8px 22px rgba(0,0,0,.18)}");
  html += F(".wrap{margin:16px auto}.glass,.card,.hero-mini,.summary-link,.metric-tile,.sched-card,.zone-row,.zone-summary-card,.action-card{border-radius:8px}");
  html += F(".hero-shell{padding:22px;margin:16px 0;background:linear-gradient(135deg,var(--glass),rgba(37,99,235,.08));border-color:var(--glass-brd)}");
  html += F(".hero-title{font-size:clamp(1.7rem,3vw,2.55rem);max-width:none}.hero-text:empty{display:none}");
  html += F(".hero-mini{min-height:118px;background:var(--card);box-shadow:none}.hero-mini.hero-mini-strong{background:linear-gradient(135deg,rgba(37,99,235,.16),var(--card))}");
  html += F(".dash-nav{top:62px;background:rgba(255,255,255,.76);box-shadow:0 8px 22px rgba(15,23,42,.08)}html[data-theme='dark'] .dash-nav{background:rgba(16,29,33,.82)}");
  html += F(".card{box-shadow:0 8px 24px rgba(15,23,42,.08)}.card:hover{transform:none;box-shadow:0 10px 28px rgba(15,23,42,.12)}");
  html += F(".card h3{border-bottom:1px solid var(--line);font-size:1.02rem;letter-spacing:0}.card::before{height:2px}");
  html += F(".btn,.btn-ghost,.btn-secondary,.pill,.mini-chip,.badge,.chip,.day span,.toggle-inline,.in{border-radius:8px}");
  html += F(".summary-link,.metric-tile{background:#f8fbf9}.summary-value{letter-spacing:0}.zone-row{box-shadow:0 1px 0 rgba(15,23,42,.03)}");
  html += F(".card::before{background:linear-gradient(90deg,#2563eb,#38bdf8)}.fill{background:linear-gradient(90deg,#93c5fd,#38bdf8,#2563eb);box-shadow:0 0 30px rgba(56,189,248,.28) inset}");
  html += F(".summary-note{border-color:rgba(37,99,235,.2);background:linear-gradient(180deg,rgba(37,99,235,.1),rgba(37,99,235,.03))}.summary-note.is-disabled{border-color:var(--line);background:rgba(148,163,184,.12);color:var(--muted);filter:grayscale(1);opacity:.68}.zone-summary-card{background:linear-gradient(160deg,var(--panel),rgba(37,99,235,.08))}");
  html += F(".day input:checked + span{border-color:rgba(37,99,235,.46);background:linear-gradient(180deg,rgba(37,99,235,.18),rgba(37,99,235,.05));box-shadow:0 10px 20px rgba(37,99,235,.12)}");
  html += F(".sched-card input[type=checkbox]:focus-visible{box-shadow:0 0 0 3px rgba(37,99,235,.18)}html[data-theme='light'] .sched-card input[type=checkbox]::before{border-right-color:#1e40af;border-bottom-color:#1e40af}");
  html += F("html[data-theme='light'] .sched-card input[type=checkbox]:checked,html[data-theme='light'] .sched-card input[type=checkbox]:focus-visible{border-color:#2563eb;box-shadow:0 0 0 3px rgba(37,99,235,.12)}");
  html += F("html[data-theme='dark'] .summary-link,html[data-theme='dark'] .metric-tile,html[data-theme='dark'] .hero-mini{background:#102126}");
  html += F("@media(max-width:720px){.wrap{padding:0 12px;margin:12px auto}.nav .meta{width:100%}.pill,#themeBtn{flex:1 1 auto;justify-content:center}.dash-nav{top:116px;border-radius:8px}.hero-shell{padding:16px}.section-head{margin-bottom:10px}.card{padding:16px}}");
  html += F("@media(max-width:720px){.nav{padding-top:8px}.chip{font-size:.88rem}.hero-shell{padding:18px}.hero-title{max-width:none;font-size:1.95rem}.hero-mini-grid{grid-template-columns:1fr}.zone-row{grid-template-columns:1fr}.zone-row-status,.zone-row-actions{justify-content:flex-start}.zone-row-actions .btn{flex:1 1 140px}.brand-title{letter-spacing:.55px}.summary-card{min-height:auto}.summary-link{min-height:0}.summary-metric-grid{grid-template-columns:1fr 1fr}.summary-metric-grid.metric-pair{grid-template-columns:1fr}.summary-meta.status-pills{grid-template-columns:1fr}.dash-nav{top:120px;overflow:auto;flex-wrap:nowrap;padding-bottom:8px}.dash-nav a{white-space:nowrap}.sched-top{padding:16px}.sched-body{padding:16px}.sched-tools .btn{flex:1 1 140px}}");
  flush();
  html += F(".nav{background:#1e3a8a;box-shadow:0 4px 14px rgba(15,23,42,.18)}.dot{box-shadow:none}.hero-shell{background:var(--card);border-left:4px solid var(--primary);box-shadow:0 8px 22px rgba(15,23,42,.08)}.hero-title{letter-spacing:0}.hero-text{max-width:54ch}.hero-mini{background:var(--panel);box-shadow:none;border-color:var(--line)}.hero-mini.hero-mini-strong{background:var(--chip);border-color:var(--chip-brd)}.dash-nav{background:var(--card);border:1px solid var(--line);box-shadow:0 5px 14px rgba(15,23,42,.08)}.dash-nav a{border-radius:6px;background:transparent;border-color:transparent}.dash-nav a:hover{transform:none;box-shadow:none;background:rgba(37,99,235,.1)}.summary-shell{padding:0;background:transparent;border:0;box-shadow:none}.summary-card{border-top:3px solid var(--primary);background:var(--card);box-shadow:0 6px 18px rgba(15,23,42,.06)}.summary-card:nth-child(2){border-top-color:#3b82f6}.summary-card:nth-child(3){border-top-color:#0891b2}.summary-card:nth-child(4){border-top-color:#2563eb}.summary-card:nth-child(5){border-top-color:var(--primary)}.zone-row{background:var(--card);border-radius:7px;box-shadow:0 2px 8px rgba(15,23,42,.05)}.zone-row.is-active{box-shadow:0 6px 16px rgba(37,99,235,.12)}.zone-dot{box-shadow:none}.section-head h2{letter-spacing:0}.action-card{background:var(--card)}@media(max-width:720px){.nav .meta{gap:6px}.hero-shell{border-left:0;border-top:3px solid var(--primary)}.summary-shell{padding:0}.summary-grid{gap:10px}.zone-row-actions .btn{min-height:42px}}");
  html += F(".hero-grid{grid-template-columns:minmax(0,.9fr) minmax(440px,1.1fr);gap:24px}.hero-copy{padding:4px 6px}.hero-title{font-size:clamp(1.8rem,3vw,2.6rem);line-height:1.08}.hero-actions .btn{min-width:108px;text-align:center}.hero-date{padding-left:2px}.hero-mini-grid{gap:10px}.hero-mini{min-height:104px;padding:14px 15px;border-left:3px solid var(--line)}.hero-mini:nth-child(1){border-left-color:var(--primary)}.hero-mini:nth-child(2){border-left-color:#2563eb}.hero-mini:nth-child(3){border-left-color:#3b82f6}.hero-mini:nth-child(4){border-left-color:#0891b2}.hero-mini-value{font-size:1.48rem}.hero-mini-sub{white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.dash-nav{justify-content:center;margin-top:-4px}.dash-nav a{min-width:112px}.section-block{margin-top:26px}.section-head{align-items:center}.summary-grid{grid-template-columns:repeat(12,minmax(0,1fr));gap:14px}.summary-card{min-height:168px;grid-column:span 3}.summary-card.weather-card,.summary-card.next-card{grid-column:span 6;min-height:0}.summary-card h3{padding-bottom:9px}.summary-link{min-height:0;padding:0;border:0;background:transparent}.summary-link:hover .summary-value{color:var(--primary)}.summary-value{overflow-wrap:anywhere}.metric-tile{min-width:0}.weather-card .summary-metric-grid{grid-template-columns:repeat(4,minmax(0,1fr))}.weather-card .summary-metric-grid.metric-pair{grid-template-columns:repeat(2,minmax(0,1fr))}.next-card .summary-metric-grid{grid-template-columns:repeat(4,minmax(0,1fr))}.next-card .metric-wide{grid-column:span 2}.meter{height:14px;margin-top:auto}");
  flush();
  html += F(".hero-action-stack{margin-top:20px;gap:14px}.hero-date{margin-top:12px;padding-left:0;font-size:clamp(1.28rem,2vw,1.72rem);line-height:1.12;font-weight:850;color:var(--ink)}");
  html += F("@media(max-width:1060px){.hero-grid{grid-template-columns:1fr}.hero-copy{padding:0}.summary-card{grid-column:span 6}.summary-card.weather-card,.summary-card.next-card{grid-column:span 12}}@media(max-width:720px){.nav{position:relative}.dash-nav{position:sticky;top:8px;justify-content:flex-start}.hero-shell{padding:16px}.hero-mini{min-height:96px}.hero-mini-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.hero-mini-value{font-size:1.3rem}.hero-mini-sub{font-size:.82rem}.summary-card{grid-column:span 12}.weather-card .summary-metric-grid,.next-card .summary-metric-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.section-block{margin-top:22px}}@media(max-width:440px){.hero-actions{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:7px}.hero-actions .btn{min-width:0;padding:10px 7px;font-size:.9rem}.hero-mini-grid{grid-template-columns:1fr}.weather-card .summary-metric-grid.metric-pair{grid-template-columns:1fr}.next-card .metric-wide{grid-column:span 2}.condition-wrap,.wind-compass-wrap{gap:8px}}");
  html += F(":root{--gap:12px;--pad:14px;--pad-lg:16px;--radius:8px;--shadow:0 6px 18px rgba(15,23,42,.07)}");
  html += F(":root[data-theme='light']{--bg:#f4f7fb;--bg2:#fbfdff;--glass:#ffffff;--glass-brd:#d9e3ef;--card:#ffffff;--panel:#ffffff;--line:#dbe4ef;--ink:#172033;--muted:#68758a;--chip:#f1f6ff;--chip-brd:#cdddf4}");
  html += F(":root[data-theme='dark']{--bg:#0b1220;--bg2:#101827;--glass:#111c2e;--glass-brd:#293a54;--card:#111c2e;--panel:#111c2e;--line:#293a54;--ink:#e7eef9;--muted:#9aa9bd;--chip:#13233a;--chip-brd:#2f4668}");
  html += F("body{background:var(--bg);font-family:'Segoe UI',Arial,sans-serif}.wrap{max-width:1180px;margin:12px auto;padding:0 14px}.nav{padding:9px 12px;background:#1e3a8a}.brand{gap:10px}.brand-title{font-size:.96rem}.brand-sub{font-size:.68rem}.dot{width:9px;height:9px;background:#60a5fa}");
  html += F(".nav .meta{gap:6px;font-size:.82rem}.pill,.btn-ghost{padding:6px 10px;border-radius:7px}.hero-shell{margin:12px 0;padding:16px;border-left:0;border-top:3px solid var(--primary);box-shadow:var(--shadow)}.hero-grid{grid-template-columns:minmax(0,.8fr) minmax(420px,1.2fr);gap:14px}");
  html += F(".hero-kicker,.section-kicker,.summary-k,.metric-k{letter-spacing:.08em}.hero-title{font-size:clamp(1.55rem,2.4vw,2.15rem)}.hero-action-stack{margin-top:10px;gap:10px}.hero-date{margin-top:6px;font-size:1.08rem;font-weight:800}.hero-meta{margin-top:-2px}.hero-actions .btn{min-width:92px;padding:9px 12px}");
  html += F(".hero-mini-grid{gap:8px}.hero-mini{min-height:86px;padding:11px 12px;border-left:0;border-top:2px solid var(--line)}.hero-mini-label{font-size:.66rem;letter-spacing:.1em}.hero-mini-value{font-size:1.24rem}.hero-mini-sub{font-size:.8rem}.dash-nav{top:53px;margin:0 0 12px;padding:7px;gap:6px;justify-content:flex-start}.dash-nav a{min-width:0;padding:8px 10px;font-size:.75rem;border-radius:6px}");
  html += F(".section-block{margin-top:18px}.section-head{margin-bottom:9px}.section-head h2{font-size:1.12rem}.section-note{display:none}.summary-grid{gap:10px}.summary-card{min-height:132px;padding:13px;grid-column:span 3}.summary-card.weather-card,.summary-card.next-card{grid-column:span 6}.card h3{margin-bottom:8px;padding-bottom:7px;font-size:.98rem}");
  html += F(".summary-value{font-size:clamp(1.25rem,2vw,1.7rem)}.summary-support,.hint,.sub{font-size:.82rem}.summary-meta.status-pills{gap:6px}.badge,.mini-chip,.chip{padding:6px 9px;font-size:.8rem}.summary-metric-grid{gap:8px}.metric-tile{min-height:72px;padding:9px 10px}.metric-v{font-size:.98rem}.metric-v.big-metric{font-size:1.22rem}");
  html += F(".weather-icon{width:46px;height:46px;flex-basis:46px}.wind-compass{width:54px;height:54px;flex-basis:54px}.meter{height:10px}.summary-note{margin-top:8px;padding:10px 11px;border-radius:7px;font-size:.84rem}.sched-top,.sched-body{padding:13px}.sched-grid{gap:10px}.sched-card{padding:12px}.zone-list{gap:8px}.zone-row{padding:10px 12px}.action-grid{gap:10px}.action-card{min-height:132px}");
  html += F("@media(max-width:1060px){.hero-grid{grid-template-columns:1fr}.summary-card{grid-column:span 6}.summary-card.weather-card,.summary-card.next-card{grid-column:span 12}}");
  html += F("@media(max-width:720px){.wrap{padding:0 10px;margin:10px auto}.nav .meta{width:100%;display:grid;grid-template-columns:repeat(2,minmax(0,1fr))}.pill,#themeBtn{width:100%;min-width:0}.hero-shell{padding:13px}.hero-mini-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.dash-nav{top:8px;overflow:auto;flex-wrap:nowrap}.summary-card{grid-column:span 12}.weather-card .summary-metric-grid,.next-card .summary-metric-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.card{padding:13px}}");
  html += F("@media(max-width:440px){.nav .meta{grid-template-columns:1fr}.hero-actions{grid-template-columns:1fr}.hero-mini-grid{grid-template-columns:1fr}.weather-card .summary-metric-grid,.next-card .summary-metric-grid{grid-template-columns:1fr}.next-card .metric-wide{grid-column:auto}.condition-wrap,.wind-compass-wrap{align-items:flex-start}.days-grid{grid-template-columns:repeat(2,minmax(0,1fr))}}");
  flush();
  html += F(".dash-head{margin:12px 0;padding:14px;border-top:3px solid var(--primary);box-shadow:var(--shadow)}.dash-head-top{display:flex;align-items:center;justify-content:space-between;gap:14px}.dash-head-copy{min-width:0}.dash-head-title{margin:2px 0 5px;font-size:1.35rem;line-height:1.15;letter-spacing:0}.dash-head-context{display:flex;align-items:center;gap:7px;flex-wrap:wrap;color:var(--muted);font-size:.82rem;font-weight:700}.dash-head-context a{color:var(--ink)}.dash-head-actions{display:flex;gap:7px;flex:0 0 auto}.dash-head-actions .btn{padding:8px 11px;min-width:0}.dash-metrics{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));margin-top:12px;border:1px solid var(--line);border-radius:7px;overflow:hidden}.dash-metric{min-width:0;padding:9px 11px;background:var(--panel);border-left:1px solid var(--line)}.dash-metric:first-child{border-left:0}.dash-metric-k{display:block;color:var(--muted);font-size:.65rem;font-weight:800;letter-spacing:.08em;text-transform:uppercase}.dash-metric-v{display:block;margin-top:3px;font-size:1.03rem;font-weight:800;line-height:1.12;overflow-wrap:anywhere}.dash-metric-sub{display:block;margin-top:3px;color:var(--muted);font-size:.74rem;line-height:1.2;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.dash-metric-weather{display:flex;align-items:center;gap:6px}.dash-head+.dash-nav{margin-top:0}@media(max-width:760px){.dash-head-top{align-items:flex-start;flex-direction:column}.dash-head-actions{width:100%}.dash-head-actions .btn{flex:1;text-align:center}.dash-metrics{grid-template-columns:repeat(2,minmax(0,1fr))}.dash-metric:nth-child(3){border-left:0;border-top:1px solid var(--line)}.dash-metric:nth-child(4){border-top:1px solid var(--line)}}@media(max-width:440px){.dash-head-title{font-size:1.2rem}.dash-head-context{align-items:flex-start;flex-direction:column;gap:3px}.dash-head-actions .btn{padding:8px 6px;font-size:.82rem}.dash-metric{padding:8px}.dash-metric-v{font-size:.94rem}}");
  // Controller summary visual hierarchy
  html += F(".summary-shell{position:relative;padding:10px;overflow:hidden;background:linear-gradient(135deg,rgba(37,99,235,.055),rgba(8,145,178,.025) 46%,rgba(96,165,250,.045));border-color:var(--line)}");
  html += F(".summary-shell::before{content:'';position:absolute;inset:0 0 auto;height:3px;background:linear-gradient(90deg,#1e40af,#2563eb 48%,#60a5fa)}");
  html += F(".summary-card{--summary-accent:#2563eb;position:relative;isolation:isolate;overflow:hidden;border:1px solid var(--line);border-left:4px solid var(--summary-accent);border-top-width:1px;background:var(--card);box-shadow:0 5px 16px rgba(15,23,42,.06)}");
  html += F(".summary-card.weather-card{--summary-accent:#0891b2}.summary-card.next-card{--summary-accent:#2563eb}");
  html += F(".summary-card::after{content:'';position:absolute;z-index:-1;right:-72px;top:-82px;width:180px;height:180px;border-radius:50%;background:var(--summary-accent);opacity:.045;pointer-events:none}");
  html += F(".summary-card h3{min-height:42px;margin:0;padding:0 0 10px;border-bottom:1px solid var(--line);gap:10px;letter-spacing:0}");
  html += F(".summary-heading-icon{display:grid;place-items:center;flex:0 0 34px;width:34px;height:34px;border-radius:9px;background:color-mix(in srgb,var(--summary-accent) 13%,var(--card));border:1px solid color-mix(in srgb,var(--summary-accent) 28%,var(--line));color:var(--summary-accent);font-size:1.05rem;line-height:1}");
  html += F(".summary-heading-copy{display:flex;flex-direction:column;gap:2px;line-height:1.08}.summary-heading-k{color:var(--muted);font-size:.62rem;font-weight:800;letter-spacing:.1em;text-transform:uppercase}");
  html += F(".summary-card .metric-tile{border-color:var(--line);background:color-mix(in srgb,var(--summary-accent) 3.5%,var(--panel));box-shadow:0 1px 2px rgba(15,23,42,.035);transition:border-color .15s ease,background .15s ease}");
  html += F(".summary-card .metric-tile:hover{border-color:color-mix(in srgb,var(--summary-accent) 38%,var(--line));background:color-mix(in srgb,var(--summary-accent) 6%,var(--panel))}");
  html += F(".summary-card .metric-k{color:color-mix(in srgb,var(--muted) 88%,var(--summary-accent))}.summary-card .big-metric{color:var(--summary-accent)}");
  flush();
  html += F(".summary-subhead{display:flex;align-items:center;gap:9px;margin-top:2px;color:var(--summary-accent)}.summary-subhead::after{content:'';height:1px;flex:1;background:var(--line)}");
  html += F(".status-pills .badge{position:relative;border-radius:8px;background:var(--panel);font-weight:650}.status-pills .badge::before{content:'';width:8px;height:8px;border-radius:50%;background:currentColor;box-shadow:0 0 0 3px color-mix(in srgb,currentColor 13%,transparent)}.status-pills .b-ok{color:#2563eb}.status-pills .b-warn{color:#b45309}.status-pills .b-bad{color:#b91c1c}.status-pills .badge b{color:var(--ink)}");
  html += F(".next-card .summary-note{margin-top:auto;border-left:3px solid var(--summary-accent);background:color-mix(in srgb,var(--summary-accent) 6%,var(--panel));line-height:1.55}.next-card .summary-note strong{color:var(--ink)}");
  html += F("html[data-theme='dark'] .summary-shell{background:linear-gradient(135deg,rgba(37,99,235,.09),rgba(8,145,178,.035) 46%,rgba(96,165,250,.06))}html[data-theme='dark'] .summary-card{background:var(--card)}");
  html += F("@media(max-width:720px){.summary-shell{padding:8px}.summary-card{border-left-width:3px}.summary-card h3{min-height:38px}.summary-heading-icon{width:31px;height:31px;flex-basis:31px}.summary-card .metric-tile:hover{border-color:var(--line)}}@media(max-width:440px){.summary-shell{padding:6px}.summary-card{padding:12px}.summary-card::after{display:none}.summary-meta.status-pills{grid-template-columns:repeat(2,minmax(0,1fr))}.status-pills .badge{padding:7px 6px;font-size:.76rem}}");
  // Collapsible section controls and final responsive layout polish
  html += F(".section-head{flex-wrap:wrap}.section-head-copy{min-width:0}.section-summary{margin:4px 0 0;color:var(--muted);font-size:.82rem;line-height:1.35}");
  html += F(".collapsible-section-head{padding:12px 14px;border:1px solid var(--line);border-left:3px solid var(--primary);border-radius:8px;background:var(--card);box-shadow:0 4px 14px rgba(15,23,42,.055);transition:border-color .16s ease,box-shadow .16s ease}");
  html += F(".collapsible-section-head.is-open{border-color:color-mix(in srgb,var(--primary) 34%,var(--line));box-shadow:0 7px 18px rgba(37,99,235,.09)}");
  html += F(".collapse-toggle{display:inline-flex;align-items:center;justify-content:center;gap:9px;min-width:146px;min-height:40px;padding:9px 12px;border-color:color-mix(in srgb,var(--primary) 24%,var(--line));background:color-mix(in srgb,var(--primary) 5%,var(--panel));white-space:nowrap}");
  html += F(".collapse-toggle::after{content:'\\203A';display:inline-grid;place-items:center;width:22px;height:22px;border-radius:5px;background:color-mix(in srgb,var(--primary) 10%,var(--panel));color:var(--primary);font-size:1.25rem;line-height:1;transition:transform .16s ease,background .16s ease}");
  html += F(".collapse-toggle[aria-expanded='true']::after{transform:rotate(90deg);background:color-mix(in srgb,var(--primary) 16%,var(--panel))}");
  html += F(".action-card .toolbar{display:grid;grid-template-columns:repeat(auto-fit,minmax(118px,1fr));width:100%;gap:8px}.action-card .toolbar .btn{text-align:center}");
  html += F("@media(max-width:720px){.collapsible-section-head{align-items:stretch;padding:11px 12px}.collapsible-section-head .collapse-toggle{width:100%;justify-content:space-between}.sched-tools{width:100%}.sched-tools .collapse-toggle{flex:1 1 160px}.zone-row-status{justify-content:space-between}.action-grid{grid-template-columns:1fr}.action-card{min-height:0}}");
  html += F("@media(max-width:480px){.section-head h2{font-size:1.08rem}.section-summary{font-size:.78rem}.zone-row-actions{display:grid;grid-template-columns:1fr 1fr;width:100%}.zone-row-actions .btn{width:100%}.action-card .toolbar{grid-template-columns:1fr}.dash-nav a{min-width:auto}}");
  // Keep the mobile utility bar focused on identity and the two controls people use.
  // Time, date, timezone, firmware and host remain visible on wider screens; the
  // dashboard card immediately below already presents the useful mobile context.
  html += F("@media(max-width:720px){.nav{padding:9px 10px}.nav .in{flex-wrap:nowrap;gap:10px}.brand{min-width:0;flex:1 1 auto}.brand-copy{min-width:0}.brand-title,.brand-sub{white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.nav .meta{display:flex;width:auto;flex:0 0 auto;gap:6px}.nav .meta>span.pill{display:none}#btn-master{width:auto;min-width:92px;min-height:38px;padding:7px 10px;white-space:nowrap}#themeBtn{position:relative;width:38px;min-width:38px;min-height:38px;padding:0;font-size:0}#themeBtn::before{content:'\\25D0';font-size:1.05rem;line-height:1}.dash-head{margin-top:10px}}");
  html += F("@media(max-width:360px){.brand-sub{display:none}.brand-title{font-size:.88rem}.dot{width:8px;height:8px}#btn-master{min-width:84px;padding-inline:8px;font-size:.78rem}}");
  // Preserve a two-column weather summary on phones. The previous narrow-screen
  // rule expanded every small reading into a full-width row.
  html += F("@media(max-width:480px){.weather-card{gap:10px}.weather-card .summary-metric-grid,.weather-card .summary-metric-grid.metric-pair{grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.weather-card .metric-tile{min-height:70px;padding:9px 10px;gap:6px}.weather-card .metric-k{font-size:.66rem;letter-spacing:.09em}.weather-card .metric-v{font-size:.96rem}.weather-card .metric-v.big-metric{font-size:1.2rem}.weather-card .condition-tile,.weather-card .wind-dir-tile{min-height:86px}.weather-card .condition-wrap,.weather-card .wind-compass-wrap{align-items:center;gap:7px}.weather-card .wind-compass{width:52px;height:52px;flex-basis:52px}.weather-card .wind-needle{height:18px}.weather-card .summary-subhead{margin-top:0}}");
  html += F("@media(max-width:360px){.weather-card .summary-metric-grid.metric-pair{grid-template-columns:1fr}.weather-card .condition-tile,.weather-card .wind-dir-tile{min-height:76px}}");
  // Keep schedule editing dense enough for a phone without shrinking tap targets.
  // Time, duration and weekday controls stay horizontal instead of becoming rows.
  html += F("@media(max-width:480px){.sched-body{padding:8px}.sched-grid{gap:8px}.sched-card{padding:10px}.sched-card h4{margin-bottom:7px}.sched-card .rowx{margin:7px 0}.sched-card .rowx>label{margin-bottom:2px;font-size:.72rem}.sched-card .in{padding:8px 9px}.sched-card .time-spin{width:auto;max-width:100%;grid-template-columns:auto 6px auto auto;gap:5px;padding:6px}.sched-card .time-part{grid-template-columns:auto 32px}.sched-card .time-val{width:54px}.sched-card .time-arrow{width:32px}.sched-card .time-ampm{grid-column:auto;width:54px}.sched-card .duration-spin{grid-template-columns:auto auto auto auto}.sched-card .duration-spin .time-val{width:58px}.sched-card .duration-unit{margin-right:2px;font-size:.7rem}.sched-card .days-grid{grid-template-columns:repeat(4,minmax(0,1fr));gap:6px}.sched-card .day span{min-height:40px;padding:8px 5px;font-size:.7rem}.sched-card .toolbar{margin-top:9px}.sched-card .toolbar .btn{min-height:40px;padding:9px 12px}}");
  html += F("@media(max-width:350px){.sched-card .time-spin{gap:3px;padding:5px}.sched-card .time-part{grid-template-columns:auto 29px}.sched-card .time-val{width:48px}.sched-card .time-arrow{width:29px}.sched-card .time-ampm{width:48px}.sched-card .duration-spin .time-val{width:52px}.sched-card .days-grid{grid-template-columns:repeat(3,minmax(0,1fr))}}");
  // Keep delays and next-water information scannable without a long stack of
  // full-width tiles. The status badge already carries the delay cause.
  html += F("@media(max-width:480px){.next-card{gap:9px}.next-card .summary-meta.status-pills{grid-template-columns:repeat(2,minmax(0,1fr));gap:6px}.next-card .status-pills .badge{min-width:0;padding:7px 8px;overflow:hidden}.next-card .status-pills .badge b{display:block;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}.next-card .summary-metric-grid{grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.next-card .metric-wide{grid-column:1/-1}.next-card .delay-cause-tile{display:none}.next-card .metric-tile{min-height:66px;padding:9px 10px;gap:5px}.next-card .metric-k{font-size:.66rem;letter-spacing:.08em}.next-card .metric-v{font-size:.96rem}.next-card .summary-support{font-size:.76rem}.next-card .summary-note{margin-top:0;padding:9px 10px;font-size:.78rem;line-height:1.4}.next-card .smart-extra{display:none}}");
  html += F("@media(max-width:350px){.next-card .status-pills .badge{font-size:.7rem;padding-inline:6px}}");
  html += F("</style></head><body>");
  flush();

  // --- Nav ---
  html += F("<div class='nav'><div class='in'>");
  html += F("<div class='brand'><span class='dot'></span><div class='brand-copy'><span class='brand-title'>ESP32 Irrigation</span><span class='brand-sub'>Weather-aware watering control</span></div></div>");
  html += F("<div class='meta'>");
  html += F("<span class='pill' id='clock'>"); html += timeStr; html += F("</span>");
  html += F("<span class='pill'>"); html += dateStr; html += F("</span>");
  html += F("<span class='pill'>"); html += ((ti && ti->tm_isdst>0) ? "DST" : "STD"); html += F("</span>");
  html += F("<span class='pill' title='Firmware version'>FW v"); html += kFirmwareVersion; html += F("</span>");
  html += F("<span class='pill' title='IP ");
  html += WiFi.localIP().toString();
  html += F("'>espirrigation.local</span>");
  html += F("<button id='btn-master' class='pill' style='cursor:pointer;border:none' aria-pressed='");
  html += (systemMasterEnabled ? "true" : "false");
  html += F("' title='Toggle master enable/disable'>Master: <b id='master-state'>");
  html += (systemMasterEnabled ? "On" : "Off");
  html += F("</b></button>");
  html += F("<button id='themeBtn' class='btn-ghost' title='Toggle theme'>Theme</button>");
  html += F("</div></div></div>");

  // --- Compact dashboard header ---
  html += F("<div class='wrap'><section class='card dash-head'><div class='dash-head-top'><div class='dash-head-copy'><div class='section-kicker'>1-16-Zone irrigation</div><h1 class='dash-head-title'>Irrigation Valve Controller</h1><div class='dash-head-context'><span id='heroDate'>");
  html += heroDateStr;
  html += F("</span><span>Local time <strong id='upChip'>");
  html += timeStr;
  html += F("</strong></span><a id='meteoLink' href='https://open-meteo.com/en/docs?latitude=-35.1076&longitude=138.5573' target='_blank' rel='noopener'>Location <strong id='cityName'>");
  html += cityName;
  html += F("</strong></a></div></div><div class='dash-head-actions'><a class='btn' href='/setup'>Setup</a><a class='btn btn-secondary' href='/events'>Events</a><a class='btn btn-secondary' href='/diagnostics'>Diagnostics</a></div></div><div class='dash-metrics'>");
  html += F("<div class='dash-metric'><span class='dash-metric-k'>System</span><span class='dash-metric-v' id='heroMasterState'>");
  html += heroSystemValue;
  html += F("</span><span class='dash-metric-sub' id='heroMasterSub'>");
  html += heroSystemSub;
  html += F("</span></div>");
  html += F("<div class='dash-metric'><span class='dash-metric-k'>Tank Reserve</span><span class='dash-metric-v' id='heroTankValue'>");
  html += String(tankPct);
  html += F("%</span><span class='dash-metric-sub' id='heroTankSub'>");
  html += sourceModeText();
  html += F("</span></div>");
  html += F("<div class='dash-metric'><span class='dash-metric-k'>Next Start</span><span class='dash-metric-v' id='heroNextValue'>");
  html += nextWaterLabel;
  html += F("</span><span class='dash-metric-sub' id='heroNextSub'>");
  html += nextWaterSub;
  html += F("</span></div>");
  html += F("<div class='dash-metric'><span class='dash-metric-k'>Weather</span><span class='dash-metric-v dash-metric-weather'><span id='heroWeatherValue'>");
  html += heroWeatherValue;
  html += F("</span><span id='heroWeatherTrend' style='font-weight:900;'>&rarr;</span></span><span class='dash-metric-sub' id='heroWeatherSub'>");
  html += cond;
  html += F("</span></div>");
  html += F("</div></section></div>");
  html += F("<div class='wrap'><div class='dash-nav glass'><a href='#summary-section'>Summary</a><a href='#schedules-section'>Schedules</a><a href='#zones-section'>Zones</a><a href='#controls-section'>Controls</a></div></div>");

  // --- Summary cards ---
  html += F("<div class='wrap section-block' id='summary-section'><div class='section-head collapsible-section-head'><div class='section-head-copy'><div class='section-kicker'>Overview</div><h2>Controller summary</h2><p class='section-summary'>Live weather, delays and the next scheduled watering.</p></div>");
  html += F("<button class='btn btn-secondary collapse-toggle' type='button' id='summaryToggle' aria-controls='summaryBody' aria-expanded='false'>Show Summary</button></div>");
  html += F("<div id='summaryBody' class='glass section summary-shell' style='display:none'><div class='grid summary-grid'>");

  html += F("<div class='card summary-card weather-card'><h3><span class='summary-heading-icon' aria-hidden='true'>&#9728;</span><span class='summary-heading-copy'><span class='summary-heading-k'>Live conditions</span>Current Weather</span></h3><div class='summary-metric-grid'>");
  html += F("<div class='metric-tile'><span class='metric-k'>Temperature</span><div class='metric-v big-metric'><span id='tempChip'>");
  html += (isnan(temp) ? String("--") : String(temp,1)+" "+temperatureUnitChar());
  html += F("</span> <span id='tempTrend' style='font-weight:900;'>&rarr;</span></div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Feels Like</span><div class='metric-v' id='feelsChip'>");
  html += (isnan(feels) ? String("--") : String(feels,1)+" "+temperatureUnitChar());
  html += F("</div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Humidity</span><div class='metric-v' id='humChip'>");
  html += (isnan(hum) ? String("--") : String((int)hum)+" %");
  html += F("</div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Wind</span><div class='metric-v' id='windChip'>");
  html += (isnan(ws) ? String("--") : String(ws,1)+" m/s");
  html += F("</div></div>");
  html += F("</div><div class='summary-metric-grid metric-pair'>");
  html += F("<div class='metric-tile condition-tile'><span class='metric-k'>Condition</span><div class='condition-wrap'><div id='condIcon' class='weather-icon wi-unknown' aria-hidden='true'><span class='wi-main'></span><span class='wi-extra'></span></div><div class='metric-v' id='cond'>");
  html += cond.length() ? cond : String("--");
  html += F("</div></div></div>");
  html += F("<div class='metric-tile wind-dir-tile'><span class='metric-k'>Wind Direction</span><div class='wind-compass-wrap'><div id='windCompass' class='wind-compass ");
  html += isfinite(windDirDeg) ? "" : "is-empty";
  html += F("' style='--dir:");
  html += isfinite(windDirDeg) ? String(windDirDeg, 0) : String(0);
  html += F("deg'><span class='cardinal n'>N</span><span class='cardinal e'>E</span><span class='cardinal s'>S</span><span class='cardinal w'>W</span><span class='wind-needle'></span><span class='wind-center'></span></div><div class='metric-v wind-dir-text' id='windDirChip'>");
  html += windDir;
  html += F("</div></div></div></div>");
  html += F("<div class='summary-subhead'>Today</div><div class='summary-metric-grid'>");
  html += F("<div class='metric-tile'><span class='metric-k'>Low / High</span><div class='metric-split'>");
  html += F("<div class='metric-split-item'><div class='metric-v'><span id='tmin'>---</span><span class='metric-unit'>"); html += temperatureUnitChar(); html += F("</span></div></div>");
  html += F("<div class='metric-split-item'><div class='metric-v'><span id='tmax'>---</span><span class='metric-unit'>"); html += temperatureUnitChar(); html += F("</span></div></div>");
  html += F("</div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Pressure</span><div class='metric-v pressure-value'><span><span id='press'>--</span><span class='metric-unit'>hPa</span></span><span id='pressTrend' class='pressure-trend' aria-label='Pressure trend'></span></div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Sunrise</span><div class='metric-v' id='sunr'>--:--</div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Sunset</span><div class='metric-v' id='suns'>--:--</div></div></div></div>");

  // Delays + Next Water
  html += F("<div class='card summary-card next-card'><h3><span class='summary-heading-icon' aria-hidden='true'>&#9201;</span><span class='summary-heading-copy'><span class='summary-heading-k'>Automation outlook</span>Delays & Next Water</span></h3><div class='summary-meta status-pills'>");
  html += F("<div id='rainBadge' class='badge "); html += (rainDelayBlockingNow ? "b-bad" : "b-ok"); html += F("'>Rain: <b>");
  html += (rainDelayBlockingNow ? causeText : String("Off")); html += F("</b></div>");
  html += F("<div id='windBadge' class='badge "); html += (windActive ? "b-warn" : "b-ok"); html += F("'>Wind: <b>");
  html += (windActive?"Active":"Off"); html += F("</b></div></div>");
  html += F("<div class='summary-metric-grid'>");
  html += F("<div class='metric-tile metric-wide delay-cause-tile'><span class='metric-k'>Delay Cause</span><div class='metric-v' id='rainCauseBadge'>"); html += causeText; html += F("</div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Rain 1h</span><div class='metric-v'><span id='acc1h'>--</span><span class='metric-unit'>mm</span></div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Rain 24h</span><div class='metric-v'><span id='acc24'>--</span><span class='metric-unit'>mm</span></div></div>");
  html += F("<div class='metric-tile metric-wide'><span class='metric-k'>Zone & Start</span><div class='metric-v' id='nwTime'>");
  html += nextWaterLabel;
  html += F("</div><div class='summary-support'>Zone <span id='nwZone'>--</span></div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>ETA</span><div class='metric-v' id='nwETA'>--</div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Duration</span><div class='metric-v' id='nwDur'>--</div></div>");
  {
    const float smartFactorNow = smartWateringFactor();
    const int smartPctNow = smartWateringEnabled ? (int)lroundf(smartFactorNow * 100.0f) : 100;
    const int moisturePctNow = moisturePercent();
    html += F("</div><div class='summary-note");
    if (!smartWateringEnabled) html += F(" is-disabled");
    html += F("' id='smartNote'><strong>Smart Watering Status:</strong> <span id='smartStatusText'>");
    html += smartWateringEnabled ? F("Enabled") : F("Disabled");
    html += F("</span><br><strong>Current Runtime Factor:</strong> <span id='smartFactorText'>");
    html += String(smartPctNow);
    html += F("%</span><span id='smartSkipText'>");
    if (smartWateringEnabled && smartFactorNow <= 0.0f) {
      html += F(" - starts will be skipped by the current smart rules");
    }
    html += F("</span><br><strong>Ground Moisture:</strong> <span id='smartMoistureText'>");
    if (!moistureProbeEnabled) {
      html += F("Disabled");
    } else if (moisturePctNow < 0) {
      html += F("No valid reading");
    } else {
      html += String(moisturePctNow);
      html += moistureUseMeteo ? F("% water volume (Open-Meteo 0-1 cm), skip at ") : F("% wet, skip at ");
      html += String(moistureSkipThresholdPct);
      html += F("%");
    }
    html += F("</span><span class='smart-extra'><br><strong>Applies To:</strong> Scheduled and manual runs.</span></div></div>");
  }
  html += F("</div></div></div>"); // end grid / summary shell / wrap
  flush();

  // ---------- Schedules (collapsible) ----------
  static const char* DLBL[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  auto pad2 = [](int v) -> String {
    if (v < 0) v = 0;
    if (v < 10) {
      char b[3];
      snprintf(b, sizeof(b), "0%d", v);
      return String(b);
    }
    return String(v);
  };
  html += F("<div class='wrap section-block' id='schedules-section'><div class='section-head'><div><div class='section-kicker'>Scheduling</div><h2>Zone schedules</h2></div>");
  html += F("<p class='section-note'></p></div>");
  html += F("<div class='card sched sched-shell'><div class='sched-top'><div class='sched-top-copy'><div class='section-kicker'>Planner</div><h3>Weekly schedule editor</h3><p>Set watering days, start times and run duration for each zone.</p></div>");
  html += F("<div class='sched-tools'><button class='btn btn-secondary collapse-toggle' type='button' id='schedToggle' aria-controls='schedBody' aria-expanded='false'>Show Schedules</button><button class='btn' id='btn-save-all' title='Save all zone schedules'>Save All</button></div></div>");
  html += F("<div id='schedBody' class='sched-ctr sched-body' style='display:none'>");
  html += F("<div class='sched-grid'>");

  for (int z=0; z<(int)zonesCount; ++z) {
    html += F("<div class='sched-card'><h4><span class='sched-badge'>Z");
    html += String(z + 1);
    html += F("</span> ");
    html += zoneNames[z];
    html += F("</h4><form method='POST' action='/submit'>");
    html += F("<input type='hidden' name='onlyZone' value='"); html += String(z); html += F("'>");

    html += F("<div class='rowx'><label>Wind Delay</label><div class='field'><label class='toggle-inline'><input type='checkbox' name='zoneWindDelay");
    html += String(z); html += F("' "); html += (zoneWindDelayEnabled[z] ? "checked" : "");
    html += F("> Enable</label><small>Uses the wind switch and threshold in Setup.</small></div></div>");
    // Name
    html += F("<div class='rowx'><label>Name</label><div class='field'>");
    html += F("<input class='in' type='text' name='zoneName"); html += String(z);
    html += F("' value='"); html += zoneNames[z]; html += F("' maxlength='32' style='flex:1;min-width:200px'>");
    html += F("</div></div>");

    // Start 1
    html += F("<div class='rowx'><label>Start Time 1 -</label><div class='field inline'>");
    html += F("<div class='time-spin' data-time-spin>");
    html += F("<input type='hidden' name='startHour"); html += String(z);
    html += F("' value='"); html += pad2(startHour[z]); html += F("'>");
    html += F("<input type='hidden' name='startMin"); html += String(z);
    html += F("' value='"); html += pad2(startMin[z]); html += F("'>");
    html += F("<div class='time-part'><button class='time-btn time-val' type='button' data-role='hour' title='Hour'>12</button><button class='time-btn time-arrow' type='button' data-part='hour' data-dir='1' title='Hour up'>&#9650;</button><button class='time-btn time-arrow' type='button' data-part='hour' data-dir='-1' title='Hour down'>&#9660;</button></div>");
    html += F("<span class='time-colon'>:</span>");
    html += F("<div class='time-part'><button class='time-btn time-val' type='button' data-role='min' title='Minute'>00</button><button class='time-btn time-arrow' type='button' data-part='min' data-dir='1' title='Minute up'>&#9650;</button><button class='time-btn time-arrow' type='button' data-part='min' data-dir='-1' title='Minute down'>&#9660;</button></div>");
    html += F("<button class='time-btn time-ampm' type='button' data-role='ampm' title='Toggle AM/PM'>AM</button></div>");
    html += F("</div></div>");

    // Start 2 (hide inputs unless enabled)
    html += F("<div class='rowx'><label>Start Time 2 -</label><div class='field inline'>");
    html += F("<span id='start2fields"); html += String(z); html += F("' style='display:");
    html += (enableStartTime2[z] ? "inline-flex" : "none");
    html += F(";align-items:center;gap:8px'>");
    html += F("<div class='time-spin' data-time-spin>");
    html += F("<input type='hidden' name='startHour2"); html += String(z);
    html += F("' value='"); html += pad2(startHour2[z]); html += F("'>");
    html += F("<input type='hidden' name='startMin2"); html += String(z);
    html += F("' value='"); html += pad2(startMin2[z]); html += F("'>");
    html += F("<div class='time-part'><button class='time-btn time-val' type='button' data-role='hour' title='Hour'>12</button><button class='time-btn time-arrow' type='button' data-part='hour' data-dir='1' title='Hour up'>&#9650;</button><button class='time-btn time-arrow' type='button' data-part='hour' data-dir='-1' title='Hour down'>&#9660;</button></div>");
    html += F("<span class='time-colon'>:</span>");
    html += F("<div class='time-part'><button class='time-btn time-val' type='button' data-role='min' title='Minute'>00</button><button class='time-btn time-arrow' type='button' data-part='min' data-dir='1' title='Minute up'>&#9650;</button><button class='time-btn time-arrow' type='button' data-part='min' data-dir='-1' title='Minute down'>&#9660;</button></div>");
    html += F("<button class='time-btn time-ampm' type='button' data-role='ampm' title='Toggle AM/PM'>AM</button></div>");
    html += F("</span>");
    html += F("<label class='toggle-inline'><input type='checkbox' name='enableStartTime2");
    html += String(z); html += F("' "); html += (enableStartTime2[z] ? "checked" : "");
    html += F("> Enable</label></div></div>");

    // Duration
    html += F("<div class='rowx'><label>Run Time 1 -</label><div class='field inline'>");
    html += F("<div class='time-spin duration-spin' data-duration-spin>");
    html += F("<input type='hidden' name='durationMin"); html += String(z);
    html += F("' value='"); html += String(durationMin[z]); html += F("'>");
    html += F("<input type='hidden' name='durationSec"); html += String(z);
    html += F("' value='"); html += pad2(durationSec[z]); html += F("'>");
    html += F("<div class='time-part'><button class='time-btn time-val' type='button' data-role='dmin' title='Minutes'>0</button><button class='time-btn time-arrow' type='button' data-part='min' data-dir='1' title='Minutes up'>&#9650;</button><button class='time-btn time-arrow' type='button' data-part='min' data-dir='-1' title='Minutes down'>&#9660;</button></div><span class='duration-unit'>min</span>");
    html += F("<div class='time-part'><button class='time-btn time-val' type='button' data-role='dsec' title='Seconds'>00</button><button class='time-btn time-arrow' type='button' data-part='sec' data-dir='1' title='Seconds up'>&#9650;</button><button class='time-btn time-arrow' type='button' data-part='sec' data-dir='-1' title='Seconds down'>&#9660;</button></div><span class='duration-unit'>sec</span></div>");
    html += F("</div></div>");

    // Duration 2 (used when Start 2 fires)
    html += F("<div class='rowx dur2row' id='dur2row"); html += String(z); html += F("' style='display:");
    html += (enableStartTime2[z] ? "grid" : "none");
    html += F("'><label>Run Time 2 -</label><div class='field inline'>");
    html += F("<div class='time-spin duration-spin' data-duration-spin>");
    html += F("<input type='hidden' name='duration2Min"); html += String(z);
    html += F("' value='"); html += String(duration2Min[z]); html += F("'>");
    html += F("<input type='hidden' name='duration2Sec"); html += String(z);
    html += F("' value='"); html += pad2(duration2Sec[z]); html += F("'>");
    html += F("<div class='time-part'><button class='time-btn time-val' type='button' data-role='dmin' title='Minutes'>0</button><button class='time-btn time-arrow' type='button' data-part='min' data-dir='1' title='Minutes up'>&#9650;</button><button class='time-btn time-arrow' type='button' data-part='min' data-dir='-1' title='Minutes down'>&#9660;</button></div><span class='duration-unit'>min</span>");
    html += F("<div class='time-part'><button class='time-btn time-val' type='button' data-role='dsec' title='Seconds'>00</button><button class='time-btn time-arrow' type='button' data-part='sec' data-dir='1' title='Seconds up'>&#9650;</button><button class='time-btn time-arrow' type='button' data-part='sec' data-dir='-1' title='Seconds down'>&#9660;</button></div><span class='duration-unit'>sec</span></div>");
    html += F("</div></div>");

    // Days
    html += F("<div class='rowx'><label>Days</label><div class='days-grid'>");
    for (int d=0; d<7; ++d) {
      html += F("<label class='day'><input type='checkbox' name='day"); html += String(z); html += "_"; html += String(d);
      html += F("' "); html += (days[z][d] ? "checked" : ""); html += F("><span>"); html += DLBL[d]; html += F("</span></label>");
    }
    html += F("</div></div>");

    // Actions
    html += F("<div class='toolbar' style='justify-content:flex-end'><button class='btn' type='submit'>Save Zone</button></div>");
    html += F("</form></div>");
    flush();
  }

  html += F("</div>"); // .sched-grid
  html += F("<div class='sched-tools' style='justify-content:flex-end;margin-top:14px'><button class='btn' id='btn-save-all-bottom' type='button' title='Save all zone schedules'>Save All</button></div>");
  html += F("</div></div></div>"); // #schedBody, .card.sched, .wrap
  flush();

  // --- Live Zones ---
  html += F("<div class='wrap section-block' id='zones-section'><div class='section-head'><div><div class='section-kicker'>Live Control</div><h2>Zone overview</h2></div>");
  html += F("<p class='section-note'></p></div>");
  html += F("<div class='card zones-shell'>");

  html += F("<div class='zone-list-head'><h3>Quick Zone Control</h3><div class='zone-list-note'>Compact live rows with manual on/off control.</div></div>");
  html += F("<div class='zone-list'>");

  for (int z = 0; z < (int)zonesCount; z++) {
    unsigned long total = zoneRunTotalSec[z];
    if (total == 0) total = durationForSlot(z, 1);
    unsigned long rem = total;
    if (zoneActive[z]) {
      unsigned long elapsed = (millis() - zoneStartMs[z]) / 1000UL;
      rem = (elapsed < total ? total - elapsed : 0);
    }

    char valueBuf[16];
    fmtMMSS(valueBuf, sizeof(valueBuf), zoneActive[z] ? rem : total);

    html += F("<div class='zone-row ");
    html += (zoneActive[z] ? "is-active" : "");
    html += F("' id='zone-");
    html += String(z);
    html += F("-row'>");

    html += F("<div class='zone-row-main'><span class='zone-index'>Z");
    html += String(z + 1);
    html += F("</span><span id='zone-");
    html += String(z);
    html += F("-dot' class='zone-dot ");
    html += (zoneActive[z] ? "on" : "");
    html += F("'></span><div class='zone-row-copy'><div class='zone-row-name'>");
    html += zoneNames[z];
    html += F("</div><div class='zone-row-sub' id='zone-");
    html += String(z);
    html += F("-meta'>");
    html += (zoneActive[z] ? "Remaining" : "Run time");
    html += F("</div></div></div>");

    html += F("<div class='zone-row-status'><div id='zone-");
    html += String(z);
    html += F("-state' class='badge ");
    html += (zoneActive[z] ? "b-ok" : "");
    html += F("'>");
    html += (zoneActive[z] ? "Running" : "Ready");
    html += F("</div><div class='zone-row-rem' id='zone-");
    html += String(z);
    html += F("-rem'>");
    html += valueBuf;
    html += F("</div></div>");

    html += F("<div class='zone-row-actions'>");
    html += F("<button type='button' class='btn' id='zone-");
    html += String(z);
    html += F("-on' onclick='toggleZone(");
    html += String(z);
    html += F(",1)'");
    if (zoneActive[z]) html += F(" disabled");
    html += F(">On</button>");
    html += F("<button type='button' class='btn btn-danger' id='zone-");
    html += String(z);
    html += F("-off' onclick='toggleZone(");
    html += String(z);
    html += F(",0)'");
    if (!zoneActive[z]) html += F(" disabled");
    html += F(">Off</button>");
    html += F("</div>");

    html += F("</div>");
    flush();
  }

  html += F("</div></div></div>");
  flush();

  // --- System Controls ---
  html += F("<div class='wrap section-block controls-shell' id='controls-section'><div class='section-head'><div><div class='section-kicker'>Actions</div><h2>Controller controls</h2></div>");
  html += F("<p class='section-note'></p></div>");
  html += F("<div class='action-grid'>");
  html += F("<div class='card action-card'><div class='action-copy'><h3>Quick Actions</h3><p>Clear active delays, resume automation, or restart the controller.</p></div><div class='toolbar'><button class='btn btn-secondary' id='btn-clear-delays'>Clear Delays</button><button class='btn btn-secondary' id='btn-resume'>Resume</button><button class='btn btn-danger' id='rebootBtn'>Reboot</button></div></div>");
  html += F("<div class='card action-card'><div class='action-copy'><h3>Deeper Setup</h3><p>Jump into configuration or event history when you need wiring, forecast, timezone.</p></div><div class='toolbar'><a class='btn' href='/setup'>Open Setup</a><a class='btn btn-secondary' href='/events'>View Events</a></div></div>");
  html += F("</div></div>");
  flush();

  // --- JS ---
  html += F("<script>");
  html += F("function pad(n){return n<10?'0'+n:n;}");
  html += F("let _devEpoch=null; let _tickTimer=null; let _lastTemp=null; let _lastTempTrend='\\u2192'; let _wifiSeeded=false;");
  html += F("function startDeviceClock(seedSec){_devEpoch=seedSec;if(_tickTimer)clearInterval(_tickTimer);");
  html += F("const draw=()=>{if(_devEpoch==null)return; const d=new Date(_devEpoch*1000);");
  html += F("const h=pad(d.getHours()),m=pad(d.getMinutes()),s=pad(d.getSeconds());");
  html += F("const el=document.getElementById('clock'); if(el) el.textContent=h+':'+m+':'+s; _devEpoch++;};");
  html += F("draw(); _tickTimer=setInterval(draw,1000);} ");
  html += F("function fmtClock12(epoch, offsetMin){");
  html += F(" if(typeof epoch!=='number' || epoch<=0) return '--:--';");
  html += F(" const t=new Date((epoch + (offsetMin||0)*60)*1000);");
  html += F(" let h=t.getUTCHours(); const m=t.getUTCMinutes(); const s=t.getUTCSeconds();");
  html += F(" const am=(h>=12)?'PM':'AM'; h=h%12; if(h===0) h=12;");
  html += F(" return pad(h)+':'+pad(m)+':'+pad(s)+' '+am;");
  html += F("} ");
  html += F("function fmtDeviceDate(epoch,offsetMin){if(typeof epoch!=='number'||epoch<=0)return'Date unavailable';const t=new Date((epoch+(offsetMin||0)*60)*1000);const days=['Sunday','Monday','Tuesday','Wednesday','Thursday','Friday','Saturday'];const months=['January','February','March','April','May','June','July','August','September','October','November','December'];return days[t.getUTCDay()]+', '+pad(t.getUTCDate())+' '+months[t.getUTCMonth()]+' '+t.getUTCFullYear();}");
  html += F("let _busy=false; async function postJson(url,payload){const body=payload?JSON.stringify(payload):\"{}\";");
  html += F("return fetch(url,{method:'POST',headers:{'Content-Type':'application/json','Cache-Control':'no-cache'},body});}");
  html += F("async function postForm(url, body){const opts={method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'}};");
  html += F("if(body)opts.body=body; return fetch(url,opts);} ");
  html += F("function addRipple(e){const t=e.currentTarget; if(t.disabled) return; const rect=t.getBoundingClientRect();");
  html += F("const size=Math.max(rect.width,rect.height); const x=(e.clientX|| (rect.left+rect.width/2)) - rect.left - size/2;");
  html += F("const y=(e.clientY|| (rect.top+rect.height/2)) - rect.top - size/2;");
  html += F("const r=document.createElement('span'); r.className='ripple'; r.style.width=size+'px'; r.style.height=size+'px';");
  html += F("r.style.left=x+'px'; r.style.top=y+'px'; const old=t.querySelector('.ripple'); if(old) old.remove(); t.appendChild(r);");
  html += F("setTimeout(()=>{r.remove();},520);}");
  html += F("document.querySelectorAll('.btn,.btn-ghost,.pill').forEach(el=>{el.addEventListener('pointerdown',addRipple);});");
  html += F("async function toggleZone(z,on){if(_busy)return;_busy=true;try{await postJson('/valve/'+(on?'on/':'off/')+z,{t:Date.now()});setTimeout(refreshStatus,200);}catch(e){console.error(e);}finally{_busy=false;}}");

  html += F("const btnReboot=document.getElementById('rebootBtn');");
  html += F("if(btnReboot){btnReboot.addEventListener('click',async()=>{if(confirm('Reboot controller now?')){try{await postJson('/reboot',{t:Date.now()});}catch(e){}}});} ");
  html += F("document.getElementById('btn-clear-delays')?.addEventListener('click',async()=>{await postForm('/clear_delays','a=1');setTimeout(refreshStatus,200);});");
  html += F("document.getElementById('btn-pause-24')?.addEventListener('click',async()=>{await postForm('/pause','sec=86400');setTimeout(refreshStatus,200);});");
  html += F("document.getElementById('btn-pause-7d')?.addEventListener('click',async()=>{await postForm('/pause','sec='+(7*86400));setTimeout(refreshStatus,200);});");
  html += F("document.getElementById('btn-resume')?.addEventListener('click',async()=>{await postForm('/resume','x=1');setTimeout(refreshStatus,200);});");

  // Master pill
  html += F("const btnMaster=document.getElementById('btn-master');");
  html += F("if(btnMaster){btnMaster.addEventListener('click',async()=>{");
  html += F("  const cur=(btnMaster.getAttribute('aria-pressed')==='true'); const turnOn=!cur;");
  html += F("  try{await fetch('/master',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:turnOn?'on=1':''});");
  html += F("      btnMaster.setAttribute('aria-pressed',turnOn?'true':'false');");
  html += F("      const st=document.getElementById('master-state'); if(st) st.textContent=turnOn?'On':'Off';");
  html += F("  }catch(e){console.error(e);} ");
  html += F("});}");

  // Theme
  html += F("function applyTheme(t){document.documentElement.setAttribute('data-theme',t==='dark'?'dark':'light');}");
  html += F("(function(){let saved=localStorage.getItem('theme');");
  html += F("if(saved!=='light'&&saved!=='dark'){saved=(window.matchMedia&&window.matchMedia('(prefers-color-scheme: dark)').matches)?'dark':'light';");
  html += F("localStorage.setItem('theme',saved);}applyTheme(saved);})();");
  html += F("document.getElementById('themeBtn')?.addEventListener('click',()=>{const cur=(document.documentElement.getAttribute('data-theme')==='dark')?'dark':'light';");
  html += F("const nxt=(cur==='dark')?'light':'dark';applyTheme(nxt);localStorage.setItem('theme',nxt);});");

  html += F("function toLocalHHMM(epoch){if(!epoch||epoch===0)return'--:--'; const d=new Date(epoch*1000); return pad(d.getHours())+':'+pad(d.getMinutes());}");
  html += F("function nextWaterDayLabel(epoch){if(!epoch||epoch===0)return'--'; const d=new Date(epoch*1000); const now=new Date();");
  html += F("const start=new Date(now.getFullYear(),now.getMonth(),now.getDate()); const target=new Date(d.getFullYear(),d.getMonth(),d.getDate());");
  html += F("const diff=Math.round((target-start)/86400000); if(diff===0) return 'Today'; if(diff===1) return 'Tomorrow';");
  html += F("return d.toLocaleDateString(undefined,{weekday:'short', day:'2-digit', month:'short'});}"); 
  html += F("function nextWaterStartLabel(epoch){if(!epoch||epoch===0)return'--'; return nextWaterDayLabel(epoch)+' '+toLocalHHMM(epoch);}");
  html += F("function fmtDurCompact(s){ if(!(s>0)) return '--'; const m=Math.floor(s/60), sec=s%60; return pad(m)+'m '+pad(sec)+'s'; }");
  html += F("function setWindCompass(deg){const c=document.getElementById('windCompass');if(!c)return;const n=Number(deg);const ok=Number.isFinite(n);c.classList.toggle('is-empty',!ok);if(ok)c.style.setProperty('--dir',(((n%360)+360)%360)+'deg');}");
  html += F("function weatherIconClass(text){const t=String(text||'').toLowerCase();if(!t||t==='--')return'wi-unknown';if(t.includes('thunder'))return'wi-storm';if(t.includes('freez')||t.includes('ice')||t.includes('rime'))return'wi-freezing';if(t.includes('snow'))return'wi-snow';if(t.includes('shower'))return'wi-showers';if(t.includes('drizzle'))return'wi-drizzle';if(t.includes('rain'))return'wi-rain';if(t.includes('fog')||t.includes('mist')||t.includes('haze'))return'wi-fog';if(t.includes('overcast'))return'wi-overcast';if(t.includes('partly')||t.includes('mainly'))return'wi-partly';if(t.includes('cloud'))return'wi-cloud';if(t.includes('clear')||t.includes('sun'))return'wi-clear';return'wi-unknown';}");
  html += F("function setWeatherIcon(text){const el=document.getElementById('condIcon');if(!el)return;el.className='weather-icon '+weatherIconClass(text);el.title=text||'Unknown';}");
  html += F("function wifiQuality(v,c){if(c===false)return'Disconnected';if(typeof v!=='number')return'Connected';if(v>=-50)return'Excellent';if(v>=-60)return'Strong';if(v>=-67)return'Good';if(v>=-75)return'Fair';if(v>=-82)return'Weak';return'Very weak';}");
  html += F("function updateWifiSummary(st){const rssi=document.getElementById('rssiChip');const rq=document.getElementById('rssiQuality');const connected=(typeof st.wifiConnected==='boolean')?st.wifiConnected:undefined;const v=(typeof st.rssi==='number')?st.rssi:null;if(rssi)rssi.textContent=(v===null?'--':v)+' dBm';if(rq)rq.textContent=wifiQuality(v,connected);}");
  html += F("async function refreshWifiStatus(){try{const r=await fetch('/status',{cache:'no-store'});updateWifiSummary(await r.json());}catch(e){}}");
  html += F("async function refreshStatus(){try{const r=await fetch('/status',{cache:'no-store'});const st=await r.json();");
  html += F("if(!_wifiSeeded){updateWifiSummary(st);_wifiSeeded=true;}");
  html += F("if(typeof st.deviceEpoch==='number' && st.deviceEpoch>0 && _devEpoch===null){ startDeviceClock(st.deviceEpoch); }");
  html += F("const rb=document.getElementById('rainBadge');const wb=document.getElementById('windBadge');");
  html += F("if(rb){const rc=st.rainDelayCause||'Active';rb.className='badge '+(st.rainDelayActive?'b-bad':'b-ok');rb.innerHTML='Rain: <b>'+(st.rainDelayActive?rc:'Off')+'</b>';}");
  html += F("if(wb){wb.className='badge '+(st.windDelayActive?'b-warn':'b-ok');wb.innerHTML='Wind: <b>'+(st.windDelayActive?'Active':'Off')+'</b>';}");
  html += F("const cause=document.getElementById('rainCauseBadge'); if(cause) cause.textContent=st.rainDelayCause||'Off';");
  html += F("const pct=st.tankPct||0; const tf=document.getElementById('tankFill'); const tl=document.getElementById('tankPctLabel');");
  html += F("if(tf) tf.style.width=Math.max(0,Math.min(100,pct))+'%'; if(tl) tl.textContent=pct+'%';");
  html += F("const src=document.getElementById('srcChip'); if(src) src.textContent=st.sourceMode||'';");
  html += F("const heroTank=document.getElementById('heroTankValue'); const heroTankSub=document.getElementById('heroTankSub');");
  html += F("if(heroTank) heroTank.textContent=pct+'%'; if(heroTankSub) heroTankSub.textContent=st.sourceMode||'';");
  html += F("const up=document.getElementById('upChip'); if(up) up.textContent=fmtClock12(st.deviceEpoch, st.utcOffsetMin);");
  html += F("const heroDate=document.getElementById('heroDate');if(heroDate)heroDate.textContent=fmtDeviceDate(st.deviceEpoch,st.utcOffsetMin);");
  // Location chip + Open-Meteo link
  html += F("const cityEl=document.getElementById('cityName'); const cityLink=document.getElementById('meteoLink');");
  html += F("if(typeof st.cityName==='string' && st.cityName.length){");
  html += F("  if(cityEl) cityEl.textContent=st.cityName;");
  html += F("}");
  html += F("if(cityLink){");
  html += F("  const lat=Number.isFinite(st.lat)?st.lat:null;");
  html += F("  const lon=Number.isFinite(st.lon)?st.lon:null;");
  html += F("  if(lat!==null && lon!==null){ cityLink.href='https://open-meteo.com/en/docs?latitude='+lat+'&longitude='+lon; }");
  html += F("  else { cityLink.href='https://open-meteo.com/en/docs?latitude=-35.1076&longitude=138.5573'; }");
  html += F("}");




  // 1h & 24h
  html += F("var acc1h=document.getElementById('acc1h');");
  html += F("if(acc1h){var v=(typeof st.rain1hNow==='number')?st.rain1hNow:NaN;acc1h.textContent=isNaN(v)?'--':v.toFixed(1);}");
  html += F("const acc24=document.getElementById('acc24'); if(acc24){ const v=(typeof st.rain24hActual==='number')?st.rain24hActual:(typeof st.rain24h==='number'?st.rain24h:NaN); acc24.textContent=isNaN(v)?'--':v.toFixed(1);} ");

  html += F("let activeCount=0; if(Array.isArray(st.zones)){ st.zones.forEach((z,idx)=>{");
  html += F("if(z.active){ activeCount++; }");
  html += F("const stateEl=document.getElementById('zone-'+idx+'-state'); const remEl=document.getElementById('zone-'+idx+'-rem'); const dotEl=document.getElementById('zone-'+idx+'-dot'); const rowEl=document.getElementById('zone-'+idx+'-row'); const metaEl=document.getElementById('zone-'+idx+'-meta');");
  html += F("if(stateEl){stateEl.className='badge '+(z.active?'b-ok':'');stateEl.textContent=z.active?'Running':'Ready';}");
  html += F("if(dotEl){dotEl.className='zone-dot'+(z.active?' on':'');} if(rowEl){rowEl.classList.toggle('is-active',!!z.active);}");
  html += F("if(metaEl) metaEl.textContent=z.active?'Remaining':'Run time';");
  html += F("if(remEl){ remEl.textContent=z.active?fmtDurCompact(z.remaining||0):fmtDurCompact(z.totalSec||0); }");
  html += F("const onBtn=document.getElementById('zone-'+idx+'-on'); const offBtn=document.getElementById('zone-'+idx+'-off');");
  html += F("if(onBtn) onBtn.disabled=!!z.active; if(offBtn) offBtn.disabled=!z.active;");
  html += F("}); }");


  // Weather stats
  html += F("const tempUnit=(st.tempUnit==='F')?'F':'C'; const showTemp=v=>(tempUnit==='F'?(v*9/5+32):v);");
  html += F("const tmin=document.getElementById('tmin'); const tmax=document.getElementById('tmax'); const sunr=document.getElementById('sunr'); const suns=document.getElementById('suns'); const press=document.getElementById('press'); const pressTrend=document.getElementById('pressTrend');");
  html += F("if(tmin) tmin.textContent=showTemp(st.tmin??0).toFixed(0);");
  html += F("if(tmax) tmax.textContent=showTemp(st.tmax??0).toFixed(0);");
  html += F("if(sunr) sunr.textContent = st.sunriseLocal || '--:--';");
  html += F("if(suns) suns.textContent = st.sunsetLocal  || '--:--';");
  html += F("if(press){ const p=st.pressure; press.textContent=(typeof p==='number' && p>0)?p.toFixed(0):'--'; }");
  html += F("if(pressTrend){ const pt=(typeof st.pressureTrend==='number')?st.pressureTrend:0; pressTrend.textContent=pt>0?'\\u2191':(pt<0?'\\u2193':''); pressTrend.className='pressure-trend '+(pt>0?'up':(pt<0?'down':'')); pressTrend.title=pt>0?'Pressure rising':(pt<0?'Pressure falling':''); }");
  html += F("const tempEl=document.getElementById('tempChip'); const trendEl=document.getElementById('tempTrend');");
  html += F("const heroWeather=document.getElementById('heroWeatherValue'); const heroWeatherTrend=document.getElementById('heroWeatherTrend'); const heroWeatherSub=document.getElementById('heroWeatherSub');");
  html += F("if(tempEl){ const v=st.temp;");
  html += F("  if(typeof v==='number'){");
  html += F("    tempEl.textContent=showTemp(v).toFixed(1)+' '+tempUnit;");
  html += F("    if(_lastTemp!==null){ const d=v-_lastTemp; if(d>0.1) _lastTempTrend='\\u2191'; else if(d<-0.1) _lastTempTrend='\\u2193'; }");
  html += F("    _lastTemp=v;");
  html += F("  } else { tempEl.textContent='--'; _lastTemp=null; _lastTempTrend='\\u2192'; }");
  html += F("  if(trendEl){ trendEl.textContent=_lastTempTrend; trendEl.style.color=(_lastTempTrend==='\\u2191')?'#dc2626':(_lastTempTrend==='\\u2193')?'#2563eb':'inherit'; }");
  html += F("  if(heroWeatherTrend){ heroWeatherTrend.textContent=_lastTempTrend; heroWeatherTrend.style.color=(_lastTempTrend==='\\u2191')?'#dc2626':(_lastTempTrend==='\\u2193')?'#2563eb':'inherit'; }");
  html += F("}");
  html += F("const feelsEl=document.getElementById('feelsChip'); if(feelsEl){ const v=st.feels_like; feelsEl.textContent=(typeof v==='number')?showTemp(v).toFixed(1)+' '+tempUnit:'--'; }");
  html += F("const humEl=document.getElementById('humChip'); if(humEl){ const v=st.humidity; humEl.textContent=(typeof v==='number')?Math.round(v)+' %':'--'; }");
  html += F("const windEl=document.getElementById('windChip'); if(windEl){ const v=st.wind; windEl.textContent=(typeof v==='number')?v.toFixed(1)+' m/s':'--'; }");
  html += F("const windDirEl=document.getElementById('windDirChip'); if(windDirEl){ const v=(typeof st.windDirText==='string'&&st.windDirText.length)?st.windDirText:'--'; windDirEl.textContent=v; const m=/\\(([-0-9.]+) deg\\)/.exec(v); setWindCompass((typeof st.windDirDeg==='number')?st.windDirDeg:(m?Number(m[1]):NaN)); }");
  html += F("const condEl=document.getElementById('cond');");
  html += F("const cd=(typeof st.condDesc==='string' && st.condDesc.length)?st.condDesc:'';");
  html += F("const cm=(typeof st.condMain==='string' && st.condMain.length)?st.condMain:'';");
  html += F("const condText=cd||cm||'--'; if(condEl){ condEl.textContent=condText; } setWeatherIcon(condText);");
  html += F("if(heroWeather){ const v=st.temp; heroWeather.textContent=(typeof v==='number')?showTemp(v).toFixed(1)+' '+tempUnit:'--'; }");
  html += F("if(heroWeatherSub) heroWeatherSub.textContent=(condText!=='--')?condText:'Weather data syncing';");

  // keep master pill synced
  html += F("const bm=document.getElementById('btn-master'); const ms=document.getElementById('master-state');");
  html += F("if(bm && ms && typeof st.masterOn==='boolean'){ bm.setAttribute('aria-pressed', st.masterOn?'true':'false'); ms.textContent = st.masterOn?'On':'Off'; }");
  html += F("const hm=document.getElementById('heroMasterState'); const hms=document.getElementById('heroMasterSub');");
  html += F("if(hm){ if(st.systemPaused){ hm.textContent='Paused'; if(hms) hms.textContent='Schedules are temporarily suspended'; }");
  html += F("else if(st.masterOn){ hm.textContent='Master On'; if(hms) hms.textContent=activeCount?(activeCount+' zone'+(activeCount===1?'':'s')+' running'):'Automation ready'; }");
  html += F("else { hm.textContent='Master Off'; if(hms) hms.textContent='Automation blocked'; } }");

  // Smart watering status
  html += F("(function(){ const note=document.getElementById('smartNote'); const status=document.getElementById('smartStatusText'); const factor=document.getElementById('smartFactorText'); const skip=document.getElementById('smartSkipText'); const moisture=document.getElementById('smartMoistureText');");
  html += F("const enabled=!!st.smartWatering; const pct=(typeof st.smartFactor==='number')?Math.round(st.smartFactor*100):100;");
  html += F("if(note) note.classList.toggle('is-disabled',!enabled); if(status) status.textContent=enabled?'Enabled':'Disabled'; if(factor) factor.textContent=(enabled?pct:100)+'%';");
  html += F("if(skip) skip.textContent=(enabled&&pct<=0)?' - starts will be skipped by the current smart rules':'';");
  html += F("if(moisture){ if(!st.moistureEnabled) moisture.textContent='Disabled'; else if(typeof st.moisturePct!=='number'||st.moisturePct<0) moisture.textContent='No valid reading'; else moisture.textContent=Math.round(st.moisturePct)+(st.moistureSource==='meteo'?'% water volume (Open-Meteo 0-1 cm)':'% wet')+(st.moistureSkip?' - skip active':''); }");
  html += F("})();");

  // Next Water
  html += F("(function(){ const zEl=document.getElementById('nwZone'); const tEl=document.getElementById('nwTime'); const eEl=document.getElementById('nwETA'); const dEl=document.getElementById('nwDur');");
  html += F("const hv=document.getElementById('heroNextValue'); const hs=document.getElementById('heroNextSub');");
  html += F("const epoch=st.nextWaterEpoch|0; const zone=st.nextWaterZone; const name=st.nextWaterName||(Number.isInteger(zone)?('Z'+(zone+1)):'--'); const dur=st.nextWaterDurSec|0;");
  html += F("function fmtDur(s){ if(s<=0) return '--'; const m=Math.floor(s/60), sec=s%60; return pad(m)+'m '+pad(sec)+'s'; }");
  html += F("function fmtETA(delta){ if(delta<=0) return 'now'; const h=Math.floor(delta/3600), m=Math.floor((delta%3600)/60); if(h>0) return h+'h '+m+'m'; return m+'m'; }");
  html += F("const masterOff=(st.masterOn===false);");
  html += F("if(zEl) zEl.textContent=masterOff?'--':((zone>=0&&zone<255)?name:'--'); if(tEl) tEl.textContent=masterOff?'Master off':nextWaterStartLabel(epoch); if(dEl) dEl.textContent=masterOff?'--':fmtDur(dur);");
  html += F("let nowEpoch=(typeof st.deviceEpoch==='number'&&st.deviceEpoch>0&&_devEpoch!=null)?_devEpoch:Math.floor(Date.now()/1000);");
  html += F("if(eEl) eEl.textContent=(masterOff||!epoch)?'--':fmtETA(epoch-nowEpoch);");
  html += F("if(hv) hv.textContent=masterOff?'Master off':nextWaterStartLabel(epoch);");
  html += F("if(hs) hs.textContent=masterOff?'Automation blocked':(epoch?(name+(dur>0?(' - '+fmtDur(dur)):'')):(st.rainDelayActive?'Waiting for rain delay to clear':'No queued run'));");
  html += F("})();");

  html += F("}catch(e){} } setInterval(refreshStatus,1000); refreshStatus(); setInterval(refreshWifiStatus,60000);");

  // expose zonesCount & Save All
  html += F("const ZC="); html += String(zonesCount); html += F(";");
  html += F("function attachRepeat(btn,fn){let t=0,i=0,ptr=false;const stop=()=>{clearTimeout(t);clearInterval(i);t=i=0;};btn.addEventListener('pointerdown',e=>{if(e.button&&e.button!==0)return;ptr=true;e.preventDefault();btn.setPointerCapture?.(e.pointerId);fn(e);t=setTimeout(()=>{i=setInterval(()=>fn(e),90);},360);});btn.addEventListener('click',e=>{if(ptr){ptr=false;return;}fn(e);});['pointerup','pointercancel','pointerleave','blur'].forEach(ev=>btn.addEventListener(ev,stop));}");
  html += F("function editDigits(btn,limit,apply){let buf='',timer=0;const clear=()=>{buf='';btn.classList.remove('is-editing');};btn.addEventListener('keydown',e=>{if(/^\\d$/.test(e.key)){e.preventDefault();clearTimeout(timer);buf=(buf+e.key).slice(-limit);btn.classList.add('is-editing');apply(parseInt(buf,10));timer=setTimeout(clear,1100);}else if(e.key==='Backspace'||e.key==='Delete'){e.preventDefault();clear();apply(0);}else if(e.key==='Enter'||e.key==='Escape'){clear();btn.blur();}});btn.addEventListener('blur',clear);}");
  html += F("function initTimeSpin(el){");
  html += F("  const hEl=el.querySelector('input[name^=\"startHour\"]'); const mEl=el.querySelector('input[name^=\"startMin\"]'); if(!hEl||!mEl)return;");
  html += F("  const hv=el.querySelector('[data-role=\"hour\"]'); const mv=el.querySelector('[data-role=\"min\"]'); const ap=el.querySelector('[data-role=\"ampm\"]');");
  html += F("  const pad=n=>String(n).padStart(2,'0'); const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));");
  html += F("  function total(){return clamp(parseInt(hEl.value||'0',10)||0,0,23)*60+clamp(parseInt(mEl.value||'0',10)||0,0,59);}");
  html += F("  function setTotal(t){t=((t%1440)+1440)%1440; const h=Math.floor(t/60), m=t%60; hEl.value=pad(h); mEl.value=pad(m); sync();}");
  html += F("  function setPart(part,v){const h=clamp(parseInt(hEl.value||'0',10)||0,0,23),m=clamp(parseInt(mEl.value||'0',10)||0,0,59); if(part==='hour'){const pm=h>=12; let hh=clamp(v,1,12)%12; hEl.value=pad(hh+(pm?12:0));} else {mEl.value=pad(clamp(v,0,59));} sync();}");
  html += F("  function sync(){const h=clamp(parseInt(hEl.value||'0',10)||0,0,23), m=clamp(parseInt(mEl.value||'0',10)||0,0,59); hEl.value=pad(h); mEl.value=pad(m); if(hv)hv.textContent=pad((h%12)||12); if(mv)mv.textContent=pad(m); if(ap)ap.textContent=h>=12?'PM':'AM';}");
  html += F("  el.querySelectorAll('[data-part]').forEach(btn=>attachRepeat(btn,()=>{const dir=parseInt(btn.dataset.dir||'0',10)||0; setTotal(total()+dir*(btn.dataset.part==='hour'?60:1));}));");
  html += F("  const keyStep=(part,dir,e)=>setTotal(total()+dir*(part==='hour'?60:(e.shiftKey?10:1)));");
  html += F("  [[hv,'hour'],[mv,'min']].forEach(([b,p])=>{if(!b)return;b.title+=' - type digits or use arrow keys';editDigits(b,p==='hour'?2:2,v=>setPart(p,v));b.addEventListener('keydown',e=>{if(e.key==='ArrowUp'){e.preventDefault();keyStep(p,1,e);}else if(e.key==='ArrowDown'){e.preventDefault();keyStep(p,-1,e);}else if(e.key==='PageUp'){e.preventDefault();setTotal(total()+(p==='hour'?180:10));}else if(e.key==='PageDown'){e.preventDefault();setTotal(total()-(p==='hour'?180:10));}else if(e.key==='Home'){e.preventDefault();setPart(p,p==='hour'?12:0);}else if(e.key==='End'){e.preventDefault();setPart(p,p==='hour'?11:59);}});});");
  html += F("  if(ap)ap.addEventListener('click',()=>setTotal(total()+720)); sync();");
  html += F("  if(ap)ap.addEventListener('keydown',e=>{if(['ArrowUp','ArrowDown',' ','Enter'].includes(e.key)){e.preventDefault();setTotal(total()+720);}});");
  html += F("}");
  html += F("document.querySelectorAll('[data-time-spin]').forEach(initTimeSpin);");
  html += F("function initDurationSpin(el){");
  html += F("  const mEl=el.querySelector('input[name*=\"duration\"][name$=\"Min\"],input[name^=\"durationMin\"],input[name^=\"duration2Min\"]'); const sEl=el.querySelector('input[name*=\"duration\"][name$=\"Sec\"],input[name^=\"durationSec\"],input[name^=\"duration2Sec\"]'); if(!mEl||!sEl)return;");
  html += F("  const mv=el.querySelector('[data-role=\"dmin\"]'); const sv=el.querySelector('[data-role=\"dsec\"]'); const pad=n=>String(n).padStart(2,'0'); const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));");
  html += F("  function total(){return clamp(parseInt(mEl.value||'0',10)||0,0,600)*60+clamp(parseInt(sEl.value||'0',10)||0,0,59);}");
  html += F("  function setTotal(t){t=clamp(t,0,600*60+59); const m=Math.floor(t/60), s=t%60; mEl.value=String(m); sEl.value=pad(s); sync();}");
  html += F("  function setPart(part,v){if(part==='min')mEl.value=String(clamp(v,0,600));else sEl.value=pad(clamp(v,0,59));sync();}");
  html += F("  function sync(){const m=clamp(parseInt(mEl.value||'0',10)||0,0,600), s=clamp(parseInt(sEl.value||'0',10)||0,0,59); mEl.value=String(m); sEl.value=pad(s); if(mv)mv.textContent=String(m); if(sv)sv.textContent=pad(s);}");
  html += F("  el.querySelectorAll('[data-part]').forEach(btn=>attachRepeat(btn,()=>{const dir=parseInt(btn.dataset.dir||'0',10)||0; setTotal(total()+dir*(btn.dataset.part==='min'?60:1));}));");
  html += F("  [[mv,'min',3],[sv,'sec',2]].forEach(([b,p,l])=>{if(!b)return;b.title+=' - type digits or use arrow keys';editDigits(b,l,v=>setPart(p,v));b.addEventListener('keydown',e=>{const big=p==='min'?300:10,small=p==='min'?60:1;if(e.key==='ArrowUp'){e.preventDefault();setTotal(total()+(e.shiftKey?big:small));}else if(e.key==='ArrowDown'){e.preventDefault();setTotal(total()-(e.shiftKey?big:small));}else if(e.key==='PageUp'){e.preventDefault();setTotal(total()+big);}else if(e.key==='PageDown'){e.preventDefault();setTotal(total()-big);}else if(e.key==='Home'){e.preventDefault();setPart(p,0);}else if(e.key==='End'){e.preventDefault();setPart(p,p==='min'?600:59);}});}); sync();");
  html += F("}");
  html += F("document.querySelectorAll('[data-duration-spin]').forEach(initDurationSpin);");
  html += F("async function saveAll(){");
  html += F("  const fd=new URLSearchParams();");
  html += F("  for(let z=0; z<ZC; z++){");
  html += F("    const q=n=>document.querySelector(`[name='${n}']`);");
  html += F("    const add=(k)=>{const el=q(k); if(el){ if((el.type||'').toLowerCase()==='checkbox'){ if(el.checked) fd.append(k,'on'); } else { fd.append(k,el.value); } } };");
  html += F("    add('zoneName'+z); add('startHour'+z); add('startMin'+z); add('startHour2'+z); add('startMin2'+z); add('durationMin'+z); add('durationSec'+z); add('duration2Min'+z); add('duration2Sec'+z);");
  html += F("    add('enableStartTime2'+z); add('zoneWindDelay'+z);");
  html += F("    for(let d=0; d<7; d++) add('day'+z+'_'+d);");
  html += F("  }");
  html += F("  try{ await fetch('/submit',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:fd.toString()}); location.reload(); }catch(e){ console.error(e); }");
  html += F("} ");
  html += F("document.getElementById('btn-save-all')?.addEventListener('click', saveAll);document.getElementById('btn-save-all-bottom')?.addEventListener('click', saveAll);");
  html += F("const summaryBody=document.getElementById('summaryBody'); const summaryToggle=document.getElementById('summaryToggle');");
  html += F("if(summaryBody && summaryToggle){ const syncSummary=()=>{ const open=summaryBody.style.display!=='none'; summaryToggle.textContent=open?'Hide Summary':'Show Summary'; summaryToggle.setAttribute('aria-expanded',open?'true':'false'); summaryToggle.closest('.collapsible-section-head')?.classList.toggle('is-open',open); };");
  html += F("summaryToggle.addEventListener('click',()=>{ const open=summaryBody.style.display!=='none'; summaryBody.style.display=open?'none':'block'; syncSummary(); }); syncSummary(); }");
  html += F("const schedBody=document.getElementById('schedBody'); const schedToggle=document.getElementById('schedToggle');");
  html += F("if(schedBody && schedToggle){ const syncSched=()=>{ const open=schedBody.style.display!=='none'; schedToggle.textContent=open?'Hide Schedules':'Show Schedules'; schedToggle.setAttribute('aria-expanded',open?'true':'false'); };");
  html += F("schedToggle.addEventListener('click',()=>{ const open=schedBody.style.display!=='none'; schedBody.style.display=open?'none':'block'; syncSched(); }); syncSched(); }");
  // Toggle Duration 2 rows when Start 2 is enabled
  html += F("for(let z=0; z<ZC; z++){");
  html += F("  const cb=document.querySelector(`[name='enableStartTime2${z}']`);");
  html += F("  const row=document.getElementById('dur2row'+z);");
  html += F("  const s2=document.getElementById('start2fields'+z);");
  html += F("  if(!cb||!row) continue;");
  html += F("  const sync=()=>{row.style.display=cb.checked?'grid':'none'; if(s2) s2.style.display=cb.checked?'inline-flex':'none';};");
  html += F("  cb.addEventListener('change', sync); sync();");
  html += F("}");

  // Weather model: toggle custom input
  html += F("const modelSel=document.getElementById('meteoModelSelect'); const customRow=document.getElementById('meteoModelCustomRow');");
  html += F("if(modelSel && customRow){ const syncModel=()=>{ customRow.style.display=(modelSel.value==='custom')?'flex':'none'; };");
  html += F("modelSel.addEventListener('change', syncModel); syncModel(); }");

  html += F("</script></body></html>");
  flush();
  server.sendContent("");
}

// Setup Page 
void handleSetupPage() {
  HttpScope _scope;
  loadConfig();
  sanitizePinConfig();
  // Stream this large page in sections. A classic ESP32 cannot reliably grow
  // one contiguous String large enough for the complete setup form.
  String html; html.reserve(12000);
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  auto flush = [&](){
    if (html.length()) {
      server.sendContent(html);
      html = "";
    }
  };
  const int uiMaxGpio = 
  #if defined(CONFIG_IDF_TARGET_ESP32)
    39;
  #else
    48;
  #endif
  String latStr = isfinite(meteoLat) ? String(meteoLat, 6) : String("");
  String lonStr = isfinite(meteoLon) ? String(meteoLon, 6) : String("");
  String setupWeatherLabel = meteoLocation.length() ? meteoLocation : String("Open-Meteo ready");
  String setupDisplayLabel = !displayEnabled ? String("Display disabled") : (displayUseTft ? String("TFT display") : String("OLED display"));
  String setupTankLabel = tankEnabled ? String("Tank Enabled") : String("City Water");
  String setupTzLabel = (tzMode == TZ_FIXED) ? String("Fixed offset") : String("POSIX timezone");
  String setupModelLabel = cleanMeteoModel(meteoModel);
  if (setupModelLabel == "best_match") setupModelLabel = "Best match";
  else setupModelLabel.replace("_", " ");
  int setupMoistureRaw = moistureRaw();
  int setupMoisturePct = moisturePercent();
  if (setupMoistureRaw >= 0 && moistureDryRaw != moistureWetRaw) {
    setupMoisturePct = constrain(map(setupMoistureRaw, moistureDryRaw, moistureWetRaw, 0, 100), 0, 100);
  }
  auto addTzOption = [&](const char* name, const char* posix) {
    html += F("<option value='");
    html += name;
    html += F("' data-posix='");
    html += posix;
    html += F("'");
    if (tzIANA == name) html += F(" selected");
    html += F(">");
    html += name;
    html += F("</option>");
  };

  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<meta name='theme-color' content='#1e40af'><meta name='color-scheme' content='light dark'>");
  html += F("<title>Setup - ESP32 Irrigation</title>");
  html += F("<style>");
  html += F("html{scroll-behavior:smooth}body{margin:0;font-family:'Trebuchet MS','Candara','Segoe UI',sans-serif;background:#0d1b2f;color:#e7f1ec;font-size:15px;line-height:1.4}");
  html += F(".wrap{max-width:1100px;margin:28px auto;padding:0 16px}");
  html += F("h1{margin:0 0 16px 0;font-size:1.7em;letter-spacing:.3px;font-weight:800}");
  html += F(".page-head{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:16px}");
  html += F(".page-head h1{margin:0}");
  html += F(".page-head-copy{display:flex;flex-direction:column;gap:4px}");
  html += F(".page-kicker{text-transform:uppercase;letter-spacing:.18em;font-size:.72rem;font-weight:800;color:#93c5fd}");
  html += F(".page-sub{color:#9ab4ad;font-size:.92rem}");
  html += F(".setup-hero{display:grid;grid-template-columns:minmax(0,1.2fr) minmax(280px,.8fr);gap:14px;padding:18px;margin:16px 0 18px;border-radius:18px;");
  html += F("background:linear-gradient(145deg,rgba(15,27,30,.86),rgba(20,69,68,.6));border:1px solid rgba(92,131,125,.24);box-shadow:0 16px 38px rgba(0,0,0,.28)}");
  html += F(".setup-hero-copy{display:flex;flex-direction:column;justify-content:center;gap:10px}");
  html += F(".setup-hero-copy h2{margin:0;font-size:1.55rem;line-height:1.08;max-width:14ch}");
  html += F(".setup-hero-copy p{margin:0;color:#b4c8c2;max-width:60ch}");
  html += F(".setup-hero.badges-only{display:block}");
  html += F(".setup-overview-title{margin:0 0 12px 0;font-size:.78rem;letter-spacing:.16em;text-transform:uppercase;font-weight:850;color:#38bdf8}");
  html += F(".setup-badges{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}");
  html += F(".setup-badge{padding:12px 13px;border-radius:15px;border:1px solid rgba(92,131,125,.24);background:linear-gradient(180deg,rgba(255,255,255,.08),rgba(147,197,253,.04))}");
  html += F(".setup-badge-k{font-size:.73rem;letter-spacing:.14em;text-transform:uppercase;color:#9ab4ad;font-weight:800}");
  html += F(".setup-badge-v{margin-top:6px;font-size:1rem;font-weight:760;color:#f0f8f3}");
  html += F(".setup-nav{display:flex;gap:10px;flex-wrap:wrap;margin:0 0 16px;padding:8px;border-radius:16px;position:sticky;top:10px;z-index:8;");
  html += F("background:rgba(12,22,24,.76);backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);border:1px solid rgba(92,131,125,.2);box-shadow:0 10px 28px rgba(0,0,0,.16)}");
  html += F(".setup-nav a{display:inline-flex;align-items:center;justify-content:center;padding:9px 14px;border-radius:999px;border:1px solid rgba(92,131,125,.24);");
  html += F("background:rgba(10,23,25,.58);color:#e7f1ec;font-size:.86rem;font-weight:760;letter-spacing:.01em;transition:transform .08s ease,background .12s ease,border-color .12s ease,box-shadow .12s ease}");
  html += F(".setup-nav a:hover{transform:translateY(-1px);background:rgba(37,99,235,.18);border-color:rgba(147,197,253,.36);box-shadow:0 10px 24px rgba(30,64,175,.16)}");
  html += F(".setup-actions-top{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin:0 0 16px;padding:10px 12px;border-radius:16px;position:sticky;top:74px;z-index:7;background:rgba(13,23,24,.82);backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);border:1px solid rgba(92,131,125,.2);box-shadow:0 12px 28px rgba(0,0,0,.18)}");
  html += F(".setup-actions-top .btn,.setup-actions-top .btn-alt{margin:0}");
  html += F(".setup-actions-top .btn[type=submit]{min-width:160px}");
  html += F(".save-confirm{display:none;margin-left:auto;padding:8px 12px;border-radius:8px;background:#dbeafe;color:#1e40af;border:1px solid #93c5fd;font-weight:800;font-size:.9rem}");
  html += F(".save-confirm.show{display:inline-flex;align-items:center}");
  html += F(".card-intro{margin:0 0 8px;color:#9ab4ad;font-size:.9rem;max-width:62ch}");
  html += F(".theme-switch{display:flex;align-items:center;gap:6px;font-weight:700;color:#d5e4de}");
  html += F(".switch{position:relative;display:inline-block;width:42px;height:24px;min-width:unset}");
  html += F(".switch input{opacity:0;width:0;height:0}");
  html += F(".slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#16262a;border:1px solid #284449;transition:.2s;border-radius:999px}");
  html += F(".slider:before{position:absolute;content:'';height:18px;width:18px;left:3px;top:2px;background:#e7f1ec;transition:.2s;border-radius:50%}");
  html += F("input:checked + .slider{background:#2563eb;border-color:#2563eb}");
  html += F("input:checked + .slider:before{transform:translateX(18px);background:#fff}");
  html += F(".card{background:#0f1d31;border:1px solid #24405f;border-radius:16px;box-shadow:0 8px 34px rgba(0,0,0,.35);padding:18px 16px;margin-bottom:16px}");
  html += F(".card.narrow{max-width:960px;margin-left:auto;margin-right:auto}");
  html += F(".card h3{margin:0 0 12px 0;font-size:1.1em;font-weight:850;letter-spacing:.35px;color:#f0f8f3;");
  html += F("padding-bottom:6px;border-bottom:2px solid #2563eb;display:flex;align-items:center;gap:8px}");
  html += F("label{display:inline-block;min-width:200px;font-size:.95rem;font-weight:600;color:#d5e4de;text-align:left}");
  // Inputs + select share the same theme
  html += F("input[type=text],input[type=number],select{background:#0c1618;color:#e7f1ec;border:1px solid #294b73;border-radius:12px;padding:9px 12px;font-size:.95rem;text-align:left}");
  html += F("option{background:#ffffff;color:#14232b}html[data-theme='dark'] option{background:#0c1618;color:#e7f1ec}");
  html += F("input[type=text],select{width:100%;max-width:520px}");
  html += F("input[type=number]{width:100%;max-width:200px}");
  html += F("input[type=text].in-wide,select.in-wide{max-width:600px}");
  html += F("input[type=text].in-med,select.in-med{max-width:440px}");
  html += F("input[type=number].in-xs{max-width:120px}");
  html += F("input[type=number].in-sm{max-width:160px}");
  html += F("input[type=number].in-md{max-width:240px}");
  html += F(".row{display:flex;align-items:center;gap:12px;margin:10px 0;padding-top:10px;border-top:1px solid rgba(41,69,74,.7);flex-wrap:wrap}.row small{color:#9ab4ad;font-size:.85rem}");
  html += F(".card h3 + .row{border-top:none;padding-top:0}");
  html += F(".btn{background:linear-gradient(180deg,#2563eb,#1e40af);color:#fff;border:1px solid rgba(0,0,0,.18);border-radius:12px;padding:10px 14px;font-weight:700;cursor:pointer;box-shadow:0 6px 16px rgba(30,64,175,.28);font-size:.95rem}");
  html += F(".btn-alt{background:#17272b;color:#e7f1ec;border:1px solid #294b73;border-radius:12px;padding:10px 14px;font-size:.95rem}");
  html += F(".btn-danger{background:linear-gradient(180deg,#ef4444,#b91c1c);border:1px solid rgba(185,28,28,.6)}");
  html += F(".btn,.btn-alt{transition:transform .06s ease,box-shadow .06s ease,filter .06s ease}");
  html += F(".btn:active,.btn-alt:active{transform:translateY(1px);box-shadow:inset 0 2px 6px rgba(0,0,0,.25)}");
  html += F(".btn,.btn-alt{position:relative;overflow:hidden}");
  html += F(".ripple{position:absolute;border-radius:999px;transform:scale(0);background:rgba(255,255,255,.35);animation:ripple .5s ease-out;pointer-events:none}");
  html += F("@keyframes ripple{to{transform:scale(3.2);opacity:0;}}");
  html += F(".grid{display:grid;grid-template-columns:1fr;gap:14px}");
  html += F(".cols2{display:grid;grid-template-columns:1fr 1fr;gap:14px}");
  html += F(".panel-split > div{padding:14px;border-radius:14px;border:1px solid rgba(92,131,125,.16);background:linear-gradient(180deg,rgba(255,255,255,.03),rgba(255,255,255,.015))}");
  html += F("@media(max-width:760px){.cols2{grid-template-columns:1fr}label{min-width:150px}input[type=text].in-wide,select.in-wide,input[type=text].in-med,select.in-med{max-width:100%}}");
  html += F(".switchline{display:flex;gap:12px;align-items:center;flex-wrap:wrap}");
  html += F("input[type=checkbox]:not(#themeToggle),input[type=radio]{appearance:none;-webkit-appearance:none;width:18px;height:18px;margin:0;flex:0 0 18px;display:inline-grid;place-content:center;cursor:pointer;");
  html += F("border:1.6px solid rgba(255,255,255,.78);background:transparent;box-shadow:none;transition:border-color .12s ease,background .12s ease,box-shadow .12s ease,transform .06s ease}");
  html += F("input[type=checkbox]:not(#themeToggle){border-radius:6px}");
  html += F("input[type=radio]{border-radius:999px}");
  html += F("input[type=checkbox]:not(#themeToggle)::before,input[type=radio]::before{content:'';display:block;transform:scale(0);transition:transform .12s ease}");
  html += F("input[type=checkbox]:not(#themeToggle)::before{width:5px;height:9px;border-right:2px solid #fff;border-bottom:2px solid #fff;transform:rotate(45deg) scale(0);margin-top:-1px}");
  html += F("input[type=radio]::before{width:8px;height:8px;border-radius:999px;background:#fff}");
  html += F("input[type=checkbox]:not(#themeToggle):checked,input[type=radio]:checked{border-color:#ffffff;background:transparent;box-shadow:none}");
  html += F("input[type=checkbox]:not(#themeToggle):checked::before{transform:rotate(45deg) scale(1)}");
  html += F("input[type=radio]:checked::before{transform:scale(1)}");
  html += F("input[type=checkbox]:not(#themeToggle):focus-visible,input[type=radio]:focus-visible{outline:none;border-color:#ffffff;box-shadow:0 0 0 3px rgba(37,99,235,.2)}");
  html += F("input[type=checkbox]:not(#themeToggle):active,input[type=radio]:active{transform:translateY(1px)}");
  html += F(".subhead{opacity:.85;margin:8px 0 6px 0;font-weight:700;font-size:.98rem}");
  html += F(".hr{height:1px;background:#24405f;margin:10px 0 8px 0;border:none}");

  // Chips + inline options for timezone mode
  html += F(".inline-options{display:flex;flex-wrap:wrap;gap:10px}");
  html += F(".chip{display:inline-flex;align-items:center;gap:6px;padding:6px 12px;border-radius:999px;background:#0c1618;border:1px solid #294b73;font-size:.85rem;color:#e7f1ec}");
  html += F(".chip input{margin:0}");
  html += F(".chip span{white-space:nowrap}");

  // Collapsible card styling
  html += F(".card,.setup-nav,.setup-actions-top,[id$='-card']{scroll-margin-top:144px}");
  html += F("details.collapse{padding:0;margin:0}");
  html += F("details.collapse summary{cursor:pointer;outline:none;list-style:none;font-weight:800;font-size:1.05rem;color:#e7f1ec;padding:4px 0}");
  html += F("details.collapse summary::-webkit-details-marker{display:none}");
  html += F("details.collapse summary:after{content:'>';margin-left:8px;transition:transform .18s ease;display:inline-block;width:26px;height:26px;line-height:24px;text-align:center;border-radius:999px;border:1px solid rgba(92,131,125,.22);background:rgba(255,255,255,.04)}");
  html += F("details.collapse[open] summary:after{transform:rotate(90deg)}");
  html += F("details.collapse[open] summary:after{background:rgba(37,99,235,.16);border-color:rgba(147,197,253,.3)}");
  html += F(".collapse-body{margin-top:10px;padding-top:4px}");

  // Help text row: let the text wrap nicely
  html += F(".helptext label{min-width:0}");
  html += F(".helptext small{max-width:520px;display:block}");

  // Make native select look like themed dropdown with a chevron
  html += F("select{appearance:none;-webkit-appearance:none;-moz-appearance:none;padding-right:32px;");
  html += F("background-image:linear-gradient(45deg,transparent 50%,#93aba4 50%),linear-gradient(135deg,#93aba4 50%,transparent 50%);");
  html += F("background-position:calc(100% - 18px) 50%,calc(100% - 13px) 50%;background-size:5px 5px,5px 5px;background-repeat:no-repeat;}");
  html += F("body{-webkit-font-smoothing:antialiased;text-rendering:optimizeLegibility}");
  html += F(".card{transition:transform .12s ease,box-shadow .12s ease}");
  html += F(".card:hover{transform:translateY(-2px);box-shadow:0 12px 28px rgba(0,0,0,.22)}");
  html += F(".btn:hover,.btn-alt:hover{filter:brightness(1.06)}");
  html += F("input:focus-visible,select:focus-visible{outline:2px solid #2563eb;outline-offset:2px}");
  html += F("a{text-decoration:none;color:inherit}");
  html += F("@media (prefers-reduced-motion: reduce){*{animation:none!important;transition:none!important}}");

  // Light theme overrides
  html += F("html[data-theme='light'] body{background:#f1f5f1;color:#1c2b2c}");
  html += F("html[data-theme='light'] .theme-switch{color:#314847}");
  html += F("html[data-theme='light'] .card{background:#ffffff;border-color:#d7e2f0;box-shadow:0 10px 26px rgba(16,24,40,.08)}");
  html += F("html[data-theme='light'] .card h3{color:#14232b;border-bottom-color:#2563eb}");
  html += F("html[data-theme='light'] label{color:#213433}");
  html += F("html[data-theme='light'] .row small{color:#607571}");
  html += F("html[data-theme='light'] input[type=text],html[data-theme='light'] input[type=number],html[data-theme='light'] select{background:#f7faf8;color:#14232b;border-color:#c9d9d0}");
  html += F("html[data-theme='light'] .chip{background:#eef6f2;border-color:#c9d9d0;color:#213433}");
  html += F("html[data-theme='light'] input[type=checkbox]:not(#themeToggle),html[data-theme='light'] input[type=radio]{border-color:#8aa59c;background:rgba(255,255,255,.78);box-shadow:inset 0 1px 0 rgba(255,255,255,.75)}");
  html += F("html[data-theme='light'] input[type=checkbox]:not(#themeToggle)::before{border-right-color:#1e40af;border-bottom-color:#1e40af}");
  html += F("html[data-theme='light'] input[type=radio]::before{background:#1e40af}");
  html += F("html[data-theme='light'] input[type=checkbox]:not(#themeToggle):checked,html[data-theme='light'] input[type=radio]:checked{border-color:#2563eb;background:#ffffff;box-shadow:0 0 0 3px rgba(37,99,235,.12)}");
  html += F("html[data-theme='light'] input[type=checkbox]:not(#themeToggle):checked::before{border-right-width:2.4px;border-bottom-width:2.4px}");
  html += F("html[data-theme='light'] input[type=checkbox]:not(#themeToggle):focus-visible,html[data-theme='light'] input[type=radio]:focus-visible{border-color:#2563eb;box-shadow:0 0 0 3px rgba(37,99,235,.14)}");
  html += F("html[data-theme='light'] .btn-alt{background:#e8efeb;color:#213433;border-color:#c9d9d0}");
  html += F("html[data-theme='light'] .btn{background:linear-gradient(180deg,#2563eb,#1e40af)}");
  html += F("html[data-theme='light'] .btn-danger{background:linear-gradient(180deg,#ef4444,#b91c1c)}");
  html += F("html[data-theme='light'] .hr{background:#d7e2f0}");
  html += F("html[data-theme='light'] .setup-hero{background:linear-gradient(145deg,rgba(255,255,255,.88),rgba(235,245,240,.86));border-color:#d7e2f0;box-shadow:0 14px 34px rgba(16,24,40,.08)}");
  html += F("html[data-theme='light'] .page-kicker{color:#2563eb}");
  html += F("html[data-theme='light'] .page-sub{color:#607571}");
  html += F("html[data-theme='light'] .setup-hero-copy p{color:#607571}");
  html += F("html[data-theme='light'] .setup-badge{background:linear-gradient(180deg,#ffffff,#f1f7f3);border-color:#d7e2f0}");
  html += F("html[data-theme='light'] .setup-badge-k{color:#607571}");
  html += F("html[data-theme='light'] .setup-badge-v{color:#14232b}");
  html += F("html[data-theme='light'] .setup-nav{background:rgba(255,255,255,.8);border-color:#d7e2f0;box-shadow:0 10px 28px rgba(16,24,40,.08)}");
  html += F("html[data-theme='light'] .setup-nav a{background:rgba(255,255,255,.88);border-color:#d7e2f0;color:#213433}");
  html += F("html[data-theme='light'] .setup-nav a:hover{background:#eef5ff;border-color:#b7d5c8;box-shadow:0 10px 24px rgba(37,99,235,.08)}");
  html += F("html[data-theme='light'] .card-intro{color:#607571}");
  html += F("html[data-theme='light'] .row{border-top-color:#e4ece7}");
  html += F("html[data-theme='light'] select{background-image:linear-gradient(45deg,transparent 50%,#607571 50%),linear-gradient(135deg,#607571 50%,transparent 50%)}");
  html += F("html[data-theme='light'] .setup-actions-top{background:rgba(255,255,255,.84);border-color:#d7e2f0;box-shadow:0 12px 28px rgba(16,24,40,.08)}");
  html += F("html[data-theme='light'] .panel-split > div{border-color:#dfe9e3;background:linear-gradient(180deg,#ffffff,#f7fbf8)}");
  html += F("html[data-theme='light'] details.collapse summary:after{border-color:#d7e2f0;background:#f7fbf8}");
  html += F("html[data-theme='light'] details.collapse[open] summary:after{background:#eef5ff;border-color:#b7d5c8}");

  // Desktop tuning
  html += F("@media(min-width:1024px){");
  html += F("body{font-size:16px;}");
  html += F(".wrap{max-width:1160px;padding:0 20px;}");
  html += F(".card{padding:20px 18px;}");
  html += F(".row{justify-content:flex-start;}");
  html += F("label{min-width:240px;}");
  html += F("input[type=text],select{max-width:560px;}");
  html += F("input[type=number]{max-width:220px;}");
  html += F("}");
  html += F("html,body{font-family:'Sora','Avenir Next','Trebuchet MS',sans-serif}");
  html += F("body{position:relative;min-height:100vh;background:radial-gradient(900px 480px at 0% -10%,rgba(37,99,235,.2),transparent),radial-gradient(900px 500px at 110% 8%,rgba(96,165,250,.14),transparent),#0d1b2f}");
  html += F(".page-head{padding:10px 14px;border-radius:15px;background:rgba(16,27,30,.48);backdrop-filter:blur(6px);-webkit-backdrop-filter:blur(6px);border:1px solid rgba(92,131,125,.24)}");
  html += F(".page-head h1{text-transform:uppercase;font-size:1.35rem;letter-spacing:1px}");
  html += F(".theme-switch{font-size:.88rem}");
  html += F(".card{position:relative;overflow:hidden}");
  html += F(".card::before{content:'';position:absolute;left:0;right:0;top:0;height:3px;background:linear-gradient(90deg,#2563eb,#38bdf8)}");
  html += F("input[type=text],input[type=number],select{font-family:'Sora','Trebuchet MS',sans-serif}");
  html += F("input[type=text]:focus-visible,input[type=number]:focus-visible,select:focus-visible{outline:2px solid #2563eb;outline-offset:1px;box-shadow:0 0 0 3px rgba(37,99,235,.18)}");
  html += F(".btn,.btn-alt{font-weight:760;letter-spacing:.16px}");
  html += F(".chip{font-weight:620}");
  html += F("details.collapse summary{display:flex;align-items:center;justify-content:space-between}");
  html += F(".card{animation:rise .42s ease both}");
  html += F(".card:nth-of-type(2){animation-delay:.04s}.card:nth-of-type(3){animation-delay:.08s}.card:nth-of-type(4){animation-delay:.12s}");
  html += F("@keyframes rise{from{opacity:0;transform:translateY(8px)}to{opacity:1;transform:translateY(0)}}");
  html += F("html[data-theme='light'] body{background:radial-gradient(900px 470px at 0% -10%,rgba(37,99,235,.18),transparent),radial-gradient(900px 450px at 110% 6%,rgba(96,165,250,.12),transparent),#f1f5f1}");
  html += F("html[data-theme='light'] .page-head{background:rgba(255,255,255,.72);border-color:#d7e2f0}");
  html += F("body{background:linear-gradient(180deg,#0d1b2f,#08111f);letter-spacing:0}.wrap{margin:20px auto}");
  html += F(".page-head,.setup-hero,.setup-nav,.setup-actions-top,.card,.panel-split>div,input[type=text],input[type=number],select,.btn,.btn-alt,.chip{border-radius:8px}");
  html += F(".page-head{background:#0f1d31;border-color:#24405f;box-shadow:0 8px 24px rgba(0,0,0,.18)}.setup-hero{background:#0f1d31;border-color:#24405f;box-shadow:0 10px 28px rgba(0,0,0,.2)}");
  html += F(".setup-badge{border-radius:8px;background:#10233d;border-color:#294b73}.setup-nav,.setup-actions-top{border-color:#24405f;box-shadow:0 8px 22px rgba(0,0,0,.18)}");
  html += F(".page-kicker{color:#93c5fd}.card{border-color:#24405f;box-shadow:0 8px 24px rgba(0,0,0,.22)}.card:hover{transform:none;box-shadow:0 10px 28px rgba(0,0,0,.26)}.card h3{border-bottom:1px solid #294b73;letter-spacing:0}");
  html += F(".slider{border-color:#294b73}.switch input:checked + .slider{background:#2563eb;border-color:#2563eb}.btn{background:linear-gradient(180deg,#2563eb,#1e40af);box-shadow:0 6px 16px rgba(37,99,235,.22)}");
  html += F(".row{gap:10px 14px}details.collapse summary:after{content:'>';border-radius:8px}input:focus-visible,select:focus-visible,input[type=text]:focus-visible,input[type=number]:focus-visible{outline-color:#38bdf8;box-shadow:0 0 0 3px rgba(37,99,235,.18)}");
  html += F("html[data-theme='light'] body{background:linear-gradient(180deg,#fbfdff,#f3f7fc)}html[data-theme='light'] .page-head,html[data-theme='light'] .setup-hero,html[data-theme='light'] .card{box-shadow:0 8px 24px rgba(16,24,40,.08)}");
  html += F("html[data-theme='light'] .page-kicker{color:#2563eb}html[data-theme='light'] .card h3{border-bottom-color:#2563eb}html[data-theme='light'] .btn{background:linear-gradient(180deg,#2563eb,#1e40af)}html[data-theme='light'] .setup-badge{background:linear-gradient(180deg,#ffffff,#eef5ff);border-color:#d7e2f0}");
  html += F(".setup-nav a:hover{background:rgba(37,99,235,.18);border-color:rgba(147,197,253,.36);box-shadow:0 10px 24px rgba(37,99,235,.16)}details.collapse[open] summary:after{background:rgba(37,99,235,.16);border-color:rgba(147,197,253,.3)}");
  html += F("input[type=checkbox]:not(#themeToggle):focus-visible,input[type=radio]:focus-visible{box-shadow:0 0 0 3px rgba(37,99,235,.2)}html[data-theme='light'] input[type=checkbox]:not(#themeToggle)::before{border-right-color:#1e40af;border-bottom-color:#1e40af}");
  html += F("html[data-theme='light'] input[type=radio]::before{background:#1e40af}html[data-theme='light'] input[type=checkbox]:not(#themeToggle):checked,html[data-theme='light'] input[type=radio]:checked{border-color:#2563eb;box-shadow:0 0 0 3px rgba(37,99,235,.12)}");
  html += F("html[data-theme='light'] input[type=checkbox]:not(#themeToggle):focus-visible,html[data-theme='light'] input[type=radio]:focus-visible{border-color:#2563eb;box-shadow:0 0 0 3px rgba(37,99,235,.14)}html[data-theme='light'] .setup-nav a:hover{background:#eef5ff;border-color:#bfdbfe;box-shadow:0 10px 24px rgba(37,99,235,.08)}");
  html += F("details.collapse summary,html[data-theme='dark'] details.collapse summary{color:#ffffff}html[data-theme='light'] details.collapse summary{color:#000000}");
  html += F("@media(max-width:760px){.page-head{padding:10px 12px}.page-head h1{font-size:1.2rem}.setup-hero{grid-template-columns:1fr}.setup-badges{grid-template-columns:1fr 1fr}.setup-nav{top:8px;flex-wrap:nowrap;overflow:auto;padding-bottom:6px}.setup-nav a{white-space:nowrap}.setup-actions-top{top:8px;z-index:9;padding:10px;border-radius:14px;background:rgba(13,23,24,.88);backdrop-filter:blur(8px);-webkit-backdrop-filter:blur(8px);border:1px solid rgba(92,131,125,.2)}.row{padding-top:8px;flex-direction:column;align-items:stretch}.row label{min-width:0;width:100%}.row .btn,.row .btn-alt{width:100%}.switchline{align-items:flex-start}}");
  html += F(":root{--setup-bg:#f3f7fc;--setup-panel:#ffffff;--setup-ink:#172033;--setup-muted:#68758a;--setup-line:#dbe4ef;--setup-blue:#2563eb}html[data-theme='dark']{--setup-bg:#0b1220;--setup-panel:#111c2e;--setup-ink:#e7eef9;--setup-muted:#9aa9bd;--setup-line:#293a54;--setup-blue:#60a5fa}.wrap{max-width:1180px;margin:0 auto;padding:0 20px}body{background:var(--setup-bg);color:var(--setup-ink)}.page-head{margin:0 -20px 16px;padding:16px 20px;border:0;border-bottom:1px solid var(--setup-line);border-radius:0;background:var(--setup-panel);box-shadow:0 4px 14px rgba(15,23,42,.06)}.page-kicker{color:var(--setup-blue)}.page-sub,.setup-hero-copy p,.card-intro,.row small{color:var(--setup-muted)}.setup-hero{display:block;margin:0 -20px 16px;padding:18px 20px;border-radius:0;background:var(--setup-panel);border:0;border-bottom:1px solid var(--setup-line);box-shadow:none}.setup-overview-title{color:var(--setup-blue);margin-bottom:10px}.setup-badges{grid-template-columns:repeat(3,minmax(0,1fr));gap:10px}.setup-badge{border-radius:7px;padding:12px;background:var(--setup-bg);border:1px solid var(--setup-line)}.setup-badge-k{color:var(--setup-muted)}.setup-badge-v{color:var(--setup-ink)}.setup-nav{top:8px;margin:0 0 10px;padding:7px;border-radius:7px;background:var(--setup-panel);border:1px solid var(--setup-line);box-shadow:0 5px 14px rgba(15,23,42,.08)}.setup-nav a{border-radius:6px;background:transparent;color:var(--setup-muted);border-color:transparent;padding:8px 11px}.setup-nav a:hover{transform:none;background:rgba(37,99,235,.1);border-color:rgba(37,99,235,.22);color:var(--setup-blue);box-shadow:none}.setup-actions-top{top:61px;margin:0 0 16px;padding:8px;border-radius:7px;background:var(--setup-panel);border:1px solid var(--setup-line);box-shadow:0 5px 14px rgba(15,23,42,.08)}.card{border-radius:7px;background:var(--setup-panel);border:1px solid var(--setup-line);box-shadow:0 6px 18px rgba(15,23,42,.06)}.card::before{height:2px;background:var(--setup-blue)}.card h3{color:var(--setup-ink)}details.collapse summary{color:var(--setup-ink);padding:7px 0;font-size:1rem}details.collapse summary:after{border-radius:6px;border-color:var(--setup-line);background:var(--setup-bg)}.collapse-body{border-top:1px solid var(--setup-line);padding-top:12px}.row{border-top-color:var(--setup-line)}label{color:var(--setup-ink)}input[type=text],input[type=number],select{background:var(--setup-bg);color:var(--setup-ink);border-color:var(--setup-line);border-radius:6px}.chip{background:var(--setup-bg);border-color:var(--setup-line);color:var(--setup-ink);border-radius:6px}.btn,.btn-alt{border-radius:6px}.btn-alt{background:var(--setup-bg);color:var(--setup-ink);border-color:var(--setup-line)}.save-confirm{margin-left:auto}@media(max-width:760px){.wrap{padding:0 12px}.page-head,.setup-hero{margin-left:-12px;margin-right:-12px;padding-left:12px;padding-right:12px}.setup-badges{grid-template-columns:repeat(2,minmax(0,1fr))}.setup-nav{top:8px}.setup-actions-top{top:58px}.setup-actions-top .save-confirm{margin-left:0}.row{padding-top:10px}}@media(max-width:440px){.setup-badges{grid-template-columns:1fr}.setup-actions-top .btn,.setup-actions-top .btn-alt{flex:1 1 100%}}</style></head><body>"); 
  // Setup-only compact styling uses the dashboard typography, palette and spacing.
  html += F("<style>html,body,input,select,button{font-family:'Segoe UI',Arial,sans-serif}html body{background:var(--setup-bg)}.wrap{margin:12px auto;padding:0 14px}html[data-theme] .page-head{margin:0 0 10px;padding:10px 12px;border-radius:7px;background:#1e3a8a;border:0;color:#fff}html .page-head h1{font-size:1rem;margin:3px 0;color:#fff}html .page-head .page-kicker,html .page-head .page-sub{color:#dbeafe}.page-sub{font-size:.78rem}.page-head-copy{min-width:0}.theme-switch{flex-shrink:0}html[data-theme] .setup-hero{margin:0 0 10px;padding:10px 12px;border:1px solid var(--setup-line);border-radius:7px;background:var(--setup-panel)}.setup-overview-title{margin-bottom:7px;font-size:.68rem}.setup-badges{grid-template-columns:repeat(3,minmax(0,1fr));gap:6px}.setup-badge{padding:7px 9px;min-width:0}.setup-badge-k{font-size:.6rem;letter-spacing:.08em}.setup-badge-v{font-size:.85rem;overflow-wrap:anywhere}#setupForm>.setup-nav{position:static;flex-wrap:wrap;overflow:visible;gap:5px;padding:6px;margin-bottom:8px}.setup-nav a{flex:1 1 auto;padding:7px 9px;font-size:.75rem}#setupForm>.setup-actions-top{position:static;gap:6px;padding:7px;margin-bottom:8px}.setup-actions-top .btn,.setup-actions-top .btn-alt{flex:0 1 auto;padding:8px 10px;font-size:.78rem}#setupForm .card.narrow{box-sizing:border-box;width:100%;min-width:0;margin:0 0 8px;padding:8px 12px;animation:none}#setupForm details.collapse>summary{min-height:30px;padding:4px 0;font-size:.88rem}#setupForm .collapse-body{padding-top:8px}.card-intro{margin:0 0 8px;font-size:.8rem}.row{gap:8px;padding-top:8px;margin:8px 0}.setup-merged{border-top:1px solid var(--setup-line);margin-top:12px;padding-top:8px}.setup-merged h3{font-size:.85rem}#setupForm [id$='-card']{scroll-margin-top:12px}input,select{max-width:100%;box-sizing:border-box}@media(max-width:760px){.wrap{padding:0 10px;margin:10px auto}.setup-badges{grid-template-columns:repeat(2,minmax(0,1fr))}.page-head{gap:8px;flex-wrap:wrap}.setup-actions-top .btn,.setup-actions-top .btn-alt{flex:1 1 auto}.row label{min-width:0}.panel-split{grid-template-columns:minmax(0,1fr)}}</style>");
  flush();

  html += F("<div class='wrap'><div class='page-head'><div class='page-head-copy'><div class='page-kicker'>Controller configuration</div><h1>System Setup</h1><div class='page-sub'>Firmware v");
  html += kFirmwareVersion;
  html += F(" - Start with zones and water source, then set weather, time, display, and hardware pins.</div></div>");
  html += F("<div class='theme-switch'><span>Light</span><label class='switch'><input type='checkbox' id='themeToggle'><span class='slider'></span></label><span>Dark</span></div>");
  html += F("</div>");
  html += F("<div class='setup-hero badges-only'><div class='setup-overview-title'>Current Configuration</div><div class='setup-badges'>");
  html += F("<div class='setup-badge'><div class='setup-badge-k'>Zone Count</div><div class='setup-badge-v'>"); html += String(zonesCount); html += F(" configured</div></div>");
  html += F("<div class='setup-badge'><div class='setup-badge-k'>Display</div><div class='setup-badge-v'>"); html += setupDisplayLabel; html += F("</div></div>");
  html += F("<div class='setup-badge'><div class='setup-badge-k'>Water Source</div><div class='setup-badge-v'>"); html += setupTankLabel; html += F("</div></div>");
  html += F("<div class='setup-badge'><div class='setup-badge-k'>Time Mode</div><div class='setup-badge-v'>"); html += setupTzLabel; html += F("</div></div>");
  html += F("<div class='setup-badge'><div class='setup-badge-k'>Forecast Site</div><div class='setup-badge-v'>"); html += setupWeatherLabel; html += F("</div></div>");
  html += F("<div class='setup-badge'><div class='setup-badge-k'>Forecast Model</div><div class='setup-badge-v'>"); html += setupModelLabel; html += F("</div></div>");
  html += F("</div></div>");
  html += F("<style>#setupForm{display:flex;flex-direction:column}.card.narrow{width:300mm;max-width:100%;align-self:center}#setupForm>.setup-nav{order:0}#setupForm>.setup-actions-top{order:1}#smart-card{order:10}#delays-card{order:20}#weather-card{order:30}#tank-card{order:40}#rain-card{order:50}#timezone-card{order:60}#pins-card{order:70}#i2c-card{order:80}#buttons-card{order:90}#display-card{order:100}#advanced-card{order:110}#mqtt-card{order:120}#ota-card{order:130}</style>");
  html += F("<form id='setupForm' action='/configure' method='POST' novalidate>");
  html += F("<div class='setup-nav'><a href='#smart-card'>Smart Watering</a><a href='#delays-card'>Delays &amp; Pause</a><a href='#weather-card'>Forecast</a><a href='#tank-card'>Water &amp; Tank</a><a href='#rain-card'>Rain Inputs</a><a href='#timezone-card'>Timezone</a><a href='#pins-card'>GPIO</a><a href='#i2c-card'>I2C</a><a href='#buttons-card'>Buttons</a><a href='#display-card'>Display</a><a href='#advanced-card'>TFT Pins</a><a href='#mqtt-card'>MQTT</a><a href='#ota-card'>Firmware</a></div>");
  html += F("<div class='setup-actions-top'><button class='btn' type='submit' id='btn-save-setup'>Save Changes</button><a class='btn-alt' href='/'>Home</a><a class='btn-alt' href='https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/?current=");
  html += kFirmwareVersion;
  html += F("' target='_blank' rel='noopener'>Web Flasher</a><a class='btn-alt' href='/update'>Browser OTA</a><button class='btn-alt' type='button' id='btn-clear-cooldown'>Clear After-Rain Delay</button><button class='btn btn-danger' type='button' onclick=\"if(confirm('Reboot controller now?'))fetch('/reboot',{method:'POST'})\">Reboot</button><span class='save-confirm' id='save-confirm'>Saved</span></div>");

  // Tank (available for all modes; water source switching works with any zone count)
  html += F("<div class='card narrow' id='tank-card'><details class='collapse'><summary>Water Source & Tank</summary><div class='collapse-body'><p class='card-intro'>Control how the controller chooses between tank and mains and where the tank sensor is connected.</p>");
  html += F("<div class='row switchline'><label>Enable Tank</label><input type='checkbox' name='tankEnabled' ");
  html += (tankEnabled ? "checked" : "");
  html += F("><small>Unchecked = ignore tank level and force mains</small></div>");
  html += F("<div id='tankCard'>");

  html += F("<div class='row switchline'><label>Water Source</label>");

  // Auto
  html += F("<label><input type='radio' name='waterMode' value='auto' ");
  if (!justUseTank && !justUseMains) html += F("checked");
  html += F("> Auto (Tank + City Water)</label>");

  // Only Tank
  html += F("<label><input type='radio' name='waterMode' value='tank' ");
  if (justUseTank) html += F("checked");
  html += F("> Only Tank</label>");

  // Only Mains
  html += F("<label><input type='radio' name='waterMode' value='mains' ");
  if (justUseMains) html += F("checked");
  html += F("> Only City Water</label>");

  html += F("<small>");
  html += "Tank/City Water switching (USE BACKFLOW PREVENTION!)";
  html += F("</small></div>");

  html += F("<div class='row'><label>Tank Low Threshold (%)</label>"
            "<input class='in-xs' type='number' min='0' max='100' name='tankThresh' value='");
  html += String(tankLowThresholdPct);
  html += F("'><small>Switch to city water if tank drops below this level</small></div>");

  #if defined(CONFIG_IDF_TARGET_ESP32)
    html += F("<div class='row'><label>Tank Level Sensor GPIO</label><input class='in-xs' type='number' min='-1' max='39' name='tankLevelPin' value='");
    html += String(tankLevelPin); html += F("'><small>-1 disables. ADC1 pin (ESP32: GPIO32-39)</small></div>");
  #else
    html += F("<div class='row'><label>Tank Level Sensor GPIO</label><input class='in-xs' type='number' min='-1' max='20' name='tankLevelPin' value='");
    html += String(tankLevelPin); html += F("'><small>-1 disables. ADC pin (ESP32-S3: GPIO1-20)</small></div>");
  #endif
  html += F("<div class='row'><label></label><a class='btn-alt' href='/tank'>Calibrate Tank</a></div>");
  html += F("</div>"); // end tankCard
  html += F("</div></details></div>");

  flush();
  // Delays & Pause + thresholds
  html += F("<div class='card narrow' id='delays-card'><details class='collapse'><summary>Weather Delays & Pause</summary><div class='collapse-body'><p class='card-intro'>Weather locks and pause controls live here, including After-Rain Delay timing and wind thresholds.</p>");
  html += F("<div class='cols2 panel-split'>");

  // Left column: toggles
  html += F("<div>");
  html += F("<div class='subhead'>Delay Toggles</div><hr class='hr'>");
  html += F("<div class='row switchline'><label>Rain Delay</label><input type='checkbox' name='rainDelay' ");
  html += (rainDelayEnabled ? "checked" : ""); html += F("></div>");
  html += F("<div class='row switchline'><label>Wind Delay</label><input type='checkbox' name='windCancelEnabled' ");
  html += (windDelayEnabled ? "checked" : ""); html += F("></div>");
  html += F("<div class='row switchline'><label>System Pause</label><input type='checkbox' name='pauseEnable' ");
  html += (systemPaused ? "checked" : ""); html += F("><small>Enable System Pause</small></div>");
  html += F("<div class='row' style='gap:8px;flex-wrap:wrap'>");
  html += F("<button class='btn' type='button' id='btn-pause-24'>Pause 24h</button>");
  html += F("<button class='btn' type='button' id='btn-pause-7d'>Pause 7d</button>");
  html += F("<button class='btn' type='button' id='btn-resume'>Unpause</button>");
  html += F("<div class='row'><label>Pause for (hours)</label><input class='in-sm' type='number' min='0' max='720' name='pauseHours' value='");
  time_t nowEp = time(nullptr);
    {
    uint32_t remain = (pauseUntilEpoch > nowEp && systemPaused) ? (pauseUntilEpoch - nowEp) : 0;
    html += String(remain/3600);
  }
  html += F("'></div>");
  html += F("</div>");
  html += F("</div>");

  // Right column: numeric thresholds / timers
  html += F("<div>");
  html += F("<div class='subhead'>Thresholds & Timers</div><hr class='hr'>");
  html += F("<div class='row'><label>Wind Threshold (m/s)</label><input class='in-sm' type='number' step='0.1' min='0' max='50' name='windSpeedThreshold' value='");
  html += String(windSpeedThreshold,1); html += F("'></div>");
  html += F("<div class='row'><label>After-Rain Delay (hours)</label><input class='in-sm' type='number' min='0' max='720' name='rainCooldownHours' value='");
  html += String(rainCooldownMin / 60);
  html += F("'><small>Delay period after rain stops</small></div>");
  html += F("<div class='row'><label>Rain Threshold 24h (mm)</label><input class='in-sm' type='number' min='0' max='200' name='rainThreshold24h' value='");
  html += String(rainThreshold24h_mm);
  html += F("'><small>After-Rain Delay if above threshold</small></div>");
  html += F("</div>");

  html += F("</div>"); // end cols2
  html += F("</div></details></div>"); // end Delays card

  flush();
  // Smart Watering
  html += F("<div class='card narrow' id='smart-card'><details class='collapse'><summary>Smart Watering</summary><div class='collapse-body'><p class='card-intro'>Adjust scheduled and manual runtimes from weather and ground-moisture rules.</p>");
  html += F("<div class='cols2 panel-split'>");
  html += F("<div>");
  html += F("<div class='subhead'>Runtime Rules</div><hr class='hr'>");
  html += F("<div class='row switchline'><label>Smart Watering</label><input type='checkbox' name='smartWatering' ");
  html += (smartWateringEnabled ? "checked" : "");
  html += F("><small>Adjust scheduled and manual runtimes using these rules</small></div>");
  html += F("<div class='row'><label>Cool Below (<span data-temp-unit>"); html += temperatureUnitChar(); html += F("</span>)</label><input class='in-sm' data-smart-temp type='number' step='0.1' name='smartCoolTemp' value='");
  html += String(temperatureForDisplay(smartCoolTempC), 1); html += F("'><small>Runtime Decrease (%)</small><input class='in-sm' type='number' min='-100' max='300' name='smartCoolPct' value='");
  html += String(smartCoolAdjustPct); html += F("'></div>");
  html += F("<div class='row'><label>Hot At/Above (<span data-temp-unit>"); html += temperatureUnitChar(); html += F("</span>)</label><input class='in-sm' data-smart-temp type='number' step='0.1' name='smartHotTemp' value='");
  html += String(temperatureForDisplay(smartHotTempC), 1); html += F("'><small>Runtime Increase (%)</small><input class='in-sm' type='number' min='-100' max='300' name='smartHotPct' value='");
  html += String(smartHotAdjustPct); html += F("'></div>");
  html += F("<div class='row'><label>Very Hot At/Above (<span data-temp-unit>"); html += temperatureUnitChar(); html += F("</span>)</label><input class='in-sm' data-smart-temp type='number' step='0.1' name='smartVeryHotTemp' value='");
  html += String(temperatureForDisplay(smartVeryHotTempC), 1); html += F("'><small>Runtime Increase (%)</small><input class='in-sm' type='number' min='-100' max='300' name='smartVeryHotPct' value='");
  html += String(smartVeryHotAdjustPct); html += F("'></div>");
  html += F("<div class='row'><label>Actual Rain Skip Above (mm)</label><input class='in-sm' type='number' step='0.1' min='0' max='200' name='smartActualRainMm' value='");
  html += String(smartActualRainSkipMm, 1); html += F("'><small>Light-rain adjustment up to this amount (%)</small><input class='in-sm' type='number' min='-100' max='300' name='smartLightRainPct' value='");
  html += String(smartLightRainAdjustPct); html += F("'></div>");
  html += F("<div class='row'><label>Forecast Rain Skip Above (mm)</label><input class='in-sm' type='number' step='0.1' min='0' max='200' name='smartForecastRainMm' value='");
  html += String(smartForecastRainSkipMm, 1); html += F("'><small>Forecast total for the next 24 hours</small></div>");
  html += F("</div>");
  html += F("<div>");
  html += F("<div class='subhead'>Ground Moisture</div><hr class='hr'>");
  html += F("<div class='row switchline'><label>Enable Ground Moisture</label><input type='checkbox' name='moistureProbeEnabled' ");
  html += (moistureProbeEnabled ? "checked" : "");
  html += F("><small>Wet soil can skip scheduled and manual starts</small></div>");
  html += F("<div class='row'><label>Moisture Source</label><select name='moistureSource'>");
  html += F("<option value='probe'"); html += (!moistureUseMeteo ? " selected" : ""); html += F(">Physical soil moisture probe</option>");
  html += F("<option value='meteo'"); html += (moistureUseMeteo ? " selected" : ""); html += F(">Open-Meteo Soil Moisture (0-1 cm)</option>");
  html += F("</select><small>Open-Meteo uses your forecast coordinates and automatic model selection. Modelled volumetric water content: 0.30 m3/m3 = 30%. Review the skip threshold for this source. GPIO and Dry/Wet calibration apply only to the physical probe.</small></div>");
  #if defined(CONFIG_IDF_TARGET_ESP32)
    html += F("<div class='row'><label>Moisture Probe GPIO</label><input class='in-xs' type='number' min='-1' max='39' name='moisturePin' value='");
    html += String(moisturePin); html += F("'><small>ADC1 pin, ESP32 GPIO32-39, or -1 disabled</small></div>");
  #else
    html += F("<div class='row'><label>Moisture Probe GPIO</label><input class='in-xs' type='number' min='-1' max='20' name='moisturePin' value='");
    html += String(moisturePin); html += F("'><small>ADC pin, ESP32-S3 GPIO1-20, or -1 disabled</small></div>");
  #endif
  html += F("<div class='row'><label>Current Moisture</label><div class='chip' id='moistureLiveRaw'>Raw: ");
  if (setupMoistureRaw < 0) html += F("--");
  else html += String(setupMoistureRaw);
  html += F("</div><div class='chip' id='moistureLivePct'>");
  html += moistureUseMeteo ? F("Water volume: ") : F("Wet: ");
  if (setupMoisturePct < 0) html += F("--");
  else { html += String(setupMoisturePct); html += F("%"); }
  html += F("</div><small id='moistureLiveHint'>");
  if (!moistureProbeEnabled) html += F("Enable ground moisture and save to start readings.");
  else if (moistureUseMeteo) html += F("Open-Meteo 0-1 cm volumetric water content; unavailable readings do not trigger a skip.");
  else if (setupMoistureRaw < 0) html += F("No valid ADC reading. Check the GPIO and wiring.");
  else html += F("Use the raw value to set Dry and Wet calibration points.");
  html += F("</small></div>");
  html += F("<div class='row'><label>Moisture Raw Dry</label><input class='in-sm' type='number' min='0' max='4095' name='moistureDryRaw' value='");
  html += String(moistureDryRaw);
  html += F("'><small>Raw ADC value measured in dry soil</small></div>");
  html += F("<div class='row'><label>Moisture Raw Wet</label><input class='in-sm' type='number' min='0' max='4095' name='moistureWetRaw' value='");
  html += String(moistureWetRaw);
  html += F("'><small>Raw ADC value measured in wet soil</small></div>");
  html += F("<div class='row'><label>Moisture Skip Above (%)</label><input class='in-sm' type='number' min='0' max='100' name='moistureSkipPct' value='");
  html += String(moistureSkipThresholdPct);
  html += F("'><small>Skip starts at or above this level. Open-Meteo uses volumetric water percentage.</small></div>");
  html += F("</div>");
  html += F("</div></div></details></div>");

  flush();
  // Physical rain & forecast
  html += F("<div class='card narrow' id='rain-card'><details class='collapse'><summary>Rain Inputs</summary><div class='collapse-body'><p class='card-intro'>Choose which rain sources can stop or delay irrigation. Open-Meteo uses the forecast location below; the physical sensor uses the GPIO here.</p>");
  html += F("<div class='row switchline'><label>Disable Open-Meteo Rain</label><input type='checkbox' name='rainForecastDisabled' ");
  html += (!rainDelayFromForecastEnabled ? "checked" : ""); html += F("><small>Checked = forecast rain will not block starts</small></div>");
  html += F("<div class='row switchline'><label>Use Physical Rain Sensor</label><input type='checkbox' name='rainSensorEnabled' "); html += (rainSensorEnabled?"checked":""); html += F("></div>");
  html += F("<div class='row'><label>Rain Sensor GPIO</label><input class='in-xs' type='number' min='0' max='"); html += String(uiMaxGpio); html += F("' name='rainSensorPin' value='"); html += String(rainSensorPin); html += F("'><small>e.g. 27</small></div>");
  html += F("<div class='row switchline'><label>Invert Sensor</label><input type='checkbox' name='rainSensorInvert' "); html += (rainSensorInvert?"checked":""); html += F("><small>Use if board is NO</small></div>");
  html += F("</div></details></div>");

  flush();
  // Weather
  html += F("<div class='card narrow' id='weather-card'><details class='collapse'><summary>Forecast Location & Model</summary><div class='collapse-body'><p class='card-intro'>Enter the irrigation site coordinates and choose the Open-Meteo forecast model used for dashboard weather, rain delay, wind delay, and Smart Watering.</p>");
  html += F("<div class='row'><label>Open-Meteo</label><a class='btn-alt' id='setupMeteoLink' href='https://open-meteo.com/en/docs?latitude=");
  html += (isfinite(meteoLat) ? String(meteoLat, 6) : String("-35.107600"));
  html += F("&longitude=");
  html += (isfinite(meteoLon) ? String(meteoLon, 6) : String("138.557300"));
  html += F("' target='_blank' rel='noopener'>Open forecast page</a><small>Click to open Open-Meteo using the current coordinates.</small></div>");
  String modelSel = cleanMeteoModel(meteoModel);
  bool modelIsKnown = isKnownMeteoModel(modelSel);
  html += F("<div class='row'><label>Location Name</label><input class='in-wide' type='text' name='meteoLocation' value='"); html += meteoLocation; html += F("'><small>Optional label for UI/logs</small></div>");
  html += F("<div class='row'><label>Latitude</label><input class='in-med' type='text' name='meteoLat' value='"); html += latStr; html += F("'><small>e.g. -35.1076</small></div>");
  html += F("<div class='row'><label>Longitude</label><input class='in-med' type='text' name='meteoLon' value='"); html += lonStr; html += F("'><small>e.g. 138.5573</small></div>");
  html += F("<div class='row'><label>Model</label><select class='in-med' name='meteoModelSelect' id='meteoModelSelect'>");
  html += F("<option value='best_match'");    html += (modelSel == "best_match" ? " selected" : ""); html += F(">Best match</option>");
  html += F("<option value='gfs_seamless'");  html += (modelSel == "gfs_seamless" ? " selected" : ""); html += F(">GFS seamless</option>");
  html += F("<option value='icon_seamless'"); html += (modelSel == "icon_seamless" ? " selected" : ""); html += F(">ICON seamless</option>");
  html += F("<option value='ecmwf_ifs025'");  html += (modelSel == "ecmwf_ifs025" ? " selected" : ""); html += F(">ECMWF IFS 0.25</option>");
  html += F("<option value='meteofrance_seamless'"); html += (modelSel == "meteofrance_seamless" ? " selected" : ""); html += F(">Meteo-France seamless</option>");
  html += F("<option value='jma_seamless'");  html += (modelSel == "jma_seamless" ? " selected" : ""); html += F(">JMA seamless</option>");
  html += F("<option value='cma_grapes_global'"); html += (modelSel == "cma_grapes_global" ? " selected" : ""); html += F(">CMA GRAPES global</option>");
  html += F("<option value='gem_seamless'");  html += (modelSel == "gem_seamless" ? " selected" : ""); html += F(">GEM seamless</option>");
  html += F("<option value='bom_access_global'"); html += (modelSel == "bom_access_global" ? " selected" : ""); html += F(">bom_access_global</option>");
  html += F("<option value='ukmo_seamless'"); html += (modelSel == "ukmo_seamless" ? " selected" : ""); html += F(">UKMO seamless</option>");
  html += F("<option value='icon_global'");   html += (modelSel == "icon_global" ? " selected" : ""); html += F(">ICON global</option>");
  html += F("<option value='icon_eu'");       html += (modelSel == "icon_eu" ? " selected" : ""); html += F(">ICON EU</option>");
  html += F("<option value='custom'");        html += (!modelIsKnown ? " selected" : ""); html += F(">custom</option>");
  html += F("</select><small>Open-Meteo forecast model</small></div>");
  html += F("<div class='row'><label>Temp/Humidity Source</label><select class='in-med' name='climateSource'>");
  html += F("<option value='meteo'"); html += (climateSource == CLIMATE_OPEN_METEO ? " selected" : ""); html += F(">Open-Meteo</option>");
  html += F("<option value='aht20'"); html += (climateSource == CLIMATE_AHT20_I2C ? " selected" : ""); html += F(">AHT20/AHT21 on I2C</option>");
  html += F("<option value='dht22'"); html += (climateSource == CLIMATE_DHT22_GPIO ? " selected" : ""); html += F(">DHT22/AM2302 on GPIO</option>");
  html += F("</select><small>Local sensors replace dashboard and Smart Watering temperature/humidity; rain and wind still use Open-Meteo.</small></div>");
  html += F("<div class='row'><label>Temperature Unit</label><select class='in-sm' name='tempUnit' id='tempUnitSelect'>");
  html += F("<option value='C'"); html += (!tempUseFahrenheit ? " selected" : ""); html += F(">Celsius (C)</option>");
  html += F("<option value='F'"); html += (tempUseFahrenheit ? " selected" : ""); html += F(">Fahrenheit (F)</option></select><small>Dashboard, screen, and Smart Watering thresholds</small></div>");
  html += F("<div class='row'><label>DHT22 GPIO</label><input class='in-xs' type='number' min='-1' max='");
  html += String(uiMaxGpio); html += F("' name='dhtSensorPin' value='"); html += String(dhtSensorPin);
  html += F("'><small>Use -1 when disconnected. AM2302 uses the same DHT22 setting.</small></div>");
  html += F("<div class='row' id='meteoModelCustomRow' style='display:");
  html += (modelIsKnown ? "none" : "flex");
  html += F("'><label>Custom Model</label><input class='in-med' type='text' name='meteoModelCustom' value='");
  html += (modelIsKnown ? "" : modelSel);
  html += F("'><small>Use an Open-Meteo models value, e.g. gfs_seamless</small></div>");
  html += F("<div class='row helptext'><label></label><small>No API key required. Enter your coordinates for Open-Meteo.</small></div>");
  html += F("</div></details></div>");

  flush();
  // Timezone
  html += F("<div class='card narrow' id='timezone-card'><details class='collapse'><summary>Timezone</summary><div class='collapse-body'><p class='card-intro'>Use a POSIX timezone string or a fixed UTC offset so schedules and weather line up with local time.</p>");

  // Mode selector - cleaner row, better labels
  html += F("<div class='row switchline'>");
  html += F("<label>Mode</label>");
  html += F("<div class='inline-options'>");

  html += F("<label class='chip'>");
  html += F("<input type='radio' name='tzMode' value='0' ");
  html += (tzMode==TZ_POSIX ? "checked" : "");
  html += F(" id='tzModePosix'>");
  html += F("<span>POSIX string</span>");
  html += F("</label>");

  html += F("<label class='chip'>");
  html += F("<input type='radio' name='tzMode' value='2' ");
  html += (tzMode==TZ_FIXED ? "checked" : "");
  html += F(">");
  html += F("<span>Fixed offset</span>");
  html += F("</label>");

  html += F("</div></div>"); // end row + inline-options

  // POSIX string input
  html += F("<div class='row'>");
  html += F("<label>Timezone</label>");
  html += F("<input class='in-wide' type='text' name='tzPosix' value='");
  html += tzPosix;
  html += F("' placeholder='ACST-9:30ACDT,M10.1.0,M4.1.0/3'>");
  html += F("</div>");

  // Help text on its own row so it wraps nicely on mobile
  html += F("<div class='row helptext'>");
  html += F("<label></label>");
  html += F("<small>Example POSIX string with DST: ACST-9:30ACDT,M10.1.0,M4.1.0/3</small>");
  html += F("</div>");

  // IANA input + themed select
  html += F("<div class='row'><label>Select Timezone</label>");
  html += F("<div style='flex:1;display:grid;gap:6px'>");
  html += F("<select class='in-med' id='tzIANASelect' name='tzIANA'><option value=''>Select from list</option>");
  addTzOption("UTC", "UTC0");
  addTzOption("Etc/UTC", "UTC0");
  addTzOption("Australia/Adelaide", "ACST-9:30ACDT,M10.1.0,M4.1.0/3");
  addTzOption("Australia/Brisbane", "AEST-10");
  addTzOption("Australia/Broken_Hill", "ACST-9:30ACDT,M10.1.0,M4.1.0/3");
  addTzOption("Australia/Darwin", "ACST-9:30");
  addTzOption("Australia/Hobart", "AEST-10AEDT,M10.1.0,M4.1.0/3");
  addTzOption("Australia/Lord_Howe", "<+1030>-10:30<+11>-11,M10.1.0,M4.1.0");
  addTzOption("Australia/Melbourne", "AEST-10AEDT,M10.1.0,M4.1.0/3");
  addTzOption("Australia/Perth", "AWST-8");
  addTzOption("Australia/Sydney", "AEST-10AEDT,M10.1.0,M4.1.0/3");
  addTzOption("Pacific/Auckland", "NZST-12NZDT,M9.5.0,M4.1.0/3");
  addTzOption("Pacific/Chatham", "<+1245>-12:45<+1345>,M9.5.0/2:45,M4.1.0/3:45");
  addTzOption("Pacific/Fiji", "<+12>-12");
  addTzOption("Pacific/Guam", "ChST-10");
  addTzOption("Pacific/Honolulu", "HST10");
  addTzOption("Pacific/Noumea", "<+11>-11");
  addTzOption("Pacific/Port_Moresby", "<+10>-10");
  addTzOption("Asia/Bangkok", "<+07>-7");
  addTzOption("Asia/Dubai", "<+04>-4");
  addTzOption("Asia/Hong_Kong", "HKT-8");
  addTzOption("Asia/Jakarta", "WIB-7");
  addTzOption("Asia/Kolkata", "IST-5:30");
  addTzOption("Asia/Manila", "PST-8");
  addTzOption("Asia/Seoul", "KST-9");
  addTzOption("Asia/Shanghai", "CST-8");
  addTzOption("Asia/Singapore", "<+08>-8");
  addTzOption("Asia/Tokyo", "JST-9");
  addTzOption("Europe/Amsterdam", "CET-1CEST,M3.5.0,M10.5.0/3");
  addTzOption("Europe/Berlin", "CET-1CEST,M3.5.0,M10.5.0/3");
  addTzOption("Europe/London", "GMT0BST,M3.5.0/1,M10.5.0");
  addTzOption("Europe/Madrid", "CET-1CEST,M3.5.0,M10.5.0/3");
  addTzOption("Europe/Paris", "CET-1CEST,M3.5.0,M10.5.0/3");
  addTzOption("Europe/Rome", "CET-1CEST,M3.5.0,M10.5.0/3");
  addTzOption("America/Anchorage", "AKST9AKDT,M3.2.0,M11.1.0");
  addTzOption("America/Chicago", "CST6CDT,M3.2.0,M11.1.0");
  addTzOption("America/Denver", "MST7MDT,M3.2.0,M11.1.0");
  addTzOption("America/Los_Angeles", "PST8PDT,M3.2.0,M11.1.0");
  addTzOption("America/New_York", "EST5EDT,M3.2.0,M11.1.0");
  addTzOption("America/Phoenix", "MST7");
  addTzOption("America/Toronto", "EST5EDT,M3.2.0,M11.1.0");
  addTzOption("America/Vancouver", "PST8PDT,M3.2.0,M11.1.0");
  addTzOption("Africa/Johannesburg", "SAST-2");
  html += F("</select>");
  html += F("</div>");
  html += F("</div>");

  html += F("<div class='row'><label>Fixed Offset (min)</label><input class='in-sm' type='number' name='tzFixed' value='");
  html += String(tzFixedOffsetMin);
  html += F("'><small>Minutes from UTC</small></div>");
  html += F("</div></details></div>"); // end Timezone card

  flush();
  // Display / Auto backlight
  html += F("<div class='card narrow' id='display-card'><details class='collapse'><summary>Display & Backlight</summary><div class='collapse-body'><p class='card-intro'>Choose the screen type, rotation, and light-sensor backlight behavior.</p>");
  html += F("<div class='row'><label>Display</label><select class='in-med' name='displayType' id='displayTypeSelect'>");
  html += F("<option value='none'");
  html += (!displayEnabled ? " selected" : "");
  html += F(">Disabled</option>");
  html += F("<option value='tft'");
  html += (displayEnabled && displayUseTft ? " selected" : "");
  html += F(">TFT (ST7789)</option>");
  html += F("<option value='oled'");
  html += (displayEnabled && !displayUseTft ? " selected" : "");
  html += F(">OLED (SSD1306)</option></select><small>Applied after reboot</small></div>");
  html += F("<div class='row'><label>Clock Format</label><select class='in-sm' name='clockFormat'>");
  html += F("<option value='24'");
  html += (clockUse24Hour ? " selected" : "");
  html += F(">24 hour</option>");
  html += F("<option value='12'");
  html += (!clockUse24Hour ? " selected" : "");
  html += F(">12 hour</option></select><small>Controls the clock shown on the screen</small></div>");
  html += F("<div class='row' data-tft-only><label>TFT Rotation</label><select class='in-sm' name='tftRotation'>");
  html += F("<option value='0'"); html += (tftRotation == 0 ? " selected" : ""); html += F(">0</option>");
  html += F("<option value='1'"); html += (tftRotation == 1 ? " selected" : ""); html += F(">1</option>");
  html += F("<option value='2'"); html += (tftRotation == 2 ? " selected" : ""); html += F(">2</option>");
  html += F("<option value='3'"); html += (tftRotation == 3 ? " selected" : ""); html += F(">3</option>");
  html += F("</select><small>ST7789 screen orientation (0-3)</small></div>");
  html += F("<div class='row' data-tft-only><label>TFT Size</label><div class='field'><input class='in-xs' type='number' min='120' max='400' name='tftWidth' value='");
  html += String(tftPanelWidth);
  html += F("'><span>x</span><input class='in-xs' type='number' min='120' max='400' name='tftHeight' value='");
  html += String(tftPanelHeight);
  html += F("'></div><small>Saved panel size. Common ST7789 sizes: 170x320, 240x320, 240x240. Applied after reboot.</small></div>");
  html += F("<div class='row switchline' data-tft-only><label>Auto Backlight (LDR)</label><input type='checkbox' name='photoAuto' ");
  html += (photoAutoEnabled ? "checked" : "");
  html += F("><small>Turn off TFT when it is dark</small></div>");
  if (tftBlPin < 0) {
    html += F("<div class='row helptext' data-tft-only><label></label><small>No BL pin set; will use display sleep (backlight stays on).</small></div>");
  }
  int photoRaw = isValidPhotoPin(photoPin) ? analogRead(photoPin) : -1;
  html += F("<div class='row' data-tft-only><label>Photo Raw</label><div class='chip'>");
  if (photoRaw < 0) html += F("--");
  else html += String(photoRaw);
  html += F("</div><small>0-4095 ADC</small></div>");
  #if defined(CONFIG_IDF_TARGET_ESP32)
    html += F("<div class='row' data-tft-only><label>Photo GPIO</label><input class='in-xs' type='number' min='32' max='39' name='photoPin' value='");
    html += String(photoPin);
    html += F("'><small>ADC1 pin (ESP32: GPIO32-39)</small></div>");
  #else
    html += F("<div class='row' data-tft-only><label>Photo-Resistor GPIO</label><input class='in-xs' type='number' min='1' max='40' name='photoPin' value='");
    html += String(photoPin);
    html += F("'><small>ESP32-S3 photo input range: GPIO1-40</small></div>");
  #endif
  html += F("<div class='row' data-tft-only><label>Dark Threshold</label><input class='in-sm' type='number' min='0' max='4095' name='photoThreshold' value='");
  html += String(photoThreshold);
  html += F("'><small>ADC raw value where screen turns off</small></div>");
  html += F("<div class='row switchline' data-tft-only><label>Invert Sensor</label><input type='checkbox' name='photoInvert' ");
  html += (photoInvert ? "checked" : "");
  html += F("><small>Enable if your LDR reads higher when dark</small></div>");
  html += F("</div></details></div>");

  flush();
  // SPI (TFT) config
  html += F("<div class='card narrow' id='advanced-card' data-tft-only><details class='collapse'><summary>TFT Display Pins</summary><div class='collapse-body'><p class='card-intro'>Advanced TFT SPI and backlight pin mapping. These pins are only used when Display is set to TFT.</p>");
  html += F("<div class='row'><label>TFT Size</label><div class='chip'>");
  html += String(tftPanelWidth); html += "x"; html += String(tftPanelHeight);
  html += F("</div><small>Saved display geometry</small></div>");
  html += F("<datalist id='tftPins'>");
  for (int p = 1; p <= uiMaxGpio; ++p) {
    if (!isUnsafeTftPin(p)) { html += F("<option value='"); html += String(p); html += F("'>"); }
  }
  html += F("</datalist>");
  html += F("<datalist id='tftPinsOrNone'><option value='-1'>");
  for (int p = 1; p <= uiMaxGpio; ++p) {
    if (!isUnsafeTftPin(p)) { html += F("<option value='"); html += String(p); html += F("'>"); }
  }
  html += F("</datalist>");
  html += F("<div class='row'><label>SCK</label><input class='in-xs' type='number' min='1' max='");
  html += String(uiMaxGpio);
  html += F("' list='tftPins' name='tftSclk' value='");
  html += String(tftSclkPin); html += F("'></div>");
  html += F("<div class='row'><label>MOSI</label><input class='in-xs' type='number' min='1' max='");
  html += String(uiMaxGpio);
  html += F("' list='tftPins' name='tftMosi' value='");
  html += String(tftMosiPin); html += F("'></div>");
  html += F("<div class='row'><label>CS</label><input class='in-xs' type='number' min='1' max='");
  html += String(uiMaxGpio);
  html += F("' list='tftPins' name='tftCs' value='");
  html += String(tftCsPin); html += F("'></div>");
  html += F("<div class='row'><label>DC</label><input class='in-xs' type='number' min='1' max='");
  html += String(uiMaxGpio);
  html += F("' list='tftPins' name='tftDc' value='");
  html += String(tftDcPin); html += F("'></div>");
  html += F("<div class='row'><label>RST</label><input class='in-xs' type='number' min='-1' max='");
  html += String(uiMaxGpio);
  html += F("' list='tftPinsOrNone' name='tftRst' value='");
  html += String(tftRstPin); html += F("'><small>-1 = not used</small></div>");
  html += F("<div class='row'><label>BL</label><input class='in-xs' type='number' min='-1' max='");
  html += String(uiMaxGpio);
  html += F("' list='tftPinsOrNone' name='tftBl' value='");
  html += String(tftBlPin); html += F("'><small>-1 = not used</small></div>");
  html += F("<div class='row'><label>LCD Brightness (%)</label><input class='in-xs' type='number' id='tftLevel' min='0' max='100' value='100'><button class='btn' type='button' id='btn-tft-bright'>Set</button></div>");
  html += F("<div class='row'><label>Self-Test</label><div class='field'>");
  html += F("<button class='btn-alt' type='button' id='tftSelfTestBtn'>TFT Self-Test</button>");
  html += F("</div></div>");
  html += F("<div class='row helptext'><label></label><small id='tftStatusLine'>Backlight: --</small></div>");
  html += F("<div class='row helptext'><label></label><small>Changing TFT pins requires reboot. ");
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    html += F("ESP32-S3: avoid 0/3/45/46 (strapping) and 26-37 (flash/PSRAM, module-dependent). GPIO19/20 are allowed but will take over the USB pins.");
  #else
    html += F("Use output-capable GPIO 1-");
    html += String(uiMaxGpio);
    html += F(" excluding 19/20 (USB), 0/45/46 (strapping), 9-14 & 35-38 (flash/PSRAM).");
  #endif
  html += F("</small></div>");
  html += F("</div></details></div>");

  flush();
  // I2C config
  html += F("<div class='card narrow' id='i2c-card'><details class='collapse'><summary>I2C Relay Expander Pins</summary><div class='collapse-body'><p class='card-intro'>SDA and SCL for the PCF8574 relay expanders. Leave these alone unless your expander wiring is different.</p>");
  html += F("<div class='row'><label>SDA</label><input class='in-xs' type='number' min='0' max='48' name='i2cSda' value='");
  html += String(i2cSdaPin); html += F("'></div>");
  html += F("<div class='row'><label>SCL</label><input class='in-xs' type='number' min='0' max='48' name='i2cScl' value='");
  html += String(i2cSclPin); html += F("'></div>");
  html += F("<div class='row helptext'><label></label><small>Changing I2C pins requires reboot. Avoid strapping pins and SPI flash/PSRAM pins. KC868 boards commonly use SDA 4 and SCL 15.</small></div>");
  html += F("</div></details></div>");

  // Zones, run mode, and GPIO fallback pins
  html += F("<div class='card narrow' id='pins-card'><details class='collapse'><summary>Zones & Relay GPIO</summary><div class='collapse-body'><p class='card-intro'>Configure zone operation and assign direct ESP32 GPIO outputs for zone, city-water, tank, and power-supply relays.</p>");
  html += F("<div class='row'><label>Zone Count</label><input class='in-xs' type='number' min='1' max='");
  html += String(MAX_ZONES);
  html += F("' name='zonesMode' value='");
  html += String(zonesCount);
  html += F("'><small>Tank/Mains works with any zone count. Up to ");
  html += String(MAX_ZONES);
  html += F(" zones supported.</small></div>");
  html += F("<div class='row switchline'><label>Run Mode</label>");
  html += F("<label><input type='checkbox' name='runConcurrent' "); html += (runZonesConcurrent ? "checked" : "");
  html += F("> Run Zones Together</label><small>Unchecked = one at a time. Ensure the power supply can handle multiple valves before enabling concurrent operation.</small></div>");
  html += F("<div class='grid'>");
  for (uint8_t i=0;i<MAX_ZONES;i++){
    html += F("<div class='row switchline'><label>Zone "); html += String(i+1);
    html += F(" GPIO</label><input class='in-xs' type='number' min='-1' max='"); html += String(uiMaxGpio); html += F("' name='zonePin"); html += String(i);
    html += F("' value='"); html += String(zonePins[i]); html += F("'>");
    html += F("<label class='chip'><input type='checkbox' name='zonePinLow"); html += String(i); html += F("' ");
    html += (zoneGpioActiveLow[i] ? "checked" : "");
    html += F("><span>LOW = ON</span></label><small>-1 = unused</small></div>");
  }
  html += F("<div class='row helptext'><label></label><small>Use output-capable GPIOs only. Avoid boot strapping pins, flash pins, and any pins already used by display, I2C, sensors, or buttons.</small></div>");
  html += F("<div class='row switchline'><label>City Water Relay GPIO</label><input class='in-xs' type='number' min='-1' max='"); html += String(uiMaxGpio); html += F("' name='mainsPin' value='");
  html += String(mainsPin); html += F("'><label class='chip'><input type='checkbox' name='mainsPinLow' ");
  html += (mainsGpioActiveLow ? "checked" : "");
  html += F("><span>LOW = ON</span></label><small>-1 disables. City water relay. Use a check/backflow prevention device.</small></div>");
  html += F("<div class='row switchline'><label>Tank Relay GPIO</label><input class='in-xs' type='number' min='-1' max='"); html += String(uiMaxGpio); html += F("' name='tankPin' value='");
  html += String(tankPin); html += F("'><label class='chip'><input type='checkbox' name='tankPinLow' ");
  html += (tankGpioActiveLow ? "checked" : "");
  html += F("><span>LOW = ON</span></label><small>-1 disables. Tank pump/source relay output.</small></div>");
  html += F("<div class='row switchline'><label>Power Supply Relay GPIO</label><input class='in-xs' type='number' min='-1' max='"); html += String(uiMaxGpio); html += F("' name='powerSupplyPin' value='");
  html += String(powerSupplyPin); html += F("'><label class='chip'><input type='checkbox' name='powerSupplyPinLow' ");
  html += (powerSupplyActiveLow ? "checked" : "");
  html += F("><span>LOW = ON</span></label><small>-1 disables. Turns on while any zone or source relay is active.</small></div>");
  html += F("<div class='row'><label>Legacy Default</label><div class='chip'>");
  html += (gpioActiveLow ? "LOW = ON" : "HIGH = ON");
  html += F("</div><small>Used only when loading older saved configs that do not have per-pin polarity values yet.</small></div>");

  html += F("</div>");
  html += F("<div class='row' style='justify-content:flex-end;gap:8px'>");
  html += F("<button class='btn' type='submit'>Save</button>");
  html += F("</div>");
  html += F("</div></details></div>");

  flush();
  // Manual buttons
  html += F("<div class='card narrow' id='buttons-card'><details class='collapse'><summary>Physical Button Pins</summary><div class='collapse-body'><p class='card-intro'>Optional input pins for local controls. Buttons use INPUT_PULLUP, so wire the button to pull the pin LOW when pressed.</p>");
  html += F("<div class='row switchline'><label>Select Button GPIO</label><input class='in-xs' type='number' min='-1' max='"); html += String(uiMaxGpio); html += F("' name='manualSelectPin' value='");
  html += String(manualSelectPin);
  html += F("'><small>-1 to disable. Uses INPUT_PULLUP; press = LOW.</small></div>");
  html += F("<div class='row switchline'><label>Start/Stop Button GPIO</label><input class='in-xs' type='number' min='-1' max='"); html += String(uiMaxGpio); html += F("' name='manualStartPin' value='");
  html += String(manualStartPin);
  html += F("'><small>Toggles the selected zone on/off.</small></div>");
  html += F("<div class='row'><label>Selected Zone</label><div class='sub'>");
  html += zoneNames[manualSelectedZone % zonesCount];
  html += F(" (cycles with Select button)</div></div>");
  html += F("</div></details></div>");

  

  flush();
  // MQTT
  html += F("<div class='card narrow' id='mqtt-card'><details class='collapse'><summary>MQTT Integration</summary><div class='collapse-body'><p class='card-intro'>Publish controller status and accept simple commands from Home Assistant or another MQTT client.</p>");
  html += F("<div class='row switchline'><label>Enable MQTT</label><input type='checkbox' name='mqttEnabled' "); html += (mqttEnabled ? "checked" : ""); html += F("></div>");
  html += F("<div class='row'><label>Broker Host</label><input class='in-wide' type='text' name='mqttBroker' value='"); html += mqttBroker; html += F("'></div>");
  html += F("<div class='row'><label>Port</label><input class='in-xs' type='number' name='mqttPort' value='"); html += String(mqttPort); html += F("'></div>");
  html += F("<div class='row'><label>User</label><input class='in-med' type='text' name='mqttUser' value='"); html += mqttUser; html += F("'></div>");
  html += F("<div class='row'><label>Password</label><input class='in-med' type='text' name='mqttPass' value='"); html += mqttPass; html += F("'></div>");
  html += F("<div class='row'><label>Base Topic</label><input class='in-med' type='text' name='mqttBase' value='"); html += mqttBase; html += F("'><small>e.g. espirrigation</small></div>");
  html += F("</div></details></div>");

#if ENABLE_OTA
  html += F("<div class='card narrow' id='ota-card'><details class='collapse'><summary>Firmware Updates</summary><div class='collapse-body'><p class='card-intro'>Install a compiled application image directly from a browser, with ArduinoOTA retained as a second network update method.</p>");
  html += F("<div class='row'><label>Browser OTA</label><div class='sub'>");
  html += otaPassword.length() ? F("Password configured") : F("Disabled until a password is set");
  html += F("</div></div><div class='row'><label>OTA Username</label><div class='sub'><code>admin</code></div></div>");
  html += F("<div class='row'><label>New OTA Password</label><input class='in-med' type='password' name='otaPassword' minlength='8' maxlength='64' autocomplete='new-password' placeholder='Leave blank to keep current password'><small>8-64 characters. Used by both Browser OTA and ArduinoOTA. The current password is never displayed.</small></div>");
  html += F("<div class='row'><label>Upload Firmware</label><div><a class='btn-alt' href='/update'>Open Browser OTA</a><small>Save a new password before opening the uploader.</small></div></div>");
  html += F("</div></details></div>");
#endif

  html += F("</form>");

  // quick-actions + timezone JS
  html += F("<script>");
  // Merge related sections without changing control names, IDs or form ownership.
  html += F("[['smart-card','Watering & Delays',['delays-card']],['tank-card','Water & Rain',['rain-card']],['weather-card','Forecast & Time',['timezone-card']],['pins-card','Hardware & Buttons',['i2c-card','buttons-card']],['display-card','Display & TFT Pins',['advanced-card']]].forEach(([id,title,children])=>{const parent=document.getElementById(id);if(!parent)return;parent.querySelector('summary').textContent=title;const body=parent.querySelector('.collapse-body');children.forEach(childId=>{const child=document.getElementById(childId);if(!child)return;const heading=document.createElement('h3');heading.textContent=child.querySelector('summary').textContent;const content=child.querySelector('.collapse-body');const section=document.createElement('section');section.id=childId;section.className='setup-merged';if(child.hasAttribute('data-tft-only'))section.setAttribute('data-tft-only','');section.append(heading);while(content.firstChild)section.append(content.firstChild);body.append(section);child.remove();document.querySelector('.setup-nav a[href=\\\"#'+childId+'\\\"]')?.remove();});const link=document.querySelector('.setup-nav a[href=\\\"#'+id+'\\\"]');if(link)link.textContent=title;});");
  html += F("function revealSetupSection(){const id=location.hash.slice(1);const target=document.getElementById(id);if(!target||!target.closest('#setupForm'))return;let node=target;while(node){if(node.tagName==='DETAILS')node.open=true;node=node.parentElement;}const details=target.querySelector('details');if(details)details.open=true;target.scrollIntoView({block:'start'});}document.querySelector('.setup-nav').addEventListener('click',event=>{const link=event.target.closest('a');if(!link)return;event.preventDefault();history.replaceState(null,'',link.hash);revealSetupSection();});window.addEventListener('hashchange',revealSetupSection);if(location.hash)revealSetupSection();");
  html += F("async function post(path, body){try{await fetch(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});}catch(e){console.error(e)}}");
  html += F("const g=id=>document.getElementById(id);");
  html += F("let setupTempUnit='"); html += temperatureUnitChar(); html += F("';const tempUnitSel=g('tempUnitSelect');");
  html += F("tempUnitSel?.addEventListener('change',()=>{const next=tempUnitSel.value==='F'?'F':'C';if(next===setupTempUnit)return;document.querySelectorAll('[data-smart-temp]').forEach(el=>{const v=parseFloat(el.value);if(Number.isFinite(v))el.value=(next==='F'?(v*9/5+32):(v-32)*5/9).toFixed(1);});document.querySelectorAll('[data-temp-unit]').forEach(el=>el.textContent=next);setupTempUnit=next;});");
  html += F("function addRipple(e){const t=e.currentTarget; if(t.disabled) return; const rect=t.getBoundingClientRect();");
  html += F("const size=Math.max(rect.width,rect.height); const x=(e.clientX|| (rect.left+rect.width/2)) - rect.left - size/2;");
  html += F("const y=(e.clientY|| (rect.top+rect.height/2)) - rect.top - size/2;");
  html += F("const r=document.createElement('span'); r.className='ripple'; r.style.width=size+'px'; r.style.height=size+'px';");
  html += F("r.style.left=x+'px'; r.style.top=y+'px'; const old=t.querySelector('.ripple'); if(old) old.remove(); t.appendChild(r);");
  html += F("setTimeout(()=>{r.remove();},520);}");
  html += F("document.querySelectorAll('.btn,.btn-alt').forEach(el=>{el.addEventListener('pointerdown',addRipple);});");
  html += F("const savedFlag=sessionStorage.getItem('setupSaved');if(savedFlag){sessionStorage.removeItem('setupSaved');const c=g('save-confirm');if(c){c.classList.add('show');setTimeout(()=>c.classList.remove('show'),3500);}}");
  html += F("g('setupForm')?.addEventListener('submit',()=>{sessionStorage.setItem('setupSaved','1');const b=g('btn-save-setup');if(b){b.textContent='Saving...';setTimeout(()=>{b.disabled=true;},0);}});");
  html += F("g('btn-toggle-backlight')?.addEventListener('click',()=>post('/toggleBacklight','x=1'));");
  html += F("g('btn-clear-cooldown')?.addEventListener('click',async()=>{const b=g('btn-clear-cooldown');const old=b?b.textContent:'';if(b){b.disabled=true;b.textContent='Clearing...';}try{await post('/clear_cooldown','x=1');if(b)b.textContent='After-Rain Delay Cleared';setTimeout(()=>location.reload(),350);}catch(e){console.error(e);if(b){b.disabled=false;b.textContent='Clear Failed';setTimeout(()=>{b.textContent=old;},1500);}}});");
  html += F("g('btn-pause-24')?.addEventListener('click',()=>post('/pause','sec=86400'));");
  html += F("g('btn-pause-7d')?.addEventListener('click',()=>post('/pause','sec='+(7*86400)));");
  html += F("g('btn-resume')?.addEventListener('click',()=>post('/resume','x=1'));");
  html += F("g('btn-tft-bright')?.addEventListener('click',async()=>{const v=g('tftLevel'); if(!v) return; let n=parseInt(v.value||'100'); if(isNaN(n)) n=100; n=Math.min(100,Math.max(0,n)); v.value=n; try{await post('/tft_brightness','level='+n);}catch(e){console.error(e)}});");
  html += F("g('tftSelfTestBtn')?.addEventListener('click',async()=>{");
  html += F("  if(!confirm('Run TFT self-test now?')) return;");
  html += F("  try{const r=await fetch('/tft_selftest'); const t=await r.text(); alert(t||'TFT self-test done');}catch(e){alert('TFT self-test failed');}");
  html += F("  loadTftStatus();");
  html += F("});");
  html += F("const displayTypeSel=g('displayTypeSelect');");
  html += F("function syncDisplaySetup(){const isTft=displayTypeSel&&displayTypeSel.value==='tft';document.querySelectorAll('[data-tft-only]').forEach(el=>{el.style.display=isTft?'':'none';});}");
  html += F("displayTypeSel?.addEventListener('change',syncDisplaySetup);syncDisplaySetup();");

  html += F("async function loadTftStatus(){");
  html += F("  const el=g('tftStatusLine'); if(!el) return;");
  html += F("  try{const r=await fetch('/status'); const st=await r.json();");
  html += F("    const pin=st.tftBlPin; const pwm=!!st.tftPwm; const on=!!st.tftBlOn; const disp=!!st.tftDisplayOn;");
  html += F("    const pct=(typeof st.tftBrightnessPct==='number')?st.tftBrightnessPct:0;");
  html += F("    if(typeof pin==='number' && pin>=0){");
  html += F("      const mode=pwm?'PWM':'ON/OFF'; el.textContent='Backlight: '+mode+' '+pct+'% (pin '+pin+') '+(on?'on':'off');");
  html += F("    } else {");
  html += F("      el.textContent='Backlight: display '+(disp?'on':'off');");
  html += F("    }");
  html += F("  }catch(e){ el.textContent='Backlight: --'; }");
  html += F("}");
  html += F("async function loadMoistureStatus(){");
  html += F("  const rawEl=g('moistureLiveRaw'); const pctEl=g('moistureLivePct'); const hint=g('moistureLiveHint');");
  html += F("  if(!rawEl&&!pctEl&&!hint) return;");
  html += F("  try{const r=await fetch('/status'); const st=await r.json();");
  html += F("    const en=!!st.moistureEnabled; const raw=(typeof st.moistureRaw==='number')?st.moistureRaw:-1; const pct=(typeof st.moisturePct==='number')?st.moisturePct:-1;");
  html += F("    if(rawEl) rawEl.textContent='Raw: '+(en&&raw>=0?raw:'--');");
  html += F("    if(pctEl) pctEl.textContent=(st.moistureSource==='meteo'?'Water volume: ':'Wet: ')+(en&&pct>=0?(Math.round(pct)+'%'):'--');");
  html += F("    if(hint){");
  html += F("      if(!en) hint.textContent='Enable ground moisture and save to start readings.';");
  html += F("      else if(st.moistureSource==='meteo') hint.textContent=pct<0?'Open-Meteo soil moisture unavailable. Skip inactive.':'Open-Meteo 0-1 cm volumetric water content.'+(st.moistureSkip?' Skip is active.':'');");
  html += F("      else if(raw<0) hint.textContent='No valid ADC reading. Check the GPIO and wiring.';");
  html += F("      else hint.textContent='Use the raw value to set Dry and Wet calibration points.'+(st.moistureSkip?' Skip is active.':'');");
  html += F("    }");
  html += F("  }catch(e){ if(hint) hint.textContent='Moisture reading unavailable.'; }");
  html += F("}");

  // === Timezone loading from Nayarsystems posix_tz_db with fallback ===
  html += F("const TZ_DB_URL='https://raw.githubusercontent.com/nayarsystems/posix_tz_db/master/zones.json';");
  html += F("const tzInput=document.getElementsByName('tzIANA')[0]||null;");
  html += F("const tzPosixInput=document.getElementsByName('tzPosix')[0]||null;");
  html += F("const tzSel=g('tzIANASelect');");

  // Built-in zones are shown immediately; the remote database can add more.
  html += F("const BUILTIN_ZONES={");
  html += F("'UTC':'UTC0','Etc/UTC':'UTC0',");
  html += F("'Australia/Adelaide':'ACST-9:30ACDT,M10.1.0,M4.1.0/3',");
  html += F("'Australia/Brisbane':'AEST-10',");
  html += F("'Australia/Broken_Hill':'ACST-9:30ACDT,M10.1.0,M4.1.0/3',");
  html += F("'Australia/Darwin':'ACST-9:30',");
  html += F("'Australia/Hobart':'AEST-10AEDT,M10.1.0,M4.1.0/3',");
  html += F("'Australia/Lord_Howe':'<+1030>-10:30<+11>-11,M10.1.0,M4.1.0',");
  html += F("'Australia/Melbourne':'AEST-10AEDT,M10.1.0,M4.1.0/3',");
  html += F("'Australia/Perth':'AWST-8',");
  html += F("'Australia/Sydney':'AEST-10AEDT,M10.1.0,M4.1.0/3',");
  html += F("'Pacific/Auckland':'NZST-12NZDT,M9.5.0,M4.1.0/3',");
  html += F("'Pacific/Chatham':'<+1245>-12:45<+1345>,M9.5.0/2:45,M4.1.0/3:45',");
  html += F("'Pacific/Fiji':'<+12>-12',");
  html += F("'Pacific/Guam':'ChST-10',");
  html += F("'Pacific/Honolulu':'HST10',");
  html += F("'Pacific/Noumea':'<+11>-11',");
  html += F("'Pacific/Port_Moresby':'<+10>-10',");
  html += F("'Asia/Bangkok':'<+07>-7',");
  html += F("'Asia/Dubai':'<+04>-4',");
  html += F("'Asia/Hong_Kong':'HKT-8',");
  html += F("'Asia/Jakarta':'WIB-7',");
  html += F("'Asia/Kolkata':'IST-5:30',");
  html += F("'Asia/Manila':'PST-8',");
  html += F("'Asia/Seoul':'KST-9',");
  html += F("'Asia/Shanghai':'CST-8',");
  html += F("'Asia/Singapore':'<+08>-8',");
  html += F("'Asia/Tokyo':'JST-9',");
  html += F("'Europe/Amsterdam':'CET-1CEST,M3.5.0,M10.5.0/3',");
  html += F("'Europe/Berlin':'CET-1CEST,M3.5.0,M10.5.0/3',");
  html += F("'Europe/London':'GMT0BST,M3.5.0/1,M10.5.0',");
  html += F("'Europe/Madrid':'CET-1CEST,M3.5.0,M10.5.0/3',");
  html += F("'Europe/Paris':'CET-1CEST,M3.5.0,M10.5.0/3',");
  html += F("'Europe/Rome':'CET-1CEST,M3.5.0,M10.5.0/3',");
  html += F("'America/Anchorage':'AKST9AKDT,M3.2.0,M11.1.0',");
  html += F("'America/Chicago':'CST6CDT,M3.2.0,M11.1.0',");
  html += F("'America/Denver':'MST7MDT,M3.2.0,M11.1.0',");
  html += F("'America/Los_Angeles':'PST8PDT,M3.2.0,M11.1.0',");
  html += F("'America/New_York':'EST5EDT,M3.2.0,M11.1.0',");
  html += F("'America/Phoenix':'MST7',");
  html += F("'America/Toronto':'EST5EDT,M3.2.0,M11.1.0',");
  html += F("'America/Vancouver':'PST8PDT,M3.2.0,M11.1.0',");
  html += F("'Africa/Johannesburg':'SAST-2'");
  html += F("};");

  // Helper to populate the select + sync inputs from a map of { IANA: POSIX }
  html += F("function buildTzOptions(zones){");
  html += F(" if(!tzSel||!tzInput) return;");
  html += F(" const current=tzInput.value||'';");
  html += F(" tzSel.innerHTML='<option value=\"\">Select from list</option>';");

  html += F(" const names=Object.keys(zones).sort();");
  html += F(" names.forEach(name=>{");
  html += F("   const opt=document.createElement('option');");
  html += F("   opt.value=name;");
  html += F("   opt.textContent=name;");
  html += F("   opt.dataset.posix=zones[name];");
  html += F("   if(current===name) opt.selected=true;");
  html += F("   tzSel.appendChild(opt);");
  html += F(" });");

  // If tzInput already has a valid value, sync POSIX
  html += F(" if(current && zones[current] && tzPosixInput){");
  html += F("   tzInput.value=current;");
  html += F("   tzPosixInput.value=zones[current];");
  html += F(" }");

  // Try to guess browser zone if empty and available
  html += F(" if(!current && window.Intl && Intl.DateTimeFormat){");
  html += F("   const guess=Intl.DateTimeFormat().resolvedOptions().timeZone;");
  html += F("   if(guess && zones[guess]){");
  html += F("     tzInput.value=guess;");
  html += F("     if(tzPosixInput) tzPosixInput.value=zones[guess];");
  html += F("     for(const opt of tzSel.options){");
  html += F("       if(opt.value===guess){opt.selected=true;break;}");
  html += F("     }");
  html += F("   }");
  html += F(" }");

  // When user picks a zone, update IANA + POSIX fields
  html += F(" tzSel.addEventListener('change',()=>{");
  html += F("   const val=tzSel.value;");
  html += F("   if(!val) return;");
  html += F("   tzInput.value=val;");
  html += F("   const posix=zones[val]||tzSel.options[tzSel.selectedIndex]?.dataset.posix||'';");
  html += F("   if(tzPosixInput && posix) tzPosixInput.value=posix;");
  html += F("   const mode=g('tzModePosix'); if(mode) mode.checked=true;");
  html += F(" });");
  html += F("}"); // end buildTzOptions

  // Tank card always shown now (no toggle)

  html += F("async function loadTimezones(){");
  html += F(" if(!tzSel||!tzInput) return;");

  html += F(" buildTzOptions(BUILTIN_ZONES);");
  html += F(" if(Intl.supportedValuesOf){try{const intlZones=Intl.supportedValuesOf('timeZone');intlZones.forEach(n=>{if(!BUILTIN_ZONES[n])BUILTIN_ZONES[n]='';});buildTzOptions(BUILTIN_ZONES);}catch(e){}}");

  html += F(" try{");
  html += F("   const res=await fetch(TZ_DB_URL);");
  html += F("   if(!res.ok) throw new Error('HTTP '+res.status);");
  html += F("   const zones=await res.json();");
  html += F("   buildTzOptions(Object.assign({},BUILTIN_ZONES,zones));");
  html += F(" }catch(e){");
  html += F("   console.error('tz load failed, using built-in zones',e);");
  html += F(" }");
  html += F("}");

  html += F("function initThemeToggle(){");
  html += F("  let saved=localStorage.getItem('theme');");
  html += F("  if(saved!=='light'&&saved!=='dark'){saved=(window.matchMedia&&window.matchMedia('(prefers-color-scheme: dark)').matches)?'dark':'light';}");
  html += F("  document.documentElement.setAttribute('data-theme', saved);");
  html += F("  const cb=document.getElementById('themeToggle');");
  html += F("  if(cb){cb.checked=(saved==='dark');cb.addEventListener('change',()=>{");
  html += F("    const next=cb.checked?'dark':'light';document.documentElement.setAttribute('data-theme', next);localStorage.setItem('theme', next);");
  html += F("  });}");
  html += F("}");
  html += F("initThemeToggle();");
  html += F("loadTimezones();");
  html += F("loadTftStatus();");
  html += F("loadMoistureStatus(); setInterval(loadMoistureStatus,3000);");
  // === END TZ CODE ===

  html += F("</script>");

  html += F("</div></body></html>");

  flush();
  server.sendContent("");
}

// ---------- Schedule POST (per-zone card or full form) ----------
void handleSubmit() {
  HttpScope _scope;
  auto clampInt = [](int v, int lo, int hi) -> int {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  };

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
      if (server.hasArg("startHour"+String(z)))  startHour[z]  = clampInt(server.arg("startHour"+String(z)).toInt(), 0, 23);
      if (server.hasArg("startMin"+String(z)))   startMin[z]   = clampInt(server.arg("startMin"+String(z)).toInt(), 0, 59);
      if (server.hasArg("startHour2"+String(z))) startHour2[z] = clampInt(server.arg("startHour2"+String(z)).toInt(), 0, 23);
      if (server.hasArg("startMin2"+String(z)))  startMin2[z]  = clampInt(server.arg("startMin2"+String(z)).toInt(), 0, 59);
      if (server.hasArg("durationMin"+String(z))) durationMin[z]=clampInt(server.arg("durationMin"+String(z)).toInt(), 0, 600);
      if (server.hasArg("durationSec"+String(z))) durationSec[z]=clampInt(server.arg("durationSec"+String(z)).toInt(), 0, 59);
      if (server.hasArg("duration2Min"+String(z))) duration2Min[z]=clampInt(server.arg("duration2Min"+String(z)).toInt(), 0, 600);
      if (server.hasArg("duration2Sec"+String(z))) duration2Sec[z]=clampInt(server.arg("duration2Sec"+String(z)).toInt(), 0, 59);
      enableStartTime2[z] = server.hasArg("enableStartTime2"+String(z));
      zoneWindDelayEnabled[z] = server.hasArg("zoneWindDelay"+String(z));
      // Days
      for (int d=0; d<7; d++) days[z][d] = server.hasArg("day"+String(z)+"_"+String(d));

      saveSchedule(); saveConfig();
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
    if (server.hasArg("startHour"+String(z)))  startHour[z]  = clampInt(server.arg("startHour"+String(z)).toInt(), 0, 23);
    if (server.hasArg("startMin"+String(z)))   startMin[z]   = clampInt(server.arg("startMin"+String(z)).toInt(), 0, 59);
    if (server.hasArg("startHour2"+String(z))) startHour2[z] = clampInt(server.arg("startHour2"+String(z)).toInt(), 0, 23);
    if (server.hasArg("startMin2"+String(z)))  startMin2[z]  = clampInt(server.arg("startMin2"+String(z)).toInt(), 0, 59);
    if (server.hasArg("durationMin"+String(z))) durationMin[z]=clampInt(server.arg("durationMin"+String(z)).toInt(), 0, 600);
    if (server.hasArg("durationSec"+String(z))) durationSec[z]=clampInt(server.arg("durationSec"+String(z)).toInt(), 0, 59);
    if (server.hasArg("duration2Min"+String(z))) duration2Min[z]=clampInt(server.arg("duration2Min"+String(z)).toInt(), 0, 600);
    if (server.hasArg("duration2Sec"+String(z))) duration2Sec[z]=clampInt(server.arg("duration2Sec"+String(z)).toInt(), 0, 59);
    enableStartTime2[z] = server.hasArg("enableStartTime2"+String(z));
    zoneWindDelayEnabled[z] = server.hasArg("zoneWindDelay"+String(z));
  }
  saveSchedule(); saveConfig();
  server.sendHeader("Location","/",true); server.send(302,"text/plain","");
}

static String htmlEscape(const String& s) {
  String out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') out += F("&amp;");
    else if (c == '<') out += F("&lt;");
    else if (c == '>') out += F("&gt;");
    else if (c == '"') out += F("&quot;");
    else if (c == '\'') out += F("&#39;");
    else out += c;
  }
  return out;
}

static String compactRunDetailText(const String& temp, const String& hum, const String& wind, const String& cond, String city) {
  city.replace(F("; Current Runtime Factor: "), F("; Factor "));
  city.replace(F("; Ground Moisture: "), F("; Moisture "));
  city.replace(F("No valid reading"), F("No reading"));

  String details;
  details.reserve(96 + city.length());
  details += temp; details += F("C, ");
  details += hum; details += F("%, ");
  details += wind; details += F("m/s, ");
  details += cond;
  if (city.length()) {
    details += F(" @ ");
    details += city;
  }
  return details;
}

static const char* eventLabel(const String& ev, const String& src) {
  if (ev == "START") return "Start";
  if (ev == "STOPPED") return "Stopped";
  if (ev == "CANCELLED") {
    if (src == "RAIN") return "Rain Delay";
    if (src == "SMART") return "Smart Skip";
    if (src == "NO_PIN") return "No Pin";
    if (src == "BLOCKED") return "Blocked";
    return "Cancelled";
  }
  if (ev == "QUEUED") {
    if (src == "WIND") return "Wind Delay";
    if (src == "ACTIVE RUN") return "Queued";
    return "Queued";
  }
  return "Event";
}

static const char* eventChipClass(const String& ev) {
  if (ev == "START") return "start";
  if (ev == "STOPPED") return "stopped";
  if (ev == "CANCELLED") return "cancelled";
  if (ev == "QUEUED") return "queued";
  return "queued";
}

// ---------- Event Log Page ----------
void handleLogPage() {
    // Prepare runtime display strings
    String schedRuntime = formatRuntimeClock(totalScheduledRuntimeSec);
    String manualRuntime = formatRuntimeClock(totalManualRuntimeSec);
  HttpScope _scope;
  File f = LittleFS.open("/events.csv","r");
  const bool hasEventLog = (bool)f;

  String html; html.reserve(18000);
  html += F("<!doctype html><html lang='en' data-theme='light'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<meta name='theme-color' content='#1e40af'><meta name='color-scheme' content='light dark'>");
  html += F("<title>ESP32 Irrigation Events</title>");
  html += F("<style>");
  html += F(":root[data-theme='light']{--bg:#edf3ef;--bg2:#f8fbf8;--glass:rgba(255,255,255,.58);--glass-brd:rgba(122,149,140,.32);--panel:#ffffff;--line:#d3dfd9;");
  html += F("--card:#ffffff;--ink:#14232b;--muted:#5f736f;--primary:#2563eb;--primary-2:#1e40af;--ok:#2563eb;--warn:#c97a1a;--bad:#d9485f;");
  html += F("--chip:#edf4ff;--chip-brd:#c9daf5;--ring:#dcebe5;--ring2:#b4d4cb;--shadow:0 18px 40px rgba(20,47,45,.14)}");
  html += F(":root[data-theme='dark']{--bg:#08111f;--bg2:#0d1d21;--glass:rgba(12,28,31,.62);--glass-brd:rgba(84,123,118,.3);--panel:#0f1e22;--line:#214147;");
  html += F("--card:#0f1e22;--ink:#e6f0ec;--muted:#9ab4ad;--primary:#38bdf8;--primary-2:#0284c7;--ok:#60a5fa;--warn:#f0ae4d;--bad:#ff6b7d;");
  html += F("--chip:#10233d;--chip-brd:#24444a;--ring:#123036;--ring2:#1d5152;--shadow:0 18px 40px rgba(0,0,0,.42)}");
  html += F("*{box-sizing:border-box}html,body{margin:0;padding:0}");
  html += F("html{scroll-behavior:smooth}");
  html += F("body{background:radial-gradient(1200px 600px at 10% -5%,var(--bg2),transparent),radial-gradient(1200px 700px at 100% 0%,var(--ring),transparent),radial-gradient(900px 500px at -10% 80%,var(--ring2),transparent),var(--bg);");
  html += F("color:var(--ink);font-family:'Trebuchet MS','Candara','Segoe UI',sans-serif;line-height:1.35;-webkit-font-smoothing:antialiased;text-rendering:optimizeLegibility}");
  html += F("a{text-decoration:none;color:inherit}");
  html += F(".wrap{max-width:1280px;margin:20px auto;padding:0 16px}");
  html += F(".nav{position:sticky;top:0;z-index:10;padding:10px 12px 12px;background:linear-gradient(180deg,rgba(0,0,0,.25),transparent),var(--primary-2);box-shadow:0 16px 36px rgba(0,0,0,.25)}");
  html += F(".nav .in{max-width:1280px;margin:0 auto;display:flex;align-items:center;justify-content:space-between;gap:12px;color:#fff;flex-wrap:wrap}");
  html += F(".brand{display:flex;align-items:center;gap:8px;font-weight:800;letter-spacing:.2px;font-size:1.12rem}");
  html += F(".brand-copy{display:flex;flex-direction:column;gap:2px}");
  html += F(".brand-title{text-transform:uppercase;letter-spacing:.9px;font-size:1rem;line-height:1}");
  html += F(".brand-sub{font-size:.74rem;font-weight:650;color:rgba(255,255,255,.8);letter-spacing:.08em;text-transform:uppercase}");
  html += F(".dot{width:12px;height:12px;border-radius:999px;background:#93c5fd;box-shadow:0 0 14px #60a5fa}");
  html += F(".nav .meta{display:flex;gap:8px;flex-wrap:wrap;align-items:center;font-weight:650;font-size:.88rem}");
  html += F(".pill{display:inline-flex;align-items:center;gap:6px;background:rgba(255,255,255,.16);border:1px solid rgba(255,255,255,.28);border-radius:999px;padding:7px 12px}");
  html += F(".btn-ghost{background:rgba(255,255,255,.14);border:1px solid rgba(255,255,255,.35);color:#fff;border-radius:10px;padding:8px 14px;font-weight:700;cursor:pointer;font-size:.92rem}");
  html += F(".glass{background:var(--glass);backdrop-filter:blur(12px);-webkit-backdrop-filter:blur(12px);border:1px solid var(--glass-brd);border-radius:20px;box-shadow:var(--shadow)}");
  html += F(".section{padding:18px}");
  html += F(".hero-shell{position:relative;overflow:hidden;padding:clamp(18px,3vw,30px);margin:18px 0}");
  html += F(".hero-shell::before,.hero-shell::after{content:'';position:absolute;border-radius:999px;pointer-events:none;filter:blur(8px)}");
  html += F(".hero-shell::before{width:220px;height:220px;right:-50px;top:-40px;background:radial-gradient(circle,rgba(233,173,73,.2),transparent 68%)}");
  html += F(".hero-shell::after{width:260px;height:260px;left:-90px;bottom:-100px;background:radial-gradient(circle,rgba(56,189,248,.2),transparent 68%)}");
  html += F(".hero-grid{display:grid;grid-template-columns:minmax(0,1.15fr) minmax(320px,.85fr);gap:18px;align-items:stretch}");
  html += F(".hero-copy,.hero-mini-grid{position:relative;z-index:1}");
  html += F(".hero-copy{display:flex;flex-direction:column;justify-content:center;gap:14px}");
  html += F(".hero-kicker{text-transform:uppercase;letter-spacing:.22em;font-size:.72rem;font-weight:800;color:var(--primary)}");
  html += F(".hero-title{margin:0;font-size:clamp(1.95rem,4vw,3rem);line-height:1.02;max-width:11ch}");
  html += F(".hero-text{margin:0;max-width:60ch;color:var(--muted);font-size:1rem}");
  html += F(".hero-actions,.toolbar{display:flex;gap:10px;flex-wrap:wrap}");
  html += F(".hero-mini-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}");
  html += F(".hero-mini{min-height:132px;padding:16px;border-radius:18px;border:1px solid var(--glass-brd);background:linear-gradient(180deg,rgba(255,255,255,.58),rgba(255,255,255,.14));box-shadow:0 14px 30px rgba(19,33,68,.12);display:flex;flex-direction:column;justify-content:space-between;backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px)}");
  html += F(".hero-mini.hero-mini-strong{background:linear-gradient(135deg,rgba(37,99,235,.2),rgba(96,165,250,.1))}");
  html += F(".hero-mini-label{text-transform:uppercase;letter-spacing:.16em;font-size:.74rem;font-weight:800;color:var(--muted)}");
  html += F(".hero-mini-value{font-size:1.72rem;font-weight:800;line-height:1.02;font-variant-numeric:tabular-nums}");
  html += F(".hero-mini-sub{color:var(--muted);font-size:.92rem}");
  html += F(".section-head{display:flex;align-items:flex-end;justify-content:space-between;gap:14px;max-width:1280px;margin:0 auto 14px;padding:0 2px;flex-wrap:wrap}");
  html += F(".section-kicker{text-transform:uppercase;letter-spacing:.18em;font-size:.72rem;font-weight:800;color:var(--primary)}");
  html += F(".section-head h2{margin:4px 0 0;font-size:1.35rem;line-height:1.08}");
  html += F(".section-note{margin:0;max-width:42ch;color:var(--muted);font-size:.92rem}");
  html += F(".card{background:var(--card);border:1px solid var(--glass-brd);border-radius:20px;box-shadow:var(--shadow);padding:18px;transition:transform .12s ease,box-shadow .12s ease}");
  html += F(".card:hover{transform:translateY(-2px);box-shadow:0 16px 34px rgba(0,0,0,.18)}");
  html += F(".btn{display:inline-flex;align-items:center;justify-content:center;gap:8px;background:linear-gradient(180deg,var(--primary),var(--primary-2));color:#fff;border:1px solid rgba(0,0,0,.08);border-radius:13px;padding:11px 16px;font-weight:800;cursor:pointer;box-shadow:0 8px 20px rgba(0,0,0,.22);font-size:1rem}");
  html += F(".btn-secondary{background:transparent;color:var(--ink);border:1px solid var(--line);box-shadow:none}");
  html += F(".btn-danger{background:linear-gradient(180deg,#ef4444,#b91c1c);border-color:rgba(185,28,28,.6)}");
  html += F(".btn-warn{background:linear-gradient(180deg,#d97706,#92400e);border-color:rgba(146,64,14,.55)}");
  html += F(".btn,.btn-ghost,.pill{transition:transform .06s ease,box-shadow .06s ease,filter .06s ease;touch-action:manipulation;position:relative;overflow:hidden}");
  html += F(".btn:hover{filter:brightness(1.05)}");
  html += F(".btn:active:not(:disabled),.btn-ghost:active,.pill:active{transform:translateY(1px);box-shadow:inset 0 2px 6px rgba(0,0,0,.25)}");
  html += F(".btn:focus-visible,.btn-ghost:focus-visible,.pill:focus-visible{outline:2px solid var(--primary);outline-offset:2px}");
  html += F(".toolbar{margin:14px 0 0}");
  html += F(".toolbar form{display:inline-flex;margin:0}");
  html += F(".filter-toggle{display:inline-flex;align-items:center;gap:10px;padding:10px 14px;border-radius:14px;border:1px solid var(--glass-brd);background:rgba(255,255,255,.08);color:var(--ink);font-weight:700}");
  html += F(".filter-toggle input[type=checkbox]{appearance:none;-webkit-appearance:none;width:18px;height:18px;margin:0;flex:0 0 18px;display:inline-grid;place-content:center;cursor:pointer;border:1.6px solid rgba(255,255,255,.7);border-radius:6px;background:rgba(255,255,255,.06)}");
  html += F(".filter-toggle input[type=checkbox]::before{content:'';display:block;width:5px;height:9px;border-right:2px solid #fff;border-bottom:2px solid #fff;transform:rotate(45deg) scale(0);margin-top:-1px;transition:transform .12s ease}");
  html += F(".filter-toggle input[type=checkbox]:checked{border-color:#ffffff;background:transparent;box-shadow:none}");
  html += F(".filter-toggle input[type=checkbox]:checked::before{transform:rotate(45deg) scale(1)}");
  html += F(".filter-toggle input[type=checkbox]:focus-visible{outline:none;border-color:#ffffff;box-shadow:0 0 0 3px rgba(37,99,235,.2)}");
  html += F(".filter-toggle small{display:block;color:var(--muted);font-weight:600;font-size:.78rem}");
  html += F(".table-wrap{overflow:auto;border-radius:18px;border:1px solid var(--glass-brd);background:linear-gradient(180deg,rgba(255,255,255,.18),rgba(255,255,255,.05));box-shadow:var(--shadow)}");
  html += F("table{width:100%;border-collapse:separate;border-spacing:0;min-width:820px}");
  html += F("th,td{padding:12px 14px;border-bottom:1px solid var(--line);font-size:.94rem;text-align:left;white-space:nowrap;vertical-align:top}");
  html += F("th{position:sticky;top:0;z-index:1;background:rgba(30,64,175,.92);color:#fff;text-transform:uppercase;letter-spacing:.08em;font-size:.75rem}");
  html += F("tbody tr{background:rgba(255,255,255,.04)}");
  html += F("tbody tr:nth-child(even){background:rgba(255,255,255,.02)}");
  html += F("tbody tr:hover{background:rgba(37,99,235,.09)}");
  html += F("td:first-child{font-family:'JetBrains Mono','Consolas',monospace;font-size:.82rem}");
  html += F("td:last-child{white-space:normal;min-width:260px;color:var(--muted)}");
  html += F(".event-chip{display:inline-flex;align-items:center;justify-content:center;min-width:88px;padding:6px 10px;border-radius:999px;border:1px solid var(--chip-brd);font-size:.78rem;font-weight:800;letter-spacing:.08em;text-transform:uppercase}");
  html += F(".event-chip.start{background:rgba(37,99,235,.12);border-color:rgba(37,99,235,.3);color:var(--ok)}");
  html += F(".event-chip.stopped{background:rgba(217,72,95,.12);border-color:rgba(217,72,95,.32);color:var(--bad)}");
  html += F(".event-chip.cancelled{background:rgba(245,158,11,.12);border-color:rgba(245,158,11,.34);color:var(--warn)}");
  html += F(".event-chip.queued{background:rgba(56,189,248,.12);border-color:rgba(56,189,248,.32);color:var(--primary)}");
  html += F(".muted{color:var(--muted)}");
  html += F(".empty-state{padding:28px 22px;text-align:center;color:var(--muted)}");
  html += F(".ripple{position:absolute;border-radius:999px;transform:scale(0);background:rgba(255,255,255,.35);animation:ripple .5s ease-out;pointer-events:none}");
  html += F("@keyframes ripple{to{transform:scale(3.2);opacity:0;}}");
  html += F("html[data-theme='dark'] .hero-mini{background:linear-gradient(180deg,rgba(15,30,34,.96),rgba(15,30,34,.9));box-shadow:0 14px 30px rgba(0,0,0,.22)}");
  html += F("html[data-theme='dark'] .hero-mini.hero-mini-strong{background:linear-gradient(135deg,rgba(37,99,235,.18),rgba(16,39,43,.94))}");
  html += F("html[data-theme='dark'] .table-wrap{background:linear-gradient(180deg,rgba(12,28,31,.9),rgba(12,28,31,.72))}");
  html += F("html[data-theme='dark'] tbody tr{background:rgba(255,255,255,.015)}");
  html += F("html[data-theme='light'] .filter-toggle{background:rgba(255,255,255,.82)}");
  html += F("html[data-theme='light'] .filter-toggle input[type=checkbox]{border-color:#8aa59c;background:rgba(255,255,255,.78);box-shadow:inset 0 1px 0 rgba(255,255,255,.75)}");
  html += F("html[data-theme='light'] .filter-toggle input[type=checkbox]::before{border-right-color:#1e40af;border-bottom-color:#1e40af}");
  html += F("html[data-theme='light'] .filter-toggle input[type=checkbox]:checked{border-color:#2563eb;background:#ffffff;box-shadow:0 0 0 3px rgba(37,99,235,.12)}");
  html += F("html[data-theme='light'] th{background:rgba(30,64,175,.94)}");
  html += F("html[data-theme='light'] tbody tr{background:rgba(255,255,255,.8)}");
  html += F("html[data-theme='light'] tbody tr:nth-child(even){background:rgba(248,251,248,.92)}");
  html += F("@media(max-width:980px){.hero-grid{grid-template-columns:1fr}.hero-mini-grid{grid-template-columns:repeat(2,minmax(0,1fr))}}");
  html += F(":root{--shadow:0 10px 28px rgba(15,23,42,.12)}:root[data-theme='light']{--bg:#f3f7fc;--bg2:#fbfdff;--glass:rgba(255,255,255,.78);--glass-brd:#d8e3dd;--line:#dbe6f3;--ring:#e6eefb;--ring2:#f3e8cf}");
  html += F(":root[data-theme='dark']{--bg:#0a1114;--bg2:#111d21;--glass:rgba(16,29,33,.78);--glass-brd:#264148;--card:#101d21;--line:#263f45;--chip:#13282d;--chip-brd:#2b4a51;--shadow:0 12px 30px rgba(0,0,0,.34)}");
  html += F(":root[data-theme='light']{--bg:#f3f7fc;--bg2:#fbfdff;--glass-brd:#d7e2f0;--line:#dbe6f3;--primary:#2563eb;--primary-2:#1e40af;--chip:#edf4ff;--chip-brd:#c9daf5;--ring:#e6eefb;--ring2:#dbeafe}");
  html += F(":root[data-theme='dark']{--bg:#08111f;--bg2:#0d1b2f;--glass:rgba(13,27,47,.8);--glass-brd:#24405f;--card:#0f1d31;--line:#263f60;--primary:#38bdf8;--primary-2:#2563eb;--chip:#10233d;--chip-brd:#294b73}");
  html += F("body{background:linear-gradient(180deg,var(--bg2),var(--bg));letter-spacing:0}.hero-shell::before,.hero-shell::after{display:none}");
  html += F(".nav{background:linear-gradient(180deg,#1e40af,#172554);box-shadow:0 8px 22px rgba(0,0,0,.18)}.wrap{margin:16px auto}");
  html += F(".glass,.card,.hero-mini,.table-wrap,.btn,.btn-ghost,.pill,.filter-toggle,.event-chip{border-radius:8px}");
  html += F(".hero-shell{padding:22px;background:linear-gradient(135deg,var(--glass),rgba(37,99,235,.08));border-color:var(--glass-brd)}.hero-title{font-size:clamp(1.7rem,3vw,2.45rem);max-width:none}");
  html += F(".hero-mini{min-height:112px;background:var(--card);box-shadow:none}.card{box-shadow:0 8px 24px rgba(15,23,42,.08)}.card:hover{transform:none;box-shadow:0 10px 28px rgba(15,23,42,.12)}");
  html += F(".table-wrap{box-shadow:0 8px 24px rgba(15,23,42,.08)}th{background:#1e40af}tbody tr:hover{background:rgba(37,99,235,.09)}td:last-child{min-width:320px}.btn{box-shadow:0 6px 16px rgba(37,99,235,.22)}");
  html += F(".event-chip.queued{background:rgba(37,99,235,.12);border-color:rgba(37,99,235,.32);color:var(--primary)}.filter-toggle input[type=checkbox]:focus-visible{box-shadow:0 0 0 3px rgba(37,99,235,.2)}");
  html += F("html[data-theme='light'] .filter-toggle input[type=checkbox]::before{border-right-color:#1e40af;border-bottom-color:#1e40af}html[data-theme='light'] .filter-toggle input[type=checkbox]:checked{border-color:#2563eb;box-shadow:0 0 0 3px rgba(37,99,235,.12)}");
  html += F("html[data-theme='light'] th{background:#1e40af}");
  html += F("html[data-theme='dark'] .hero-mini{background:#102126}html[data-theme='dark'] .table-wrap{box-shadow:0 12px 30px rgba(0,0,0,.34)}");
  html += F(".hero-shell{padding:16px 18px;margin:12px 0 14px}.hero-grid{grid-template-columns:minmax(0,.78fr) minmax(520px,1.22fr);gap:16px;align-items:center}.hero-copy{gap:10px}.hero-kicker{letter-spacing:.12em;font-size:.66rem}.hero-title{font-size:clamp(1.55rem,2.4vw,2.15rem);line-height:1.08}.hero-text{font-size:.92rem;max-width:48ch}.hero-actions{gap:8px}.hero-actions .btn{padding:9px 13px;font-size:.9rem;min-width:92px}.hero-mini-grid{grid-template-columns:repeat(4,minmax(0,1fr));gap:8px}.hero-mini{min-height:82px;padding:10px 12px;border-left:0;border-top:2px solid var(--line)}.hero-mini.hero-mini-strong{border-top-color:var(--primary)}.hero-mini-label{font-size:.62rem;letter-spacing:.1em}.hero-mini-value{font-size:1.28rem;line-height:1}.hero-mini-sub{font-size:.76rem;line-height:1.22}.section-head{align-items:center;margin-bottom:9px}.section-head h2{font-size:1.16rem}.section-note{font-size:.82rem;max-width:34ch}");
  html += F("@media(max-width:1080px){.hero-grid{grid-template-columns:1fr}.hero-mini-grid{grid-template-columns:repeat(4,minmax(0,1fr))}}@media(max-width:760px){.hero-shell{padding:14px}.hero-mini-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.section-note{display:none}}@media(max-width:440px){.hero-mini-grid{grid-template-columns:1fr}.hero-actions{display:grid;grid-template-columns:1fr}.hero-actions .btn{width:100%}}");
  html += F(".log-head{display:grid;grid-template-columns:minmax(240px,.9fr) minmax(0,1.8fr);gap:14px;align-items:start;margin:12px 0 14px;padding:14px 16px}.log-title{margin:0;font-size:1.55rem;line-height:1.1}.log-copy{margin:5px 0 10px;color:var(--muted);font-size:.9rem;max-width:52ch}.log-actions{display:flex;gap:8px;flex-wrap:wrap}.log-actions .btn{padding:9px 13px;font-size:.9rem;min-width:92px}.metric-strip{display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:8px}.metric{min-width:0;border:1px solid var(--line);border-top:2px solid var(--primary);border-radius:8px;padding:9px 10px;background:var(--panel)}.metric:nth-child(2){border-top-color:#2563eb}.metric:nth-child(3){border-top-color:#dc2626}.metric:nth-child(4){border-top-color:#3b82f6}.metric:nth-child(5){border-top-color:#0891b2}.metric:nth-child(6){border-top-color:#f59e0b}.metric-k{text-transform:uppercase;letter-spacing:.1em;font-size:.58rem;font-weight:850;color:var(--muted);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.metric-v{margin-top:4px;font-size:1.18rem;line-height:1;font-weight:850;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}.metric-s{margin-top:4px;color:var(--muted);font-size:.72rem;line-height:1.18}.metric-latest{grid-column:span 2}.section-head{align-items:center;margin-bottom:9px}.section-head h2{font-size:1.16rem}.section-note{font-size:.82rem;max-width:34ch}");
  html += F("@media(max-width:1080px){.log-head{grid-template-columns:1fr}.metric-strip{grid-template-columns:repeat(3,minmax(0,1fr))}.metric-latest{grid-column:span 1}}@media(max-width:640px){.metric-strip{grid-template-columns:repeat(2,minmax(0,1fr))}.log-head{padding:13px}.section-note{display:none}}@media(max-width:420px){.metric-strip{grid-template-columns:1fr}.log-actions{display:grid;grid-template-columns:1fr}.log-actions .btn{width:100%}}");
  html += F("@media(max-width:720px){.wrap{padding:0 12px}.nav{padding:10px}.hero-mini-grid{grid-template-columns:1fr}.toolbar,.hero-actions{flex-direction:column}.toolbar form{display:flex}.btn,.btn-ghost{width:100%}.section{padding:16px}th,td{padding:10px 12px}}");
  html += F("@media (prefers-reduced-motion: reduce){*{animation:none!important;transition:none!important}}");
  html += F("</style></head><body>");

  int eventCount = 0;
  int startCount = 0;
  int stopCount = 0;
  int manualRunCount = 0;
  int scheduledRunCount = 0;
  String latestTs = "-";
  bool logTruncatedForPage = false;
  size_t logBytesShown = 0;

  if (f) {
    const size_t maxLogPageBytes = 32768;
    const size_t fileSize = f.size();
    logBytesShown = fileSize;
    if (fileSize > maxLogPageBytes) {
      logTruncatedForPage = true;
      logBytesShown = maxLogPageBytes;
      f.seek(fileSize - maxLogPageBytes);
      if (f.position() > 0) f.readStringUntil('\n');
    }
  }

  String eventRows;
  eventRows.reserve(9000);

  while (f && f.available()) {
    String line=f.readStringUntil('\n');
    if (line.length()<5) continue;
    line.trim();

    int i1=line.indexOf(','), i2=line.indexOf(',',i1+1), i3=line.indexOf(',',i2+1), i4=line.indexOf(',',i3+1);
    int i5=line.indexOf(',',i4+1), i6=line.indexOf(',',i5+1), i7=line.indexOf(',',i6+1), i8=line.indexOf(',',i7+1), i9=line.indexOf(',',i8+1);
    if (i1 < 0 || i2 < 0 || i3 < 0 || i4 < 0 || i5 < 0) continue;

    String ts   = line.substring(0,i1);
    String zone = line.substring(i1+1,i2);
    String ev   = line.substring(i2+1,i3);
    String src  = line.substring(i3+1,i4);
    String rd   = line.substring(i4+1,i5);
    if (!isRunLogEvent(ev.c_str())) continue;
    const bool isManualRunEvent = (src == "MANUAL");

    String temp =(i6>i5)?line.substring(i5+1,i6):"";
    String hum  =(i7>i6)?line.substring(i6+1,i7):"";
    String wind =(i8>i7)?line.substring(i7+1,i8):"";
    String cond =(i9>i8)?line.substring(i8+1,i9):"";
    String city =(i9>=0)?line.substring(i9+1):"";

    String details = temp.length() ? compactRunDetailText(temp, hum, wind, cond, city) : "n/a";

    eventCount++;
    latestTs = ts;
    if (ev == "START") startCount++;
    if (ev == "STOPPED") stopCount++;
    if (isManualRunEvent) manualRunCount++;
    else scheduledRunCount++;

    String row;
    row.reserve(320);
    row += F("<tr data-manual='"); row += (isManualRunEvent ? "1" : "0");
    row += F("'><td>"); row += htmlEscape(ts);
    row += F("</td><td>"); row += htmlEscape(zone);
    row += F("</td><td><span class='event-chip ");
    row += eventChipClass(ev);
    row += F("'>");
    row += eventLabel(ev, src);
    row += F("</span>");
    row += F("</td><td>"); row += htmlEscape(src);
    row += F("</td><td>"); row += htmlEscape(rd);
    row += F("</td><td>"); row += htmlEscape(details); row += F("</td></tr>");

    // Keep newest events at the top of the table.
    if (eventRows.length() < 12000) eventRows = row + eventRows;
  }
  if (f) f.close();
  html += F("<nav class='nav'><div class='in'><div class='brand'><span class='dot'></span><div class='brand-copy'><span class='brand-title'>ESP32 Irrigation</span><span class='brand-sub'>Event History</span></div></div><div class='meta'><span id='eventCountBadge' class='pill'>");
  html += String(eventCount);
  html += F(" filtered events</span><button id='themeBtn' class='btn-ghost' title='Toggle theme'>Theme</button></div></div></nav>");
  html += F("<div class='wrap'>");
  html += F("<section class='card log-head'><div><div class='section-kicker'>System History</div><h1 class='log-title'>Irrigation run log</h1><p class='log-copy'>Manual and scheduled starts and stops only. Download CSV keeps the complete log.</p><div class='log-actions'><a class='btn' href='/'>Home</a><a class='btn btn-secondary' href='/setup'>Setup</a>");
  if (hasEventLog) html += F("<a class='btn btn-secondary' href='/download/events.csv'>Download CSV</a>");
  html += F("</div></div><div class='metric-strip'>");
  html += F("<div class='metric metric-latest'><div class='metric-k'>Latest</div><div id='latestEventValue' class='metric-v'>");
  html += latestTs;
  html += F("</div><div class='metric-s'>Latest run entry</div></div>");
  html += F("<div class='metric'><div class='metric-k'>Starts</div><div id='startCountValue' class='metric-v'>");
  html += String(startCount);
  html += F("</div><div class='metric-s'>Run starts</div></div>");
  html += F("<div class='metric'><div class='metric-k'>Stops</div><div id='stopCountValue' class='metric-v'>");
  html += String(stopCount);
  html += F("</div><div class='metric-s'>Run stops</div></div>");
  html += F("<div class='metric'><div class='metric-k'>Manual</div><div class='metric-v'>");
  html += String(manualRunCount);
  html += F("</div><div class='metric-s'>Entries</div></div>");
  html += F("<div class='metric'><div class='metric-k'>Scheduled</div><div class='metric-v'>");
  html += String(scheduledRunCount);
  html += F("</div><div class='metric-s'>Entries</div></div>");
  html += F("<div class='metric'><div class='metric-k'>Run Time</div><div class='metric-v'>");
  html += manualRuntime; html += F("</div><div class='metric-s'>Manual</div></div>");
  html += F("<div class='metric'><div class='metric-k'>Run Time</div><div class='metric-v'>");
  html += schedRuntime; html += F("</div><div class='metric-s'>Scheduled</div></div>");
  html += F("</div></section>");
  html += F("<div class='section-head'><div><div class='section-kicker'>Audit Trail</div><h2>Recent runs</h2></div><p class='section-note'>Newest manual and scheduled starts and stops stay at the top.</p></div>");
  if (logTruncatedForPage) {
    html += F("<section class='card'><p class='section-note'>Showing the most recent ");
    html += String(logBytesShown / 1024);
    html += F(" KB of the event log to keep the page responsive. Use Download CSV for the complete log.</p></section>");
  }
  html += F("<section class='card'><div class='toolbar'><form method='POST' action='/clearevents'><button class='btn btn-danger' type='submit'>Clear Events</button></form><form method='POST' action='/stopall'><button class='btn btn-warn' type='submit'>Stop All</button></form><label class='filter-toggle'><input id='hideManualRuns' type='checkbox'><span>Hide Manual Starts/Stops<small>Filter manual run entries from this view</small></span></label></div><div class='table-wrap'><table><thead><tr>");
  html += F("<th>Time</th><th>Zone</th><th>Event</th><th>Source</th><th>Rain Delay</th><th>Details</th></tr></thead><tbody>");
  html += eventRows;
  html += F("<tr id='events-empty-row'"); html += (eventCount == 0 ? "" : " hidden");
  html += F("><td colspan='6' class='empty-state'>No manual or scheduled run entries are available yet. Starts and stops will appear here after the controller records them.</td></tr>");
  html += F("</tbody></table></div></section></div>");
  html += F("<script>");
  html += F("function applyTheme(t){document.documentElement.setAttribute('data-theme',t==='dark'?'dark':'light');}");
  html += F("(function(){let saved=localStorage.getItem('theme');if(saved!=='dark'&&saved!=='light'){saved=(window.matchMedia&&window.matchMedia('(prefers-color-scheme: dark)').matches)?'dark':'light';localStorage.setItem('theme',saved);}applyTheme(saved);})();");
  html += F("document.getElementById('themeBtn')?.addEventListener('click',()=>{const cur=(document.documentElement.getAttribute('data-theme')==='dark')?'dark':'light';const nxt=(cur==='dark')?'light':'dark';applyTheme(nxt);localStorage.setItem('theme',nxt);});");
  html += F("const manualFilterKey='eventsHideManualRuns';");
  html += F("const hideManualBox=document.getElementById('hideManualRuns');");
  html += F("const rows=[...document.querySelectorAll('tbody tr[data-manual]')];");
  html += F("function refreshEventView(){const hideManual=!!hideManualBox?.checked;let visible=0,starts=0,stops=0;let latest='-';");
  html += F("rows.forEach(row=>{const isManual=row.dataset.manual==='1';const keep=!(hideManual&&isManual);row.hidden=!keep;if(!keep)return;visible++;const cells=row.cells;if(visible===1&&cells[0])latest=cells[0].textContent.trim();const ev=(cells[2]?.textContent||'').trim().toUpperCase();if(ev==='START')starts++;else if(ev==='STOPPED')stops++;});");
  html += F("const empty=document.getElementById('events-empty-row'); if(empty) empty.hidden=visible!==0;");
  html += F("const badge=document.getElementById('eventCountBadge'); if(badge) badge.textContent=visible+' filtered events';");
  html += F("const latestEl=document.getElementById('latestEventValue'); if(latestEl) latestEl.textContent=latest;");
  html += F("const startEl=document.getElementById('startCountValue'); if(startEl) startEl.textContent=String(starts);");
  html += F("const stopEl=document.getElementById('stopCountValue'); if(stopEl) stopEl.textContent=String(stops);}");
  html += F("if(hideManualBox){hideManualBox.checked=localStorage.getItem(manualFilterKey)==='1';hideManualBox.addEventListener('change',()=>{localStorage.setItem(manualFilterKey,hideManualBox.checked?'1':'0');refreshEventView();});refreshEventView();}");
  html += F("function addRipple(e){const t=e.currentTarget; if(t.disabled) return; const rect=t.getBoundingClientRect();");
  html += F("const size=Math.max(rect.width,rect.height); const x=(e.clientX|| (rect.left+rect.width/2)) - rect.left - size/2;");
  html += F("const y=(e.clientY|| (rect.top+rect.height/2)) - rect.top - size/2;");
  html += F("const r=document.createElement('span'); r.className='ripple'; r.style.width=size+'px'; r.style.height=size+'px';");
  html += F("r.style.left=x+'px'; r.style.top=y+'px'; const old=t.querySelector('.ripple'); if(old) old.remove(); t.appendChild(r);");
  html += F("setTimeout(()=>{r.remove();},520);}");
  html += F("document.querySelectorAll('.btn,.btn-ghost').forEach(el=>{el.addEventListener('pointerdown',addRipple);});");
  html += F("</script></body></html>");
  server.send(200,"text/html",html);
}

// ---------- Tank Calibration Page ----------
void handleTankCalibration() {
  HttpScope _scope;
  int raw = readTankRaw();
  int pct = tankPercent();

  String html; html.reserve(4800);
  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>Tank Calibration</title>");
  html += F("<style>:root{color-scheme:light dark;--bg:#f3f7fc;--panel:#fff;--ink:#172033;--muted:#68758a;--line:#dbe4ef;--blue:#2563eb;--blue-dark:#1e3a8a;--warn:#b45309}");
  html += F("@media(prefers-color-scheme:dark){:root{--bg:#0b1220;--panel:#111c2e;--ink:#e7eef9;--muted:#9aa9bd;--line:#293a54;--blue:#60a5fa;--blue-dark:#1d4ed8}}");
  html += F("*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font-family:'Segoe UI',Arial,sans-serif;line-height:1.45}.wrap{max-width:760px;margin:0 auto;padding:20px}");
  html += F(".nav{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap;margin:-20px -20px 24px;padding:16px 20px;background:var(--blue-dark);color:#fff}.nav h1{margin:0;font-size:1.35rem}.links{display:flex;gap:7px}.nav a,.links-bottom a{color:inherit;text-decoration:none;font-weight:700}.nav a{border:1px solid rgba(255,255,255,.3);padding:7px 10px;border-radius:7px;font-size:.9rem}");
  html += F(".card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:18px;box-shadow:0 6px 18px rgba(15,23,42,.08);margin-bottom:14px}.k{color:var(--muted);font-size:.76rem;text-transform:uppercase;letter-spacing:.08em;font-weight:800}.reading{display:flex;align-items:end;justify-content:space-between;gap:16px;margin:5px 0 14px}.reading strong{font-size:2.7rem;line-height:1;font-variant-numeric:tabular-nums}.mono{font-family:Consolas,monospace;font-variant-numeric:tabular-nums}");
  html += F(".meter{height:16px;border-radius:99px;background:var(--line);overflow:hidden}.fill{height:100%;width:");
  html += String(constrain(pct, 0, 100));
  html += F("%;background:var(--blue);transition:width .25s ease}.stats,.steps{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:14px}.stat,.step{border:1px solid var(--line);border-radius:8px;padding:12px}.stat strong{display:block;margin-top:3px;font-size:1.15rem}.steps{grid-template-columns:1fr 1fr;margin-top:0}.step h2{margin:0 0 6px;font-size:1rem}.step p,.note{margin:0 0 12px;color:var(--muted);font-size:.9rem}.btn{width:100%;border:0;border-radius:7px;padding:11px 14px;background:var(--blue);color:#fff;font-weight:800;cursor:pointer;font-size:.95rem}.btn:hover{filter:brightness(1.08)}.warning{color:var(--warn);font-weight:700}.links-bottom{display:flex;justify-content:space-between;gap:12px;margin-top:16px;color:var(--blue)}");
  html += F(".card{position:relative;overflow:hidden}.card:before{content:'';position:absolute;left:0;right:0;top:0;height:3px;background:var(--blue)}.reading strong{color:var(--blue)}.step{background:rgba(37,99,235,.04)}.step:nth-child(2){background:rgba(96,165,250,.06)}.step:nth-child(2) .btn{background:#2563eb}.live-dot{color:#2563eb;font-weight:800}");
  html += F("@media(max-width:600px){.wrap{padding:14px}.nav{margin:-14px -14px 18px;padding:14px}.links{width:100%}.links a{flex:1;text-align:center}.steps,.stats{grid-template-columns:1fr}.reading strong{font-size:2.2rem}}</style></head><body><main class='wrap'>");
  html += F("<header class='nav'><h1>Tank calibration</h1><div class='links'><a href='/'>Home</a><a href='/setup'>Setup</a><a href='/diagnostics'>Diagnostics</a></div></header><section class='card'>");

  html += F("<div class='k'>Live tank reading</div><div class='reading'><strong id='pct'>"); html += String(pct); html += F("%</strong><span class='mono' id='raw'>Raw "); html += String(raw); html += F("</span></div><div class='meter'><div class='fill' id='fill'></div></div><div class='stats'><div class='stat'><span class='k'>Empty point</span><strong class='mono' id='empty'>"); html += String(tankEmptyRaw); html += F("</strong></div><div class='stat'><span class='k'>Full point</span><strong class='mono' id='full'>"); html += String(tankFullRaw); html += F("</strong></div><div class='stat'><span class='k'>Sensor GPIO</span><strong class='mono'>"); html += String(tankLevelPin); html += F("</strong></div></div></section><section class='card'><div class='k'>Two-point calibration</div><p class='note'>Put the tank in the stated physical condition, wait for the reading to settle, then capture that point. The averaged value is saved to flash.</p><div class='steps'><div class='step'><h2>1. Empty tank</h2><p>Tank empty or at the chosen minimum level.</p>");
  if (!isValidAdcPin(tankLevelPin)) {
    #if defined(CONFIG_IDF_TARGET_ESP32)
      html += F("<p style='color:#f59e0b'>Invalid ADC pin. Set tank sensor GPIO to 32-39 (ESP32) in Setup.</p>");
    #else
      html += F("<p style='color:#f59e0b'>Invalid ADC pin. Set tank sensor GPIO to 1-20 (ESP32-S3) in Setup.</p>");
    #endif
  } else {
    html += F("<p>Level: <b>"); html += String(pct); html += F("%</b></p>");
  }
  html += F("<form method='POST' action='/setTankEmpty'><button class='btn' type='submit'>Capture empty point</button></form></div><div class='step'><h2>2. Full tank</h2><p>Tank full at the maximum level you want to measure.</p><form method='POST' action='/setTankFull'><button class='btn' type='submit'>Capture full point</button></form></div></div></section><div class='links-bottom'><a href='/'>Back to dashboard</a><a href='/diagnostics'>Open diagnostics</a></div></main>");
  html += F("<script>");
  html += F("async function update(){try{const r=await fetch('/status',{cache:'no-store'});const s=await r.json();const p=Math.max(0,Math.min(100,Number(s.tankPct)||0));document.getElementById('pct').textContent=p+'%';document.getElementById('fill').style.width=p+'%';document.getElementById('raw').textContent='Raw '+(s.tankRaw==null?'--':s.tankRaw);document.getElementById('empty').textContent=s.tankEmptyRaw==null?'--':s.tankEmptyRaw;document.getElementById('full').textContent=s.tankFullRaw==null?'--':s.tankFullRaw;}catch(e){}}update();setInterval(update,1000);");
  html += F("</script></body></html>");

  server.send(200,"text/html",html);
}

// ---------- Config & Schedule ----------

static String _safeReadLine(File& f) {
  if (!f.available()) return String("");
  String s = f.readStringUntil('\n');
  s.trim();
  return s;
}

void loadConfig() {
  g_configLoadedFromFs = false;
  if (!LittleFS.exists("/config.txt")) return;
  File f = LittleFS.open("/config.txt", "r");
  if (!f) return;
  g_configLoadedFromFs = true;

  String s;
  meteoLat = -35.1076f;
  meteoLon = 138.5573f;
  meteoLocation = "Adelaide";

  // Legacy & core (now: Open-Meteo coords)
  if ((s = _safeReadLine(f)).length()) {
    float v = s.toFloat();
    if (v >= -90.0f && v <= 90.0f) meteoLat = v;
  }
  if ((s = _safeReadLine(f)).length()) {
    float v = s.toFloat();
    if (v >= -180.0f && v <= 180.0f) meteoLon = v;
  }
  if ((s = _safeReadLine(f)).length()) tzOffsetHours = s.toFloat(); // legacy

  if ((s = _safeReadLine(f)).length()) rainDelayEnabled     = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) windSpeedThreshold   = s.toFloat();
  if ((s = _safeReadLine(f)).length()) windDelayEnabled     = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) justUseTank          = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) justUseMains         = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) tankEmptyRaw         = s.toInt();
  if ((s = _safeReadLine(f)).length()) tankFullRaw          = s.toInt();
  if ((s = _safeReadLine(f)).length()) {
    int z = s.toInt();
    if (z < 1) z = 1;
    if (z > (int)MAX_ZONES) z = MAX_ZONES;
    zonesCount = (uint8_t)z;
  }
  if ((s = _safeReadLine(f)).length()) rainSensorEnabled = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) {
    int p = s.toInt();
    if (isValidGpioPin(p)) rainSensorPin = p;
  }
  if ((s = _safeReadLine(f)).length()) rainSensorInvert  = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) {
    int th = s.toInt();
    if (th >= 0 && th <= 100) tankLowThresholdPct = th;
  }
  if ((s = _safeReadLine(f)).length()) tankEnabled = (s.toInt() == 1);

  // GPIO fallback pins
  for (int i = 0; i < MAX_ZONES; i++) {
    if ((s = _safeReadLine(f)).length()) {
      int p = s.toInt();
      if (p == -1 || isValidOutputPin(p)) zonePins[i] = p;
    }
  }
  if ((s = _safeReadLine(f)).length()) {
    int p = s.toInt();
    if (p == -1 || isValidOutputPin(p)) mainsPin = p;
  }
  if ((s = _safeReadLine(f)).length()) {
    int p = s.toInt();
    if (p == -1 || isValidOutputPin(p)) tankPin  = p;
  }
  if (f.available()) {
    if ((s = _safeReadLine(f)).length()) {
      int p = s.toInt();
      manualSelectPin = (p == -1 || isValidGpioPin(p)) ? p : -1;
    }
  }
  if (f.available()) {
    if ((s = _safeReadLine(f)).length()) {
      int p = s.toInt();
      manualStartPin = (p == -1 || isValidGpioPin(p)) ? p : -1;
    }
  }

  // Zone names
  for (int i = 0; i < MAX_ZONES && f.available(); i++) {
    String nm = _safeReadLine(f);
    if (nm.length()) zoneNames[i] = nm;
  }

  // trailing (older "new" ones)
  if (f.available()) { if ((s = _safeReadLine(f)).length()) rainDelayFromForecastEnabled = (s.toInt() == 1); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) systemPaused                 = (s.toInt() == 1); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) pauseUntilEpoch             = (uint32_t)s.toInt(); }

  // Timezone block
  if (f.available()) { if ((s = _safeReadLine(f)).length()) tzMode            = (TZMode)s.toInt(); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) tzPosix          = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) tzIANA           = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) tzFixedOffsetMin = (int16_t)s.toInt(); }

  // NEW trailing: master / cooldown / threshold / run mode / MQTT
  if (f.available()) { if ((s = _safeReadLine(f)).length()) systemMasterEnabled = (s.toInt() == 1); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) rainCooldownMin     = s.toInt(); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) rainThreshold24h_mm = s.toInt(); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) runZonesConcurrent  = (s.toInt() == 1); }

  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttEnabled = (s.toInt() == 1); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttBroker  = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttPort    = (uint16_t)s.toInt(); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttUser    = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttPass    = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttBase    = s; }

  // ? NEW: relay polarity (optional trailing line, backwards compatible)
  if (f.available()) { 
    if ((s = _safeReadLine(f)).length()) 
      relayActiveHigh = (s.toInt() == 1);
  }

  // NEW: persisted GPIO polarity for fallback mode
  if (f.available()) {
    if ((s = _safeReadLine(f)).length())
      gpioActiveLow = (s.toInt() == 1);
  }

  // Default per-output polarity to the legacy global value, then override if present.
  for (int i = 0; i < MAX_ZONES; ++i) zoneGpioActiveLow[i] = gpioActiveLow;
  mainsGpioActiveLow = gpioActiveLow;
  tankGpioActiveLow  = gpioActiveLow;

  // Read the rest of the file so older configs can safely skip the new per-pin polarity block.
  String tail[64];
  int tailCount = 0;
  while (f.available() && tailCount < (int)(sizeof(tail) / sizeof(tail[0]))) {
    tail[tailCount++] = _safeReadLine(f);
  }
  int tailIdx = 0;
  const int perOutputPolarityCount = MAX_ZONES + 2;

  auto nextTail = [&](String& out) -> bool {
    if (tailIdx >= tailCount) return false;
    out = tail[tailIdx++];
    return true;
  };

  bool hasPerOutputPolarityBlock = tailCount >= perOutputPolarityCount;
  for (int i = 0; hasPerOutputPolarityBlock && i < perOutputPolarityCount; ++i) {
    String v = tail[i];
    v.trim();
    hasPerOutputPolarityBlock = (v == "0" || v == "1");
  }

  if (hasPerOutputPolarityBlock) {
    for (int i = 0; i < MAX_ZONES; ++i) {
      if (nextTail(s) && s.length()) zoneGpioActiveLow[i] = (s.toInt() == 1);
    }
    if (nextTail(s) && s.length()) mainsGpioActiveLow = (s.toInt() == 1);
    if (nextTail(s) && s.length()) tankGpioActiveLow  = (s.toInt() == 1);
  }

  // NEW: tank level sensor pin (ADC)
  if (nextTail(s) && s.length()) {
    int p = s.toInt();
    if (p == -1 || isValidAdcPin(p)) tankLevelPin = p;
  }

  // NEW: photoresistor auto-backlight (optional trailing lines)
  if (nextTail(s) && s.length()) photoAutoEnabled = (s.toInt() == 1);
  if (nextTail(s) && s.length()) {
    int p = s.toInt();
    photoPin = isValidPhotoPin(p) ? p : -1;
  }
  if (nextTail(s) && s.length()) photoThreshold = s.toInt();
  if (nextTail(s) && s.length()) photoInvert = (s.toInt() == 1);

  // NEW: TFT SPI pins (optional trailing lines)
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidTftSignalPin(p))   tftSclkPin = p; }
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidTftSignalPin(p))   tftMosiPin = p; }
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidTftSignalPin(p))   tftCsPin   = p; }
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidTftSignalPin(p))   tftDcPin   = p; }
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidOptionalTftPin(p)) tftRstPin  = p; }
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidOptionalTftPin(p)) tftBlPin   = p; }

  // NEW: Open-Meteo location label (optional trailing line)
  if (nextTail(s) && s.length()) meteoLocation = cleanName(s);
  // NEW: Open-Meteo model (optional trailing line)
  if (nextTail(s) && s.length()) meteoModel = cleanMeteoModel(s);
  // NEW: I2C pins (optional trailing lines)
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidGpioPin(p)) i2cSdaPin = p; }
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidGpioPin(p)) i2cSclPin = p; }
  // NEW: display mode (optional trailing line)
  if (nextTail(s) && s.length()) displayUseTft = (s.toInt() == 1);
  // NEW: TFT rotation (optional trailing line)
  if (nextTail(s) && s.length()) {
      int r = s.toInt();
      if (r < 0) r = 0;
      if (r > 3) r = 3;
      tftRotation = (uint8_t)r;
  }
  // NEW: TFT panel width/height (optional trailing lines)
  if (nextTail(s) && s.length()) {
    int v = s.toInt();
    if (isValidTftDimension(v)) tftPanelWidth = (int16_t)v;
  }
  if (nextTail(s) && s.length()) {
    int v = s.toInt();
    if (isValidTftDimension(v)) tftPanelHeight = (int16_t)v;
  }
  // NEW: screen clock format (optional trailing line, 1=24-hour, 0=12-hour)
  if (nextTail(s) && s.length()) clockUse24Hour = (s.toInt() != 0);
  // NEW: switched power supply GPIO (optional trailing lines)
  if (nextTail(s) && s.length()) {
    int p = s.toInt();
    powerSupplyPin = (p == -1 || isValidOutputPin(p)) ? p : -1;
  }
  if (nextTail(s) && s.length()) powerSupplyActiveLow = (s.toInt() == 1);
  // NEW: display enabled flag (optional trailing line, 1=enabled, 0=disabled)
  if (nextTail(s) && s.length()) displayEnabled = (s.toInt() != 0);
  // NEW: smart watering runtime adjustment (optional trailing line)
  if (nextTail(s) && s.length()) smartWateringEnabled = (s.toInt() == 1);
  // Smart watering thresholds and percentage adjustments (optional trailing lines)
  if (nextTail(s) && s.length()) { float v=s.toFloat(); if (v >= -30.0f && v <= 60.0f) smartCoolTempC=v; }
  if (nextTail(s) && s.length()) { int v=s.toInt(); if (v >= -100 && v <= 300) smartCoolAdjustPct=v; }
  if (nextTail(s) && s.length()) { float v=s.toFloat(); if (v >= -30.0f && v <= 60.0f) smartHotTempC=v; }
  if (nextTail(s) && s.length()) { int v=s.toInt(); if (v >= -100 && v <= 300) smartHotAdjustPct=v; }
  if (nextTail(s) && s.length()) { float v=s.toFloat(); if (v >= -30.0f && v <= 60.0f) smartVeryHotTempC=v; }
  if (nextTail(s) && s.length()) { int v=s.toInt(); if (v >= -100 && v <= 300) smartVeryHotAdjustPct=v; }
  if (nextTail(s) && s.length()) { float v=s.toFloat(); if (v >= 0.0f && v <= 200.0f) smartActualRainSkipMm=v; }
  if (nextTail(s) && s.length()) { int v=s.toInt(); if (v >= -100 && v <= 300) smartLightRainAdjustPct=v; }
  if (nextTail(s) && s.length()) { float v=s.toFloat(); if (v >= 0.0f && v <= 200.0f) smartForecastRainSkipMm=v; }
  if (nextTail(s) && s.length()) tempUseFahrenheit = (s.toInt() == 1);
  // NEW: ground moisture probe for Smart Watering (optional trailing lines)
  if (nextTail(s) && s.length()) moistureProbeEnabled = (s.toInt() == 1);
  if (nextTail(s) && s.length()) {
    int p = s.toInt();
    moisturePin = (p == -1 || isValidAdcPin(p)) ? p : -1;
  }
  if (nextTail(s) && s.length()) {
    int v = s.toInt();
    if (v >= 0 && v <= 4095) moistureDryRaw = v;
  }
  if (nextTail(s) && s.length()) {
    int v = s.toInt();
    if (v >= 0 && v <= 4095) moistureWetRaw = v;
  }
  if (nextTail(s) && s.length()) {
    int v = s.toInt();
    if (v >= 0 && v <= 100) moistureSkipThresholdPct = v;
  }
  // NEW: local temperature/humidity source (optional trailing line)
  if (nextTail(s) && s.length()) climateSource = parseClimateSource(s);
  // DHT22/AM2302 data GPIO (optional trailing line)
  if (nextTail(s) && s.length()) {
    int p = s.toInt();
    dhtSensorPin = (p == -1 || isValidOutputPin(p)) ? p : -1;
  }
  // Password shared by Browser OTA and ArduinoOTA (optional trailing line).
  if (nextTail(s)) {
    s.trim();
    s.replace("\r", "");
    s.replace("\n", "");
    if (s.length() >= 8 && s.length() <= 64) otaPassword = s;
  }
  if (nextTail(s) && s.length()) moistureUseMeteo = (s == "meteo");

  if (!(smartCoolTempC < smartHotTempC && smartHotTempC < smartVeryHotTempC)) {
    smartCoolTempC = 18.0f;
    smartHotTempC = 28.0f;
    smartVeryHotTempC = 34.0f;
  }

  f.close();

  // Derive hours from minutes (for UI & cooldown logic)
  if (rainCooldownMin < 0) rainCooldownMin = 0;
  rainCooldownHours = (uint8_t)(rainCooldownMin / 60);
}

void saveConfig() {
  File f = LittleFS.open("/config.txt", "w");
  if (!f) return;
  g_configLoadedFromFs = true;

  if (zonesCount < 1) zonesCount = 1;
  if (zonesCount > MAX_ZONES) zonesCount = MAX_ZONES;

  // Core / legacy order (now: Open-Meteo coords)
  if (isValidLatLon(meteoLat, meteoLon)) {
    f.println(String(meteoLat, 6));
    f.println(String(meteoLon, 6));
  } else {
    f.println("");
    f.println("");
  }
  f.println(String(tzOffsetHours, 3));

  f.println(rainDelayEnabled ? 1 : 0);
  f.println(String(windSpeedThreshold, 3));
  f.println(windDelayEnabled ? 1 : 0);
  f.println(justUseTank ? 1 : 0);
  f.println(justUseMains ? 1 : 0);
  f.println(tankEmptyRaw);
  f.println(tankFullRaw);
  f.println(zonesCount);
  f.println(rainSensorEnabled ? 1 : 0);
  f.println(rainSensorPin);
  f.println(rainSensorInvert ? 1 : 0);
  f.println(tankLowThresholdPct);
  f.println(tankEnabled ? 1 : 0);

  // ? REMOVE this line from here (it breaks the alignment!)
  // f.println(relayActiveHigh ? 1 : 0);

  for (int i = 0; i < MAX_ZONES; i++) f.println(zonePins[i]);
  f.println(mainsPin);
  f.println(tankPin);
  f.println(manualSelectPin);
  f.println(manualStartPin);

  for (int i = 0; i < MAX_ZONES; i++) f.println(zoneNames[i]);

  // trailing
  f.println(rainDelayFromForecastEnabled ? 1 : 0);
  f.println(systemPaused ? 1 : 0);
  f.println(pauseUntilEpoch);

  // Timezone
  f.println((int)tzMode);
  f.println(tzPosix);
  f.println(tzIANA);
  f.println(tzFixedOffsetMin);

  // Master / cooldown / threshold / run mode / MQTT
  // keep minutes & hours in sync
  rainCooldownMin   = (int)rainCooldownHours * 60;
  if (rainCooldownMin < 0) rainCooldownMin = 0;

  f.println(systemMasterEnabled ? 1 : 0);
  f.println(rainCooldownMin);
  f.println(rainThreshold24h_mm);
  f.println(runZonesConcurrent ? 1 : 0);

  f.println(mqttEnabled ? 1 : 0);
  f.println(mqttBroker);
  f.println(mqttPort);
  f.println(mqttUser);
  f.println(mqttPass);
  f.println(mqttBase);

  // ? NEW: relay polarity written as trailing line to match loadConfig()
  f.println(relayActiveHigh ? 1 : 0);
  // NEW: persist GPIO polarity for fallback mode
  f.println(gpioActiveLow ? 1 : 0);
  // NEW: persist per-output GPIO polarity for fallback mode
  for (int i = 0; i < MAX_ZONES; ++i) f.println(zoneGpioActiveLow[i] ? 1 : 0);
  f.println(mainsGpioActiveLow ? 1 : 0);
  f.println(tankGpioActiveLow ? 1 : 0);
  // NEW: tank level sensor pin (ADC)
  f.println(tankLevelPin);
  // NEW: photoresistor auto-backlight
  f.println(photoAutoEnabled ? 1 : 0);
  f.println(photoPin);
  f.println(photoThreshold);
  f.println(photoInvert ? 1 : 0);
  // NEW: TFT SPI pins
  f.println(tftSclkPin);
  f.println(tftMosiPin);
  f.println(tftCsPin);
  f.println(tftDcPin);
  f.println(tftRstPin);
  f.println(tftBlPin);
  // NEW: Open-Meteo location label
  f.println(cleanName(meteoLocation));
  // NEW: Open-Meteo model
  f.println(cleanMeteoModel(meteoModel));
  // NEW: I2C pins
  f.println(i2cSdaPin);
  f.println(i2cSclPin);
  // NEW: display mode (1=TFT, 0=OLED)
  f.println(displayUseTft ? 1 : 0);
  // NEW: TFT rotation (0..3)
  f.println((int)tftRotation);
  // NEW: TFT panel width/height
  f.println((int)tftPanelWidth);
  f.println((int)tftPanelHeight);
  // NEW: screen clock format (1=24-hour, 0=12-hour)
  f.println(clockUse24Hour ? 1 : 0);
  // NEW: switched power supply GPIO
  f.println(powerSupplyPin);
  f.println(powerSupplyActiveLow ? 1 : 0);
  // NEW: display enabled flag (1=enabled, 0=disabled)
  f.println(displayEnabled ? 1 : 0);
  // NEW: smart watering runtime adjustment
  f.println(smartWateringEnabled ? 1 : 0);
  f.println(String(smartCoolTempC, 2));
  f.println(smartCoolAdjustPct);
  f.println(String(smartHotTempC, 2));
  f.println(smartHotAdjustPct);
  f.println(String(smartVeryHotTempC, 2));
  f.println(smartVeryHotAdjustPct);
  f.println(String(smartActualRainSkipMm, 2));
  f.println(smartLightRainAdjustPct);
  f.println(String(smartForecastRainSkipMm, 2));
  f.println(tempUseFahrenheit ? 1 : 0);
  // NEW: ground moisture probe for Smart Watering
  f.println(moistureProbeEnabled ? 1 : 0);
  f.println(moisturePin);
  f.println(moistureDryRaw);
  f.println(moistureWetRaw);
  f.println(moistureSkipThresholdPct);
  // NEW: local temperature/humidity source
  f.println(climateSourceValue());
  f.println(dhtSensorPin);
  f.println(otaPassword);
  f.println(moistureUseMeteo ? "meteo" : "probe");

  f.close();
}

void loadSchedule() {
  File f = LittleFS.open("/schedule.txt","r");
  if (!f) return;

  for (int i=0; i<MAX_ZONES; i++) {
    String line=f.readStringUntil('\n'); line.trim(); if (!line.length()) continue;

    int idx=0; int tcount=0; int tokens[32];
    while (idx < (int)line.length() && tcount < 32) {
      int nidx=line.indexOf(',', idx); if (nidx<0) nidx=line.length();
      String sv=line.substring(idx,nidx); sv.trim();
      tokens[tcount++] = sv.toInt();
      idx = (nidx<(int)line.length()) ? nidx+1 : nidx;
    }
    auto tok=[&](int k,int def)->int{ return (k < tcount) ? tokens[k] : def; };

    startHour[i]    = tok(0, startHour[i]);
    startMin[i]     = tok(1, startMin[i]);
    startHour2[i]   = tok(2, startHour2[i]);
    startMin2[i]    = tok(3, startMin2[i]);
    durationMin[i]  = tok(4, durationMin[i]);
    durationSec[i]  = tok(5, durationSec[i]);
    duration2Min[i] = tok(6, durationMin[i]);
    duration2Sec[i] = tok(7, durationSec[i]);

    int enIdx = (tcount >= 9) ? 8 : 6; // compatibility when duration2 fields absent
    enableStartTime2[i] = (tok(enIdx, enableStartTime2[i]) == 1);

    zoneWindDelayEnabled[i] = tok(16, 1) == 1;
    int dayStart = enIdx + 1;
    for (int d=0; d<7; d++) {
      days[i][d] = (tok(dayStart + d, days[i][d]) == 1);
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
    f.print(duration2Min[i]);f.print(',');
    f.print(duration2Sec[i]);f.print(',');
    f.print(enableStartTime2[i] ? '1' : '0');
    for (int d=0; d<7; d++){ f.print(','); f.print(days[i][d] ? '1' : '0'); }
    f.print(','); f.print(zoneWindDelayEnabled[i] ? '1' : '0');
    f.println();
  }
  f.close();
}

void handleConfigure() {
  HttpScope _scope;
  String displayCfgErr;
  String requestedOtaPassword;
  bool changeOtaPassword = false;

#if ENABLE_OTA
  if (server.hasArg("otaPassword")) {
    requestedOtaPassword = server.arg("otaPassword");
    requestedOtaPassword.trim();
    requestedOtaPassword.replace("\r", "");
    requestedOtaPassword.replace("\n", "");
    if (requestedOtaPassword.length()) {
      if (requestedOtaPassword.length() < 8 || requestedOtaPassword.length() > 64) {
        displayCfgErr += "OTA password must contain 8 to 64 characters.\n";
      } else if (otaPassword.length() && !otaHttpAuthenticate()) {
        return;
      } else {
        changeOtaPassword = true;
      }
    }
  }
#endif

    // NEW: snapshot current values before applying POST changes
  float  oldLat    = meteoLat;
  float  oldLon    = meteoLon;
  String oldLoc    = meteoLocation;
  String oldModel  = meteoModel;
  LocalClimateSource oldClimateSource = climateSource;
  int oldDhtSensorPin = dhtSensorPin;
  int oldTftSclk = tftSclkPin;
  int oldTftMosi = tftMosiPin;
  int oldTftCs   = tftCsPin;
  int oldTftDc   = tftDcPin;
  int oldTftRst  = tftRstPin;
  int oldTftBl   = tftBlPin;
  int oldTftWidth = tftPanelWidth;
  int oldTftHeight = tftPanelHeight;
  int oldTftRotation = tftRotation;
  int oldI2cSda  = i2cSdaPin;
  int oldI2cScl  = i2cSclPin;
  bool oldDisplayEnabled = displayEnabled;
  bool oldDisplayUseTft = displayUseTft;
  bool oldClockUse24Hour = clockUse24Hour;
  bool oldTempUseFahrenheit = tempUseFahrenheit;

  // Weather (Open-Meteo)
  if (server.hasArg("meteoLocation")) {
    meteoLocation = cleanName(server.arg("meteoLocation"));
  }
  if (server.hasArg("meteoModelSelect")) {
    String sel = cleanMeteoModel(server.arg("meteoModelSelect"));
    if (sel == "custom" && server.hasArg("meteoModelCustom")) {
      meteoModel = cleanMeteoModel(server.arg("meteoModelCustom"));
    } else {
      meteoModel = cleanMeteoModel(sel);
    }
  } else if (server.hasArg("meteoModel")) {
    meteoModel = cleanMeteoModel(server.arg("meteoModel"));
  }
  if (server.hasArg("meteoLat")) {
    String s = server.arg("meteoLat"); s.trim();
    if (!s.length()) {
      meteoLat = NAN;
    } else {
      float v = s.toFloat();
      if (v >= -90.0f && v <= 90.0f) meteoLat = v;
    }
  }
  if (server.hasArg("meteoLon")) {
    String s = server.arg("meteoLon"); s.trim();
    if (!s.length()) {
      meteoLon = NAN;
    } else {
      float v = s.toFloat();
      if (v >= -180.0f && v <= 180.0f) meteoLon = v;
    }
  }
  // Fallback: allow "lat,lon" in the latitude field
  if (!isValidLatLon(meteoLat, meteoLon) && server.hasArg("meteoLat")) {
    float la = NAN, lo = NAN;
    if (parseLatLon(server.arg("meteoLat"), la, lo)) {
      meteoLat = la;
      meteoLon = lo;
    }
  }
  if (server.hasArg("climateSource")) {
    climateSource = parseClimateSource(server.arg("climateSource"));
  }
  if (server.hasArg("dhtSensorPin")) {
    int p = -1;
    if (parseRequiredIntArg(server.arg("dhtSensorPin"), p, displayCfgErr, F("DHT22 GPIO"))) {
      if (p == -1 || isValidOutputPin(p)) dhtSensorPin = p;
      else displayCfgErr += "DHT22 GPIO invalid or not output-capable: " + String(p) + "\n";
    }
  }

  // Zones mode (1..MAX_ZONES, 4 keeps tank/mains behaviour)
  if (server.hasArg("zonesMode")) {
    int z = server.arg("zonesMode").toInt();
    if (z < 1) z = 1;
    if (z > (int)MAX_ZONES) z = MAX_ZONES;
    zonesCount = (uint8_t)z;
  }

  // Run mode (sequential vs concurrent)
  runZonesConcurrent = server.hasArg("runConcurrent");
  smartWateringEnabled = server.hasArg("smartWatering");

  const bool submittedFahrenheit = server.hasArg("tempUnit")
                                    ? (server.arg("tempUnit") == "F")
                                    : tempUseFahrenheit;
  auto submittedTempToC = [&](float value) -> float {
    return submittedFahrenheit ? (value - 32.0f) * 5.0f / 9.0f : value;
  };
  float coolTemp = server.hasArg("smartCoolTemp") ? submittedTempToC(server.arg("smartCoolTemp").toFloat()) : smartCoolTempC;
  float hotTemp = server.hasArg("smartHotTemp") ? submittedTempToC(server.arg("smartHotTemp").toFloat()) : smartHotTempC;
  float veryHotTemp = server.hasArg("smartVeryHotTemp") ? submittedTempToC(server.arg("smartVeryHotTemp").toFloat()) : smartVeryHotTempC;
  if (coolTemp >= -30.0f && veryHotTemp <= 60.0f && coolTemp < hotTemp && hotTemp < veryHotTemp) {
    smartCoolTempC = coolTemp;
    smartHotTempC = hotTemp;
    smartVeryHotTempC = veryHotTemp;
  }
  if (server.hasArg("smartCoolPct")) {
    int v = server.arg("smartCoolPct").toInt();
    if (v >= -100 && v <= 300) smartCoolAdjustPct = v;
  }
  if (server.hasArg("smartHotPct")) {
    int v = server.arg("smartHotPct").toInt();
    if (v >= -100 && v <= 300) smartHotAdjustPct = v;
  }
  if (server.hasArg("smartVeryHotPct")) {
    int v = server.arg("smartVeryHotPct").toInt();
    if (v >= -100 && v <= 300) smartVeryHotAdjustPct = v;
  }
  if (server.hasArg("smartActualRainMm")) {
    float v = server.arg("smartActualRainMm").toFloat();
    if (v >= 0.0f && v <= 200.0f) smartActualRainSkipMm = v;
  }
  if (server.hasArg("smartLightRainPct")) {
    int v = server.arg("smartLightRainPct").toInt();
    if (v >= -100 && v <= 300) smartLightRainAdjustPct = v;
  }
  if (server.hasArg("smartForecastRainMm")) {
    float v = server.arg("smartForecastRainMm").toFloat();
    if (v >= 0.0f && v <= 200.0f) smartForecastRainSkipMm = v;
  }
  moistureProbeEnabled = server.hasArg("moistureProbeEnabled");
  if (server.hasArg("moistureSource")) moistureUseMeteo = (server.arg("moistureSource") == "meteo");
  // Settings may change coordinates or source; discard the old site reading.
  meteoMoisturePct = -1;
  meteoMoistureAttempted = false;
  if (server.hasArg("moisturePin")) {
    int p = server.arg("moisturePin").toInt();
    moisturePin = (p == -1 || isValidAdcPin(p)) ? p : -1;
  }
  if (server.hasArg("moistureDryRaw")) {
    int v = server.arg("moistureDryRaw").toInt();
    if (v >= 0 && v <= 4095) moistureDryRaw = v;
  }
  if (server.hasArg("moistureWetRaw")) {
    int v = server.arg("moistureWetRaw").toInt();
    if (v >= 0 && v <= 4095) moistureWetRaw = v;
  }
  if (server.hasArg("moistureSkipPct")) {
    int v = server.arg("moistureSkipPct").toInt();
    if (v >= 0 && v <= 100) moistureSkipThresholdPct = v;
  }

  // Rain forecast gate
  rainDelayFromForecastEnabled = !server.hasArg("rainForecastDisabled");

  // Delay toggles
  rainDelayEnabled = server.hasArg("rainDelay");
  windDelayEnabled = server.hasArg("windCancelEnabled");

  // Physical rain sensor
  rainSensorEnabled = server.hasArg("rainSensorEnabled");
  if (server.hasArg("rainSensorPin")) {
    int pin = server.arg("rainSensorPin").toInt();
    if (isValidGpioPin(pin)) rainSensorPin = pin;
  }
  rainSensorInvert = server.hasArg("rainSensorInvert");

  // Wind threshold
  if (server.hasArg("windSpeedThreshold")) {
    windSpeedThreshold = server.arg("windSpeedThreshold").toFloat();
    if (windSpeedThreshold < 0) windSpeedThreshold = 0;
  }

  // Rain cooldown (hours ? minutes + uint8)
  if (server.hasArg("rainCooldownHours")) {
    int h = server.arg("rainCooldownHours").toInt();
    if (h < 0) h = 0;
    rainCooldownHours = (uint8_t)h;
    rainCooldownMin   = h * 60;
  }

  // 24h rain threshold (mm)
  if (server.hasArg("rainThreshold24h")) {
    int mm = server.arg("rainThreshold24h").toInt();
    if (mm < 0) mm = 0;
    rainThreshold24h_mm = mm;
  }

  // Pause handling
  bool resumeNow = server.hasArg("resumeNow");
  int pauseHours = server.hasArg("pauseHours") ? server.arg("pauseHours").toInt() : 0;
  time_t nowEp   = time(nullptr);

  if (resumeNow) {
    systemPaused    = false;
    pauseUntilEpoch = 0;
  } else {
    if (server.hasArg("pauseEnable")) {
      if (pauseHours > 0) {
        systemPaused    = true;
        pauseUntilEpoch = nowEp + (time_t)pauseHours * 3600;
      } else {
        // 0 hours + checkbox = "until manual resume"
        systemPaused    = true;
        pauseUntilEpoch = 0;
      }
    } else {
      // Checkbox not ticked ? clear pause
      systemPaused    = false;
      pauseUntilEpoch = 0;
    }
  }

  // Water mode
  if (server.hasArg("waterMode")) {
    String wm = server.arg("waterMode");
    if (wm == "tank") {
      justUseTank  = true;
      justUseMains = false;
    } else if (wm == "mains") {
      justUseTank  = false;
      justUseMains = true;
    } else { // auto
      justUseTank  = false;
      justUseMains = false;
    }
  }

  // Tank low threshold
  if (server.hasArg("tankThresh")) {
    int th = server.arg("tankThresh").toInt();
    if (th < 0)   th = 0;
    if (th > 100) th = 100;
    tankLowThresholdPct = th;
  }
  tankEnabled = server.hasArg("tankEnabled");

  // GPIO fallback pins
  for (int i = 0; i < MAX_ZONES; i++) {
    String key = "zonePin" + String(i);
    if (server.hasArg(key)) {
      int p = server.arg(key).toInt();
      if (p == -1 || isValidOutputPin(p)) zonePins[i] = p;
    }
    String polarityKey = "zonePinLow" + String(i);
    zoneGpioActiveLow[i] = server.hasArg(polarityKey);
  }
  if (server.hasArg("mainsPin")) {
    int p = server.arg("mainsPin").toInt();
    if (p == -1 || isValidOutputPin(p)) mainsPin = p;
  }
  mainsGpioActiveLow = server.hasArg("mainsPinLow");
  if (server.hasArg("tankPin")) {
    int p = server.arg("tankPin").toInt();
    if (p == -1 || isValidOutputPin(p)) tankPin = p;
  }
  tankGpioActiveLow = server.hasArg("tankPinLow");
  if (server.hasArg("powerSupplyPin")) {
    int p = server.arg("powerSupplyPin").toInt();
    if (p == -1 || isValidOutputPin(p)) powerSupplyPin = p;
  }
  powerSupplyActiveLow = server.hasArg("powerSupplyPinLow");
  if (server.hasArg("tankLevelPin")) {
    int p = server.arg("tankLevelPin").toInt();
    if (p == -1 || isValidAdcPin(p)) tankLevelPin = p;
  }
  // Photoresistor auto-backlight
  photoAutoEnabled = server.hasArg("photoAuto");
  if (server.hasArg("photoPin")) {
    int p = server.arg("photoPin").toInt();
    if (isValidPhotoPin(p)) photoPin = p;
  }
  if (server.hasArg("photoThreshold")) {
    int th = server.arg("photoThreshold").toInt();
    if (th < 0) th = 0;
    if (th > 4095) th = 4095;
    photoThreshold = th;
  }
  photoInvert = server.hasArg("photoInvert");
  if (server.hasArg("displayType")) {
    String dm = server.arg("displayType");
    dm.toLowerCase();
    if (dm == "none") {
      displayEnabled = false;
    } else if (dm == "tft") {
      displayEnabled = true;
      displayUseTft = true;
    } else if (dm == "oled") {
      displayEnabled = true;
      displayUseTft = false;
    }
  }
  if (server.hasArg("clockFormat")) {
    clockUse24Hour = (server.arg("clockFormat") != "12");
  }
  if (server.hasArg("tempUnit")) {
    tempUseFahrenheit = (server.arg("tempUnit") == "F");
  }
  if (server.hasArg("tftRotation")) {
    int r = server.arg("tftRotation").toInt();
    if (r < 0) r = 0;
    if (r > 3) r = 3;
    tftRotation = (uint8_t)r;
  }
  if (server.hasArg("tftWidth")) {
    int v = 0;
    if (parseRequiredIntArg(server.arg("tftWidth"), v, displayCfgErr, F("TFT width"))) {
      if (isValidTftDimension(v)) tftPanelWidth = (int16_t)v;
      else displayCfgErr += "TFT width out of range: " + String(v) + " (allowed 120..400)\n";
    }
  }
  if (server.hasArg("tftHeight")) {
    int v = 0;
    if (parseRequiredIntArg(server.arg("tftHeight"), v, displayCfgErr, F("TFT height"))) {
      if (isValidTftDimension(v)) tftPanelHeight = (int16_t)v;
      else displayCfgErr += "TFT height out of range: " + String(v) + " (allowed 120..400)\n";
    }
  }
  // TFT SPI pins (apply on reboot)
  if (server.hasArg("tftSclk")) {
    int p = 0;
    if (parseRequiredIntArg(server.arg("tftSclk"), p, displayCfgErr, F("TFT SCK"))) {
      if (isValidTftSignalPin(p)) tftSclkPin = p;
      else displayCfgErr += "TFT SCK invalid/unsafe: GPIO" + String(p) + "\n";
    }
  }
  if (server.hasArg("tftMosi")) {
    int p = 0;
    if (parseRequiredIntArg(server.arg("tftMosi"), p, displayCfgErr, F("TFT MOSI"))) {
      if (isValidTftSignalPin(p)) tftMosiPin = p;
      else displayCfgErr += "TFT MOSI invalid/unsafe: GPIO" + String(p) + "\n";
    }
  }
  if (server.hasArg("tftCs")) {
    int p = 0;
    if (parseRequiredIntArg(server.arg("tftCs"), p, displayCfgErr, F("TFT CS"))) {
      if (isValidTftSignalPin(p)) tftCsPin = p;
      else displayCfgErr += "TFT CS invalid/unsafe: GPIO" + String(p) + "\n";
    }
  }
  if (server.hasArg("tftDc")) {
    int p = 0;
    if (parseRequiredIntArg(server.arg("tftDc"), p, displayCfgErr, F("TFT DC"))) {
      if (isValidTftSignalPin(p)) tftDcPin = p;
      else displayCfgErr += "TFT DC invalid/unsafe: GPIO" + String(p) + "\n";
    }
  }
  if (server.hasArg("tftRst")) {
    int p = 0;
    if (parseRequiredIntArg(server.arg("tftRst"), p, displayCfgErr, F("TFT RST"))) {
      if (isValidOptionalTftPin(p)) tftRstPin = p;
      else displayCfgErr += "TFT RST invalid/unsafe: GPIO" + String(p) + "\n";
    }
  }
  if (server.hasArg("tftBl")) {
    int p = 0;
    if (parseRequiredIntArg(server.arg("tftBl"), p, displayCfgErr, F("TFT BL"))) {
      if (isValidOptionalTftPin(p)) tftBlPin = p;
      else displayCfgErr += "TFT BL invalid/unsafe: GPIO" + String(p) + "\n";
    }
  }
  if (displayCfgErr.length()) {
    server.send(400, "text/plain", displayCfgErr);
    return;
  }
#if ENABLE_OTA
  if (changeOtaPassword) {
    otaPassword = requestedOtaPassword;
    ArduinoOTA.setPassword(otaPassword.c_str());
  }
#endif
  if (server.hasArg("manualSelectPin")) {
    int p = server.arg("manualSelectPin").toInt();
    if (p == -1 || isValidGpioPin(p)) manualSelectPin = p;
  }
  if (server.hasArg("manualStartPin")) {
    int p = server.arg("manualStartPin").toInt();
    if (p == -1 || isValidGpioPin(p)) manualStartPin = p;
  }

  // Keep the legacy global polarity aligned for backward-compatible saves.
  gpioActiveLow = false;
  for (int i = 0; i < MAX_ZONES; ++i) {
    if (zoneGpioActiveLow[i]) { gpioActiveLow = true; break; }
  }
  if (mainsGpioActiveLow || tankGpioActiveLow) gpioActiveLow = true;
  if (powerSupplyActiveLow) gpioActiveLow = true;

  // Timezone mode + values
  if (server.hasArg("tzMode")) {
    int m = server.arg("tzMode").toInt();
    if (m < 0) m = 0;
    if (m > 2) m = 2;
    tzMode = (TZMode)m;
  }
  if (server.hasArg("tzPosix")) {
    String v = server.arg("tzPosix");
    v.trim();
    if (v.length()) tzPosix = v;
  }
  if (server.hasArg("tzIANA")) {
    String v = server.arg("tzIANA");
    v.trim();
    if (v.length()) {
      tzIANA = v;
      const char* mapped = posixForIanaZone(tzIANA);
      if (mapped) {
        tzPosix = mapped;
        tzMode = TZ_POSIX;
      }
    }
  }
  if (server.hasArg("tzFixed")) {
    tzFixedOffsetMin = (int16_t)server.arg("tzFixed").toInt();
  }

  // MQTT
  mqttEnabled = server.hasArg("mqttEnabled");
  if (server.hasArg("mqttBroker")) mqttBroker = server.arg("mqttBroker");
  if (server.hasArg("mqttPort"))   mqttPort   = (uint16_t)server.arg("mqttPort").toInt();
  if (server.hasArg("mqttUser"))   mqttUser   = server.arg("mqttUser");
  if (server.hasArg("mqttPass"))   mqttPass   = server.arg("mqttPass");
  if (server.hasArg("mqttBase"))   mqttBase   = server.arg("mqttBase");

  bool tftPinsChanged = (oldTftSclk != tftSclkPin ||
                         oldTftMosi != tftMosiPin ||
                         oldTftCs   != tftCsPin   ||
                         oldTftDc   != tftDcPin   ||
                         oldTftRst  != tftRstPin  ||
                         oldTftBl   != tftBlPin);
  bool tftGeometryChanged = (oldTftWidth != tftPanelWidth ||
                             oldTftHeight != tftPanelHeight);

  if (server.hasArg("i2cSda")) {
    int p = server.arg("i2cSda").toInt();
    if (isValidGpioPin(p)) i2cSdaPin = p;
  }
  if (server.hasArg("i2cScl")) {
    int p = server.arg("i2cScl").toInt();
    if (isValidGpioPin(p)) i2cSclPin = p;
  }

  bool i2cPinsChanged = (oldI2cSda != i2cSdaPin || oldI2cScl != i2cSclPin);
  bool displayModeChanged = (oldDisplayEnabled != displayEnabled || oldDisplayUseTft != displayUseTft);
  bool clockFormatChanged = (oldClockUse24Hour != clockUse24Hour);
  bool tempUnitChanged = (oldTempUseFahrenheit != tempUseFahrenheit);
  bool climateSourceChanged = (oldClimateSource != climateSource);
  bool dhtPinChanged = (oldDhtSensorPin != dhtSensorPin);
  bool tftRotationChanged = (oldTftRotation != (int)tftRotation);

  // Persist and re-apply runtime things
  validatePinMap();
  saveConfig();
  initManualButtons();
  initGpioPinsForZones();
  if (!displayModeChanged && !tftGeometryChanged && displayEnabled && displayUseTft && tftRotationChanged) {
    tft.setRotation(tftRotation);
  }
  if (i2cPinsChanged || climateSourceChanged || dhtPinChanged) {
    localAhtReady = false;
    localAhtLastAttemptMs = 0;
    resetLocalDhtSensor();
    localClimateLastReadMs = 0;
    localClimateValid = false;
    localTempC = NAN;
    localHumidityPct = NAN;
  }
  applyLocalClimateOverride(true);
  if (clockFormatChanged || tempUnitChanged || climateSourceChanged || dhtPinChanged) {
    g_forceHomeReset = true;
    lastScreenRefresh = 0;
  }

  // --- NEW: if location/coords changed, force a weather refresh ---
  auto coordChanged = [](float a, float b) -> bool {
    if (!isfinite(a) && !isfinite(b)) return false;
    if (!isfinite(a) || !isfinite(b)) return true;
    return fabsf(a - b) > 0.0001f;
  };
  if (coordChanged(oldLat, meteoLat) || coordChanged(oldLon, meteoLon) || (oldLoc != meteoLocation) || (oldModel != meteoModel) || climateSourceChanged || dhtPinChanged) {
    // Clear current / forecast caches and timers
    cachedWeatherData   = "";
    cachedForecastData  = "";
    lastWeatherUpdate   = 0;
    lastForecastUpdate  = 0;

    // Reset derived metrics so /status & UI show clean values until next fetch
    rain1hNow           = 0.0f;
    rain3hNow           = 0.0f;
    curTempC            = NAN;
    curFeelsC           = NAN;
    curPressureHpa      = NAN;
    prevPressureHpa     = NAN;
    curPressureTrend    = 0;
    curWindMs           = NAN;
    curGustMs           = NAN;
    curWindDirDeg       = NAN;
    curHumidityPct      = -1;
    curWeatherCode      = -1;
    curUtcOffsetSec     = 0;
    curWeatherValid     = false;
    rainNext12h_mm      = 0.0f;
    rainNext24h_mm      = 0.0f;
    popNext12h_pct      = -1;
    nextRainIn_h        = -1;
    maxGust24h_ms       = 0.0f;
    todayMin_C          = NAN;
    todayMax_C          = NAN;
    todaySunrise        = 0;
    todaySunset         = 0;
    clearRainHistoryState();
    applyLocalClimateOverride(true);
  }


  applyTimezoneAndSNTP();  // re-sync NTP with new TZ
  mqttSetup();             // reconfigure client; loop() will reconnect

  server.sendHeader("Location", "/setup", true);
  server.send(302, "text/plain", "");

  if (tftPinsChanged || tftGeometryChanged || i2cPinsChanged || displayModeChanged) {
    Serial.println("[CFG] Display/pin mapping changed, restarting to apply...");
    delay(200);
    ESP.restart();
  }
}
void handleClearEvents() {
  HttpScope _scope;
  totalScheduledRuntimeSec = 0;
  totalManualRuntimeSec = 0;
  if (LittleFS.exists("/events.csv")) {
    LittleFS.remove("/events.csv");
  }
  server.sendHeader("Location", "/events", true);
  server.send(302, "text/plain", "");
}

static uint32_t readBootCount() {
  if (!LittleFS.exists("/boot_count.txt")) return 0;
  File f = LittleFS.open("/boot_count.txt", "r");
  if (!f) return 0;
  String value = f.readString();
  f.close();
  value.trim();
  if (!value.length()) return 0;
  return (uint32_t)strtoul(value.c_str(), nullptr, 10);
}

static bool saveBootCount(uint32_t count) {
  File f = LittleFS.open("/boot_count.txt", "w");
  if (!f) {
    Serial.println("[BOOT] Unable to persist boot count");
    return false;
  }
  const bool saved = f.print(count) > 0;
  f.close();
  return saved;
}

static void updateBootCount() {
  bootCount = readBootCount();
  if (bootCount != 0xFFFFFFFFUL) ++bootCount;
  saveBootCount(bootCount);
}

static bool isValidUpdateReportEventId(const String& value) {
  if (value.length() != 32) return false;
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  }
  return true;
}

static String makeUpdateReportEventId() {
  char value[33];
  snprintf(value, sizeof(value), "%08lx%08lx%08lx%08lx",
           (unsigned long)esp_random(), (unsigned long)esp_random(),
           (unsigned long)esp_random(), (unsigned long)esp_random());
  return String(value);
}

static bool saveUpdateReportState(bool reported) {
  File f = LittleFS.open(kUpdateReportStatePath, "w");
  if (!f) return false;
  f.println(F("UPDATE_REPORT1"));
  f.println(kFirmwareVersion);
  f.println(updateReportEventId);
  f.println(reported ? '1' : '0');
  f.close();
  return true;
}

static void initUpdateReportState() {
  String savedVersion;
  String savedEventId;
  bool reported = false;

  File f = LittleFS.open(kUpdateReportStatePath, "r");
  if (f) {
    String magic = f.readStringUntil('\n'); magic.trim();
    savedVersion = f.readStringUntil('\n'); savedVersion.trim();
    savedEventId = f.readStringUntil('\n'); savedEventId.trim();
    String savedReported = f.readStringUntil('\n'); savedReported.trim();
    reported = savedReported == "1";
    f.close();
    if (magic != "UPDATE_REPORT1") savedVersion = "";
  }

  if (savedVersion == kFirmwareVersion && isValidUpdateReportEventId(savedEventId)) {
    updateReportEventId = savedEventId;
    updateReportPending = !reported;
  } else {
    updateReportEventId = makeUpdateReportEventId();
    updateReportPending = true;
    saveUpdateReportState(false);
  }
  updateReportNextAttemptMs = millis() + UPDATE_REPORT_BOOT_DELAY_MS;
}

static void tickUpdateReport() {
  if (!updateReportPending || WiFi.status() != WL_CONNECTED) return;
  const uint32_t now = millis();
  if ((int32_t)(now - updateReportNextAttemptMs) < 0) return;
  updateReportNextAttemptMs = now + UPDATE_REPORT_RETRY_MS;

  JsonDocument doc;
  doc["version"] = kFirmwareVersion;
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    doc["board"] = "esp32-s3-devkitc-1";
  #else
    doc["board"] = "esp32-dev";
  #endif
  doc["eventId"] = updateReportEventId;
  String payload;
  serializeJson(doc, payload);

  WiFiClientSecure secure;
  secure.setInsecure();
  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(5000);
  if (!http.begin(secure, kUpdateReportUrl)) return;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("User-Agent", String("ESP32-Irrigation/") + kFirmwareVersion);
  const int code = http.POST(payload);
  http.end();

  if (code >= 200 && code < 300) {
    updateReportPending = false;
    saveUpdateReportState(true);
    Serial.printf("[UPDATE] Firmware %s success reported\n", kFirmwareVersion);
  } else {
    Serial.printf("[UPDATE] Success report retry scheduled (HTTP %d)\n", code);
  }
}
