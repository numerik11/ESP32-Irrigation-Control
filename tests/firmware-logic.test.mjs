import assert from "node:assert/strict";
import path from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";

import {
  compileFirmwareFunctions,
  extractFunction,
  readFirmware,
} from "./helpers/firmware-source.mjs";

const testDirectory = path.dirname(fileURLToPath(import.meta.url));
const repositoryRoot = path.resolve(testDirectory, "..");
const esp32 = await readFirmware(
  path.join(repositoryRoot, "firmware", "ESP32-Irrigation", "ESP32-Irrigation.ino"),
);
const esp8266 = await readFirmware(
  path.join(repositoryRoot, "firmware", "ESP8266-Irrigation", "ESP8266-Irrigation.ino"),
);

test("ESP32 GPIO polarity produces the safe relay levels", () => {
  const { gpioLevelForPolarity } = compileFirmwareFunctions(
    esp32,
    ["gpioLevelForPolarity"],
    { LOW: 0, HIGH: 1 },
  );

  assert.equal(gpioLevelForPolarity(true, true), 0, "active-low ON");
  assert.equal(gpioLevelForPolarity(false, true), 1, "active-low OFF");
  assert.equal(gpioLevelForPolarity(true, false), 1, "active-high ON");
  assert.equal(gpioLevelForPolarity(false, false), 0, "active-high OFF");
});

test("ESP32 primary and secondary runtimes include seconds and use fallback", () => {
  const globals = {
    MAX_ZONES: 16,
    enableStartTime2: [true, true],
    durationMin: [2, 4],
    durationSec: [15, 5],
    duration2Min: [1, 0],
    duration2Sec: [30, 0],
  };
  const { durationForSlot } = compileFirmwareFunctions(esp32, ["durationForSlot"], globals);

  assert.equal(durationForSlot(0, 1), 135);
  assert.equal(durationForSlot(0, 2), 90);
  assert.equal(durationForSlot(1, 2), 245, "zero-length slot 2 falls back to slot 1");
  assert.equal(durationForSlot(-1, 1), 0);
  assert.equal(durationForSlot(16, 1), 0);

  globals.durationMin[0] = -4;
  globals.durationSec[0] = -1;
  assert.equal(durationForSlot(0, 1), 0, "negative persisted values are made safe");
});

test("ESP32 weather codes map to display groups and wet/dry decisions", () => {
  const { meteoCodeToMain, meteoCodeToDesc, meteoCodeIsWet } = compileFirmwareFunctions(
    esp32,
    ["meteoCodeToMain", "meteoCodeToDesc", "meteoCodeIsWet"],
  );

  assert.equal(meteoCodeToMain(0), "Clear");
  assert.equal(meteoCodeToMain(48), "Fog");
  assert.equal(meteoCodeToMain(63), "Rain");
  assert.equal(meteoCodeToMain(85), "Snow");
  assert.equal(meteoCodeToMain(95), "Thunder");
  assert.equal(meteoCodeToMain(49), "Unknown");
  assert.equal(meteoCodeToDesc(82), "Violent showers");
  assert.equal(meteoCodeToDesc(999), "Unknown");

  for (const code of [51, 67, 71, 77, 80, 86, 95, 99]) {
    assert.equal(meteoCodeIsWet(code), true, `weather code ${code} is wet`);
  }
  for (const code of [-1, 0, 50, 68, 78, 87, 94]) {
    assert.equal(meteoCodeIsWet(code), false, `weather code ${code} is dry`);
  }
});

test("ESP32 compass conversion normalizes angles and handles invalid data", () => {
  const { normalizeDegrees360, meteoWindDirectionToCompass } = compileFirmwareFunctions(
    esp32,
    ["normalizeDegrees360", "meteoWindDirectionToCompass"],
  );

  assert.equal(normalizeDegrees360(-45), 315);
  assert.equal(normalizeDegrees360(720), 0);
  assert.equal(meteoWindDirectionToCompass(0), "N");
  assert.equal(meteoWindDirectionToCompass(11.24), "N");
  assert.equal(meteoWindDirectionToCompass(11.25), "NNE");
  assert.equal(meteoWindDirectionToCompass(90), "E");
  assert.equal(meteoWindDirectionToCompass(-45), "NW");
  assert.equal(meteoWindDirectionToCompass(Number.NaN), "");
});

test("ESP32 smart watering skips, scales, and combines weather adjustments", () => {
  function factor(overrides = {}) {
    const state = {
      smartWateringEnabled: true,
      rainNext24h_mm: 0,
      smartActualRainSkipMm: 8,
      smartForecastRainSkipMm: 10,
      smartCoolTempC: 12,
      smartHotTempC: 28,
      smartVeryHotTempC: 36,
      smartCoolAdjustPct: -20,
      smartHotAdjustPct: 25,
      smartVeryHotAdjustPct: 50,
      smartLightRainAdjustPct: -50,
      actualRain: 0,
      soilWet: false,
      referenceTemp: 20,
      ...overrides,
    };
    const { smartWateringFactor } = compileFirmwareFunctions(
      esp32,
      ["smartWateringFactor"],
      {
        ...state,
        last24hActualRain: () => state.actualRain,
        isSoilWetForSmartSkip: () => state.soilWet,
        smartWateringReferenceTempC: () => state.referenceTemp,
      },
    );
    return smartWateringFactor();
  }

  assert.equal(factor({ smartWateringEnabled: false }), 1);
  assert.equal(factor({ actualRain: 8.1 }), 0);
  assert.equal(factor({ rainNext24h_mm: 10.1 }), 0);
  assert.equal(factor({ soilWet: true }), 0);
  assert.equal(factor({ referenceTemp: 10 }), 0.8);
  assert.equal(factor({ referenceTemp: 30 }), 1.25);
  assert.equal(factor({ referenceTemp: 40 }), 1.5);
  assert.equal(factor({ referenceTemp: 30, actualRain: 2 }), 0.625);
});

test("ESP32 status LED pulse and wrapped flash windows honor boundaries", () => {
  const clock = { now: 0 };
  const { statusPixelPulseLevel, statusPixelWindowOn } = compileFirmwareFunctions(
    esp32,
    ["statusPixelPulseLevel", "statusPixelWindowOn"],
    { millis: () => clock.now },
  );

  assert.equal(statusPixelPulseLevel(1000, 10, 110), 10);
  clock.now = 250;
  assert.equal(statusPixelPulseLevel(1000, 10, 110), 60);
  clock.now = 500;
  assert.equal(statusPixelPulseLevel(1000, 10, 110), 110);
  assert.equal(statusPixelPulseLevel(1, 10, 110), 110);
  assert.equal(statusPixelPulseLevel(1000, 110, 10), 10);

  clock.now = 950;
  assert.equal(statusPixelWindowOn(1000, 900, 200), true);
  clock.now = 50;
  assert.equal(statusPixelWindowOn(1000, 900, 200), true);
  clock.now = 100;
  assert.equal(statusPixelWindowOn(1000, 900, 200), false);
  assert.equal(statusPixelWindowOn(0, 0, 10), false);
});

test("ESP32 publishes the Home Assistant discovery contract from its status payload", () => {
  const status = extractFunction(esp32, "mqttPublishStatus");
  const discovery = extractFunction(esp32, "mqttPublishHomeAssistantDiscovery");
  const publishConfig = extractFunction(esp32, "mqttPublishHomeAssistantConfig");
  const reconnect = extractFunction(esp32, "mqttEnsureConnected");

  for (const field of [
    "rain24hActual",
    "cooldownRemaining",
    "masterOn",
    "paused",
    "rainActive",
    "windActive",
    "zones",
    "active",
  ]) {
    assert.match(status, new RegExp(`\\[\\"${field}\\"\\]`), `${field} is published in MQTT status`);
  }

  const expectedEntities = [
    ["sensor", "espirrigation_rainfall_24h"],
    ["sensor", "espirrigation_cooldown"],
    ["binary_sensor", "espirrigation_master"],
    ["binary_sensor", "espirrigation_paused"],
    ["binary_sensor", "espirrigation_rain_active"],
    ["binary_sensor", "espirrigation_wind_active"],
  ];
  for (const [component, uniqueId] of expectedEntities) {
    assert.ok(discovery.includes(`"${component}", "${uniqueId}"`), `${uniqueId} is discovered`);
  }

  for (const templateField of [
    "value_json.rain24hActual",
    "value_json.cooldownRemaining",
    "value_json.masterOn",
    "value_json.paused",
    "value_json.rainActive",
    "value_json.windActive",
    "value_json.zones[",
  ]) {
    assert.ok(discovery.includes(templateField), `${templateField} template is configured`);
  }

  assert.match(discovery, /i\s*<\s*\(int\)zonesCount/, "switches follow the configured zone count");
  assert.match(discovery, /cmd\/zone\//, "switch commands use the existing zone command topic");
  assert.match(discovery, /i\s*=\s*\(int\)zonesCount;\s*i\s*<\s*\(int\)MAX_ZONES/, "stale zone discoveries are removed");
  assert.match(publishConfig, /_mqtt\.publish\(topic\.c_str\(\), payload\.c_str\(\), true\)/, "discovery is retained");
  assert.match(reconnect, /mqttTryPublishHomeAssistantDiscovery\(now\)/, "discovery runs after MQTT connects");
});

test("ESP32 browser OTA is authenticated, writes an app image, and restarts only after success", () => {
  const authenticate = extractFunction(esp32, "otaHttpAuthenticate");
  const updatePage = extractFunction(esp32, "handleOtaUpdatePage");
  const upload = extractFunction(esp32, "handleOtaUploadData");
  const result = extractFunction(esp32, "handleOtaUploadResult");
  const configure = extractFunction(esp32, "handleConfigure");
  const loadConfig = extractFunction(esp32, "loadConfig");
  const saveConfig = extractFunction(esp32, "saveConfig");

  assert.match(esp32, /#include\s+<Update\.h>/, "ESP32 Update support is included");
  assert.match(esp32, /server\.on\("\/update",\s*HTTP_GET,\s*handleOtaUpdatePage\)/);
  assert.match(
    esp32,
    /server\.on\("\/update",\s*HTTP_POST,\s*handleOtaUploadResult,\s*handleOtaUploadData\)/,
  );

  assert.match(authenticate, /!otaPassword\.length\(\)/, "an empty password disables browser OTA");
  assert.match(authenticate, /server\.authenticate\("admin",\s*otaPassword\.c_str\(\)\)/);
  assert.match(authenticate, /server\.requestAuthentication\(\)/);
  assert.match(updatePage, /otaHttpAuthenticate\(\)/, "the update page is protected");
  assert.match(updatePage, /Minimal SPIFFS \(Large APPS with OTA\)/);

  assert.match(upload, /endsWith\("\.bin"\)/, "only an application binary is accepted");
  assert.match(upload, /pendingStart\[z\]\s*=\s*false/);
  assert.match(upload, /turnOffZone\(z\)/, "active valves are stopped before writing flash");
  assert.match(upload, /Update\.begin\(UPDATE_SIZE_UNKNOWN,\s*U_FLASH\)/);
  assert.match(upload, /Update\.write\(upload\.buf,\s*upload\.currentSize\)/);
  assert.match(upload, /Update\.end\(true\)/);
  assert.match(upload, /Update\.abort\(\)/);

  assert.match(result, /otaUploadAuthorized\s*&&\s*otaUploadStarted/);
  assert.match(result, /otaUploadComplete\s*&&\s*!otaUploadFailed\s*&&\s*!Update\.hasError\(\)/);
  assert.match(result, /ESP\.restart\(\)/);
  assert.doesNotMatch(upload, /ESP\.restart\(\)/, "partial uploads must not restart the controller");

  assert.match(configure, /OTA password must contain 8 to 64 characters/);
  assert.match(configure, /ArduinoOTA\.setPassword\(otaPassword\.c_str\(\)\)/);
  assert.match(loadConfig, /otaPassword\s*=\s*s/);
  assert.match(saveConfig, /f\.println\(otaPassword\)/);
  assert.match(
    esp32,
    /otaPassword\.length\(\)\s*&&\s*!otaHttpAuthenticate\(\)/,
    "saved credentials protect config.txt downloads",
  );
});

test("ESP8266 clamps configuration and classifies pins and rain codes", () => {
  const { clampInt, validZonePin, weatherCodeIsWet } = compileFirmwareFunctions(
    esp8266,
    ["clampInt", "validZonePin", "weatherCodeIsWet"],
  );

  assert.equal(clampInt(-1, 0, 10), 0);
  assert.equal(clampInt(5, 0, 10), 5);
  assert.equal(clampInt(11, 0, 10), 10);
  assert.equal(validZonePin(-1), false);
  assert.equal(validZonePin(0), true);
  assert.equal(validZonePin(16), true);
  assert.equal(validZonePin(17), false);
  assert.equal(weatherCodeIsWet(51), true);
  assert.equal(weatherCodeIsWet(99), true);
  assert.equal(weatherCodeIsWet(100), false);
});

test("ESP8266 runtime and minute helpers handle limits and slot selection", () => {
  const globals = {
    MAX_ZONES: 6,
    durationMin: [10, -1, 999],
    duration2Min: [3, 5, 1],
    clampInt: (value, low, high) => Math.min(high, Math.max(low, value)),
  };
  const { durationForSlot, minuteEpoch, weatherRulesArmed } = compileFirmwareFunctions(
    esp8266,
    ["durationForSlot", "minuteEpoch", "weatherRulesArmed"],
    { ...globals, weatherLat: 0, weatherLon: 0 },
  );

  assert.equal(durationForSlot(0, 1), 600);
  assert.equal(durationForSlot(0, 2), 180);
  assert.equal(durationForSlot(1, 1), 0);
  assert.equal(durationForSlot(2, 1), 36_000, "duration is capped at 600 minutes");
  assert.equal(durationForSlot(6, 1), 0);
  assert.equal(minuteEpoch(125), 120);
  assert.equal(weatherRulesArmed(), false);

  const armed = compileFirmwareFunctions(esp8266, ["weatherRulesArmed"], {
    weatherLat: 0.02,
    weatherLon: 0,
  });
  assert.equal(armed.weatherRulesArmed(), true);
});

test("ESP8266 scheduler starts due slots once and respects disabled days", () => {
  const calls = [];
  const clock = { tm_wday: 1, tm_hour: 6, tm_min: 30 };
  const state = {
    zonesCount: 2,
    lastStartMinuteEpoch: [0, 0],
    days: [
      [false, true, false, false, false, false, false],
      [false, false, false, false, false, false, false],
    ],
    startHour: [6, 6],
    startMin: [30, 30],
    startHour2: [7, 7],
    startMin2: [45, 45],
    enableStart2: [true, true],
    durationForSlot: () => 60,
    turnOnZone: (zone, slot) => {
      calls.push([zone, slot]);
      return true;
    },
    localtime_r: () => clock,
  };
  const { processDueStartsAtMinute } = compileFirmwareFunctions(
    esp8266,
    ["processDueStartsAtMinute"],
    state,
    {
      replacements: [
        [/localtime_r\s*\(&minuteMark,\s*&t\s*\);/, "t = localtime_r(minuteMark);"],
      ],
    },
  );

  const primaryMinute = 10_020;
  processDueStartsAtMinute(primaryMinute);
  assert.deepEqual(calls, [[0, 1]]);
  assert.equal(
    calls.some(([zone]) => zone === 1),
    false,
    "a zone disabled on Monday does not start",
  );
  assert.deepEqual(state.lastStartMinuteEpoch, [primaryMinute, 0]);

  processDueStartsAtMinute(primaryMinute);
  assert.deepEqual(calls, [[0, 1]], "same schedule minute is not triggered twice");

  clock.tm_hour = 7;
  clock.tm_min = 45;
  const secondaryMinute = primaryMinute + 75 * 60;
  processDueStartsAtMinute(secondaryMinute);
  assert.deepEqual(calls, [[0, 1], [0, 2]], "enabled secondary slot starts at 07:45");
  assert.deepEqual(state.lastStartMinuteEpoch, [secondaryMinute, 0]);
});
