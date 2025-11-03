Key Features
Dashboard

Current time/date with ACST/ACDT tag (auto DST via SNTP; Adelaide TZ baked in by default).

Tank level (percent) with Auto:Mains / Auto:Tank / Force state.

Live weather (OpenWeather Current): temp, humidity, wind, condition.

Forecast roll-ups (OneCall): rain next 12/24h, POP(12h), max gust (24h), today min/max, sunrise/sunset.

Next Water: server-computed next run (zone, start time, ETA, duration).

Rain delay (sensor + weather) and wind delay badges with cause.

Live zone status/progress bars and manual On/Off buttons.

Zones & Schedules

4-zone mode (Zones 1-4 + Mains and Tank master valves on relays 5&6).

6-zone mode (six real zones).

Two start times per zone (optional Start 2), per-day enable, minute-precision.

Per-zone duration (min + sec).

Queued starts: if a zone is delayed by rain/wind or an active run, it auto-starts later.

Zone names stored in LittleFS (editable in UI).

Weather, Rain & Wind

OpenWeather Current + OneCall:

rain12h, rain24h, pop12h, nextRainInH, gust24h, tmin/tmax, sunrise/sunset.

Rain delay sources: physical sensor (invert option) + weather conditions (Rain/Drizzle/Thunderstorm or rain amount).

Wind delay: configurable threshold (m/s).

Live cause text (Sensor, OpenWeather, Both, Disabled).

Hardware & I/O

Works with KC868-A6 (PCF8574 at 0x24 relays, 0x22 inputs).

Automatic I²C health check with debounce → GPIO fallback if unstable.

All zone/mains/tank GPIO pins configurable in Setup (fallback mode).

OLED status screens (Home / Rain Delay).

Web, OTA & Services

Modern Web UI (responsive, light/dark toggle).

mDNS: http://espirrigation.local (plus raw IP).

WiFiManager captive portal: ESPIrrigationAP (first boot or failure).

OTA updates enabled (ESP32-Irrigation host).

Event Logger to CSV with weather snapshot per event (downloadable).

What’s New (vs. older versions)

Server-authored time for the UI (authoritative /status fields):
deviceEpoch, utcOffsetMin, isDST, tzAbbrev, plus server-formatted sunriseLocal/sunsetLocal.

Next Water now computed on the controller and exposed at /status:

nextWaterEpoch, nextWaterZone, nextWaterName, nextWaterDurSec.

Cleaner rain/wind logic with explicit cause reporting and sensor + weather fusion.

I²C→GPIO fallback with health debounce and one-click I²C tools (/i2c-test, /i2c-scan).

Manual control endpoints per zone (/valve/on/<z>, /valve/off/<z>), Stop All, and Toggle OLED backlight.

Downloads for config, schedule, and events (/download/*), plus tiny time probe (/api/time) and whereami endpoint.

UI polish: progress bars, badges, light/dark theme, improved tank meter, better error defaults.

Materials

7-core irrigation cable from controller to solenoid pit/box.

6 irrigation solenoids (match your supply; e.g. 12V DC or 24V AC).
Example: 12V DC micro-solenoids powered by the same 12V 1.5A input supply.

KC868-A6 (recommended) or ESP32 dev board + 6-relay module.

Tank level sensor (0–3.3V output to ESP32 ADC).

Wiring Instructions (Typical)

Tie all solenoid grounds/returns to the power supply GND/COM.

Feed 12/24V into each relay COM terminal; wire the solenoid’s hot lead to N.O..

Relays 1–4 → Zones 1–4. Relay 5 → Mains, Relay 6 → Tank (4-zone master valves).

Tank level sensor → IO36 (A1). Do not exceed 3.3V.

Flashing the Controller
1) Install ESP32 Boards in Arduino IDE

File → Preferences → Additional Boards URLs:
https://dl.espressif.com/dl/package_esp32_index.json

Tools → Board → Boards Manager… → search ESP32 by Espressif Systems → Install.

2) Select a Board

Tools → Board → ESP32 Dev Module (works for KC868-A6 too)

Suggested options: Flash 80 MHz, Upload 115200–921600, Partition: Default (4MB), correct COM port.

3) (KC868-A6) Install PCF8574 Library

Download Kincony PCF8574 zip: https://www.kincony.com/forum/attachment.php?aid=1697

Sketch → Include Library → Add .ZIP Library… → pick the file.

4) Upload

Open the sketch (single .ino) and click Upload.

5) First-Run Wi-Fi

Open Serial Monitor @ 115200 to watch logs.

Connect to ESPIrrigationAP; the portal should pop up.
If not: browse to https://192.168.4.1
 → select your Wi-Fi + password.

6) Access & Configure

OLED shows the assigned IP on boot; or run arp -a to find it.

Browse to the IP (or http://espirrigation.local).

Go to Setup:

Enter OpenWeather API Key and City ID.

Adjust zones (4/6), wind/rain/tank options, GPIO pins, and sensor settings.

Return to Home and configure days, start times, and durations per zone.

Useful Endpoints

/ – Dashboard + schedule editor + manual controls

/status – JSON snapshot (all state + Next Water, weather roll-ups, time/TZ)

/setup – Setup page (API keys, zones, GPIO, rain/wind, sensor, etc.)

/events – Event log (download via /download/events.csv)

/tank – Tank calibration (Set Empty / Set Full)

/download/config.txt, /download/schedule.txt, /download/events.csv

/i2c-test, /i2c-scan, /api/time, /whereami, /reboot, /stopall

/valve/on/<z>, /valve/off/<z> (0-based index)

Notes & Tips

KC868-A6 mapping: I²C PCF8574 relays at 0x24, inputs at 0x22; active-LOW relay logic is handled in code.

If I²C becomes unstable, the controller auto-switches to GPIO fallback using pins set in Setup.

Start 2 is optional per zone—handy for split runs or morning/evening watering.

Event CSV includes a weather snapshot (temp, humidity, wind, condition, city) at the time of each event.

DST & clocks: The device is the single source of truth; the web UI never guesses time.

Credits & Links

KC868-A6: https://www.kincony.com/esp32-6-channel-relay-module-kc868-a6.html

OpenWeather: https://openweathermap.org
