ESP32 4/6-Zone Irrigation Controller
--------------------------------------------------------------------------------------------------------------------

Smart irrigation controller for ESP32 that drives up to 6 zones, supports Tank/Mains selection, 
shows live weather and tank status and serves a modern web UI with manual overrides and scheduling.

Default tank analog pin: IO36 (A1) on KC868-A6 and most ESP32 boards.
Max input: Tank sensor output must be ≤ 3.3V to the ESP32 ADC.

Weather API: You’ll need Wi-Fi and a free OpenWeather API key → https://home.openweathermap.org/users/sign_up

Key Features
--------------------------------------------------------------------------------------------------------------------

- Dashboard

Current time/date with ACST/ACDT tag (auto DST via SNTP).

Tank level (%) with Auto:Mains / Auto:Tank / Force state.

Live weather (OpenWeather Current): temperature, humidity, wind, condition.

Next Water: next run (zone, start time, ETA, duration) computed on the device.

Rain delay (sensor + weather) and wind delay badges with cause.

Zone status/progress bars and manual On/Off buttons.

--------------------------------------------------------------------------------------------------------------------

- Zones & Schedules

4-zone mode (Zones 1–4) + Mains and Tank master valves on relays 5 & 6.

6-zone mode (six zones).

Two start times per zone (optional Start 2), per-day enable, minute precision.

Per-zone duration (minutes + seconds).

Queued starts: delayed zones (rain/wind or active run) auto-start later.

Zone names editable and saved in LittleFS.

Rain delay sources: physical sensor (invert option) + weather conditions (Rain/Drizzle/Thunderstorm or rain amount).

Wind delay: configurable threshold (m/s).

--------------------------------------------------------------------------------------------------------------------

- Hardware & I/O

KC868-A6 support (PCF8574 at 0x24 relays, 0x22 inputs).

Automatic I²C health check with debounce → GPIO fallback for generic ESP32 boards.

All zone/mains/tank GPIO pins configurable in Setup (fallback mode).

OLED status screens (Home / Rain Delay).

WiFiManager captive portal: ESPIrrigationAP (first boot or failure).

OTA updates enabled (hostname ESP32-Irrigation).

Event Logger to CSV with weather snapshot per event (downloadable).

--------------------------------------------------------------------------------------------------------------------

- Materials

7-core irrigation cable from controller to solenoid pit/box.

6 irrigation solenoids (match your supply; 12V DC or 24V AC).
Example: 12V DC micro-solenoids powered by the same 12V 1.5A supply.

KC868-A6 (recommended) or ESP32 dev board + 6-relay module.

Tank level sensor with 0–3.3V output (to ESP32 ADC).

--------------------------------------------------------------------------------------------------------------------

- Wiring (Typical)

Tie all solenoid grounds/returns to the power supply GND/COM.

Feed 12/24V into each relay COM terminal; wire the solenoid hot lead to N.O..

Relays 1–4 → Zones 1–4. Relay 5 → Mains, Relay 6 → Tank (4-zone master valves).

Tank level sensor → IO36 (A1). Do not exceed 3.3V.

Rain sensor IO27

--------------------------------------------------------------------------------------------------------------------

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

Sketch → Include Library → Add .ZIP Library… → select the file.

4) Upload

Open the sketch (single .ino) and click Upload.

5) First-Run Wi-Fi

Open Serial Monitor @ 115200 to watch logs.

Connect to ESPIrrigationAP; captive portal should pop up.
If not, browse to https://192.168.4.1
, select your Wi-Fi SSID + password.

6) Access & Configure

OLED shows the assigned IP on boot (or run arp -a to find it).

Browse to the IP shown on serial or screen at startup goto-

- Setup page:

Enter OpenWeather API Key and City ID.

Adjust zones (4/6), wind/rain/tank options, GPIO pins, sensor settings.

Home page: configure days, start times, and durations per zone.

--------------------------------------------------------------------------------------------------------------------

- Useful Endpoints
  
Endpoint	Description
/	Dashboard, schedule editor, manual controls
/status	JSON snapshot: device time/TZ, Next Water, zones, tank, weather roll-ups
/setup	Setup page (API keys, zones, GPIO, rain/wind, sensor, etc.)
/events	Event log (table view)
/tank	Tank calibration (Set Empty / Full)
/download/config.txt	Download raw config
/download/schedule.txt	Download schedule
/download/events.csv	Download event log CSV
/i2c-test / /i2c-scan	I²C relay pulse test / I²C bus scan
/api/time	Tiny time probe (local/UTC)
/whereami	IP/SSID/RSSI/Mode snapshot
/reboot	Reboot controller
/stopall	Stop all running zones
/valve/on/<z>	Manual start zone <z> (0-based)
/valve/off/<z>	Manual stop zone <z> (0-based)

- Credits & Links

KC868-A6: https://www.kincony.com/esp32-6-channel-relay-module-kc868-a6.html

OpenWeather: https://openweathermap.org
