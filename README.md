# ESP32 4/6-Zone Irrigation Controller

Smart irrigation controller for ESP32 driving up to 6 zones with Tank/Mains selection, live weather + tank status, a modern web UI, manual overrides, and on-device scheduling.

---

## Highlights

- **Dashboard**
  - Tank level (%) with `Auto:Mains` / `Auto:Tank` / `Force` state
  - Live weather (OpenWeather Current): temperature, humidity, wind, condition
  - **Next Water**: next scheduled run (zone, start, ETA, duration) computed on device
  - Delay badges for rain & wind with cause
  - Zone cards with progress and manual On/Off

- **Zones & Schedules**
  - 4-zone mode (Zones 1–4) **+** Mains/Tank master valves on relays 5 & 6  
  - 6-zone mode (six zones)
  - Two start times per zone (optional Start 2), per-day enable, minute precision
  - Per-zone duration (minutes + seconds)
  - **No queuing:** delayed/blocked or overlapping starts are cancelled and logged
  - Editable zone names stored in LittleFS

- **Delays & Sensors**
  - Rain sources: physical sensor (invert option) + weather conditions (Rain/Drizzle/Thunderstorm or rain amount)
  - Wind delay: configurable threshold (m/s)
  - Rain cooldown and 24h forecast threshold (mm)
  - Rolling actual rainfall stats (1h / 24h)

- **Hardware & I/O**
  - KC868-A6 support (PCF8574 @ 0x24 relays, 0x22 inputs)
  - Automatic I²C health + debounce → GPIO fallback for generic ESP32 boards
  - All zone/mains/tank GPIO pins configurable in Setup (fallback mode)
  - OLED status screens (Home / Rain Delay)

- **Networking & UX**
  - WiFiManager captive portal: `ESPIrrigationAP` (first boot/failure)
  - mDNS: `http://espirrigation.local/`
  - OTA updates (hostname `ESP32-Irrigation`)
  - Event logger to CSV (weather snapshot per event, downloadable)

---

## Requirements

- **Board:** ESP32 Dev Module (KC868-A6 recommended)
- **Tank analog pin:** IO36 (A1) (≤ 3.3V ADC)
- **Rain sensor:** IO27 (configurable)
- **Power:** Matches your solenoids (e.g., 12V DC or 24V AC; keep logic at 3.3V)
- **Weather API:** Free OpenWeather API key → https://home.openweathermap.org/users/sign_up

---

## Materials

- KC868-A6 (recommended) or ESP32 dev board + 6-relay module  
- 6 irrigation solenoids (12V DC or 24V AC to match supply)  
- 7-core irrigation cable to the pit/box  
- Tank level sensor with 0–3.3V output to ESP32 ADC

---

## Wiring (Typical)

- Tie all solenoid returns to supply GND/COM.  
- Feed 12/24V into each relay COM; solenoid hot lead to N.O.  
- Relays 1–4 → Solenoids 1–4  
- Relay 5 → **Mains** solenoid, Relay 6 → **Tank** solenoid (4-zone master valves)  
- Tank level sensor → IO36 (A1). **Do not exceed 3.3V.**  
- Rain sensor → IO27 (configurable)

---

## Flashing the Controller

1. **Install ESP32 Boards (Arduino IDE)**  
   *File → Preferences → Additional Boards URLs:*  
   `https://dl.espressif.com/dl/package_esp32_index.json`  
   *Tools → Board → Boards Manager… →* install **ESP32 by Espressif Systems**.

2. **Select a Board**  
   *Tools → Board →* **ESP32 Dev Module** (works for KC868-A6)  
   Suggested: Flash 80 MHz, Upload 115200–921600, Partition: Default (4MB).

3. **(KC868-A6) PCF8574 Library**  
   Download Kincony PCF8574 zip: <https://www.kincony.com/forum/attachment.php?aid=1697>  
   *Sketch → Include Library → Add .ZIP Library…*

4. **Upload**  
   Open the sketch and click **Upload**.

5. **First-Run Wi-Fi**  
   Serial Monitor @ 115200. Connect to **ESPIrrigationAP** (captive portal),  
   or browse to **http://192.168.4.1** and set SSID + password.

6. **Access & Configure**  
   OLED shows assigned IP (or use `arp -a`).  
   - **Setup**: enter OpenWeather API Key + City ID, timezone, zones (4/6), wind/rain/tank, GPIO pins, sensors.  
   - **Home**: set days, start times, durations per zone.

---

## Behaviour (Scheduling & Delays)

- Starts during **rain/wind**, **pause**, **master off**, or **rain cooldown** → **CANCELLED** (logged)  
- Starts that would **overlap** another running zone when not using concurrent mode → **CANCELLED** (logged)  
- Manual “On” commands also respect the same rules and cancel when blocked  
- “Next Water” still reflects the next eligible schedule (no queueing)

---

## Useful Endpoints

- `/` — Dashboard  
- `/setup` — Setup page (API keys, zones, GPIO, sensors, delays, MQTT, timezone)  
- `/status` — JSON snapshot (device time/TZ, Next Water, zones, tank, weather roll-ups)  
- `/events` — Event log (table)  
- `/tank` — Tank calibration (Set Empty/Full)  
- `/download/config.txt` — Download raw config  
- `/download/schedule.txt` — Download schedule  
- `/download/events.csv` — Download event log CSV  
- `/i2c-test` — I²C relay pulse test  
- `/i2c-scan` — I²C bus scan  
- `/api/time` — Local/UTC time probe  
- `/whereami` — IP/SSID/RSSI/Mode  
- `/reboot` — Reboot controller  
- `/stopall` — Stop all running zones  
- `/valve/on/<z>` — Manual start zone `<z>` (0-based)  
- `/valve/off/<z>` — Manual stop zone `<z>` (0-based)

---

## Notes

- **Tank ADC:** default IO36 (A1). Keep ≤ **3.3V** to the ESP32 ADC.  
- **mDNS:** `http://espirrigation.local/` after joining your Wi-Fi.  
- **OTA:** enabled (hostname `ESP32-Irrigation`).  
- **Fallback:** If I²C expanders aren’t detected, GPIO fallback is enabled and pins from Setup are used.

---

## Links

- KC868-A6: <https://www.kincony.com/esp32-6-channel-relay-module-kc868-a6.html>  
- OpenWeatherMap: <https://openweathermap.org>
