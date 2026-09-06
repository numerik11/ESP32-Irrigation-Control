![Platform](https://img.shields.io/badge/Platform-ESP32%20%7C%20ESP32--S3-blue)
![Zones](https://img.shields.io/badge/Zones-1%E2%80%9316-green)
![Web UI](https://img.shields.io/badge/Web%20UI-Local-orange)
![Weather](https://img.shields.io/badge/Weather-Aware-success)
![OTA](https://img.shields.io/badge/Updates-OTA-informational)
![MQTT](https://img.shields.io/badge/MQTT-Supported-purple)

# 🌱 ESP32 DIY 1–16 Zone Irrigation Controller

A flexible, locally controlled irrigation system for **1–16 solenoid valves**. It combines per-zone scheduling, weather-aware delays, optional tank/mains switching, sensor support, MQTT, and a responsive web dashboard.

Schedules run on the controller, so watering continues even when the internet is unavailable.

## Firmware 2.8

System Setup now uses the home page's compact styling, with seven grouped settings sections and navigation that wraps on mobile. Firmware Updates starts collapsed. Updated ESP32 and ESP32-S3 images are available through the web flasher and Browser OTA downloads.

## 🚀 Install from your browser

No Arduino IDE is required. Connect the ESP32 by USB and open the:

### 👉 [ESP32 Irrigation Web Flasher](https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/)
After flashing:

1. Connect to the **ESPIrrigationAP** Wi-Fi network.
2. Open `http://192.168.4.1` and enter your Wi-Fi details.
3. Wait for the controller to restart.
4. Open `http://espirrigation.local` or use the IP address assigned by your router.
5. Configure the zones, GPIO pins, weather location, and schedules.
6. Manually test every output before enabling automatic watering.

## 📸 Interface

### Dashboard

See controller status, active and upcoming zones, progress, weather, tank level, water source, and delay reasons at a glance.

<p align="center">
<img width="862" height="899" alt="image" src="https://github.com/user-attachments/assets/cf75fb58-65f0-445e-abe4-f65609e4c525" />
</p>

### Setup

Configure zones, schedules, GPIO assignments, relay polarity, weather rules, sensors, MQTT, and displays from your browser.

<p align="center">
<img width="609" height="821" alt="image" src="https://github.com/user-attachments/assets/6d97ff3c-84df-40e8-a4f5-4e66dac3346f" />
</p>

### Events

Review watering and system activity, including weather-related delays, and export the log as CSV.

<p align="center">
<img width="650" height="492" alt="image" src="https://github.com/user-attachments/assets/1bc1d7e9-1449-4f20-a51b-a0eaa4b347e2" />
</p>

## ✨ Features

- 1–16 named irrigation zones
- Two start times per zone, with independent runtimes
- Seven-day schedules with minute-and-second runtime control
- Sequential or concurrent operation
- Manual zone and master controls
- Rain, 24-hour rainfall, cooldown, and wind delays
- Cool, Normal, Hot, and Very Hot runtime adjustment
- Optional soil-moisture and temperature/humidity sensors
- Optional tank-level monitoring and automatic tank/mains selection
- Live weather data from Open-Meteo
- Next-event calculation, event logging, and CSV export
- Local web dashboard, MQTT, and OTA updates
- Optional TFT, I²C OLED, and I²C LCD displays
- ESP32, ESP32-S3, KC868-A6, and KC868-A8 support
- Local schedules continue without internet access

## ⏱️ Scheduling and smart watering

Each zone has editable days, two start times, and separate durations. **Sequential mode** runs one valve at a time and suits most systems. **Concurrent mode** can run multiple valves, but only when the transformer, relays, pipework, and water supply can handle the combined load.

With latitude and longitude configured, Open-Meteo can provide temperature, apparent temperature, humidity, wind, pressure, rainfall, daily high/low, sunrise, sunset, and current conditions.

Scheduled watering can be blocked by:

- Current rain or an active physical rain sensor
- A configured rainfall threshold
- The after-rain cooldown period
- Excessive wind
- Master OFF or system pause

Manual watering remains available where permitted. Optional temperature and moisture rules can also adjust runtimes for current conditions.

## 💧 Tank and mains control

An optional level sensor can help select between a rainwater tank and mains supply. Available modes include **Auto: Tank**, **Auto: Mains**, **Force Tank**, and **Force Mains**.

## 🔌 Wiring

### Controller wiring diagram

<p align="center">
<img width="1000" alt="ESP32 Irrigation Controller Wiring Diagram" src="https://github.com/user-attachments/assets/df3b2d56-5021-43df-a429-55fd85ebc625" />
</p>

### ESP32-S3, six relays, and 24 V AC example

<p align="center">
<img width="650" alt="ESP32-S3 6 Zone 24V AC Irrigation Controller" src="https://github.com/user-attachments/assets/f9d2c234-1e8a-4ff2-8e8d-51a5bdf73905" />
</p>

### TFT Display

<p align="center">
<img width="215" height="113" alt="PXL_20260808_043755753~2" src="https://github.com/user-attachments/assets/42fbe90b-aeb4-48c1-9cfa-cd5753487815" />
</p>

> ⚠️ Wiring and available GPIOs vary between boards, relay modules, and irrigation systems. Confirm pin assignments, relay polarity, and voltage requirements before applying power.

## 🖥️ Supported hardware

| Hardware | Description |
| --- | --- |
| **ESP32** | Standard development-board installation |
| **ESP32-S3** | Newer ESP32-S3 installation |
| **ESP32 + 240×320 SPI TFT** | Full-colour status display |
| **ESP32 + I²C OLED** | Compact, low-pin-count display |
| **ESP8266 + I²C LCD** | Lightweight LCD controller version |
| **KC868-A6 / A8** | ESP32 relay controllers with screw terminals |

Typical builds use an ESP32 or ESP32-S3, a 4/6/8/16-channel relay module, 1–16 irrigation valves, a suitable 12 or 24 V AC/DC supply, irrigation cable, terminal blocks, fuse protection, and a weatherproof enclosure.

Optional equipment includes tank, rain, soil-moisture, and temperature/humidity sensors; tank and mains valves; and TFT, OLED, or LCD displays. A photoresistor can switch or dim an enclosure display when the door is closed.

### GPIO and relay polarity

GPIOs can be assigned to zone outputs, tank and mains valves, sensors, and displays. Outputs can be configured as **active HIGH** or **active LOW**; many ESP32 relay modules are LOW = ON. GPIO changes may require a reboot.

### KC868

KC868 configurations use PCF8574 I/O expanders with automatic detection, configurable polarity, and GPIO fallback where supported. Default addresses are:

```text
0x24 = Relay outputs
0x22 = Inputs
```

Manual compilation requires the [Kincony PCF8574 library](https://www.kincony.com/forum/attachment.php?aid=1697).

## 🌐 Networking and integration

| Service | Address or hostname |
| --- | --- |
| Setup access point | `ESPIrrigationAP` |
| Setup portal | `http://192.168.4.1` |
| Local dashboard | `http://espirrigation.local/` |
| OTA hostname | `ESP32-Irrigation` |

MQTT supports integrations such as Home Assistant, Node-RED, custom dashboards, and other home-automation systems.

When MQTT is enabled, the controller publishes retained Home Assistant discovery
configuration after connecting to the broker. Home Assistant automatically creates
the rainfall and cooldown sensors, master/pause/rain/wind binary sensors, and one
switch for every configured irrigation zone. Zone switches use the controller's
configured names and existing `espirrigation/cmd/zone/<index>` command topics.

### Web endpoints

| Path | Description |
| --- | --- |
| `/` | Main dashboard |
| `/setup` | Controller configuration |
| `/status` | JSON system status |
| `/events` | Event log |
| `/tank` | Tank sensor calibration |
| `/update` | Password-protected browser OTA uploader |
| `/download/events.csv` | Download event log |
| `/i2c-test` | I²C/relay test |
| `/stopall` | Stop all active zones |
| `/valve/on/<z>` | Start a zone manually |
| `/valve/off/<z>` | Stop a zone manually |
| `/reboot` | Reboot the controller |

## 🛠️ Manual installation

To compile with Arduino IDE:

1. Add `https://dl.espressif.com/dl/package_esp32_index.json` to **Additional Boards Manager URLs**.
2. Install **ESP32 by Espressif Systems**.
3. Select **ESP32 Dev Module** or **ESP32S3 Dev Module**, as appropriate.
4. Select **Minimal SPIFFS (Large APPS with OTA)** so the firmware has enough program space while retaining two OTA application slots.

> Do not select **Huge APP** or **No OTA** if you intend to update firmware over the network; those layouts normally provide only one application slot.

### Browser OTA (ArduinoOTA alternative)

The firmware also provides an HTTP updater, so Arduino IDE network discovery and
the ArduinoOTA protocol are not required:

1. In **Setup → Firmware Updates**, save an OTA password of 8–64 characters.
2. Compile with **Minimal SPIFFS (Large APPS with OTA)** and use **Sketch → Export Compiled Binary**.
3. Open `http://<controller-ip>/update` in a browser.
4. Sign in with username `admin` and the OTA password, select the application `.bin`, and install it.

The controller stops active valves before writing, validates the image, switches
the boot slot only after a complete upload, and then restarts. Keep the controller
powered throughout the update. The password also secures ArduinoOTA and access to
the downloaded configuration file; it is stored locally in the controller's
configuration just like the MQTT password.

An existing installation can use this route only after firmware containing the
route has been installed. In particular, an older ESP32 image using a single-app
partition cannot convert itself to an OTA layout. Flash the full Minimal SPIFFS
image once over USB/Web Serial (erasing the device so the partition table is
replaced); subsequent application updates can use `/update`. ArduinoOTA remains
available as a fallback when it works on the local network.

### Faster OTA build verification on Windows

The PowerShell helper builds with Minimal SPIFFS and keeps Arduino's normal
persistent compilation cache enabled:

```powershell
.\tests\verify-ota.ps1
```

The default checks the ESP32 Dev Module. Use `-Target esp32s3` for the S3 or
`-Target all` before publishing both images. The first build can take several
minutes; later builds with the same board core and partition setting reuse the
cache and should be substantially faster.

## ⚠️ Electrical safety

- Use a transformer or power supply that matches the valve rating and can support every valve that may run concurrently.
- Never apply more than **3.3 V** directly to an ESP32 GPIO. Use a voltage divider, level converter, or suitable signal-conditioning circuit for 5 V, 10 V, or 0–10 V sensor outputs.
- Solenoids are inductive loads. Use a correctly oriented flyback diode for DC valves. For AC valves, use an appropriately rated MOV or RC snubber—not a DC flyback diode.
- Keep solenoid wiring away from ESP32 and sensor wiring to reduce resets, interference, and electrical noise.
- Enclose, insulate, and protect any mains wiring according to local requirements. Keep it physically separated from low-voltage wiring and use a qualified person where required.

## 🛠️ Troubleshooting

| Problem | Checks |
| --- | --- |
| ESP32 resets when a valve switches | Check suppression, supply capacity, grounding, relay ratings, and cable separation. |
| Relay operates backwards | Change the output polarity in **Setup**. |
| `espirrigation.local` does not open | Use the IP address shown in your router's connected-device list. |
| Weather does not update | Check internet access, latitude, longitude, timezone, and DNS. Local schedules still operate. |
| Wrong valve activates | Check zone numbering, GPIO assignments, relay wiring, and HIGH/LOW polarity. |

## 🔗 Links

- [Web Flasher](https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/)
- [GitHub repository](https://github.com/hjennerway/ESP32-Irrigation-Controller)

If this project helps you, consider giving it a ⭐ on GitHub. Bug reports, testing, and suggestions are welcome.

Beau
