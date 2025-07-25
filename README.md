Irrigation controller degsined to use KC868 6 relay module from Aliexpress.


Can also be used with any ESP32 module with the addition of a 6 relay module, you can assign pins in the setup page.
IO36 is Default Analog pin for Tank Level Sensor on K6-KC868 and with ESP32 Boards.

Youll need to get a free API Key for weather data at: 

[openweathermap.org ](https://home.openweathermap.org/users/sign_up)

Key Features:
----
Dashboard:
----
Tank levels, weather (OpenWeatherMap), current zone status, rain/wind delays, and scheduled runs.

Zones:
----
Configure schedules for up to 6 irrigation zones.

Manual override buttons for each zone (“On”/“Off”).

Setup Page:
----
Set API keys, city/region, time offsets, wind/rain/tank options, and GPIO assignments.

Other:
----
Works with ESP32 (Arduino IDE framework).

Tank Level calibration, save Empty and Full Readings.

----
Materials Required:
----

- 7 Core irrigation wire to run from controller to solenoid valve box
  
- 6 Irrigatoion Solenoids (Powered by seperate AC12/24V source, depending on irrigation solenoid power reqirements) Ive used 20mm 12v DC Microsolenoids using the same power input scource of 12v 1.5a.
  
- KC868 6 Channel relay board with case. Or ESP32 Controller with 6 Relay Module
  

----
Wiring Instrustions:
----

- Wire all solenoid wires/grounds to power source ground.
- Wire 24/12v to all "COM" screw termials for relays, then wire irrigation solenoids to the N.O screw terminals
- Relays 1 - 4 = Zones 1 - 4 - Relay 5 - Mains, Relay 6 - Tank.
- Tank level sensor to A1. (3.3V MAX?)

----
Flashing Code to Controller:
----

- To upload the project to the Kilcoy A6 or ESP32 board use Arduino IDE software, follow these steps.

----

1. Install/Add the ESP32 Board in Arduino IDE

Open Arduino IDE.
Go to File > Preferences.
In the Additional Boards Manager URLs field, add the following link (if it's not already there):

https://dl.espressif.com/dl/package_esp32_index.json

Once added You should now be able to Search for ESP32 in boards manager,
click Install on the esp32 by Espressif Systems package.

Go to: Tools > Board > Board Manager.
---

2. Select the ESP32 Dev Module Board for K6 or select your esp module if using a different module.

After installing the ESP32 board package, go to Tools > Board ESP32 Dev Module from the list of boards.
Set the following options:

Port: Select the COM port corresponding to your ESP32 board.
Flash Frequency: 80 MHz (default).
Upload Speed: 115200 or 921600.
Partition Scheme: Default (4MB).

---

3. Install the PCF8574 Library (FOR K6-KC868 I2C)

Go to PCF8574 Library Download below:

https://www.kincony.com/forum/attachment.php?aid=1697

Download the library file (it should be in .zip format).
In Arduino IDE, go to Sketch > Include Library > Add .ZIP Library....
Select the downloaded .zip file and click Open.

This will add the PCF8574 (K6-KC868) library to your Arduino IDE.


---

4. Upload Your Code

Now that your ESP32 board (ESP32 Dev Module Board for K6) is selected and all necessary libraries are installed, you can upload the code.
Open ESP32-Irrigation.ino or Irrigation6Zone.ino file with Arduino IDE.
Click the Upload arrow button in the Arduino IDE to upload the code to your ESP32 board.


---

5. Check for Successful Upload

After the code is successfully uploaded, you can open the Serial Monitor (set to 115200 baud rate) to check for any output or errors.
You should now also see "ESPIrrigationAP" in your wifi menu on your phone or pc connect to it,
Wifi manager page should popup automatically if not got Goto: https://192.168.4.1 then scan for your wifi router name select and input your password.

---

6. Access the System

The OLED will show the IP its connected to on startup. 
Or type "arp -a" into command prompt and find it in the list.
Type this IP into a browser to access the irrigation control homepage goto setup page,
Enter your details into the setup page (City ID, API Key and Timezone) Once setup is saved goto home page to setup and save times, days, ect.

<img width="236" height="58" alt="image" src="https://github.com/user-attachments/assets/eb369697-5fc7-436d-93eb-d64fb0faf5b2" /> - City ID

By following these steps, your ESP32-based smart irrigation system will be configured. If you encounter any issues or need further clarification, feel free to ask!


---

https://www.kincony.com/esp32-6-channel-relay-module-kc868-a6.html
<img width="791" height="754" alt="image" src="https://github.com/user-attachments/assets/ed356fc6-4ed2-4b8e-8a93-de206ef08a92" />

Kilkony KC868-K6

---

![51BA8viI0BL _SX522_](https://github.com/user-attachments/assets/3ca35811-27b2-4bfd-a91e-8748b7463eb3) 
Tank level sensor

---

![download](https://github.com/user-attachments/assets/634f39fa-968c-493c-b1b5-f588702cd1ed) 
Relays

---

![download](https://github.com/user-attachments/assets/b3d3e541-8df6-4f3f-af2c-38f72cae96a2)  

ESP32 NodeMCU

---

<img width="491" height="793" alt="image" src="https://github.com/user-attachments/assets/f79826b6-1607-4128-b457-18ad0a56ca95" />
4 Zone w/Tank/Main Control

<img width="621" height="816" alt="image" src="https://github.com/user-attachments/assets/55c5281c-fc18-4f83-958a-1e78213eee66" />
6 Zone

---

<img width="354" height="638" alt="image" src="https://github.com/user-attachments/assets/531b57cc-fbef-48b7-88c3-558be41420d7" />
Setup

---

<img width="343" height="294" alt="image" src="https://github.com/user-attachments/assets/64398358-6fa6-4831-9fb2-2bdc4a0b0a8e" />
Tank Calibration

---

<img width="758" height="584" alt="image" src="https://github.com/user-attachments/assets/c8ee1952-1950-4785-acd9-3eb0f2e94e7e" />
Event Logger

---
