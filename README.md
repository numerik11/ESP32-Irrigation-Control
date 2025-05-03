This irrigation controller is degsined to use a Kilcony A6 ESP32 controlled 6 relay module: https://www.kincony.com/esp32-6-channel-relay-module-kc868-a6.html

![image](https://github.com/user-attachments/assets/fe8432ff-ee40-48f5-b75d-dcd55b703e60)

---

Ground solenoids at power scorce ground, 12v to each COM on relay, and solenoids to the N.O side of the relay for each zone/tank/main...  

Tank level sensor to A1 3.3v only 

Relays 1-4 Zones, Relay 5 - Mains, Relay 6 - Tank.

N.O - COM - N.C.

---

To upload the project to the Kilcoy A6-ESP32 board use Arduino IDE software, follow these steps to guide you through selecting the right board, installing the necessary libraries, and uploading the code to your ESP32.

Steps to Upload the Project to ESP32 via Arduino IDE:

---

1. Install/Add the ESP32 Board in Arduino IDE

Open Arduino IDE.

Go to File > Preferences.

In the Additional Boards Manager URLs field, add the following link (if it's not already there):

https://dl.espressif.com/dl/package_esp32_index.json

Go to Tools > Board > Board Manager.

Search for ESP32 and click Install on the esp32 by Espressif Systems package.



---

2. Select the ESP32 Dev Module Board

After installing the ESP32 board package, go to Tools > Board and select ESP32 Dev Module from the list of boards.

Set the following options (for a typical ESP32 Dev board):

Port: Select the COM port corresponding to your ESP32 board.

Flash Frequency: 80 MHz (default).

Upload Speed: 115200 or 921600.

Partition Scheme: Default (4MB).




---

3. Install the PCF8574 Library

Go to PCF8574 Library Download below:

https://www.kincony.com/forum/attachment.php?aid=1697

Download the library file (it should be in .zip format).

In Arduino IDE, go to Sketch > Include Library > Add .ZIP Library....

Select the downloaded .zip file and click Open.


This will add the PCF8574 library to your Arduino IDE.


---

4. Upload Your Code

Now that your ESP32 board is selected and the necessary libraries are installed, you can upload the project code.

Copy and paste the ESP32 irrigation system code (from the project description) into Arduino IDE.

Make sure that the necessary pins and configuration settings are correct (i.e., PCF8574 I/O expander and relay settings).

Click the Upload button in the Arduino IDE to upload the code to your ESP32 board.



---

5. Check for Successful Upload

After the code is successfully uploaded, open the Serial Monitor (set to 115200 baud rate) to check for any output or errors.
You should now see "ESPIrrigationAP" in your wifi menu connect to it, the wifi manager page should load automaticly if not got Goto: https://192.168.4.1 scan select and input your wifi SSID and Password

---

6. Check Connections

Ensure that the LCD is properly connected to the ESP32 board via I2C (SDA, SCL, VCC, and GND).


---

7. Access the System

Once the ESP32 connects to your Wi-Fi, it will start communicating with Adafruit IO. You can monitor and control the system via the Adafruit IO dashboard.

Use the web interface or app to control irrigation zones and monitor weather conditions.



---

8. Test the System

Test the system by activating the irrigation zones via the Adafruit IO dashboard.

Verify that the relays and solenoids turn on/off as expected, and that the system reacts correctly to weather-based automation.



---

By following these steps, your ESP32-based smart irrigation system will be successfully uploaded and configured to work with Adafruit IO and the PCF8574 I/O expander. If you encounter any issues or need further clarification, feel free to ask!

