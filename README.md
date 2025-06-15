Irrigation controller degsined to use KC868 6 relay module from aliexpress.


Can be used with any ESP32 controller with 6 relays assign pins in setup page.

If other ESP32 is used it will detect the absent i2c conrollers on the A6 board and use GPIO programmed in settings. 

IO36 Default Analog pin for Tank Level Sensor on A6 and with ESP32 Boards.


---


-Materials Required-


1.) 7 Core irrigation wire to run from controller to solenoid box

2.) 6 Solenoids (MUST BE 12v DC Solenoid Valves (unless you have seperate AC12/24V power scource). 

3.) KC868 6 Channel relay board with case. Or ESP32 Controller with 6 Relay Module

----

https://www.kincony.com/esp32-6-channel-relay-module-kc868-a6.html
![image](https://github.com/user-attachments/assets/113cedeb-a453-42a8-809c-a522808daa87)

----

![download](https://github.com/user-attachments/assets/634f39fa-968c-493c-b1b5-f588702cd1ed)    Relays

![download](https://github.com/user-attachments/assets/b3d3e541-8df6-4f3f-af2c-38f72cae96a2)    ESP32 NodeMCU

----

- Wire all solenoid wires/grounds to power source ground.

- Wire 24/12v to all "COM" screw termials for relays, then wire solenoids to the N.O screw terminals

- Relays 1 - 4 = Zones 1 - 4 

- Relay 5 - Mains, Relay 6 - Tank.

- N.O - COM - N.C.

- Tank level sensor to A1. (3.3V MAX?)

----

![51BA8viI0BL _SX522_](https://github.com/user-attachments/assets/3ca35811-27b2-4bfd-a91e-8748b7463eb3)

----

- Flashing Code to Controller - 


To upload the project to the Kilcoy A6 or ESP32 board use Arduino IDE software, follow these steps.


----


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

Click the Upload button in the Arduino IDE to upload the code to your ESP32 board.


---

5. Check for Successful Upload

After the code is successfully uploaded, open the Serial Monitor (set to 115200 baud rate) to check for any output or errors.
You should now see "ESPIrrigationAP" in your wifi menu connect to it, the wifi manager page should load automatically if not got Goto: https://192.168.4.1 scan for your wifi router select and input your password.

---

6. Access the System

The OLED will show the IP its connected to on startup. 

Type this IP into a browser to access the user interface,

enter your details into the setup page (Openweathermap.org City and API Key, Timezone, ect.) one saved goto main page to setup and save times, days, ect.

By following these steps, your ESP32-based smart irrigation system will be successfully uploaded and configured to work with Adafruit IO and the PCF8574 I/O expander. If you encounter any issues or need further clarification, feel free to ask!

![setuppng](https://github.com/user-attachments/assets/da8d36a7-e759-4e1a-8728-806d3cfdf084)

![Webio](https://github.com/user-attachments/assets/54be9b2d-0afc-45a5-a8e6-372c8818dddc)
---




