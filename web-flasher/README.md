# ESP32 Irrigation Firmware Update

This folder contains a browser-based USB firmware installer for the ESP32 irrigation firmware.

Goto: https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/

## Boards

- ESP32 Dev Module
- ESP32-S3 DevKitC-1

## Use

Use Chrome or Edge. ESP Web Tools needs Web Serial, so Firefox and Safari will not work.

When GitHub Pages is enabled for this repository, open:

```text
https://numerik11.github.io/ESP32-Irrigation-Controller/web-flasher/
```

If that URL returns `404`, GitHub Pages has not been enabled or has not finished deploying yet.

For local testing:

```powershell
cd web-flasher
python -m http.server 8080
```

Then open:

```text
http://localhost:8080
```

Choose the correct board, click Install, and select the ESP serial port when the browser asks.

## Browser OTA

Choose the matching board, then download the application .bin or copy its GitHub firmware URL. Enter the controller's IP address or hostname in the online updater and select **Open controller OTA** to open its /update page in a new tab. The default address is espirrigation.local. Sign in with the OTA password configured on the controller, then upload the file or install from the copied URL.

The online updater does not start an OTA installation automatically. The browser must be able to reach the controller's address.

## Refresh Binaries

After rebuilding firmware, copy the generated files from `.pio/build/<env>/` into the matching board folder:

- `bootloader.bin`
- `partitions.bin`
- `firmware.bin`

Also include `boot_app0.bin` from:

```text
%USERPROFILE%\.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin
```
