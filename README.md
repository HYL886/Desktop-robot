# Desktop Robot

ESP32-C3 desktop robot firmware with:

- L298N mini motor driver control
- SSD1315 0.96 inch OLED eye animation
- Local WiFi hotspot web controller
- Forward, backward, left, right, stop commands
- Sleep, wiggle, and curious random motion modes

## Hardware Pins

| ESP32-C3 GPIO | Connected To | Function |
| --- | --- | --- |
| GPIO 0 | Motor driver IN1 | Left motor forward |
| GPIO 1 | Motor driver IN2 | Left motor backward |
| GPIO 2 | Motor driver IN3 | Right motor forward |
| GPIO 3 | Motor driver IN4 | Right motor backward |
| GPIO 10 | Motor driver STBY | Motor enable |
| GPIO 8 | OLED SDA | I2C data |
| GPIO 9 | OLED SCL | I2C clock |

## Build

Use ESP-IDF and set the target to ESP32-C3:

```powershell
. C:\ESP-IDF\.espressif\v5.5.4\esp-idf\export.ps1
idf.py set-target esp32c3
idf.py build
```

## Flash

Replace `COMx` with the board port:

```powershell
idf.py -p COMx flash monitor
```

## Web Control

After boot, connect your phone or computer to the robot hotspot:

```text
SSID: 桌面机器人
URL:  http://192.168.4.1
```

This firmware only provides the robot's local hotspot web page. It does not connect to home WiFi.
