## Program Description

* ReadIMU() & ReadHeartRate() contantly read the sensors and update the global variables.
* SendXML(sub_device_id) sends a custom XML message. The message is broken up into several smaller messages, because the larger string seemed to be too much for the Arduino to handle
* UpdateOLED() updates the display with stored sensor information
* SetVibrationMotor(bool) sets the vibration motor on/off

## Heart Rate Sensor
The heart rate sensor is finicky, and seems to work better at 115200 baud for some reason. You need to hold your finger down on it for some time before it shows a result.

Library: 
https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library

Tutorial:
https://learn.sparkfun.com/tutorials/sparkfun-photodetector-max30101-hookup-guide

## IMU
Library:
https://github.com/adafruit/Adafruit_MPU6050

## Display
The typical Adafruit SSD1306 OLED driver was too large for this project. I found a smaller, simpler driver:
https://github.com/g3rb3n/SSD1306

## Bluetooth
I had Bluetooth in and working via SoftwareSerial, and had it so the stream can be passed to SendXML function. I took it out due to space issue, but it might be okay now that
the OLED memory issue has been corrected. Would be easy to add back in.
