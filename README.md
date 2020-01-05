# Overview
This code allows the Arduino device to store the time, and speed of the GPS unit and accelerometer values in m/s^2 to a micro SD card where the file creates under the name GPSData.TXT.

# Dependency
This code requires the installation of the [Adafruit_GPS.h](https://learn.adafruit.com/adafruit-ultimate-gps-featherwing/arduino-library) library for it to work 

# Pin Layout
Here you will find the pin for each modal and its corresponding pin on the Arduino

## GPS Breakout:
- VIN to +5V
- GND to Ground
- RX to digital 2
- TX to digital 3

## SD Card Breakout:
- MOSI to digital 11
- MISO to digital 12
- SCK (CLK) to digital 13
- CS to digital 4

## Accelerometer:
- GND to Ground
- X_OUT to analog 0
- Y_OUT to analog 1
- Z_OUT to analog 2
- VCC to +5V

## LED:
- digital 8

## 2X16LCD: 
- enable 5
- rs     A3
- d4     6
- d5     7
- d6     8
- d7     9

## ButtonPin:
- A4
