# SBUS2-Telemetry
Arduino Library for receiving SBUS and SBUS2 Frames and transmit Telemetry Data on Atmega328P

## Supported Sensors
- Temp125 Sensor -> Temperatures from -16384°C to + 16383°C
- SBS-01RM -> Rotations per Minute (RPM) from 0 to 393210
- Curr-1678 -> Voltage (0V to 655.3V) / Current (0A to 163.0A) / Capacity Sensor (0mAh to 32767 mAh)
- GPS -1675 -> Speed (0km/h to 999km/h) / Altitude (-16384m to +16383m) / Vario (-3276,8m/s to +3276.7m/s) / LON&LAT (Degree Minutes)

## Supported Radio Systems
- T14SG
- T18MZ
- T10J
- ...

## Supported Receivers
- R7003SB
- R7008SB
- R3006SB
- ...

## Supported MCU
- Arduino Pro Mini 8MHz (with external Inverters)
- Arduino Pro Mini 16MHz (with external Inverters)
- ...


### Inverter Schematic

SBUS2       ---  --- 1K <---Inverter <--- TXD/D1

          |       
          |
          -------> Inverter ---> RXD/D0



## Version

0.1     created

0.2     Inverter instead of 3-State Buffer

0.3     16MHz support, new API

1.0     Pre-Release


## Original Code for Atmel Studio 7
https://bitbucket.org/iBartk/multisensor/overview

Author: Bart Keser

Project: Castle Creation Telemetry to Futaba Telemetry Converter

Ported to Arduino by BrushlessPower

