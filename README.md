# SBUS2-Telemetry Library
Arduino Library for receiving SBUS and SBUS2 Frames and transmit Telemetry Data with Atmega328P MCU
The Library uses the U(S)ART Interrupt and Timer2 -> You can't use Serial in your Sketch!
Please use Softserial instead.

## Setting up the Example Sketch
- Before you start flashing and coding, please Setup you Futaba Radio Control
  - Select Modulation-> FASSTest -> 14CH and bind your Receiver
  - You should see the SerialNumber of the Receiver
  - In Telemetry Screen, you should see the RX Voltage
  - Got to "Sensors" Menu and activate the Inactive Slots
  - See #defines in the example Sketch -> #define TEMPRATURE_SLOT   1 
  - Set Slot1 (Inactive) to Temp125 Sensor
  - Do the same with all other Sensors defined in the Example Sketch
- Now build up you Arduino Hardware
  - Build your Inverter Circuit
  - Flash the Sktech -> On Arduino Pro Mini you must flash without attached Inverter!
  - Attach The Inverter to RX and TX on the Arduino Board
  - Attach the Inverter to SBUS2 Port on your Receiver
- Power up
  - You can power your Futaba Receiver from Arduino (5V)
  - Or you power your Arduino from Futaba Receiver (BEC)
  - You will See Telemetry Data on your Telemetry Screen
  
## Setting up a custom Sketch
You can't set every Sensor to every Slot!
There are Sensors which use 1 Slot and other Sensors use 3 or 8 Slots
After receiving the SBUS Frame it's possible to transmit 8 Telemetry Slots
There are 32 different Telemetry Slots available

### Example
- Slot 0 is always RX Voltage-> So you cant use Slot0 for custom Sensors
- So you have 7 Slots left (Slot 1 to Slot 7)
- So you cant use a 8 Slot Sensor (GPS) on Slot 1!
- GPS has to be on Slot 8, or Slot 16, or Slot 24
- On Slot 1 you could set a Temperature Sensor (1 Slot Sensor)
- On Slot 2 you could set a RPM Sensor (1 Slot Sensor)
- On Slot 3 you could set a Power Sensor (3 Slot Sensor)
- The Power Sensor uses Slot 3, Slot 4 and Slot 5 (3 Slot's!)
- The next free Slot would be Slot 6
- Slot 6 can't be a Power Sensor, because you have just 2 Slots free!

### The easiest Way for a Working Setup
- First set all Sensors in your Futaba Radio Control
- Your Futaba System will keep the things clear.
- After you set up all your Favorit Sensors in your Radio, you can #define the used Slot's in the Sketch
- Take a look which Radio support which Sensor  ![PDF](https://github.com/BrushlessPower/SBUS2-Telemetry/blob/master/futaba-sensors.pdf)


## Supported Sensors
- Temp125 -> Temperatures from -16384°C to + 16383°C
- SBS/01TE -> Temperatures from -16384°C to + 16383°C
- F1713 -> Temperatures from -16384°C to + 16383°C
- SBS-01RM -> Rotations per Minute (RPM) from 0 to 393210
- SBS/01RB -> Rotations per Minute (RPM) from 0 to 393210
- SBS-01RO -> Rotations per Minute (RPM) from 0 to 393210
- Curr-1678 -> Voltage (0V to 655.3V) / Current (0A to 163.0A) / Capacity Sensor (0mAh to 32767 mAh)
- F1678 -> Voltage (0V to 655.3V) / Current (0A to 163.0A) / Capacity Sensor (0mAh to 32767 mAh)
- GPS-1675 -> Speed (0km/h to 999km/h) / Altitude (-16384m to +16383m) / Vario (-3276,8m/s to +3276.7m/s) / LON&LAT (Degree Minutes)
- F1675 -> Speed (0km/h to 999km/h) / Altitude (-16384m to +16383m) / Vario (-3276,8m/s to +3276.7m/s) / LON&LAT (Degree Minutes)
- SBS/01V -> comming soon
- F1672 -> comming soon
- F1712 -> comming soon


## Unsupported Sensors
- SBS-01G
- SBS/01S
- SBS-0 1 TAS
- F1677
- SBS/01C
- P-SBS/01T
- SBS/02A
- SBS/01A

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
- ESP32 (https://github.com/BrushlessPower/SBUS2-Telemetry-ESP32)
- ...


## Inverter Schematic

![correct inverter](https://github.com/BrushlessPower/SBUS2-Telemetry/blob/master/SBUS2_inverter.png)


## Guide for Library Development
The Futaba Telemetry Protocol has very hard Timings:
- Slot 0 must be send 2ms after the last Byte of the SBUS(2) Frame 
- same for Slot 8, Slot 16 and Slot 24
- After every Slot's there must be 325µs Pause to the next Slot
- So you have to receive the SBUS Frame in a UART Interrupt
- And with the Last Byte of the Frame you have to Start a Timer with 2ms
- With every Timer Interrupt you have to set the next Timer Interrupt to 660µs
- If the Timer doesn't in this conditions, you do not need to start developing the Slot transmission
- You need to have a Logic Analyser or Oscilloscope
- SBUS(2) Level Voltage is 3,3V -> Do not work with 5V Level Signals!

## Aditional Informations about Futaba's SBUS and SBUS2
- SBUS(2) is 100000 Baud with 8 data bits, even parity and 2 stop bits
- SBUS(2) Signal is inverted UART
- SBUS is a Frame with 25 Bytes
  - Byte [0] is 0x0F
  - Byte [1-22] is Servo Channel Data
  - Byte [23] is DigiChannel 17&18 + Stus Bits
  - Byte [24] is 0x00
- SBUS2 is a SBUS Frame but with a diffent End Byte (Byte24)
  - Byte [24] is 0x04, 0x14, 0x24, 0x34
  - Byte 24 controls the Number of the Telemetry Slot's
    - 0x04 -> Slot 0 to Slot 7
    - 0x14 -> Slot 8 to Slot 15
    - 0x24 -> Slot 16 to Slot 23
    - 0x34 -> Slot 24 to Slot 31

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

