#include <Arduino.h>
#include <SBUS2.h>

#define RPM_SLOT          7     // 1 Slot Sensor

unsigned long sysmillis;
uint8_t lowbyte= 0;
uint8_t highbyte = 0;
uint16_t value = 0;


void setup()
{
  // put your setup code here, to run once:
  
#if defined (ESP32)
  Serial.begin(115200);
  Serial.println("Setup SBUS2...");
#endif // ESP32

  /*
   * Simulation Mode
   * - Never use this Mode together with a connected RX (R7008,7003,.....)
   * - Output SBUS2 Frames to Simulate a RX for testing Sensors without radio
   */
  SBUS2_Simulation(25,26);          // SBUS2 Simulation on RX = IO25 and TX = IO26

  sysmillis = millis();
}

void loop()
{
    /* DO YOUR STUFF HERE */
    if(millis() > sysmillis){
      sysmillis += 1000;
      for(uint8_t i = 0; i<32;i++){                     // Check all 32 Slots for valid Telemetry Data
        if(SBUS2_get_Slot(i, &highbyte, &lowbyte)){     // if true, new data is availabe on this Slot
          value = ((highbyte << 8) | lowbyte);          // See send_RPM(uint8_t port, uint32_t RPM) calculation in SBUS2.cpp
          Serial.print("Slot: ");
          Serial.print(i);
          Serial.print(" LowByte = ");
          Serial.print(lowbyte,HEX);
          Serial.print(" HighByte = ");
          Serial.print(highbyte,HEX);
          Serial.print(" Value = ");
          Serial.println(value);                        // Value shows the Raw value and not necessarily the physical value (depends on Sensor)
        }
      }
    }

    if(SBUS2_Ready()){                                  // SBUS2 Frame available -> Ready for transmit Telemetry    
      send_RPM(RPM_SLOT,(uint16_t)500);                 // RPM = 600-> rounding Error +/- 3 RPM  
    }
    else
    {
        // No SBUS2 Frames
    }

} // End of Loop()
