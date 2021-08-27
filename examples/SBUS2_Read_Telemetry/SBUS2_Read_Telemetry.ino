#include <Arduino.h>
#include <SBUS2.h>

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

  SBUS2_Setup(25,26);     // For ESP32 set RX and TX Pin Number

  sysmillis = millis();
}

void loop()
{
    /* DO YOUR STUFF HERE */

    /*
     * The Read Function should always show SLOT0 -> The RX Voltage and External RX Voltage (alternate from Frame to Frame)
     */
    if(millis() > sysmillis){
      sysmillis += 1000;
      for(uint8_t i = 0; i<32;i++){                     // Check all 32 Slots for valid Telemetry Data
        if(SBUS2_get_Slot(i, &highbyte, &lowbyte)){     // if true, new data is availabe on this Slot
          value = ((highbyte << 8) | lowbyte);          
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

} // End of Loop()
