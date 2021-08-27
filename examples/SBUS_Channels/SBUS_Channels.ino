#include <Arduino.h>
#include <SBUS2.h>

uint8_t FrameErrorRate = 0;
int16_t channel[18] = {0};


void setup()
{
  // put your setup code here, to run once:
#if defined (ESP32)
  Serial.begin(115200);
  Serial.println("Setup SBUS2...");
#endif // ESP32

  SBUS2_Setup(25,26);     // For ESP32 set RX and TX Pin Number
  //SBUS2_Setup();        // Default Pin Number and Atmega328

}

void loop()
{
    /* DO YOUR STUFF HERE */

    if(SBUS_Ready()){                               // SBUS Frames available -> Ready for getting Servo Data
      for(uint8_t i = 0; i<10; i++){
        channel[i] = SBUS2_get_servo_data(i);        // Channel = Servo Value of Channel 5
      }
      FrameErrorRate = SBUS_get_FER();
    }
  
#if defined (ESP32)
    delay(1000);
    Serial.print("Servo Channels");
    for(uint8_t i = 0; i<10; i++){
      Serial.print(" ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(channel[i]);
    }
    Serial.println();
    Serial.print("Frame Error Rate: ");
    Serial.println(FrameErrorRate);
#endif // ESP32

} // End of Loop()
