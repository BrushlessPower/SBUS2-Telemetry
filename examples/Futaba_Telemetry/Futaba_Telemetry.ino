#include <Arduino.h>
#include <SBUS2.h>


#define TEMPRATURE_SLOT   1     // 1 Slot Sensor
#define RPM_SLOT          1     // 1 Slot Sensor
#define CURRENT_SLOT      3     // 3 Slot Sensor
      //CURRENT_SLOT      4
      //CURRENT_SLOT      5
#define ERROR_SLOT        6     // 1 Slot Sensor

#define GPS_SLOT          8     // 8 Slot Sensor

#define VOLTAGE_SLOT      16    // 2 Slot Sensor
#define VARIO_SLOT        18    // 2 Slot Sensor

// Koordinaten Berlin Fernsehturm  = Grad Minuten  = Dezimalgrad = Grad Minuten Sekunden
int32_t latitude = 52312499;  // = N 52° 31.2499 = N 52.520833 = N 52° 31' 14.9988"
int32_t longitude = 13245658; // = E 13° 24.5658 = E 13.409430 = E 13° 24' 33.9480"

int16_t channel = 0;
uint8_t FrameErrorRate = 0;

void setup()
{
    // put your setup code here, to run once:
    pinMode(16,OUTPUT);
    digitalWrite(16,LOW);
    
#if defined (ESP32)
    Serial.begin(115200);
    Serial.println("Setup SBUS2...");
#endif // ESP32

    SBUS2_Setup(25,26);
    //SBUS2_Setup();
    
#if defined (ESP32)
    Serial.println("OK");
#endif // ESP32

    while(!SBUS2_Ready()){
      // Wait for valid SBUS2 Signal
#if defined (ESP32)
    delay(2000);
    Serial.println("Wait for SBUS2");
#endif // ESP32
    }
    
    // GPS Distance and Altitude is set to Zero with the first GPS Telemetry Data
    // GPS Distance and Altitude is calculated with changing Latitude, Longitude and Altitude
    /*for(uint8_t i = 0; i< 10; i++){
      send_f1675_gps(GPS_SLOT, (uint16_t)0, (int16_t)0, (int16_t) 0, (latitude+100), (longitude+100));           // Speed = 0km/h, Altitude = 0m, Vario = 0m/s
      send_f1672_vario(VARIO_SLOT, (int16_t) 0, (int16_t) 0);
      delay(50);
    }*/

}

void loop()
{
    uint16_t uart_dropped_frame = 0;
    bool transmision_dropt_frame = false;
    bool failsave = false;
    uint8_t lowbyte= 0;
    uint8_t highbyte = 0;
    uint16_t rpm = 0;

    /* DO YOUR STUFF HERE */
#if defined (ESP32)
    delay(1000);
    Serial.print("Servo Channel 2: ");
    Serial.println(channel);
    Serial.print("Frame Error Rate: ");
    Serial.println(FrameErrorRate);
    //SBUS2_print_Raw();
    SBUS2_get_Slot(RPM_SLOT, &lowbyte, &highbyte);
    rpm = ((highbyte << 8) | lowbyte) * 6;          // See send_RPM(uint8_t port, uint32_t RPM) calculation in SBUS2.cpp
    Serial.print("Slot: ");
    Serial.print(RPM_SLOT);
    Serial.print(" LowByte: ");
    Serial.print(lowbyte,HEX);
    Serial.print(" HighByte: ");
    Serial.print(highbyte,HEX);
    Serial.print(" RPM: ");
    Serial.println(rpm);
#endif // ESP32

    if(SBUS_Ready(true)){                             // SBUS Frames available -> Ready for getting Servo Data
      channel = SBUS2_get_servo_data( 2 );        // Channel = Servo Value of Channel 5
    }
  
    
    if(SBUS2_Ready(true)){                                                                                                // SBUS2 Frame available -> Ready for transmit Telemetry                                                                                    // set pin D13 (LED ON) -> SBUS2 Frames OK                                      
      FrameErrorRate = SBUS2_get_FER();
      
      //send_temp125(TEMPRATURE_SLOT, (int16_t) 50);
      
      SBUS2_get_status(&uart_dropped_frame, &transmision_dropt_frame, &failsave);                                     // Check SBUS(2) Status
      if((uart_dropped_frame > 1) ||(transmision_dropt_frame != 0) || (failsave != 0) ){
          //send_alarm_as_temp125(ERROR_SLOT, ((failsave*1000) + (transmision_dropt_frame*100) + uart_dropped_frame));
#if defined (ESP32)
          Serial.print("UART dropped Frame: ");
          Serial.println(uart_dropped_frame);
          Serial.print("Transmission dropped Frame: ");
          Serial.println(transmision_dropt_frame);
          Serial.print("Failsave: ");
          Serial.println(failsave);
#endif // ESP32
          uart_dropped_frame = 0;
          transmision_dropt_frame = false;
          failsave = false;
        }
      
  
      //send_F1672(VARIO_SLOT , 1234, (float)23.45);                                                                    // Altitude = 1234m; Vario = 23,45 m/s; 
  
      //send_f1675_gps(GPS_SLOT, (uint16_t)50, (int16_t)1000, (int16_t) 200, latitude, longitude);
      Serial.println("send Telemetry");
      send_RPM(RPM_SLOT,(uint16_t)600);                                                                               // RPM = 600-> rounding Error +/- 3 RPM
      send_s1678_current(CURRENT_SLOT,(uint16_t)2345,(uint16_t)15000,(uint16_t)1234);                                 // Current = 23.45A, Capacity = 15000mAh, Voltage = 12.34V
  
      //send_voltage(VOLTAGE_SLOT,(uint16_t)128, (uint16_t)255);                                                        // Voltage1 = 12.8V, Voltage2 = 25.5V

      // Select Port 8, 16 or 24
      // example below is 
      // 2110 => 21.1V
      // 1330 => 1.33 Ah used
      // 600 => 600 RPM 
      // 2345 = Current 23.45A
      // 30 = Temp 30 degress
      // 25 = BecTemp 25 degress
      // 1345 = BecCurrent 13.45 degress
      // 54 = 54% PWM
      // Send Scorpion and Kontronik
      //   send_scorpion(8, (uint16_t) 2110, (uint16_t) 1330, (uint16_t)600, (uint16_t)2345, (uint16_t) 30, (uint16_t) 25, (uint16_t)1345, (uint16_t) 54);
      //   send_kontronik(16, (uint16_t) 2110, (uint16_t) 1330, (uint16_t)600, (uint16_t)2345, (uint16_t) 30, (uint16_t) 25, (uint16_t)1345, (uint16_t) 54);
    }
    else
    {
        // No SBUS2 Frames
    }


} // End of Loop()
