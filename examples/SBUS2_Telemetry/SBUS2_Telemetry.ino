#include <Arduino.h>
#include <SBUS2.h>

#define TEMPRATURE_SLOT   1     // 1 Slot Sensor
#define RPM_SLOT          2     // 1 Slot Sensor
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

uint8_t FrameErrorRate = 0;




void setup()
{  
  // put your setup code here, to run once:

#if defined (ESP32)
  Serial.begin(115200);
  Serial.println("Setup SBUS2...");
#endif // ESP32

  SBUS2_Setup(25,26);     // For ESP32 set RX and TX Pin Number
  //SBUS2_Setup();        // Default Pin Number and Atmega328

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
  /* DO YOUR STUFF HERE */
#if defined (ESP32)
    delay(1000);
    Serial.print("Frame Error Rate: ");
    Serial.println(FrameErrorRate);
#endif // ESP32   

    if(SBUS2_Ready()){                                    // SBUS2 Frame available -> Ready for transmit Telemetry  
        
      FrameErrorRate = SBUS2_get_FER();                   // get Frame Error Rate
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
  
      //send_f1675_gps(GPS_SLOT, (uint16_t)50, (int16_t)1000, (int16_t) 200, latitude, longitude);                      // Speed = 50km/h, Altitude = 1000m, Vario = 200m/s, lat, lon
      
      send_RPM(RPM_SLOT,(uint16_t)600);                                                                               // RPM = 600-> rounding Error +/- 3 RPM
      //send_s1678_current(CURRENT_SLOT,(uint16_t)2345,(uint16_t)15000,(uint16_t)1234);                                 // Current = 23.45A, Capacity = 15000mAh, Voltage = 12.34V
  
      //send_voltage(VOLTAGE_SLOT,(uint16_t)128, (uint16_t)255);                                                        // Voltage1 = 12.8V, Voltage2 = 25.5V
      
    }
    else{
          // No SBUS2 Frames
    }

} // End of Loop()
