#include "SBUS2.h"
#include "SBUS_usart.h"
#include <AltSoftSerial.h>

#define CURRENT_SLOT      3
#define RPM_SLOT          2
#define ERROR_SLOT        6
#define TEMPRATURE_SLOT   1

#define GPS_SLOT          8

#define TEMPRATURE_SLOT8   8
#define TEMPRATURE_SLOT9   9
#define TEMPRATURE_SLOT10   10
#define TEMPRATURE_SLOT11   11
#define TEMPRATURE_SLOT12   12
#define TEMPRATURE_SLOT13   13
#define TEMPRATURE_SLOT14   14
#define TEMPRATURE_SLOT15   15

// Dezimal Koordinaten Berlin Fernsehturm
int32_t latitude = 52520833;    // 52° 31' 14.9988"
int32_t longitude = 13409430;   // 13° 24' 33.9480"

// AltSoftSerial always uses these pins:
//
// Board          Transmit  Receive   PWM Unusable
// -----          --------  -------   ------------
// Teensy 3.0 & 3.1  21        20         22
// Teensy 2.0         9        10       (none)
// Teensy++ 2.0      25         4       26, 27
// Arduino Uno        9         8         10
// Arduino Leonardo   5        13       (none)
// Arduino Mega      46        48       44, 45
// Wiring-S           5         6          4
// Sanguino          13        14         12

AltSoftSerial altSerial;


void setup() {
  // put your setup code here, to run once:

  pinMode(2, OUTPUT);             // set pin D2 as Output  
  pinMode(10, OUTPUT);            // set pin D10 as Output  
  pinMode(13, OUTPUT);            // set pin D10 as Output  
  digitalWrite(2, HIGH);          // set pin D2 High = Inverted UART signal
  digitalWrite(10, LOW);          // Debug
  digitalWrite(13, LOW);          // Debug
  SBUS2_Setup();
  altSerial.begin(19200);
  
}

void loop() {
  int16_t channel = 0;
  uint16_t uart_dropped_frame = 0;
  bool transmision_dropt_frame = false;
  bool failsave = false;

  //digitalWrite(10, HIGH);       // Debug
  // put your main code here, to run repeatedly:
  delay(1000);       // Simulation of User Code, for example measuring Voltage, Temperature, Current, .... -> Maybe causing dropped frames
  
  
  //if(SBUS2_Ready()){
    //SBUS2_get_all_servo_data(&channels[0]);
    channel = SBUS2_get_servo_data( 2 );        // Channel = Sbus Value of Channel 2
    //if (channel != -1 ){
      //digitalWrite(10, LOW);       // Debug
      SBUS2_get_status(&uart_dropped_frame, &transmision_dropt_frame, &failsave);
      send_alarm_as_temp125(ERROR_SLOT, ((failsave*1000) + (transmision_dropt_frame*100) + uart_dropped_frame));      // Warning with over Temp at Error Slot
      
      send_temp125(TEMPRATURE_SLOT, (int16_t)50);                                     // Temperature [°C]
      send_RPM(RPM_SLOT,(uint16_t)600);                                               // RPM = 600-> rounding Error +/- 3 RPM
      send_s1678_current(CURRENT_SLOT,(uint16_t)20,(uint16_t)15000,(uint16_t)1230);   // Current = 20A, Capacity = 15000mAh, Voltage = 12.30V
      
      //send_f1675_gps(GPS_SLOT, (uint16_t)50, (int16_t)1000, (int16_t) 200);           // Speed = 50km/h, Altitude = 1000m, Vario = 200m/s
      send_f1675_gps(GPS_SLOT, (uint16_t)50, (int16_t)1000, (int16_t) 200, latitude, longitude);           // Speed = 50km/h, Altitude = 1000m, Vario = 200m/s

      /*send_temp125(TEMPRATURE_SLOT8, (int16_t)50);                                                                     // Temperature [°C]
      send_temp125(TEMPRATURE_SLOT9, (int16_t)50); 
      send_temp125(TEMPRATURE_SLOT10, (int16_t)50); 
      send_temp125(TEMPRATURE_SLOT11, (int16_t)50); 
      send_temp125(TEMPRATURE_SLOT12, (int16_t)50); 
      send_temp125(TEMPRATURE_SLOT13, (int16_t)50); 
      send_temp125(TEMPRATURE_SLOT14, (int16_t)50);
      send_temp125(TEMPRATURE_SLOT15, (int16_t)50);*/
    //}
  //} 
  //channel = 1000;
  altSerial.println(channel);
  
} // End of Loop()
