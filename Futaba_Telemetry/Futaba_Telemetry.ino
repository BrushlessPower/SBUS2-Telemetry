#include "SBUS2.h"
#include "SBUS_usart.h"

#define CURRENT_SLOT      3
#define RPM_SLOT          2
#define ERROR_SLOT        6
#define TEMPRATURE_SLOT   1

void setup() {
  // put your setup code here, to run once:

  pinMode(2, OUTPUT);           // set pin D2 as Output  
  digitalWrite(2, HIGH);       // set pin D2 High = Inverted UART signal
  SBUS2_Setup();
  
}

void loop() {
  int16_t channel = 0;
  uint16_t uart_dropped_frame = false;
  bool transmision_dropt_frame = false;
  bool failsave = false;
  
  // put your main code here, to run repeatedly:
  delay(100);       // Simulation of User Code, for example measuring Voltage, Temperature, Current, .... -> Maybe causing dropped frames
  
  if(SBUS2_Ready()){
    channel = SBUS2_get_servo_data( 2 );        // Channel = Sbus Value of Channel 2
    if (channel != -1 ){
      SBUS2_get_status(&uart_dropped_frame, &transmision_dropt_frame, &failsave);
      send_alarm_as_temp125(ERROR_SLOT, ((failsave*1000) + (transmision_dropt_frame*100) + uart_dropped_frame));      // Warning with over Temp at Error Slot
      send_temp125(TEMPRATURE_SLOT, (int16_t)50);                                                                     // Temperature [°C]
      send_RPM(RPM_SLOT,(uint16_t)600);                                                                               // RPM -> rounding Error +/- 3 RPM
      send_s1678_current(CURRENT_SLOT,(uint16_t)20,(uint16_t)15000,(uint16_t)1230);                                   // Current 20A, Capacity 15000mAh, Voltage 12.30V
    }
  } 

  
} // End of Loop()
