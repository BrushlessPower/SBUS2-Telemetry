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
  SBUS2_Setup(CURRENT_SLOT, TEMPRATURE_SLOT, RPM_SLOT, ERROR_SLOT);
}

void loop() {
  // put your main code here, to run repeatedly:
    SBUS2_loop();
}
