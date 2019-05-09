/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*  @ Project : MultiSensor
*  @ File Name : SBUS2.cpp
*  @ Date : 6/12/2013
*  @ Author : Bart Keser
*
*/

#include "SBUS2.h"
#include "SBUS_usart.h"

#define SBUS2_TEL_DATA_SIZE        3
//#define CURRENT_SENSOR_SLOT     0x04
//#define TEMPERATURE_SENSOR_SLOT 0x0A
//#define RPM_SENSOR_SLOT         0x0A

//static void     do_servo_pulse_callback(uint32_t counter);

// absolute timer for capacity calculation
//static volatile uint32_t  absTime        = 0;
//static volatile bool      do_servo_pulse = false;

/*static uint8_t currentPort     = 0;
static uint8_t temperaturePort = 0;
static uint8_t rpmPort         = 0;
static uint8_t alarmPort       = 0;*/

//static int16_t   SBUS_throttle;

/*void SBUS2_Setup(uint8_t current_port, 
				 uint8_t temperature_port, 
				 uint8_t rpm_port, 
				 uint8_t alarm_port 
				 )
{
   currentPort     = current_port;
   temperaturePort = temperature_port;
   rpmPort         = rpm_port;
   alarmPort       = alarm_port;
   SBUS2_uart_setup(do_servo_pulse_callback);
}*/

void SBUS2_Setup()
{
  SBUS2_uart_setup(/*do_servo_pulse_callback*/);
}

/*void send_RPM(uint16_t RPM)
{
   int16_t value =  0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x00, 0x00 };

   value =  RPM / 6;
   bytes[2] = value >> 8;
   bytes[1] = value;
   SBUS2_transmit_telemetry_data( rpmPort , bytes);
}*/

void send_RPM(uint8_t port, uint16_t RPM)
{
   int16_t value =  0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x00, 0x00 };

   value =  RPM / 6;
   bytes[2] = value >> 8;
   bytes[1] = value;
   SBUS2_transmit_telemetry_data( port , bytes);
}

/*void send_temp125(int16_t temp)
{
   int16_t value=  0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };

   value = temp | 0x4000;
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( temperaturePort , bytes);
}*/

void send_temp125(uint8_t port, int16_t temp)
{
   int16_t value=  0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };

   value = temp | 0x4000;
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( port , bytes);
}

/*void send_alarm_as_temp125(int16_t alarm)
{
   int16_t value=  0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };

   value = alarm | 0x4000;
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( alarmPort , bytes);
}*/

void send_alarm_as_temp125(uint8_t port, int16_t alarm)
{
   int16_t value=  0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };

   value = alarm | 0x4000;
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( port , bytes);
}

/*void send_s1678_current(uint16_t current, uint16_t capacity, uint16_t voltage)
{
   uint16_t value = 0;
   uint32_t local = 0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };
 
   
   // CURRENT
   local = ((uint32_t)current) * 100 ;
   value = (uint16_t)local;   
   if ( value > 0x3FFF )
   {
      // max current is 163.83
      value = 0x3FFF;
   }  
   bytes[1] = value >> 8;
   bytes[1] = bytes[1] | 0x40;
   bytes[1] = bytes[1] & 0x7F;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( currentPort , bytes);

   //VOLTAGE
   local = ((uint32_t)voltage) * 100;
   value = (uint16_t)local;   
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( currentPort+1 , bytes);

   // CAPACITY
   local = (uint32_t)capacity;
   value = (uint16_t)local;   
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( currentPort+2 , bytes);
}*/

void send_s1678_current(uint8_t port, uint16_t current, uint16_t capacity, uint16_t voltage)
{
   uint16_t value = 0;
   uint32_t local = 0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };
 
   
   // CURRENT
   local = ((uint32_t)current) * 100 ;
   value = (uint16_t)local;   
   if ( value > 0x3FFF )
   {
      // max current is 163.83
      value = 0x3FFF;
   }  
   bytes[1] = value >> 8;
   bytes[1] = bytes[1] | 0x40;
   bytes[1] = bytes[1] & 0x7F;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( port , bytes);

   //VOLTAGE
   local = ((uint32_t)voltage) * 1;
   value = (uint16_t)local;   
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( port+1 , bytes);

   // CAPACITY
   local = (uint32_t)capacity;
   value = (uint16_t)local;   
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( port+2 , bytes);
}

void send_f1675_gps(uint8_t port, uint16_t speed, int16_t altitude, int16_t vario, int32_t latitude, int32_t longitude)
{
   uint16_t value1 = 0;
   int16_t  value2 = 0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };
 
   
   // SPEED
   value1 = speed | 0x4000;
   if ( value1 > 0x43E7 )
   {
      // max Speed is 999 km/h
      value1 = 0x43E7;
   }  
   bytes[1] = value1 >> 8;
   bytes[2] = value1;
   SBUS2_transmit_telemetry_data( port , bytes);

   //HIGHT
   value2 = altitude | 0x4000;
   bytes[1] = value2 >> 8;
   bytes[2] = value2;
   SBUS2_transmit_telemetry_data( port+1 , bytes);

   //TIME -> 12:30 ? MSBit Unix?
   bytes[1] = 0x0C;
   bytes[2] = 0x1E;
   SBUS2_transmit_telemetry_data( port+2 , bytes);

   // VARIO
   value2 = vario * 10;   
   bytes[1] = value2 >> 8;
   bytes[2] = value2;
   SBUS2_transmit_telemetry_data( port+3 , bytes);

   // LATITUDE
   bytes[1] = (uint8_t)(latitude/1000000);
   //bytes[1] = 0x34;         // 52°
   value1 = (uint16_t)(latitude%1000000);
   if(latitude >= 0){
    bytes[2] = ((value1 >> 12) & 0x0f);    // North
   }
   else{
    bytes[2] = ((value1 >> 12) & 0x1f);    // South
   }
   //bytes[2] = 0x07;         // N (0x0X) or S (0x1X)
   SBUS2_transmit_telemetry_data( port+4 , bytes);

   bytes[1] = ((value1 >> 8) & 0xff);
   //bytes[1] = 0xF2;
   bytes[2] = value1 & 0xff;
   //bytes[2] = 0x71;
   SBUS2_transmit_telemetry_data( port+5 , bytes);

   // LONGITUDE
   bytes[1] = (uint8_t)(longitude/1000000);
   //bytes[1] = 0x0D;         // 13°
   value1 = (uint16_t)(longitude%1000000);
   if(longitude >= 0){
    bytes[2] = ((value1 >> 12) & 0x0f);    // Eath
   }
   else{
    bytes[2] = ((value1 >> 12) & 0x1f);    // West
   }
   //bytes[2] = 0x06;         // E (0x0X) or W (0x1X)
   SBUS2_transmit_telemetry_data( port+6 , bytes);

   bytes[1] = ((value1 >> 8) & 0xff);
   //bytes[1] = 0x3E;
   bytes[2] = value1 & 0xff;
   //bytes[2] = 0xF0;
   SBUS2_transmit_telemetry_data( port+7 , bytes);
}

/*void do_servo_pulse_callback(uint32_t counter)
{
   do_servo_pulse = true;
   absTime = counter;
}*/


/*void SBUS2_loop()
{
   int16_t channel = 0;
   uint16_t uart_dropped_frame = false;
   bool transmision_dropt_frame = false;
   bool failsave = false;

   if (do_servo_pulse)
   {
      do_servo_pulse = false;

      channel = SBUS2_get_servo_data( 2 );
      SBUS_throttle = channel;
      if (channel != -1 )
      {
         if ( alarmPort != 0 )
         {
            SBUS2_get_status(&uart_dropped_frame, &transmision_dropt_frame, &failsave);
            send_alarm_as_temp125( (failsave*1000) + (transmision_dropt_frame*100) + uart_dropped_frame);     // Warning with over Temp at Error Slot
         }
         send_temp125(50);	                    //Temperature [°C]
         send_RPM(600);                         // RPM -> rounding Error +/- 3 RPM
         send_s1678_current((uint16_t)20,		    //Current   [A]
                            (uint16_t)15000,		//Capacity  [mah]
                            (uint16_t)10);	    //Voltage   [V]
      }
   }
}*/

/*bool SBUS2_Ready()
{
  if (do_servo_pulse)
   {
    do_servo_pulse = false;
    return true;
   }
   else{
    return false;
   }
}*/
