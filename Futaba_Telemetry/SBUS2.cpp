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
*/

#include "SBUS2.h"
#include "SBUS_usart.h"

#define SBUS2_TEL_DATA_SIZE        3


void SBUS2_Setup()
{
  SBUS2_uart_setup();
}


void send_RPM(uint8_t port, uint16_t RPM)
{
   int16_t value =  0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x00, 0x00 };

   value =  RPM / 6;
   bytes[2] = value >> 8;
   bytes[1] = value;
   SBUS2_transmit_telemetry_data( port , bytes);
}


void send_temp125(uint8_t port, int16_t temp)
{
   int16_t value=  0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };

   value = temp | 0x4000;
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( port , bytes);
}


void send_alarm_as_temp125(uint8_t port, int16_t alarm)
{
   int16_t value=  0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };

   value = alarm | 0x4000;
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( port , bytes);
}

void send_s1678_current(uint8_t port, uint16_t current, uint16_t capacity, uint16_t voltage)
{
   uint16_t value = 0;
   uint32_t local = 0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };
 
   
   // CURRENT
   local = ((uint32_t)current) * 1 ;
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
   uint32_t  value3 = 0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };
 
   
   // SPEED -> Bit 14(bytes[1] bit7) -> GPS Valid or not
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

   //TIME -> 12:34:56 Uhr = 12*60*60 + 34*60 + 56 = 45296 = 0xB0F0
   bytes[1] = 0xB0;
   bytes[2] = 0xF0;
   SBUS2_transmit_telemetry_data( port+2 , bytes);

   // VARIO
   value2 = vario * 10;   
   bytes[1] = value2 >> 8;
   bytes[2] = value2;
   SBUS2_transmit_telemetry_data( port+3 , bytes);

   // LATITUDE
   bytes[1] = (uint8_t)(latitude/1000000);
   value3 = (uint32_t)(latitude%1000000);
   if(latitude >= 0){
    bytes[2] = ((value3 >> 16) & 0x0f);    // North
   }
   else{
    bytes[2] = ((value3 >> 16) & 0x1f);    // South
   }
   SBUS2_transmit_telemetry_data( port+4 , bytes);

   bytes[1] = ((value3 >> 8) & 0xff);
   bytes[2] = value3 & 0xff;
   SBUS2_transmit_telemetry_data( port+5 , bytes);

   // LONGITUDE
   bytes[1] = (uint8_t)(longitude/1000000);
   value3 = (uint32_t)(longitude%1000000);
   if(longitude >= 0){
    bytes[2] = ((value3 >> 16) & 0x0f);    // Eath
   }
   else{
    bytes[2] = ((value3 >> 16) & 0x1f);    // West
   }
   SBUS2_transmit_telemetry_data( port+6 , bytes);

   bytes[1] = ((value3 >> 8) & 0xff);
   bytes[2] = value3 & 0xff;
   SBUS2_transmit_telemetry_data( port+7 , bytes);
}
