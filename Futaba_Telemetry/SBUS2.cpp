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

uint8_t SBUS2_get_FER(){
  uint8_t fer = SBUS_get_FER();
  return fer;
}
uint8_t SBUS2_get_RSSI(){
   uint8_t rssi = SBUS_get_RSSI();
   return rssi;
}

void send_RPM(uint8_t port, uint32_t RPM)
{
   uint32_t value =  0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x00, 0x00 };

   value =  RPM / 6;
   if(value > 0xffff){
    value = 0xffff;
   }
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
void send_SBS01T(uint8_t port, int16_t temp){
  int16_t value=  0;
  uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };

  value = temp | 0x8000;
  value = value + 100;
  bytes[1] = value;// >> 8;
  bytes[2] = value >> 8;
  SBUS2_transmit_telemetry_data( port , bytes);
}


void send_voltage(uint8_t port,uint16_t voltage1, uint16_t voltage2)
{
   uint16_t value = 0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };
   
   // VOLTAGE1
   value = voltage1 | 0x8000; 
   if ( value > 0x9FFF ){
     value = 0x9FFF; // max voltage is 819.1
   }
   else if(value < 0x8000){
     value = 0x8000;
   }
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( port , bytes);

   //VOLTAGE2
   value = voltage2;
   if ( value > 0x1FFF ){
     value = 0x1FFF; // max voltage is 819.1
   }  
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( port+1 , bytes);
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
   bool latitude_pos = false;
   bool longitude_pos = false;
   int32_t _latitude = latitude;
   int32_t _longitude = longitude;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };
 
   
   // SPEED -> Bit 14(bytes[1] bit7) -> GPS Valid or not
   value1 = speed | 0x4000;
   if (value1 > 0x43E7 ){
      value1 = 0x43E7;  // max Speed is 999 km/h
   }  
   else if( value1 < 0x4000){
     value1 = 0x4000;
   }
   bytes[1] = value1 >> 8;
   bytes[2] = value1;
   SBUS2_transmit_telemetry_data( port , bytes);

   //HIGHT
   value2 = altitude | 0x4000;
   if(value2 > 0x7FFF ){
     value2 = 0x7FFF;  // max Speed is 999 km/h
   }  
   else if( value2 < 0xC000){
     value2 = 0xC000;
   }
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
   if(latitude >= 0){
    latitude_pos = true;
   }
   else{
    latitude_pos = false;
    latitude = latitude * -1;
   }
   bytes[1] = (uint8_t)(latitude/1000000);
   value3 = (uint32_t)(latitude%1000000);
   if(latitude_pos){
    bytes[2] = ((value3 >> 16) & 0x0f);    // North
   }
   else{
    bytes[2] = ((value3 >> 16) | 0x1f);    // South
   }
   SBUS2_transmit_telemetry_data( port+4 , bytes);

   bytes[1] = ((value3 >> 8) & 0xff);
   bytes[2] = value3 & 0xff;
   SBUS2_transmit_telemetry_data( port+5 , bytes);

   // LONGITUDE
   if(longitude >= 0){
    longitude_pos = true;
   }
   else{
    longitude_pos = false;
    longitude = longitude * -1;
   }
   bytes[1] = (uint8_t)(longitude/1000000);
   value3 = (uint32_t)(longitude%1000000);
   if(longitude_pos){
    bytes[2] = ((value3 >> 16) & 0x0f);    // Eath
   }
   else{
    bytes[2] = ((value3 >> 16) | 0x1f);    // West
   }
   SBUS2_transmit_telemetry_data( port+6 , bytes);

   bytes[1] = ((value3 >> 8) & 0xff);
   bytes[2] = value3 & 0xff;
   SBUS2_transmit_telemetry_data( port+7 , bytes);
}

void send_f1672_vario(uint8_t port, int16_t altitude, int16_t vario)
{
   int16_t value = 0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };

   // VARIO
   value = vario;   
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( port, bytes);
   
   //HIGHT
   value = altitude | 0x4000;
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( port + 1, bytes);
}

void send_f1712_vario(uint8_t port, int16_t altitude, int16_t vario)
{
   int16_t  value = 0;
   uint8_t bytes[SBUS2_TEL_DATA_SIZE] = {0x03, 0x40, 0x00 };

   // VARIO
   value = vario;   
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( port , bytes);
   
   //HIGHT
   value = altitude | 0x4000;
   bytes[1] = value >> 8;
   bytes[2] = value;
   SBUS2_transmit_telemetry_data( port + 1 , bytes);
   
}


void send_alarm_as_temp125(uint8_t port, int16_t alarm)
{
   send_temp125(port, alarm);
}
void send_SBS01TE(uint8_t port, int16_t temp){
  send_temp125(port, temp);
}
void send_F1713(uint8_t port, int16_t temp){
  send_temp125(port, temp);
}

void send_SBS01RB(uint8_t port, uint32_t RPM){
  send_RPM(port, RPM);
}
void send_SBS01RM(uint8_t port, uint32_t RPM){
  send_RPM(port, RPM);
}
void send_SBS01RO(uint8_t port, uint32_t RPM){
  send_RPM(port, RPM);
}
void send_SBS01R(uint8_t port, uint32_t RPM){
  send_RPM(port, RPM);
}

void send_F1678(uint8_t port, uint16_t current, uint16_t capacity, uint16_t voltage){
  send_s1678_current(port, current, capacity, voltage);
}
void send_s1678_current(uint8_t port, float current, uint16_t capacity, float voltage){
  send_s1678_current(port, (uint16_t)(current * 100), capacity, (uint16_t)(voltage * 100));
}
void send_F1678(uint8_t port, float current, uint16_t capacity, float voltage){
  send_s1678_current(port, (uint16_t)(current * 100), capacity, (uint16_t)(voltage * 100));
}
void send_SBS01V(uint8_t port,uint16_t voltage1, uint16_t voltage2){
  send_voltage(port, voltage1, voltage2);
}
void send_SBS01V(uint8_t port,float voltage1, float voltage2){
  send_voltage(port, (uint16_t)(voltage1 * 10), (uint16_t)(voltage2 * 10));
}
void send_voltage(uint8_t port,float voltage1, float voltage2){
  send_voltage(port, (uint16_t)(voltage1 * 10), (uint16_t)(voltage2 * 10));
}
void send_SBS01C(uint8_t port, uint16_t current, uint16_t capacity, uint16_t voltage){
  send_s1678_current(port, current, capacity, voltage);
}
void send_SBS01C(uint8_t port, float current, uint16_t capacity, float voltage){
  send_s1678_current(port, (uint16_t)(current * 100), capacity, (uint16_t)(voltage * 100));
}

void send_f1712_vario(uint8_t port, int16_t altitude, float vario){
  send_f1712_vario(port, altitude, (int16_t)(vario * 10));
}
void send_f1672_vario(uint8_t port, int16_t altitude, float vario){
  send_f1672_vario(port, altitude, (int16_t)(vario * 100));
}
void send_F1712(uint8_t port, int16_t altitude, int16_t vario){
  send_f1712_vario(port, altitude, vario);
}
void send_F1712(uint8_t port, int16_t altitude, float vario){
  send_f1712_vario(port, altitude, (int16_t)(vario * 10));
}
void send_F1672(uint8_t port, int16_t altitude, int16_t vario){
  send_f1672_vario(port, altitude, vario);
}
void send_F1672(uint8_t port, int16_t altitude, float vario){
  send_f1672_vario(port, altitude, (int16_t)(vario * 100));
}

void send_F1675(uint8_t port, uint16_t speed, int16_t hight, int16_t vario, int8_t lat_deg, float lat_min, int8_t lon_deg, float lon_min){
  bool Lat_Negative = false;
  bool Lon_Negative = false;
  if(lat_deg < 0){
    Lat_Negative = true;
    lat_deg = lat_deg * -1;
  }
  if(lon_deg < 0){
    Lon_Negative = true;
    lon_deg = lon_deg * -1;
  }
  if(lat_min < 0){
    Lat_Negative = true;
    lat_min = lat_min * -1;
  }
  if(lon_min < 0){
    Lon_Negative = true;
    lon_min = lon_min * -1;
  }
  int32_t _latitude_deg = lat_deg;
  int32_t _longitude_deg = lon_deg;
  int32_t _latitude_min = lat_min * 10000;
  int32_t _longitude_min = lon_min * 10000;
  int32_t _latitude = _latitude_deg * 1000000;
  int32_t _longitude = _longitude_deg * 1000000;
  _latitude = _latitude + _latitude_min;
  _longitude = _longitude + _longitude_min;
  if(Lat_Negative){
    _latitude = _latitude * -1;
  }
  if(Lon_Negative){
    _longitude = _longitude * -1;
  }
  send_f1675_gps(port, speed, hight, vario, _latitude, _longitude);
}
void send_F1675(uint8_t port, uint16_t speed, int16_t hight, int16_t vario, int8_t lat_deg, int32_t lat_min, int8_t lon_deg, int32_t lon_min){
  bool Lat_Negative = false;
  bool Lon_Negative = false;
  if(lat_deg < 0){
    Lat_Negative = true;
    lat_deg = lat_deg * -1;
  }
  if(lon_deg < 0){
    Lon_Negative = true;
    lon_deg = lon_deg * -1;
  }
  if(lat_min < 0){
    Lat_Negative = true;
    lat_min = lat_min * -1;
  }
  if(lon_min < 0){
    Lon_Negative = true;
    lon_min = lon_min * -1;
  }
  int32_t _latitude_deg = lat_deg;
  int32_t _longitude_deg = lon_deg;
  int32_t _latitude = _latitude_deg * 1000000;
  int32_t _longitude = _longitude_deg * 1000000;
  _latitude = _latitude + lat_min;
  _longitude = _longitude + lon_min;
  if(Lat_Negative){
    _latitude = _latitude * -1;
  }
  if(Lon_Negative){
    _longitude = _longitude * -1;
  }
  send_f1675_gps(port, speed, hight, vario, _latitude, _longitude);
}
void send_F1675(uint8_t port, uint16_t speed, int16_t hight, int16_t vario, int32_t latitude, int32_t longitude){
  int32_t _latitude = latitude;
  int32_t _longitude = longitude;
  int32_t _latitude_deg = _latitude/1000000;
  int32_t _longitude_deg = _longitude/1000000;
  int32_t _latitude_min = _latitude%1000000;
  int32_t _longitude_min = _longitude%1000000;
  _latitude = _latitude_deg * 1000000;
  _longitude = _longitude_deg * 1000000;
  _latitude = _latitude + ((_latitude_min * 60)/100);
  _longitude = _longitude + ((_longitude_min * 60)/100);
  send_f1675_gps(port, speed, hight, vario, _latitude, _longitude);
}
void send_F1675(uint8_t port, uint16_t speed, int16_t hight, int16_t vario, float latitude, float longitude){
  int32_t _latitude = latitude * 1000000;
  int32_t _longitude = longitude * 1000000;
  int32_t _latitude_deg = _latitude/1000000;
  int32_t _longitude_deg = _longitude/1000000;
  int32_t _latitude_min = _latitude%1000000;
  int32_t _longitude_min = _longitude%1000000;
  _latitude = _latitude_deg * 1000000;
  _longitude = _longitude_deg * 1000000;
  _latitude = _latitude + ((_latitude_min * 60)/100);
  _longitude = _longitude + ((_longitude_min * 60)/100);
  send_f1675_gps(port, speed, hight, vario, _latitude, _longitude);
}
