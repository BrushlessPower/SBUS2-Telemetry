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
#include <Arduino.h>

#define NUMBER_OF_CHANNELS 18

void SBUS2_uart_setup();
void SBUS2_transmit_telemetry_data( uint8_t slot, uint8_t data[3] );
int16_t SBUS2_get_servo_data( uint8_t channel);
void SBUS2_get_status( uint16_t *uart_dropped_frame, bool *transmision_dropt_frame, bool *failsave );
bool SBUS2_Ready();
bool SBUS_Ready();
uint8_t SBUS_get_FER();
uint8_t SBUS_get_RSSI();
