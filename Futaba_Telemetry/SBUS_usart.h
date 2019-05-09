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
*  @ File Name : SBUS_usart.h
*  @ Date : 6/12/2013
*  @ Author : Bart Keser
*
*/
#ifndef _SBUS2_USART_H
#define _SBUS2_USART_H

#include <avr/io.h>

#define NUMBER_OF_CHANNELS 18

void SBUS2_uart_setup(/*void (*start_pulse)(uint32_t counter)*/);
void SBUS2_transmit_telemetry_data( uint8_t slot, uint8_t data[3] );
//bool SBUS2_get_all_servo_data( uint16_t channels[NUMBER_OF_CHANNELS] );
int16_t SBUS2_get_servo_data( uint8_t channel);
void SBUS2_get_status( uint16_t *uart_dropped_frame, bool *transmision_dropt_frame, bool *failsave );
bool SBUS2_Ready();

#endif
