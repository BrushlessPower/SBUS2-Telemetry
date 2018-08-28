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
*  @ File Name : SBUS2.h
*  @ Date : 6/12/2013
*  @ Author : Bart Keser
*
*/
#if !defined(_SBUS2_H)
#define _SBUS2_H

#include <avr/io.h>

void SBUS2_Setup(uint8_t current_port, 
				 uint8_t temperature_port, 
				 uint8_t rpm_port, 
				 uint8_t alarm_port, 
				 uint8_t current_cal, 
				 uint8_t NumberOfPoles);
void SBUS2_loop();

#endif  //_SBUS2_H
