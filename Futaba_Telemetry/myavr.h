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
*  @ File Name : myavr.h
*  @ Date : 6/12/2013
*  @ Author : Bart Keser
*
*/

#ifndef MYAVR_H_
#define MYAVR_H_

#include <avr/io.h>
#include <stdint.h>

#define PIN_0 0x01
#define PIN_1 0x02
#define PIN_2 0x04
#define PIN_3 0x08
#define PIN_4 0x10
#define PIN_5 0x20
#define PIN_6 0x40
#define PIN_7 0x80

#define LOW  0
#define HIGH 1

#define PinBasOutput(a) \
	DDRB  |= a;

#define PinBasInput(a) \
	DDRB  &= ~a;

#define PinBOutput(a,b) \
	if (b == HIGH)      \
		PORTB |=  a;    \
	else                \
		PORTB &= ~a;

inline bool PinBInput(uint8_t pin)
{
    return ((PINB & pin) == 0) ? LOW : HIGH;
}

#define PinCasOutput(a) \
    DDRC  |= a;

#define PinCasInput(a) \
    DDRC  &= ~a;

#define PinCOutput(a,b) \
    if (b == HIGH)      \
        PORTC |=  a;    \
    else                \
        PORTC &= ~a;

inline uint8_t PinCInput(uint8_t pin)
{
    return ((PINC & pin) > 0);
}

#define PinDasOutput(a) \
    DDRD  |= a;

#define PinDasInput(a) \
    DDRD  &= ~a;

#define PinDOutput(a,b) \
    if (b == HIGH)      \
        PORTD |=  a;    \
    else                \
        PORTD &= ~a;

inline uint8_t PinDInput(uint8_t pin)
{
    return ((PIND & pin) > 0);
}

#define interrupts() sei()
#define noInterrupts() cli()

#endif /* MYAVR_H_ */