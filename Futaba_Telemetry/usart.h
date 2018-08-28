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
*  @ File Name : usart.h
*  @ Date : 6/12/2013
*  @ Author : Bart Keser
*
*/

#ifndef _USART_H
#define _USART_H

#include <avr/io.h>

#define USART_BAUD 100000	

//-----------------------
#define UCSRA       UCSR0A
#define UCSRB       UCSR0B
#define UCSRC       UCSR0C
#define UDR         UDR0
#define UBRRL       UBRR0L
#define UBRRH       UBRR0H

// UCSRA
#define RXC         RXC0
#define TXC         TXC0
#define UDRE        UDRE0
#define FE          FE0
#define UPE         UPE0
#define U2X         U2X0
#define MPCM        MPCM0

// UCSRB
#define RXCIE       RXCIE0
#define TXCIE       TXCIE0
#define UDRIE       UDRIE0
#define TXEN        TXEN0
#define RXEN        RXEN0
#define UCSZ2       UCSZ02
#define RXB8        RXB80
#define TXB8        TXB80

// UCSRC
#define UMSEL1  UMSEL01
#define UMSEL0  UMSEL00
#define UPM1        UPM01
#define UPM0        UPM00
#define USBS        USBS0
#define UCSZ1       UCSZ01
#define UCSZ0       UCSZ00
#define UCPOL       UCPOL0

#endif
