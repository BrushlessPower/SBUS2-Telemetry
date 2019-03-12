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
*  @ File Name :SBUS_usart.cpp
*  @ Date : 6/12/2013
*  @ Author : Bart Keser
*
*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "SBUS_usart.h"

#define USART_BAUD              100000  

#define SLOT_DATA_LENGTH        3
#define NUMBER_OF_FRAMES        4
#define NUMBER_OF_SLOT          32
#define NUMBER_OF_SLOT_IN_FRAME 8
#define SBUS_FRAME_SIZE         25
#define SBUS_CHANNEL_SIZE       11

#define UART_RXBUFSIZE  30
#define FRAME_TIME_OUT  200 //ms

#define SLOT_TIME 90		
#define SLOT_TIME2 165		


uint8_t  swaps_slot_bits[8] = {0,4,2,6,1,5,3,7};
   
uint8_t toggle  = 0;   

// 248 is around 1 ms
uint8_t  transmit_sequence_timer[15] = {250,250,SLOT_TIME2,SLOT_TIME,SLOT_TIME,SLOT_TIME,SLOT_TIME,SLOT_TIME,SLOT_TIME,SLOT_TIME,226,226,226,226,180};
uint8_t  receive_timeout_timer = ( uint8_t ) (248.0 * (FRAME_TIME_OUT / 1000.0));

uint16_t overflow_counter = 0; // should not occur

static volatile uint8_t   rxbuf[UART_RXBUFSIZE];
static volatile bool      frame_ready = false;
static volatile uint8_t   gl_current_frame = 0;
static volatile uint16_t  uart_lost_frame = 0;

typedef enum
{
   EMPTY = 0,
   TRANSMITTING,
   AVAILABLE
} SLOT_DATA_STATUS;

typedef struct
{
   volatile bool    data_status;
   volatile uint8_t data[SLOT_DATA_LENGTH];
} SLOT_RAW_DATA;

// low resolution timer
volatile uint8_t         frameLength = 15;
volatile int8_t          previousFrame = 0;
volatile uint32_t        frameCounter = 0;


volatile uint8_t        tx_data_counter;
volatile uint8_t        buffer_index;

volatile SLOT_DATA_STATUS transmit_data_per_slot_status[NUMBER_OF_SLOT];
volatile uint8_t          transmit_data_per_slot_data[NUMBER_OF_SLOT][SLOT_DATA_LENGTH];
volatile uint8_t          gl_slot;

void sbus_uart_init();
void sbus_timer_init();
void start_receiving();
void enable_receiving();
void disable_receiving();
void start_transmit_sequencer(uint8_t frame_number);
void sbus2_send_slot(uint8_t slot);

void ISR_receive_timeout();
void ISR_transmit();

volatile void    (*do_servo_pulls)(uint32_t counter);

volatile void    (*timer_ISR)();

//*****************************************************************************
//
void SBUS2_uart_setup (void (*start_pulse)(uint32_t))
{
   uint32_t counter  = 640000;
   uint8_t  response = 0x00;
   
   noInterrupts();
   sbus_uart_init();
   sbus_timer_init();

   disable_receiving();
   
   UDR0 = 0xAA;
   response = 0x00;
   while(!(UCSR0A & (1 << UDRE0)));
   
   while ((counter > 1)  && ( response != 0x55 ))
   {
     response = UDR0;
     while ( UCSR0A & (1 << RXC0) );
     counter--;
   }      
          
   do_servo_pulls =  (volatile void    (*)(uint32_t))start_pulse;     
   start_receiving();
   
   if(response != 0x55 )
   {
      enable_receiving();       
   }   
   interrupts();
   
}

void sbus_uart_init()
{
   // set clock divider
#undef BAUD
#define BAUD USART_BAUD

#include <util/setbaud.h>

   UBRR0H = UBRRH_VALUE;
   UBRR0L = UBRRL_VALUE;

#if USE_2X
   UCSR0A |= (1 << U2X0);	// enable double speed operation
#else
   UCSR0A &= ~(1 << U2X0);	// disable double speed operation
#endif

   // set 8E2
   UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
   UCSR0B &= ~(1 << UCSZ02);

   UCSR0C |= (1 << UPM01);
   UCSR0C |= (1 << USBS0);

   // flush receive buffer
   while ( UCSR0A & (1 << RXC0) ) UDR0;

   UCSR0B |= (0 << RXEN0);
   UCSR0B |= (0 << RXCIE0);
   UCSR0B |= (1 << TXEN0);
   UCSR0B |= (1 << TXCIE0);
}

void sbus_timer_init()
{
   //memset( (void*)transmit_data_per_slot, 0, sizeof(transmit_data_per_slot));
   for(uint8_t i = 0; i < 32; i++)
   {
      transmit_data_per_slot_status[i] = EMPTY;
   }

   TCCR2B = 0x00;        //disable Timer2 while we set it up
   TCNT2  = 0;           //Reset Timer 0
   TIFR2  = 0x02;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
   TIMSK2 = 0x03;        //Timer2 INT Reg: Timer2 compare A
   TCCR2A = 0x02;        //Timer2 Control Reg A: CTC
}

uint8_t SBUS_get_frame(uint8_t *frame)
{
   if( frame_ready )
   {
      memcpy(frame, (void*)rxbuf, SBUS_FRAME_SIZE);
      frame_ready = false;
      return true;
   }
   else
   {
      return false;
   }
}

void start_receiving()
{
   TCCR2B = 0x00;        //disable Timer2 while we set it up
   buffer_index = 0;
   timer_ISR = (volatile void    (*)())ISR_receive_timeout;
   TCNT2  = 0;           //Reset Timer 0
   OCR2A = receive_timeout_timer; 
}

inline void disable_receiving()
{
   UCSR0B |= (0 << RXEN0);
   UCSR0B |= (0 << RXCIE0);
}

inline void enable_receiving()
{
   UCSR0B |= (1 << RXEN0);
   UCSR0B |= (1 << RXCIE0);
}



ISR(TIMER2_COMPA_vect)
{
   timer_ISR();
}

inline void IncreaseTimer( int8_t frameNumber)
{
   int8_t temp = (frameNumber - previousFrame) % NUMBER_OF_FRAMES;
   if (temp <= 0 )
      temp += NUMBER_OF_FRAMES;          
   if (temp > 1)
      uart_lost_frame++;
   frameCounter += temp;
   previousFrame = frameNumber;   
}

ISR (USART_RX_vect)
{
   uint8_t cdata = 0;
   frame_ready = false;
   TCNT2  = 0; //Reset Timer 2 for new usart time of char
   //enable Timer2 Control Reg B: for receive timeout this is done here because we can only start the timeout after first bye is received
   
#if F_CPU == 16000000L
  TCCR2B = 0x04;     // 16MHz clock
#elif F_CPU == 8000000L  
  TCCR2B = 0x03;     // 8MHz clock
#endif

   cdata = UDR0;   
   rxbuf[buffer_index] = cdata;
   buffer_index++;
   if (buffer_index == SBUS_FRAME_SIZE)
   {
      frame_ready = true;
      disable_receiving();
      IncreaseTimer((cdata&0x30) >> 4);
      start_transmit_sequencer(((cdata&0x30) >> 4)); // frame number needed to select correct telemetry slot

      if (do_servo_pulls != NULL )
      {
         do_servo_pulls(frameCounter * frameLength);
      }
      buffer_index = 0;
   } 
}


// receive timeout check for set packed length
void ISR_receive_timeout()
{   
   buffer_index = 0;
}

//****************************************************************
//* TRANSMIT SEQUENCER
//****************************************************************

void ISR_transmit()
{
   static  uint8_t sequence_count = 1; // first sequence step delay will be filled in when the transmit sequence is enabled
   //Increments the interrupt counter
   TCNT2  = 0; // reset at the beginning so that  there is no delay for the next segment of the sequence

   interrupts();

   if (sequence_count < 2 )
   {
      // don't do anything this is delay to the transmission slots
      OCR2A = transmit_sequence_timer[sequence_count];
   }
   else if (sequence_count < 10 ) // transmit slots
   {
      OCR2A = transmit_sequence_timer[sequence_count]; // first set next slot interrupt
      sbus2_send_slot((sequence_count-2) + (gl_current_frame * NUMBER_OF_SLOT_IN_FRAME) );
   }
   else if (sequence_count < 11 )
   {
      OCR2A = transmit_sequence_timer[sequence_count];

   }
   else if (sequence_count < 14 )
   {
      // delay to enabling receive again
      OCR2A = transmit_sequence_timer[sequence_count];
   }
   else if (sequence_count < 15 )
   {
      // delay to enabling receive again
      OCR2A = transmit_sequence_timer[sequence_count];
      frame_ready = false;  // this will give  ms to collect the servo data
      buffer_index = 0;
   }
   else
   {
      // reset transmit sequencer
      TCCR2B = 0x00;        //Disbale Timer2 while we set it up
      sequence_count = 0;  // first sequence step delay will be filled in when the transmit sequence is enabled
      enable_receiving();
      start_receiving();
   }
   sequence_count++;
   TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};

ISR(TIMER2_OVF_vect)
{
   // for debugging should not occur
   overflow_counter++;
};


void start_transmit_sequencer(uint8_t frame_number)
{
   //set transmit ISR
   timer_ISR = (volatile void    (*)())ISR_transmit;

   OCR2A  = transmit_sequence_timer[0]; //set first delay value
   
#if F_CPU == 16000000L
  TCCR2B = 0x04;     // 16MHz clock
#elif F_CPU == 8000000L  
  TCCR2B = 0x03;     // 8MHz clock
#endif

   gl_current_frame = frame_number;
}


//****************************************************************
//* TRANSMIT usart send
//****************************************************************
void sbus2_send_slot(uint8_t slot)
{
   // if data available in slot then send it
   if ( transmit_data_per_slot_status[slot] == AVAILABLE )
   {
      
      transmit_data_per_slot_status[slot] = TRANSMITTING;
      //enable_transmiting();
      //sending_slot = &transmit_data_per_slot[slot];
      gl_slot = slot;
      //send first byte
      UDR0 = transmit_data_per_slot_data[gl_slot][0]; // the reset will be done by TX ISR
      tx_data_counter = 1;
   }
}

ISR (USART_TX_vect)
{
   interrupts();
   if (transmit_data_per_slot_status[gl_slot] != EMPTY) 
   {
      if ( tx_data_counter < SLOT_DATA_LENGTH ) 
      {
         UDR0 = transmit_data_per_slot_data[gl_slot][tx_data_counter];
         tx_data_counter++;
      }
      else
      {
         // disable transmitter line and set data status to empty so that it can be written again
         //disable_transmiting();
         transmit_data_per_slot_status[gl_slot] = EMPTY;
      }
   }   
}

void SBUS2_transmit_telemetry_data( uint8_t slot, uint8_t data[SLOT_DATA_LENGTH] )
{
   uint8_t swapped_slot = 0;
   if ( transmit_data_per_slot_status[slot] != TRANSMITTING )
   {
      //swap slot ID
      swapped_slot = swaps_slot_bits[slot % 8] << 5;
      memcpy( (void*)transmit_data_per_slot_data[slot], data, SLOT_DATA_LENGTH);
      transmit_data_per_slot_data[slot][0] &= 0x1F; // reset frame slot ID
      transmit_data_per_slot_data[slot][0] |= swapped_slot;

      transmit_data_per_slot_status[slot] = AVAILABLE;
   }
}

bool SBUS2_get_all_servo_data( uint16_t channels[16] )
{
   if ( frame_ready )
   {
      uint8_t byte_in_sbus = 1;
      uint16_t bit_in_sbus = 0;
      uint8_t ch = 0;
      uint16_t bit_in_channel = 0;
      //uint16_t temp;
      uint8_t channel_data[UART_RXBUFSIZE];
      //noInterrupts();
      memcpy(channel_data, (void*)rxbuf, 24);
      //interrupts();

      for (uint8_t i=0; i<16; i++)
         channels[i] = 0;

      // process actual sbus data
      for (uint8_t i=0; i<176; i++)
      {
         //temp = 1<<bit_in_sbus;
         if (channel_data[byte_in_sbus] & (1<<bit_in_sbus))
         {
            //temp = (1<<bit_in_channel);
            channels[ch] |= (1<<bit_in_channel);
         }
         bit_in_sbus++;
         bit_in_channel++;

         if (bit_in_sbus == 8)
         {
            bit_in_sbus =0;
            byte_in_sbus++;
         }
         if (bit_in_channel == 11)
         {
            bit_in_channel =0;
            ch++;
         }
      }
      // DigiChannel 1
      if (channel_data[23] & (1<<0))
      {
         channels[16] = 1;
      }
      else
      {
         channels[16] = 0;
      }
      // DigiChannel 2
      if (channel_data[23] & (1<<1))
      {
         channels[17] = 1;
      }
      else
      {
         channels[17] = 0;
      }

      return true;
   }
   else
   {
      return false;
   }
}


void SBUS2_get_status( uint16_t *uart_dropped_frame, bool *transmision_dropt_frame, bool *failsave )
{
   if (frameCounter < 60)
   {
      uart_lost_frame = 0;
   }
   *uart_dropped_frame = uart_lost_frame;
   *transmision_dropt_frame = rxbuf[23] & 0x20 ? true : false;
   *failsave = rxbuf[23] & 0x10 ? true : false;
}

int16_t SBUS2_get_servo_data( uint8_t channel )
{
   uint8_t byte_in_sbus = 1;
   uint16_t bit_in_sbus = 0;
   uint8_t ch = 0;
   uint16_t bit_in_channel = 0;
   uint8_t channel_data[UART_RXBUFSIZE];

   uint16_t  servo = 0;
   uint8_t  start_bit  = 0;

   //noInterrupts();
   memcpy(channel_data, (void*)rxbuf, 24);
   //interrupts();

   if ( frame_ready )
   {
      start_bit  = channel * SBUS_CHANNEL_SIZE ;
      bit_in_sbus  = start_bit % 8;
      byte_in_sbus = (start_bit / 8) + 1;

      // process actual sbus data
      for (uint8_t i=start_bit; i<(start_bit+11); i++)
      {
         if (channel_data[byte_in_sbus] & (1<<bit_in_sbus))
         {
            servo |= (1<<bit_in_channel);
         }
         bit_in_sbus++;
         bit_in_channel++;

         if (bit_in_sbus == 8)
         {
            bit_in_sbus =0;
            byte_in_sbus++;
         }
         if (bit_in_channel == 11)
         {
            bit_in_channel =0;
            ch++;
         }
      }
      return (int16_t)servo;
   }
   else
   {
      return -1;
   }
}


