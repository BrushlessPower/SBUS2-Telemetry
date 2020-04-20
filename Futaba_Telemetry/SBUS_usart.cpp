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

#define UART_RXBUFSIZE  30
#define FRAME_TIME_OUT  200 //ms

#define SLOT_TIME 165   // 248 is around 1 ms
#define WAIT_TIME 250   // 248 is around 1 ms
#define TIMEOUT   (248.0 * (FRAME_TIME_OUT / 1000.0))    		


uint8_t   Slot_ID[32] = {   0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3,
                            0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
                            0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB,
                            0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB
                        };

uint16_t channels[NUMBER_OF_CHANNELS];

uint16_t overflow_counter = 0; // should not occur

static volatile uint8_t   rxbuf[UART_RXBUFSIZE];
static volatile uint8_t   sbusData[UART_RXBUFSIZE];
static volatile bool      frame_ready = false;
static volatile bool      telemetry_ready = false;
static volatile bool      sbus_ready = false;
static volatile uint8_t   gl_current_frame = 0;
static volatile uint16_t  uart_lost_frame = 0;
static volatile uint8_t   FER_count = 0;
static volatile uint8_t   FER_buf[100];

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
volatile int8_t          previousFrame = 0;
volatile uint32_t        frameCounter = 0;

volatile bool           transmit = false;

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

void SBUS2_get_all_servo_data();

void ISR_receive_timeout();
void ISR_transmit();

volatile void    (*timer_ISR)();

//*****************************************************************************
//
void SBUS2_uart_setup ()
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


void start_receiving()
{
   TCCR2B = 0x00;        //disable Timer2 while we set it up
   buffer_index = 0;
   timer_ISR = (volatile void    (*)())ISR_receive_timeout;
   TCNT2  = 0;           //Reset Timer 0
   OCR2A = TIMEOUT;
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
   if(transmit == false){
    TCNT2  = 0; //Reset Timer 2 for new usart time of char
    frame_ready = false;
   }
   
   //enable Timer2 Control Reg B: for receive timeout this is done here because we can only start the timeout after first bye is received
#if F_CPU == 16000000L
  TCCR2B = 0x04;     // 16MHz clock
#elif F_CPU == 8000000L  
  TCCR2B = 0x03;     // 8MHz clock
#endif

   cdata = UDR0;   
   rxbuf[buffer_index] = cdata;
   buffer_index++;
   if ((buffer_index == SBUS_FRAME_SIZE) && (rxbuf[0]== 0x0f))
   {
      telemetry_ready = false;
      sbus_ready = false;
      frame_ready = true;

      for(uint8_t i = 0; i < UART_RXBUFSIZE; i++){
        sbusData[i] = rxbuf[i];
      }
      
      disable_receiving();
      IncreaseTimer((cdata&0x30) >> 4);
      buffer_index = 0;
      if((rxbuf[24]&0x0f) == 0x04 ){    // Valid SBUS2 Frame
        telemetry_ready = true;
        sbus_ready = true;
        SBUS2_get_all_servo_data();
        start_transmit_sequencer(((cdata&0x30) >> 4)); // frame number needed to select correct telemetry slot
      }
      else if((rxbuf[24]&0x0f) == 0x00 ){
        telemetry_ready = false;
        sbus_ready = true;
        SBUS2_get_all_servo_data();
      }
      else{                             // Just SBUS Frame (without Telemetry)
        //digitalWrite(13, LOW);          // Debug
        telemetry_ready = false;
        sbus_ready = false;
      }
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

   if (sequence_count < 2 )
   {
      // don't do anything this is delay to the transmission slots
      OCR2A = WAIT_TIME;
      transmit = true;
   }
   else if (sequence_count < 10 ) // transmit slots
   {
      OCR2A = SLOT_TIME;
      sbus2_send_slot((sequence_count-2) + (gl_current_frame * NUMBER_OF_SLOT_IN_FRAME) );
   }
   else
   {
      transmit = false;
      buffer_index = 0;
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

   OCR2A = WAIT_TIME;
   
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
      gl_slot = slot;
      //send first byte
      UDR0 = transmit_data_per_slot_data[gl_slot][0]; // the reset will be done by TX ISR
      tx_data_counter = 1;
   }
}

ISR (USART_TX_vect)
{
   if (transmit_data_per_slot_status[gl_slot] != EMPTY) 
   {
      if ( tx_data_counter < SLOT_DATA_LENGTH ) 
      {
         UDR0 = transmit_data_per_slot_data[gl_slot][tx_data_counter];
         tx_data_counter++;
      }
      else
      {
         transmit_data_per_slot_status[gl_slot] = EMPTY;
      }
   }   
}

void SBUS2_transmit_telemetry_data( uint8_t slot, uint8_t data[SLOT_DATA_LENGTH] )
{
   if ( transmit_data_per_slot_status[slot] != TRANSMITTING ){
    
    transmit_data_per_slot_data[slot][0] = Slot_ID[slot];
    transmit_data_per_slot_data[slot][1] = data[1];
    transmit_data_per_slot_data[slot][2] = data[2];

    transmit_data_per_slot_status[slot] = AVAILABLE;
   }
}

void SBUS2_get_all_servo_data()
{
  channels[0]  = ((sbusData[1]|sbusData[2]<< 8) & 0x07FF);
  channels[1]  = ((sbusData[2]>>3|sbusData[3]<<5) & 0x07FF);
  channels[2]  = ((sbusData[3]>>6|sbusData[4]<<2|sbusData[5]<<10) & 0x07FF);
  channels[3]  = ((sbusData[5]>>1|sbusData[6]<<7) & 0x07FF);
  channels[4]  = ((sbusData[6]>>4|sbusData[7]<<4) & 0x07FF);
  channels[5]  = ((sbusData[7]>>7|sbusData[8]<<1|sbusData[9]<<9) & 0x07FF);
  channels[6]  = ((sbusData[9]>>2|sbusData[10]<<6) & 0x07FF);
  channels[7]  = ((sbusData[10]>>5|sbusData[11]<<3) & 0x07FF);
  channels[8]  = ((sbusData[12]|sbusData[13]<< 8) & 0x07FF);
  channels[9]  = ((sbusData[13]>>3|sbusData[14]<<5) & 0x07FF);
  channels[10] = ((sbusData[14]>>6|sbusData[15]<<2|sbusData[16]<<10) & 0x07FF);
  channels[11] = ((sbusData[16]>>1|sbusData[17]<<7) & 0x07FF);
  channels[12] = ((sbusData[17]>>4|sbusData[18]<<4) & 0x07FF);
  channels[13] = ((sbusData[18]>>7|sbusData[19]<<1|sbusData[20]<<9) & 0x07FF);
  channels[14] = ((sbusData[20]>>2|sbusData[21]<<6) & 0x07FF);
  channels[15] = ((sbusData[21]>>5|sbusData[22]<<3) & 0x07FF);
      
  // DigiChannel 1
  if (sbusData[23] & (1<<0)){
    channels[16] = 1;
  }
  else{
    channels[16] = 0;
  }
  // DigiChannel 2
  if (sbusData[23] & (1<<1)){
    channels[17] = 1;
  }
  else{
    channels[17] = 0;
  }
  if (sbusData[23] & 0x04){
    FER_buf[FER_count] = 1;
    FER_count ++;
    if(FER_count > 99){
      FER_count = 0;
    }
  }
  else{
    FER_buf[FER_count] = 0;
    FER_count ++;
    if(FER_count > 99){
      FER_count = 0;
    }
  }
}


void SBUS2_get_status( uint16_t *uart_dropped_frame, bool *transmision_dropt_frame, bool *failsave )
{
   if (frameCounter < 60)
   {
      uart_lost_frame = 0;
   }
   *uart_dropped_frame = uart_lost_frame;
   *transmision_dropt_frame = sbusData[23] & 0x04 ? true : false;
   *failsave = sbusData[23] & 0x08 ? true : false;
}

int16_t SBUS2_get_servo_data( uint8_t channel )
{
  if(channel < NUMBER_OF_CHANNELS){
    return channels[channel];
  }
  else{
    return -1;
  }
}

bool SBUS_Ready()
{
  if (sbus_ready)   // Checking sbus_ready Flag
   {
    return true;
   }
   else{
    return false;
   }
}

bool SBUS2_Ready()
{
  if (telemetry_ready)
   {
    return true;
   }
   else{
    return false;
   }
}

uint8_t SBUS_get_FER(){
  uint8_t fer = 0;
  for(uint8_t i = 0; i<100; i++){
    if(FER_buf[i] == 1){
      fer++;
    }
  }
  return fer;
}

uint8_t SBUS_get_RSSI(){
  uint8_t rssi = SBUS_get_FER();
  rssi = 100 - rssi;
  return rssi;
}
