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
#if defined (ESP32)
#include "SBUS_usart.h"
#include "esp32-hal.h"
#include "esp32-hal-uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_task.h"
#include "esp_task_wdt.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/timer.h"

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#define SLOT_DATA_LENGTH 3
#define NUMBER_OF_FRAMES 4
#define NUMBER_OF_SLOT 32
#define NUMBER_OF_SLOT_IN_FRAME 8
#define SBUS_FRAME_SIZE 25

#define SBUS2_UART_RX_PIN (GPIO_NUM_25)
#define SBUS2_UART_TX_PIN (GPIO_NUM_26)

#define UART_RXBUFSIZE 30

#define SLOT_TIME 660 // 660 ms
#define WAIT_TIME 2000 // 2000 ms

#define RX_BUF_SIZE 1024 // we dont need more space in buffer

bool DemoMode = false;

// 32 Slots for telemetrie Data
uint8_t Slot_ID[NUMBER_OF_SLOT] = {0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3,
                                   0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
                                   0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB,
                                   0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB};

char sbus_frame[25] = {0x0F, 0xA0, 0xA3, 0x20, 0x56, 0x2C, 0x08, 0x16, 0x50, 0x03, 0x10, 0x80, 0x00, 0x04, 0x20, 0x00, 0x01, 0x08, 0x07, 0xC8, 0x03, 0x10, 0x80, 0x02, 0x04};

// channels for connected servos at the SBUS
uint16_t channels[NUMBER_OF_CHANNELS];

static volatile uint8_t rxbuf[UART_RXBUFSIZE];
static volatile uint8_t sbusData[UART_RXBUFSIZE];
static volatile uint8_t telemetryData[256];
static volatile bool frame_ready = false;
static volatile bool telemetry_ready = false;
static volatile bool sbus_ready = false;
static volatile uint8_t gl_current_frame = 0;
static volatile uint16_t uart_lost_frame = 0;
static volatile uint8_t   FER_count = 0;
static volatile uint8_t   FER_buf[100];

static volatile uint8_t        buffer_index = 0;

static volatile uint8_t tx_pin;
uart_port_t uart_num;

typedef enum
{
   EMPTY = 0,
   TRANSMITTING,
   AVAILABLE
} SLOT_DATA_STATUS;

typedef struct
{
   bool data_status;
   uint8_t data[SLOT_DATA_LENGTH];
} SLOT_RAW_DATA;

static volatile int8_t          previousFrame = 0;
static volatile uint32_t        frameCounter = 0;

static volatile uint8_t sequence_count = 0; // first sequence step delay will be filled in when the transmit sequence is enabled

static volatile bool transmit = false;


volatile SLOT_DATA_STATUS transmit_data_per_slot_status[NUMBER_OF_SLOT];
volatile uint8_t transmit_data_per_slot_data[NUMBER_OF_SLOT][SLOT_DATA_LENGTH];

void initialize_slot_status();
void enable_receiving();
void disable_receiving();
void start_transmit_sequencer(uint8_t frame_number);

void SBUS2_get_all_servo_data();


/*******************************************************************************
 * ESP32 is able to invert the RX and TX Lines itself, so we dont need an external inverter
 * Also the HardwareSerial Library has interup driven input/output buffers and we dont need 
 * to implent this on our own. 
 * I implement the receiver part as an extra task, to avoid looping in main loop.
 * The transmitter is also implemented as a task to be able to follow the requird timings.
 * 
 */
 
uart_config_t uart_config = {
    .baud_rate = 100000,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_EVEN,
    .stop_bits = UART_STOP_BITS_2,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

timer_config_t timer_config = {
      .alarm_en = true,        //Alarm Enable
      .counter_en = false,      //If the counter is enabled it will start incrementing / decrementing immediately after calling timer_init()
      .intr_type = TIMER_INTR_LEVEL,  //Is interrupt is triggered on timer’s alarm (timer_intr_mode_t)
      .counter_dir = TIMER_COUNT_UP,  //Does counter increment or decrement (timer_count_dir_t)
      .auto_reload = true,      //If counter should auto_reload a specific initial value on the timer’s alarm, or continue incrementing or decrementing.
      .divider = 80           //Divisor of the incoming 80 MHz (12.5nS) APB_CLK clock. E.g. 80 = 1uS per timer tick
    };



/* ISR to transmit the frame */
static void IRAM_ATTR ISR_transmit_frame(void *arg);
static void IRAM_ATTR uart_intr_handle(void *arg);

//*****************************************************************************
//
void SBUS2_enable_simulation(){
	log_i("[SBUS2] Demo Mode enabled");
	DemoMode = true;
}

void SBUS2_disable_simulation(){
	log_i("[SBUS2] Demo Mode disabled");
	DemoMode = false;
}

void SBUS2_uart_setup()
{
   SBUS2_uart_setup(SBUS2_UART_RX_PIN, SBUS2_UART_TX_PIN, UART_NUM_1 );
}

void SBUS2_uart_setup(int rx, int tx)
{
   SBUS2_uart_setup(rx, tx, UART_NUM_1 );
}

void SBUS2_uart_setup(int rx, int tx, int uart)
{
   tx_pin = tx;
   uart_num = (uart_port_t)uart;
   ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
   ESP_ERROR_CHECK(uart_set_line_inverse(uart_num, UART_INVERSE_RXD|UART_INVERSE_TXD)); 
   ESP_ERROR_CHECK(uart_set_pin(uart_num, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
   gpio_set_pull_mode((gpio_num_t)rx, GPIO_PULLDOWN_ONLY);
   gpio_pulldown_en((gpio_num_t)rx);
   ESP_ERROR_CHECK(uart_driver_install(uart_num, RX_BUF_SIZE, 0,0, NULL, 0));
   ESP_ERROR_CHECK(uart_isr_free(uart_num));
   ESP_ERROR_CHECK(uart_isr_register(uart_num,uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, 0/*handle_console*/));
   ESP_ERROR_CHECK(uart_enable_rx_intr(uart_num));

    // setup transmitter ISR
    timer_init(TIMER_GROUP_0, TIMER_1, &timer_config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, SLOT_TIME);
    timer_enable_intr(TIMER_GROUP_0, TIMER_1);
    timer_isr_register(TIMER_GROUP_0, TIMER_1, &ISR_transmit_frame, NULL, 0, 0/*&s_timer_handle*/);
    timer_set_alarm(TIMER_GROUP_0,TIMER_1,TIMER_ALARM_EN);
   
    uart_flush(uart_num);

    uart_intr_config_t uart_intr;
    uart_intr.intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M | UART_RXFIFO_TOUT_INT_ENA_M;
    uart_intr.rxfifo_full_thresh = 26;
    uart_intr.rx_timeout_thresh = 2;
    uart_intr.txfifo_empty_intr_thresh = 10;
    uart_intr_config(uart_num, &uart_intr);

    initialize_slot_status();

    enable_receiving();
	
 	
	if(DemoMode){													//Nur wenn demo Mode
		uart_write_bytes(uart_num, sbus_frame, SBUS_FRAME_SIZE);
		sbus_frame[24] += 0x10;
	}
	else{
		pinMatrixOutDetach(tx_pin,false,true);
	}
}

void initialize_slot_status()
{
   for (uint8_t i = 0; i < NUMBER_OF_SLOT; i++)
   {
      transmit_data_per_slot_status[i] = EMPTY;
   }
}

void disable_receiving()
{
  sequence_count = 0;
  
  timer_set_alarm(TIMER_GROUP_0,TIMER_1,TIMER_ALARM_DIS);
  timer_pause(TIMER_GROUP_0,TIMER_1);
   
  timer_set_alarm_value(TIMER_GROUP_0,TIMER_1,WAIT_TIME);     // RX Interrupt with delay of 700µs -> 700µs + 1300µs = 2ms
  timer_set_counter_value(TIMER_GROUP_0,TIMER_1,0);
  timer_set_alarm(TIMER_GROUP_0,TIMER_1,TIMER_ALARM_EN);
  timer_start(TIMER_GROUP_0,TIMER_1);
}

void enable_receiving()
{
   // disable transmitter ISR
   /*timer_set_alarm(TIMER_GROUP_0,TIMER_1,TIMER_ALARM_DIS);
   timer_pause(TIMER_GROUP_0,TIMER_1);
   timer_set_counter_value(TIMER_GROUP_0,TIMER_1,0);*/
   
   timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
   timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, 15000);
   timer_set_alarm(TIMER_GROUP_0,TIMER_1,TIMER_ALARM_EN);

   // flush input buffer first, to avoid getting old SBUS-Frames
   //uart_flush(uart_num);
   buffer_index = 0;
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

static void IRAM_ATTR uart_intr_handle(void *arg)
{
  //digitalWrite(18,HIGH);
  pinMatrixOutDetach(tx_pin,false,true);
  uart_clear_intr_status(uart_num, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);
  uint16_t rx_fifo_len, status;
  status = UART1.int_st.val; // read UART interrupt Status
  rx_fifo_len = UART1.status.rxfifo_cnt; // read number of bytes in UART buffer
  /*Serial.print("Fifo len = ");
  Serial.println(rx_fifo_len);*/
  if(rx_fifo_len == 0){
	  return;
  }
  else if(rx_fifo_len == 25){
	  
  }
  else if(rx_fifo_len == 3){
	  
  }
  else{
	  //digitalWrite(18,HIGH);
	  /*Serial.print("Error! -> rx_fifo_len = ");
	  Serial.print(rx_fifo_len);
	  Serial.print(" Value = ");
	  for(uint8_t i=0; i<=rx_fifo_len;i++){
	 	Serial.print(rxbuf[i], HEX);
		Serial.print(", ");
	  }
	  Serial.println(" [HEX]");*/
	  uart_flush(uart_num);
	  uart_clear_intr_status(uart_num, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);
	  log_i("[SBUS2] Wrong Frame length\l\n");
	  return;
  }
  //digitalWrite(18,HIGH);

  while(rx_fifo_len){
    rxbuf[buffer_index] = UART1.fifo.rw_byte; // read all bytes
    rx_fifo_len--;
	buffer_index++;
	//Serial.print(rxbuf[buffer_index],HEX);
	//Serial.print(" ");
    /*if(rxbuf[0] == 0x0f){						// Start of SBUS(2) Frame
      buffer_index++;
    }
    else{										// Telemetry Slot?
		//digitalWrite(16,HIGH);
		buffer_index++;
    }*/
  }
    if ((buffer_index == SBUS_FRAME_SIZE) && (rxbuf[0]== 0x0f))		// SBUS Frame
     {
		//digitalWrite(18,HIGH);
        telemetry_ready = false;
        sbus_ready = false;
        frame_ready = true;
        for(uint8_t i = 0; i < UART_RXBUFSIZE; i++){
          sbusData[i] = rxbuf[i];
        }
        disable_receiving();
        IncreaseTimer((rxbuf[24]&0x30) >> 4);
        buffer_index = 0;
		//Serial.println();
        if((rxbuf[24]&0x0f) == 0x04 ){    					// Valid SBUS2 Frame
		  //digitalWrite(18,HIGH);
          telemetry_ready = true;
          sbus_ready = true;
          SBUS2_get_all_servo_data();
          start_transmit_sequencer(((rxbuf[24]&0x30) >> 4)); // frame number needed to select correct telemetry slot
        }
        else if((rxbuf[24]&0x0f) == 0x00 ){					 // Just SBUS Frame (without Telemetry)
          telemetry_ready = false;
          sbus_ready = true;
          SBUS2_get_all_servo_data();
        }
        else{                             					// Something else -> Error
          telemetry_ready = false;
          sbus_ready = false;
		  uart_flush(uart_num);
		  uart_clear_intr_status(uart_num, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);
		  log_i("[SBUS2]Invalid Frame End Byte");
        }
		//pinMatrixOutDetach(tx_pin,false,true);
     }
	 else if(buffer_index == SBUS_FRAME_SIZE){
		 uart_flush(uart_num);
		 uart_clear_intr_status(uart_num, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);
		 log_i("[SBUS2] Invalid Frame Start Byte");
	 }
	 else if((buffer_index == SLOT_DATA_LENGTH) && ((((rxbuf[0]&0x0f) == 0x03 )||(rxbuf[0]&0x0f) == 0x0B )))	// Telemetry Slot
     {
		//digitalWrite(18,HIGH);
		uint8_t temp = rxbuf[0];
		telemetryData[temp] = rxbuf[0];
		telemetryData[temp+1] = rxbuf[1];
		telemetryData[temp+2] = rxbuf[2];
		buffer_index = 0;
		//pinMatrixOutDetach(tx_pin,false,true);
		//digitalWrite(18,LOW);
	 }
	 else if(buffer_index == SLOT_DATA_LENGTH){																// Something went wrong
		 uart_flush(uart_num);
		 uart_clear_intr_status(uart_num, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);
		 log_i("[SBUS2] Invalid Slot ID");
	 }
  //}
  //digitalWrite(18,LOW);
  // after reading bytes from buffer clear UART interrupt status
  //uart_flush(uart_num);
  uart_clear_intr_status(uart_num, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);

}

/****************************************************************
 * transmit_frame
 * 
 * Implemented as an ISR to fit the strict timing requirements
 * Slot 1 has to be transmitted 2ms after receiving the SBUS2 Frame
 * Slot 2-8 has to be transmitted every 660 mirco seconds
 * Each Slot needs about 320 mircoseconds to be transmitted via uart
 * The delay between the Slots must not exceed 400 mirco seconds
 * 
 ****************************************************************/

static void IRAM_ATTR ISR_transmit_frame(void *arg)
{
  TIMERG0.int_clr_timers.t1 = 1;
  TIMERG0.hw_timer[1].config.alarm_en = 1;
  //digitalWrite(18,HIGH);
  pinMatrixOutDetach(tx_pin,false,true);
  if (sequence_count < 8) // transmit slots
  {
    uint8_t actual_slot = (sequence_count) + (gl_current_frame * NUMBER_OF_SLOT_IN_FRAME);
    // if data available in slot then send it
    if (transmit_data_per_slot_status[actual_slot] == AVAILABLE){
		//digitalWrite(18,HIGH);
		pinMatrixOutAttach(tx_pin, U1TXD_OUT_IDX, false, false);
      char buffer[SLOT_DATA_LENGTH];
      memcpy(buffer, (const void *)transmit_data_per_slot_data[actual_slot], SLOT_DATA_LENGTH);
      transmit_data_per_slot_status[actual_slot] = TRANSMITTING;
      // send the whole slot
      uart_write_bytes(uart_num, buffer, SLOT_DATA_LENGTH);
      transmit_data_per_slot_status[actual_slot] = EMPTY;
	  //digitalWrite(18,LOW);
    }
	if(sequence_count ==7){
		timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
		timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, 5150);
		//timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
		timer_set_alarm(TIMER_GROUP_0,TIMER_1,TIMER_ALARM_EN);
	}
	else{
		timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
		timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, SLOT_TIME);
		//timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
		timer_set_alarm(TIMER_GROUP_0,TIMER_1,TIMER_ALARM_EN);
	}
	
  }
  /*else if(sequence_count = 8){
	timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, 4500);
	//timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_set_alarm(TIMER_GROUP_0,TIMER_1,TIMER_ALARM_EN);
  }*/
	  
  else{
	//digitalWrite(18,HIGH);	// dummy for sending SBUS2 Frame
	if(DemoMode){
		pinMatrixOutAttach(tx_pin, U1TXD_OUT_IDX, false, false);
		uart_write_bytes(uart_num, sbus_frame, SBUS_FRAME_SIZE);
		sbus_frame[24] += 0x10;
		if(sbus_frame[24] > 0x34){
			sbus_frame[24] = 0x04;
		}
	}
	else{
		//pinMatrixOutDetach(tx_pin,false,true);
	}
		
    buffer_index = 0;
    transmit = false;
    //telemetry_ready = false;
    sequence_count = 0;

    enable_receiving();
  }
  //digitalWrite(18,LOW);
  sequence_count++;
   
}

void start_transmit_sequencer(uint8_t frame_number)
{
   gl_current_frame = frame_number;
   // we dont need more frames to be received, lets handle this one first
   disable_receiving();
}


void SBUS2_transmit_telemetry_data(uint8_t slot, uint8_t data[SLOT_DATA_LENGTH])
{
   if (transmit_data_per_slot_status[slot] != TRANSMITTING)
   {

      transmit_data_per_slot_data[slot][0] = Slot_ID[slot];
      transmit_data_per_slot_data[slot][1] = data[1];
      transmit_data_per_slot_data[slot][2] = data[2];

      transmit_data_per_slot_status[slot] = AVAILABLE;
   }
}

void SBUS2_get_all_servo_data()
{
   channels[0] = ((sbusData[1] | sbusData[2] << 8) & 0x07FF);
   channels[1] = ((sbusData[2] >> 3 | sbusData[3] << 5) & 0x07FF);
   channels[2] = ((sbusData[3] >> 6 | sbusData[4] << 2 | sbusData[5] << 10) & 0x07FF);
   channels[3] = ((sbusData[5] >> 1 | sbusData[6] << 7) & 0x07FF);
   channels[4] = ((sbusData[6] >> 4 | sbusData[7] << 4) & 0x07FF);
   channels[5] = ((sbusData[7] >> 7 | sbusData[8] << 1 | sbusData[9] << 9) & 0x07FF);
   channels[6] = ((sbusData[9] >> 2 | sbusData[10] << 6) & 0x07FF);
   channels[7] = ((sbusData[10] >> 5 | sbusData[11] << 3) & 0x07FF);
   channels[8] = ((sbusData[12] | sbusData[13] << 8) & 0x07FF);
   channels[9] = ((sbusData[13] >> 3 | sbusData[14] << 5) & 0x07FF);
   channels[10] = ((sbusData[14] >> 6 | sbusData[15] << 2 | sbusData[16] << 10) & 0x07FF);
   channels[11] = ((sbusData[16] >> 1 | sbusData[17] << 7) & 0x07FF);
   channels[12] = ((sbusData[17] >> 4 | sbusData[18] << 4) & 0x07FF);
   channels[13] = ((sbusData[18] >> 7 | sbusData[19] << 1 | sbusData[20] << 9) & 0x07FF);
   channels[14] = ((sbusData[20] >> 2 | sbusData[21] << 6) & 0x07FF);
   channels[15] = ((sbusData[21] >> 5 | sbusData[22] << 3) & 0x07FF);

   // DigiChannel 1
   if (sbusData[23] & (1 << 0))
   {
      channels[16] = 1;
   }
   else
   {
      channels[16] = 0;
   }
   // DigiChannel 2
   if (sbusData[23] & (1 << 1))
   {
      channels[17] = 1;
   }
   else
   {
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
   uart_lost_frame = 0;
   *transmision_dropt_frame = sbusData[23] & 0x04 ? true : false;
   *failsave = sbusData[23] & 0x08 ? true : false;
}

int16_t SBUS2_get_servo_data(uint8_t channel)
{
   if (channel < NUMBER_OF_CHANNELS)
   {
      return channels[channel];
   }
   else
   {
      return -1;
   }
}

bool SBUS_Ready()
{
   if (sbus_ready) // Checking sbus_ready Flag
   {
    sbus_ready = false;
    return true;
   }
   else
   {
      return false;
   }
}

bool SBUS2_Ready()
{
	//Serial.println("test");
   if (telemetry_ready)
   {
    telemetry_ready = false;
	//digitalWrite(18,LOW);
    return true;
   }
   else
   {
      return false;
   }
}

bool SBUS_Ready(bool reset)
{
  if (sbus_ready)   // Checking sbus_ready Flag
   {
	if(reset){
		sbus_ready = false;
    }
    return true;
   }
   else{
    return false;
   }
}

bool SBUS2_Ready(bool reset)
{
  if (telemetry_ready)
   {
    if(reset){
		telemetry_ready = false;
    }
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

void SBUS2_print_Raw(){
	for(uint16_t i = 0; i<256;i++){
		Serial.print(i,HEX);
		Serial.print(" ");
	}
	Serial.println();
	for(uint16_t i = 0; i<256;i++){
		Serial.print(telemetryData[i],HEX);
		Serial.print(" ");
	}
	Serial.println();
}

bool SBUS2_get_Slot(uint8_t slot, uint8_t *lowbyte, uint8_t *highbyte){
	uint8_t slotid = Slot_ID[slot];
	if(telemetryData[slotid] == slotid){
		// Sensordaten vorhanden
		*lowbyte = telemetryData[slotid+1];
		*highbyte = telemetryData[slotid+2];
		telemetryData[slotid] = 0;
		telemetryData[slotid+1] = 0;
		telemetryData[slotid+2] = 0;
		return true;
	}
	else{
		// Keine Sensordaten vorhanden
		*lowbyte = 0;
		*highbyte = 0;
		return false;
	}
}

void SBUS_disable(){
  uart_disable_rx_intr(UART_NUM_2);
  timer_set_alarm(TIMER_GROUP_0,TIMER_1,TIMER_ALARM_DIS);
}

void SBUS_enable(){
  uart_enable_rx_intr(UART_NUM_2);
  initialize_slot_status();
  enable_receiving();
}

void SBUS2_disable(){
  SBUS_disable();
}

void SBUS2_enable(){
  SBUS_enable();
}
#endif // ESP32
