/*
TITLE	: UART Communication: ATmega 2560 & ESP32
DATE	: 2019/11/12
AUTHOR	: e-Yantra Team

AIM: To send data from UART#0 of ATMega 2560 to ESP32
*/

#define F_CPUL 14745600L
#include <avr/io.h>				// Standard AVR IO Library
#include <util/delay.h>			// Standard AVR Delay Library
#include <avr/interrupt.h>		// Standard AVR Interrupt Library
#include "uart.h"				// Third Party UART Library
#include "lcd.h"
#define MESSAGE	"Initialised Tx from ATmega 2560> Hello ESP32!\n" // Message to be sent on UART #0


void init_timer2(void){
	cli();	// Turn off global interrupts

	//Setup Timer2 to fire every 1ms
	TCCR2B = 0x00;        						// Cut off Clock Source to disbale Timer2 while we set it up
	TCNT2  = 130;         						// Reset Timer Count to 130 out of 255
	TIFR2  &= ~(1 << TOV2);        				// Timer2 INT Flag Reg: Clear Timer Overflow Flag
	TIMSK2 |= (1 << TOIE2);        				// Timer2 INT Reg: Timer2 Overflow Interrupt Enable
	TCCR2A = 0x00;        						// Timer2 Control Reg A: Wave Gen Mode normal
	TCCR2B |= (1 << CS22) | (1 << CS20);        // Timer2 Control Reg B: Timer Prescaler set to 128 and Start Timer2

	sei();	// Turn on global interrupts
}


//Timer2 Overflow Interrupt Vector
ISR(TIMER2_OVF_vect) {
  count_time++;	// increment after 1 ms               
  
  // increment seconds variable after 1000 ms
  if(count_time > 999){
	seconds_time++;
	
	//uart3_puts(MESSAGE);    // Send data on UART #3 after 1 second
//	lcd_clear();
//	lcd_string(2,2,MESSAGE);
    count_time = 0;          
  }
  
  TCNT2 = 130;           	// Reset Timer to 130 out of 255
  TIFR2  &= ~(1 << TOV2);	// Timer2 INT Flag Reg: Clear Timer Overflow Flag
};

char uart3_readByte(void)
{

	uint16_t rx;
	uint8_t rx_status, rx_data;

	rx = uart3_getc();
	rx_status = (uint8_t)(rx >> 8);
	rx = rx << 8;
	rx_data = (uint8_t)(rx >> 8);

	if(rx_status == 0 && rx_data != 0){
		return rx_data;
	} else {
		return -1;
	}

}

void setup_uart(void) 
{
	init_timer2();
	front = -1;
	rear = -1;
	init_queue(); // RPC Queue is intialised to 0
	uart3_init(UART_BAUD_SELECT(115200, F_CPUL));
	uart3_flush();
	//uart3_puts(MESSAGE);
}


void init_queue()
{
	for(int i =0;i<20;i++)
		queue[i] = 0;
}
void enqueue(char item)
{
	if (rear == 19)
	{
		// printf("Can't enqueue as the queue is full\n");
		return;
	}
	else
	{
		//The first element condition
		if (front == -1)
		{
			front = 0;
		}
		rear = rear + 1;
		queue[rear] = item;
		//printf("We have enqueued %d\n",item);
		return;
	}
}

char dequeue(void)
{
	char c = 0;
	if (front == -1)
	{
		//printf("Can't dequeue as the queue is empty\n");
		return 'z';
	}
	else
	{
		// printf("We have dequeued : %d\n", queue[front]);
		c = queue[front];
		queue[front] = 0;
		front = front + 1;

		//Only happens when the last element was dequeued
		if (front > rear)
		{
			front = -1;
			rear = -1;
		}
		return c;
	}
}
