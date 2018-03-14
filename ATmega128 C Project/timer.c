/*
* timer.c
*
* Author : Matthew Witt
* Created: 6/08/2017 1:23:27 PM
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* Sets up the timer for 1ms interrupts. Modified for Hedgehog robots
*
* More Info:
* Atmel ATmega128 Datasheet:http://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void timer3Init(void)
* int delay_ms(uint16_t period_ms);
* uint8_t nbdelay_ms(uint16_t period_ms);
* int get_ms(uint32_t *timestamp);
* ISR(TIMER3_COMPA_vect)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"
#include <avr/interrupt.h>

#include "timer.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
#define TIMER3_COMPA	0x00				//Normal
#define TIMER3_COMPB	0x00				//Normal
#define TIMER3_COMPC	0x00				//Normal
#define TIMER3_FOCA		0x00				//Not used
#define TIMER3_FOCB		0x00				//Not used
#define TIMER3_WGM		0x02				//Mode 2 CTC
#define TIMER3_CLKSEL	0x03				//div by 64
#define TIMER3_OCR0A	125					//Compare every 125 counts (1ms)
#define TIMER3_OCR0B	0x00				//Not used
#define TIMER3_OCAIE	0x01				//Output compare interrupt enabled
#define TIMER3_OCBIE	0x00				//Not used
#define TIMER3_OVIE		0x00				//Not used

///////////////[Private Global Vars]////////////////////////////////////////////////////////////////
volatile uint32_t systemTimestamp = 0;		//Global timestamp (ms)
volatile uint16_t delaymsCounter = 0;		//Used by delay_ms()

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void timer3Init(void)
*
* Initializes timer3
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* See above for timer2 bit field settings
*
*/
void timer3Init(void)
{
	//See the settings for timer 3 in the defines above
	TCCR3A
	=	((TIMER3_COMPA & 0x03) << 6)
	|	((TIMER3_COMPB & 0x03) << 4)
	|	((TIMER3_WGM & 0x03) << 0);
	
	TCCR3B
	=	((TIMER3_FOCA & 0x01) << 7)
	|	((TIMER3_FOCB & 0x01) << 6)
	|	((TIMER3_WGM & 0x04) << 1)
	|	((TIMER3_CLKSEL & 0x07) << 0);
	
	ETIMSK
	=	((TIMER3_OCBIE & 0x01) << 3)
	|	((TIMER3_OCAIE & 0x01) << 4);

	OCR3A = TIMER3_OCR0A;
	OCR3B = TIMER3_OCR0B;
}

/*
* Function: int delay_ms(uint16_t period_ms)
*
* Halts execution for desired number of milliseconds.
*
* Inputs:
* period_ms is the number of milliseconds to wait
*
* Returns:
* Always returns 0
*
*/
int delay_ms(uint16_t period_ms)
{
	while(period_ms > 0)
	{
		if(delaymsCounter)
		{
			delaymsCounter = 0;
			period_ms--;
		}
	}
	return 0;
}

/*
* Function:
* uint8_t nbdelay_ms(uint16_t period_ms)
*
* Non blocking delay.
*
* Inputs:
* uint32_t period_ms
*   Delay in ms
*
* Returns:
* 0 when time is up, otherwise 1
*
* Implementation:
* A state machine governs this timer. The start state will read off the time that the delay started
* from the system time stamp. From here it moves to the wait state. If the system time stamp isn't
* greater than the start time plus the delay period, then time isn;t up, so return 1. If the desired
* amount of time has elapsed, then move to the stop state, and reset the function. Return 0 when
* done.
*
*/
uint8_t nbdelay_ms(uint16_t period_ms)
{
	enum {START, WAIT, STOP};
	static uint8_t delayState = START;
	static uint32_t startTime;
	uint32_t timeStamp;

	if(!period_ms)
	delayState = STOP;
	
	switch(delayState)
	{
		case START:
			get_ms(&startTime);
			delayState = WAIT;
			break;
		
		case WAIT:
			get_ms(&timeStamp);
			if(timeStamp > startTime + period_ms)
			delayState = STOP;
			break;
		
		case STOP:
			delayState = START;
			return 0;
			break;
	}
	return 1;
}

/*
* Function: int get_ms(uint32_t *timestamp)
*
* Outputs the system uptime generated from Timer 3.
*
* Inputs:
* address of an integer where the timestamp will be stored
*
* Returns:
* function will return 1 if invalid pointer is passed, otherwise a 0 on success
*
* Implementation:
* Retrieves the value stored in sys.timeStamp (stores the number of millisecs that have elapsed
* since power on) and drops it at the address given by *timestamp. if *timestamp is an invalid
* address then returns a 1.
*
*/
int get_ms(uint32_t *timestamp)
{
	if(!timestamp)
		return 1;
	*timestamp = systemTimestamp;
	return 0;
}

/*
* Function:
* ISR(TIMER3_COMPA_vect)
*
* Timer 2 output compare A interrupt
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Increments systemTimestamp for timing applications
*
*/
ISR(TIMER3_COMPA_vect)
{
	delaymsCounter++;	//used for delay ms
	systemTimestamp++;
}

