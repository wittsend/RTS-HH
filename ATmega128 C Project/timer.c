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
* ISR(TIMER3_COMPA_vect)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"
#include <avr/interrupt.h>

#include "timer.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
#define TIMER3_COMPA	0x00	//Normal
#define TIMER3_COMPB	0x00	//Normal
#define TIMER3_COMPC	0x00	//Normal
#define TIMER3_FOCA		0x00
#define TIMER3_FOCB		0x00
#define TIMER3_WGM		0x02	//Mode 2 CTC
#define TIMER3_CLKSEL	0x04	//div by 64
#define TIMER3_OCR0A	125		//Compare every 125 counts (1ms)
#define TIMER3_OCR0B	0x00
#define TIMER3_OCAIE	0x01	//Output compare interrupt enabled
#define TIMER3_OCBIE	0x00
#define TIMER3_OVIE		0x00

///////////////Global Vars//////////////////////////////////////////////////////////////////////////
uint32_t systemTimestamp = 0;

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
	TCCR3A
	=	((TIMER3_COMPA & 0x03) << 6)
	|	((TIMER3_COMPB & 0x03) << 4)
	|	((TIMER3_WGM & 0x03) << 0);
	
	TCCR3B
	=	((TIMER3_FOCA & 0x01) << 7)
	|	((TIMER3_FOCB & 0x01) << 6)
	|	((TIMER3_WGM & 0x04) << 1)
	|	((TIMER3_CLKSEL & 0x07) << 0);
	
	TIMSK
	=	((TIMER3_OCBIE & 0x01) << 2)
	|	((TIMER3_OCAIE & 0x01) << 1)
	|	((TIMER3_OVIE & 0x01) << 0);

	OCR3A = TIMER3_OCR0A;
	OCR3B = TIMER3_OCR0B;
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
	systemTimestamp++;
}

