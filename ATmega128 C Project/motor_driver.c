/*
* motor_driver.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/03/2018 4:49:49 PM
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* 1 or 2 liner on the purpose of the file
*
* More Info:
* Atmel ATmega128 Datasheet:http://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void funcName(void)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "avr/io.h"
#include "motor_driver.h"

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
//PWM Timer register configuration
#define TIMER1_WGM			0x07		//Fast 10-bit PWM, set on TOP
#define TIMER1_COM1A		0x02		//Clear on compare match
#define TIMER1_COM1B		0x02		//Clear on compare match
#define TIMER1_CS			0x04		//Clock select clk/256
	
//Motor control macros:
#define motorLeftDrivePin	(1<<5)		//PORTB
#define motorLeftDirPin		(1<<6)		//PORTA
#define motorRightDrivePin	(1<<6)		//PORTB
#define motorRightDirPin	(1<<7)		//PORTA

#define motorLeftSpeed		OCR1A
#define motorRightSpeed		OCR1B

#define motorLeftFwd		PORTA &= ~motorLeftDirPin
#define motorLeftRev		PORTA |= motorLeftDirPin
#define motorRightFwd		PORTA &= ~motorRightDirPin
#define motorRightRev		PORTA |= motorRightDirPin

#define motorLeftDir		PORTA & motorLeftDirPin
#define motorRightDir		PORTA & motorRightDirPin

#define motorLeftOff		PORTB &= ~motorLeftDrivePin
#define motorLeftOn			PORTB |= motorLeftDrivePin
#define motorRightOff		PORTB &= ~motorRightDrivePin
#define motorRightOn		PORTB |= motorRightDrivePin

//////////////[Private Global Variables]////////////////////////////////////////////////////////////

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function: 
* [function declaration]
*
* [brief purpose of function]
*
* Inputs:
* [input arguments and any relevant explanation]
*
* Returns:
* [return values and any relevant explanation]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void motorInit(void)
{
	
	
	
	// timer1 setup for pwm
	// enable pwm outputs, use mode 7 and prescale = 256
	// See defines above for timer settings
	TCCR1A
	|=	(TIMER1_COM1A<<6)
	|	(TIMER1_COM1B<<4)
	|	(TIMER1_WGM & 0x03);
	TCCR1B
	|=	((TIMER1_WGM & 0x0C)<<3)
	|	(TIMER1_CS);
	
	motorLeftSpeed = 0;
	motorRightSpeed = 0;
}