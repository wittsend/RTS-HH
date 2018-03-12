/*
* motor_driver.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/03/2018 4:49:49 PM
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* Hedgehog robot motor driver module
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
#include <avr/io.h>
#include "motor_driver.h"

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
//Motor PWM Timer register configuration
#define TIMER1_WGM			0x07		//Fast 10-bit PWM, set on TOP
#define TIMER1_COM1A		0x02		//Clear on compare match
#define TIMER1_COM1B		0x02		//Clear on compare match
#define TIMER1_CS			0x04		//Clock select clk/256

//Encoder External Interrupt register configuration


//Motor control macros:
#define MOTOR_LEFT_DRIVE_PIN	(1<<5)		//PORTB
#define MOTOR_LEFT_DIR_PIN		(1<<6)		//PORTA
#define MOTOR_RIGHT_DRIVE_PIN	(1<<6)		//PORTB
#define MOTOR_RIGHT_DIR_PIN		(1<<7)		//PORTA
#define MOTOR_ENC_ENABLE_PIN	(1<<3)		//PORTC

#define MOTOR_DRIVE_PORT		PORTB		
#define MOTOR_DIR_PORT			PORTA
#define MOTOR_ENC_ENABLE_PORT	PORTC

#define motorLeftSpeed			OCR1A
#define motorRightSpeed			OCR1B

#define motorLeftFwd			MOTOR_DIR_PORT &= ~MOTOR_LEFT_DIR_PIN
#define motorLeftRev			MOTOR_DIR_PORT |= MOTOR_LEFT_DIR_PIN
#define motorRightFwd			MOTOR_DIR_PORT &= ~MOTOR_RIGHT_DIR_PIN
#define motorRightRev			MOTOR_DIR_PORT |= MOTOR_RIGHT_DIR_PIN

#define motorLeftDir			MOTOR_DIR_PORT & MOTOR_LEFT_DIR_PIN
#define motorRightDir			MOTOR_DIR_PORT & MOTOR_RIGHT_DIR_PIN

#define motorLeftOff			MOTOR_DRIVE_PORT &= ~MOTOR_LEFT_DRIVE_PIN
#define motorLeftOn				MOTOR_DRIVE_PORT |= MOTOR_LEFT_DRIVE_PIN
#define motorRightOff			MOTOR_DRIVE_PORT &= ~MOTOR_RIGHT_DRIVE_PIN
#define motorRightOn			MOTOR_DRIVE_PORT |= MOTOR_RIGHT_DRIVE_PIN

#define motorEnableEncoders		MOTOR_ENC_ENABLE_PORT |= MOTOR_ENC_ENABLE_PIN
#define motorDisableEncoders	MOTOR_ENC_ENABLE_PORT &= ~MOTOR_ENC_ENABLE_PIN

//Motor direction status values
#define MOTOR_FWD				0x00
#define MOTOR_REV				0x01

//////////////[Private Global Variables]////////////////////////////////////////////////////////////
volatile unsigned int leftPulseCount, rightPulseCount; // wheel pulse counts

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
	////PIO Initialisation////
	//Set motor direction pins for output on PORTA
	DDRA
	=	MOTOR_LEFT_DIR_PIN
	|	MOTOR_RIGHT_DIR_PIN;
	
	//Set motor drive pins for output on PORTB
	DDRB
	=	MOTOR_LEFT_DRIVE_PIN
	|	MOTOR_RIGHT_DRIVE_PIN;
	
	//Set both motors to the forward direction initially
	motorLeftFwd;
	motorRightFwd;
	
	//Make sure both motors are off
	motorLeftOff;
	motorRightOff;
	
	//Enable the wheel pulse generator electronics
	DDRC
	=	MOTOR_ENC_ENABLE_PIN; 
	
	motorEnableEncoders;	
	
	////Timer1 setup for motor PWM////
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
	
	////Encoder interrupt setup////
	EICRA = 0b00001111;
	EIFR = 0b00000011; // clear interrupt flags
	EIMSK = 0b00000011; // enable INT0, INT1
}





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
ISR(INT0_vect) // left wheel pulse counter
{
	if(motorLeftDir)
		leftPulseCount--;
	else
		leftPulseCount++;
}

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
ISR(INT1_vect) // right wheel pulse counter
{
	if(motorRightDir)
		rightPulseCount--;
	else
		rightPulseCount++;
}