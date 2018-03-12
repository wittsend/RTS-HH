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
#include "robot_setup.h"
#include <stdlib.h>							//abs()
#include "motor_driver.h"

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
//Motor PWM Timer register configuration
#define TIMER1_WGM				0x07		//Fast 10-bit PWM, set on TOP
#define TIMER1_COM1A			0x02		//Clear on compare match
#define TIMER1_COM1B			0x02		//Clear on compare match
#define TIMER1_CS				0x04		//Clock select clk/256

//Encoder External Interrupt register configuration
#define INT0_ISC				0x03		//Rising edge trigger
#define INT0_IE					0x01		//Interrupt Enable
#define INT1_ISC				0x03		//Rising edge trigger
#define INT1_IE					0x01		//Interrupt Enable

//Motor control macros:
#define MOTOR_LEFT_DRIVE_PIN	(1<<5)		//PORTB
#define MOTOR_LEFT_DIR_PIN		(1<<6)		//PORTA
#define MOTOR_RIGHT_DRIVE_PIN	(1<<6)		//PORTB
#define MOTOR_RIGHT_DIR_PIN		(1<<7)		//PORTA
#define ENCODER_ENABLE_PIN		(1<<3)		//PORTC

#define MOTOR_DRIVE_PORT		PORTB		
#define MOTOR_DIR_PORT			PORTA
#define ENCODER_ENABLE_PORT		PORTC

#define ENCODER_LEFT_ISR_VECT	INT0_vect	//Interrupt vector for left wheel encoder
#define ENCODER_RIGHT_ISR_VECT	INT1_vect	//Interrupt vector for right wheel encoder

#define motorLeftSpeed			OCR1A
#define motorRightSpeed			OCR1B

#define motorLeftFwd			MOTOR_DIR_PORT &= ~MOTOR_LEFT_DIR_PIN
#define motorLeftRev			MOTOR_DIR_PORT |= MOTOR_LEFT_DIR_PIN
#define motorLeftStop			{motorLeftSpeed = 0;}
#define motorRightFwd			MOTOR_DIR_PORT &= ~MOTOR_RIGHT_DIR_PIN
#define motorRightRev			MOTOR_DIR_PORT |= MOTOR_RIGHT_DIR_PIN
#define motorRightStop			{motorRightSpeed = 0;}

#define motorLeftDir			MOTOR_DIR_PORT & MOTOR_LEFT_DIR_PIN
#define motorRightDir			MOTOR_DIR_PORT & MOTOR_RIGHT_DIR_PIN

#define motorLeftOff			MOTOR_DRIVE_PORT &= ~MOTOR_LEFT_DRIVE_PIN
#define motorLeftOn				MOTOR_DRIVE_PORT |= MOTOR_LEFT_DRIVE_PIN
#define motorRightOff			MOTOR_DRIVE_PORT &= ~MOTOR_RIGHT_DRIVE_PIN
#define motorRightOn			MOTOR_DRIVE_PORT |= MOTOR_RIGHT_DRIVE_PIN

#define motorEnableEncoders		ENCODER_ENABLE_PORT |= ENCODER_ENABLE_PIN
#define motorDisableEncoders	ENCODER_ENABLE_PORT &= ~ENCODER_ENABLE_PIN

//Motor direction status values
#define MOTOR_FWD				0x00
#define MOTOR_REV				0x01

//////////////[Private Global Variables]////////////////////////////////////////////////////////////
volatile unsigned int leftPulseCount, rightPulseCount; // wheel pulse counts

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function: 
* void motorInit(void)
*
* Initialises all hardware required to drive the motors on the Hedgehog
*
* Inputs:
* none
*
* Returns:
* none
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
	=	ENCODER_ENABLE_PIN; 
	
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
	//Set the interrupt sense modes
	EICRA 
	=	((INT1_ISC<<0)&0x03)
	|	((INT0_ISC<<0)&0x03);
	
	//Clear interrupt flags
	EIFR
	=	0xFF;
	
	//Enable interrupts
	EIMSK
	=	((INT1_IE<<0)&0x01)
	|	((INT0_IE<<0)&0x01);
}

void motorLeftDrive(signed char speed)
{
	speed = capToRangeInt(speed, -1023, 1023);	//Make sure speed is in range
	motorLeftSpeed = abs(speed);
	if(speed > 0)		//Forwards
		motorLeftFwd;
	if(speed < 0)		//Reverse
		motorLeftRev;
}

void motorRightDrive(signed char speed)
{
	speed = capToRangeInt(speed, -1023, 1023);	//Make sure speed is in range
	motorRightSpeed = abs(speed);
	if(speed > 0)		//Forwards
		motorRightFwd;
	if(speed < 0)		//Reverse
		motorRightRev;
}

void motorStop(void)
{
	motorRightDrive(0);
	motorLeftDrive(0);
}

uint8_t moveRobot(float speed, float turnRatio)
{
	int8_t rightMotorSpeed, leftMotorSpeed;
	
	//If speed is set to 0, then save processor cycles
	if(speed == 0.0)
	{
		motorStop();
		return 0;
	}
	
	//Make sure parameters are in range and correct if necessary
	speed = capToRangeFlt(speed, -100, 100);
	turnRatio = capToRangeFlt(turnRatio, -100, 100);
	
	//Calculate speed ratios
	float rotationalSpeed = speed*(turnRatio/100.0);
	float straightSpeed = abs(speed) - (abs(rotationalSpeed));
	
	//Calculate individual motor speeds
	rightMotorSpeed		= -rotationalSpeed - straightSpeed;
	leftMotorSpeed		= rotationalSpeed - straightSpeed ;
	
	//Apply speeds and directions to motors
	motorRightDrive(rightMotorSpeed);
	motorLeftDrive(leftMotorSpeed);
	
	return 0;
}

/*
* Function:
* ISR(ENCODER_LEFT_ISR_VECT)
*
* Interrupt service routine for left wheel pulse counter 
*
* Inputs:
* none
*
* Returns:
* none
*
*/
ISR(ENCODER_LEFT_ISR_VECT)
{
	//If motor is being driven backwards
	if(motorLeftDir)
		leftPulseCount--;
	else
		leftPulseCount++;
}

/*
* Function:
* ISR(ENCODER_RIGHT_ISR_VECT)
*
* Interrupt service routine for right wheel pulse counter
*
* Inputs:
* none
*
* Returns:
* none
*
*/
ISR(ENCODER_RIGHT_ISR_VECT)
{
	//If motor is being driven backwards
	if(motorRightDir)
		rightPulseCount--;
	else
		rightPulseCount++;
}