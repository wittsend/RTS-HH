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
* static void motorLeftDrive(int16_t speed);
* static void motorRightDrive(int16_t speed);
*
* void motorInit(void);
* void motorStop(void);
* uint8_t moveRobot(float speed, float turnRatio);
* int8_t getEncPulses(int8_t *leftPulses, int8_t *rightPulses);
* float nfGetDistTravelled(int8_t pulses);
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"
#include <avr/interrupt.h>
#include <stdlib.h>							//abs()
#include <math.h>							//fabs()
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

#define motorLeftFwd			(MOTOR_DIR_PORT &= ~MOTOR_LEFT_DIR_PIN)
#define motorLeftRev			(MOTOR_DIR_PORT |= MOTOR_LEFT_DIR_PIN)
#define motorLeftStop			{motorLeftSpeed = 0;}
#define motorRightFwd			(MOTOR_DIR_PORT &= ~MOTOR_RIGHT_DIR_PIN)
#define motorRightRev			(MOTOR_DIR_PORT |= MOTOR_RIGHT_DIR_PIN)
#define motorRightStop			{motorRightSpeed = 0;}

#define motorLeftDir			(MOTOR_DIR_PORT & MOTOR_LEFT_DIR_PIN)
#define motorRightDir			(MOTOR_DIR_PORT & MOTOR_RIGHT_DIR_PIN)

#define motorLeftOff			(MOTOR_DRIVE_PORT &= ~MOTOR_LEFT_DRIVE_PIN)
#define motorLeftOn				(MOTOR_DRIVE_PORT |= MOTOR_LEFT_DRIVE_PIN)
#define motorRightOff			(MOTOR_DRIVE_PORT &= ~MOTOR_RIGHT_DRIVE_PIN)
#define motorRightOn			(MOTOR_DRIVE_PORT |= MOTOR_RIGHT_DRIVE_PIN)

#define motorEnableEncoders		(ENCODER_ENABLE_PORT |= ENCODER_ENABLE_PIN)
#define motorDisableEncoders	(ENCODER_ENABLE_PORT &= ~ENCODER_ENABLE_PIN)

#define encoderEnableInterrupts (EIMSK |= 0x03)
#define encoderDisableInterrupts (EIMSK &= ~0x03)

//Motor direction status values
#define MOTOR_FWD				0x00
#define MOTOR_REV				0x01

//////////////[Private Global Variables]////////////////////////////////////////////////////////////
volatile int8_t leftPulseCount, rightPulseCount; // wheel pulse counts

//////////////[Private Functions]///////////////////////////////////////////////////////////////////
/*
* Function:
* void motorLeftDrive(int16_t speed)
*
* Drives the left motor at the given speed and direction. This function is not accessible outside
* of the motor driver as the motors should be controlled by moveRobot.
*
* Inputs:
* int16_t speed:
*	An integer between -1023 and 1023 that indicates the PWM speed to drive the motor at (1023 is
*	maximum). A negative number drives the motor in reverse.
*
* Returns:
* none
*
*/
static void motorLeftDrive(int16_t speed)
{
	speed = capToRangeInt(speed, -1023, 1023);	//Make sure speed is in range
	motorLeftSpeed = abs(speed);
	if(speed > 0) motorLeftFwd;					//Forwards
	if(speed < 0) motorLeftRev;					//Reverse
}

/*
* Function:
* void motorRightDrive(int16_t speed)
*
* Drives the right motor at the given speed and direction. This function is not accessible outside
* of the motor driver as the motors should be controlled by moveRobot.
*
* Inputs:
* int16_t speed:
*	An integer between -1023 and 1023 that indicates the PWM speed to drive the motor at (1023 is
*	maximum). A negative number drives the motor in reverse.
*
* Returns:
* none
*
*/
static void motorRightDrive(int16_t speed)
{
	speed = capToRangeInt(speed, -1023, 1023);	//Make sure speed is in range
	motorRightSpeed = abs(speed);
	if(speed > 0) motorRightFwd;				//Forwards
	if(speed < 0) motorRightRev;				//Reverse
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
	if(motorLeftDir) leftPulseCount--;
	else leftPulseCount++;
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
	if(motorRightDir) rightPulseCount--;
	else rightPulseCount++;
}

//////////////[Public Functions]////////////////////////////////////////////////////////////////////
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
	|=	((TIMER1_WGM & 0x0C)<<1)
	|	(TIMER1_CS);
	
	motorLeftSpeed = 0;
	motorRightSpeed = 0;
	
	////Encoder interrupt setup////
	//Set the interrupt sense modes
	EICRA 
	=	((INT1_ISC&0x03)<<2)
	|	((INT0_ISC&0x03)<<0);
	
	//Clear interrupt flags
	EIFR
	=	0xFF;
	
	//Enable external interrupts
	EIMSK
	=	((INT1_IE<<1)&0x01)
	|	((INT0_IE<<0)&0x01);
}

/*
* Function:
* motorStop(void)
*
* Stops all motors
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void motorStop(void)
{
	motorRightDrive(0);
	motorLeftDrive(0);
}

/*
* Function:
* uint8_t moveRobot(float speed, float turnRatio)
*
* A function to move the robot in a given direction
*
* Inputs:
* float speed:
*	A floating point value between -1023 and 1023 that indicates how fast the motors should be
*	driven. Negative numbers will attempt to drive the robot backwards.
* float turnRatio:
*   The ratio of rotation to be applied to the motion (-1023 to 1023). if turnRatio is 0, then
*   robot just drives straight at 'speed'. -1023 will have robot rotating CCW on the spot at
*   'speed'. 512 would be half and half driving forward with a CW rotational element applied. If
*   both speed and turnRatio are negative, then robot will rotate in CW (-1*-1) = 1
*
* Returns:
* Returns 0 on exit
*
*/
uint8_t moveRobot(float speed, float turnRatio)
{
	//If speed is set to 0, then save processor cycles
	if(speed == 0.0)
	{
		motorStop();
		return 0;
	}

	int16_t rightMotorSpeed, leftMotorSpeed;
	
	//Make sure parameters are in range and correct if necessary
	speed = capToRangeFlt(speed, -1023, 1023);
	turnRatio = capToRangeFlt(turnRatio, -1023, 1023);
	
	//Calculate speed ratios. Positive turn ratio will see robot veer to the right
	float rotationalSpeed = fabs(speed)*(turnRatio/1023.0);
	float straightSpeed = fabs(speed) - fabs(rotationalSpeed);
	
	//Calculate individual motor speeds
	//If statement ensures there is always a value between 0 and 1023 for the motor speeds no matter
	//the signs of speed and turnRatio 
	if(speed > 0)
	{
		rightMotorSpeed		= (int16_t)(straightSpeed + rotationalSpeed);
		leftMotorSpeed		= (int16_t)(straightSpeed - rotationalSpeed);
	} else {
		rightMotorSpeed		= (int16_t)(-straightSpeed - rotationalSpeed);
		leftMotorSpeed		= (int16_t)(-straightSpeed + rotationalSpeed);		
	}
	
	//Apply speeds and directions to motors
	motorRightDrive(rightMotorSpeed);
	motorLeftDrive(leftMotorSpeed);
	
	return 0;
}

/*
* Function:
* int8_t getEncPulses(int8_t *leftPulses, int8_t *rightPulses)
*
* Returns the number of pulses that have occurred since the last time the encoder was checked
*
* Inputs:
* int8_t *leftPulses
*	A pointer to a signed int where the number of left wheel pulses will be stored.
* int8_t *rightPulses
*	A pointer to a signed int where the number of right wheel pulses will be stored.
*
* Returns:
* 0 on exit
*
* Improvement:
* Perhaps this could return the amount of time has lapsed since the encoder pulses were last read?
*
*/
int8_t getEncPulses(int8_t *leftPulses, int8_t *rightPulses)
{
	encoderDisableInterrupts;				//Disable external ints from encoders
	*leftPulses = leftPulseCount;			//Read off left pulse count
	*rightPulses = rightPulseCount;			//Read off right pulse count
	leftPulseCount = 0;						//Clear the pulse counts
	rightPulseCount = 0;					//Clear the pulse counts
	encoderEnableInterrupts;				//Re-enable the external interrupts.
	return 0;
}