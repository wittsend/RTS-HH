/*
* robot_setup.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/03/2018 9:00:13 PM
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* This file sets up the robot's hardware and provides general functions that don't belong in one
* module or another.
*
* More Info:
* Atmel ATmega128 Datasheet:http://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void robotSetup(void);
* int32_t capToRangeInt(int32_t valueToCap, int32_t minimumVal, int32_t maximumVal)
* float capToRangeFlt(float valueToCap, float minimumVal, float maximumVal)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"

#include "motor_driver.h"
#include "timer.h"
#include "uart_driver.h"

#include <avr/interrupt.h>	//Allows the initialisation routine to enable global interrupts.

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////

//////////////[Private Global Variables]////////////////////////////////////////////////////////////
//Initialise the robot's global data structure
RobotGlobalData sys = 
{
	//Robot's FSM initial states are set here
	.state =
	{
		.main = M_IDLE,
		.gtp = GTP_START
	},
	
	//Initial position data
	.pos =
	{
		.x				= 0,
		.y				= 0,
		.heading		= 0,
		.leftPulses		= 0,
		.rightPulses	= 0,
		.leftTotal		= 0,
		.rightTotal		= 0
	},
	
	//Initialise remote command structure
	.rc =
	{
		.cmd			= RC_CMD_NONE,
		.newCmd			= 0,
		.x				= 0,
		.y				= 0
	},
	
	//PID calculation interval (ms)
	.pidUpdateInterval	= 100,
	.pidNextPIDUpdate	= 0,
	
	//System time stamp.
	.timeStamp			= 0
};

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function: 
* void robotSetup(void)
*
* Initialises all hardware on the robot by calling the initialisation routines from each driver
* module.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void robotSetup(void)
{
	timer3Init();		//Initialise the system timer	
	motorInit();		//Initialise the motor and encoder hardware
	uart0Init();		//Initialise serial comms

	sei();				//Globally enable interrupts.
	
	// display started on serial port
	uart0OutputString("\r\n**************\r\n");
	uart0OutputString("ROBOT POSITION\r\n");
	
	delay_ms(2500);		//Prevents robot from running away immediately
}

/*
* Function:
* type capToRangeInt(type valueToCap, type minimumVal, type maximumVal)
*
* Will see if a value is within a given range. If it is outside the given range, then limit the
* value to the given minimum or maximum value. Three different versions of this function operate on
* different types of variable. (Signed and unsigned integers, and single precision floating point
* numbers.
*
* Inputs:
* valueToCap:
*   The number we are checking to see if it is in range.
* minimumVal:
*   The minimumValue that we would like valueToCap to be
* maximumVal:
*   The maximum value we would like valueToCap to be.
*
* Returns:
* If valueToCap was outside the desired range, then a range limited version of valueToCap is
* returned, otherwise valueToCap is returned unmodified.
*
* Implementation:
* If valueToCap is greater than maximumVal, then make it equal maximumValue, otherwise if
* valueToCap is less than minimumValue then make it equal minimum value.
*
*/
int32_t capToRangeInt(int32_t valueToCap, int32_t minimumVal, int32_t maximumVal)
{
	if(valueToCap > maximumVal) valueToCap = maximumVal;
	if(valueToCap < minimumVal)	valueToCap = minimumVal;
	return valueToCap;
}

float capToRangeFlt(float valueToCap, float minimumVal, float maximumVal)
{
	if(valueToCap > maximumVal)	valueToCap = maximumVal;
	if(valueToCap < minimumVal)	valueToCap = minimumVal;
	return valueToCap;
}
