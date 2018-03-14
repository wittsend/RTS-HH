/*
* navigation.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 15/03/2018 10:37:41 AM
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
* void updateNavigationData(RobotGlobalData *sys);
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"
#include "timer.h"			//For getting the system time stamp.
#include "motor_driver.h"	//For reading from the wheel encoders
#include "navigation.h"

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////

//////////////[Private Global Variables]////////////////////////////////////////////////////////////

//////////////[Functions]///////////////////////////////////////////////////////////////////////////

void calcPosition(RobotGlobalData *sys)
{
	
}

/*
* Function: 
* void updateNavigationData(RobotGlobalData *sys)
*
* Retrieves navigation and time data from the robot's sensors and timer
*
* Inputs:
* RobotGlobalData *sys:
*	A pointer to the global data structure
*
* Returns:
* none
*
*/
void updateNavigationData(RobotGlobalData *sys)
{
	//Update the system time stamp from the timer driver module. This provides a level of
	//abstraction from the hardware drivers.
	sys->timeStamp = systemTimestamp;
	
	//Get the wheel encoder pulse counts from the motor driver module
	getEncPulses(&sys->pos.leftPulses, &sys->pos.rightPulses);
	
	//If movement has occurred, re-calculate position
	if(sys->pos.leftPulses || sys->pos.rightPulses)
	{
		return;
		//Calculate position of robot.
	}
}