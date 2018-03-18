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
//Robot dimensions
#define TRACK_WIDTH			0.10640	//Track width of robots wheels (m)
#define TRACK_WIDTH_P		14.9564	//Track width in pulses
#define PULSE_DIST			0.007114	//Distance travelled in one encoder pulse (m)

//////////////[Private Global Variables]////////////////////////////////////////////////////////////

//////////////[Functions]///////////////////////////////////////////////////////////////////////////

void calcPosition(RobotGlobalData *sys)
{
	float dTheta = 0;				//delta angle of each wheel in radians
	float r = 0, dx = 0, dy = 0;
	
	static int leftTotal = 0, rightTotal = 0, timeTotal = 0;

	// if there were any pulses
	if (sys->pos.leftPulses || sys->pos.rightPulses)
	{
		// calculate the new position
		//Calculate the change in heading
		dTheta = (float)(sys->pos.rightPulses - sys->pos.leftPulses)*PULSE_DIST/TRACK_WIDTH;
	
		//Calculate the change in position
		if(sys->pos.rightPulses != sys->pos.leftPulses)
		{
			//Calculate the radius of the curvature of the motion
			r	= (TRACK_WIDTH/2*(float)(sys->pos.rightPulses + sys->pos.leftPulses)*PULSE_DIST)
				/((float)(sys->pos.rightPulses - sys->pos.leftPulses)*PULSE_DIST);
			dx	= r*(sin(sys->pos.heading + dTheta) - sin(sys->pos.heading));
			dy	= r*(cos(dTheta) - cos(dTheta - sys->pos.heading));
		} else {
			dx	= PULSE_DIST*sys->pos.rightPulses*cos(sys->pos.heading);
			dy	= PULSE_DIST*sys->pos.rightPulses*sin(sys->pos.heading);
		}
	
		// add pulses to the totals
		leftTotal += sys->pos.leftPulses;
		rightTotal += sys->pos.rightPulses;
	
		//Update position structure.
		sys->pos.heading += dTheta;
		sys->pos.x += dx;
		sys->pos.y += dy;
	
		/***Move this to a comunication Module***/
		// display the new position (convert heading to degrees)
		char str[100]; // serial output string
		sprintf(str, "POS,%6.3f,%6.3f,%6.1f,%1d,%1d,%4d,%4d,%4d\r\n",
			sys->pos.x, sys->pos.y, sys->pos.heading * 180. / M_PI,
			sys->pos.leftPulses, sys->pos.rightPulses, leftTotal, rightTotal, sys->timeStamp);
		OutputString(str);	
	}
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