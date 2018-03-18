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
* void nfUpdateNavigationData(RobotGlobalData *sys);
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"
#include <stdio.h>			//sprintf
#include <math.h>			//Trig functions
#include "timer.h"			//For getting the system time stamp.
#include "motor_driver.h"	//For reading from the wheel encoders
#include "navigation.h"

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
//Robot dimensions
#define TRACK_WIDTH			0.10640		//Track width of robots wheels (m)
#define TRACK_WIDTH_H		0.0532		//Half track width (m)
#define TRACK_WIDTH_P		14.9564		//Track width in pulses
#define PULSE_DIST			0.007114	//Distance travelled in one encoder pulse (m)
#define PULSE_ANG			0.06686		//Angle rotated in one pulse (PULSE_DIST/TRACK_WIDTH) (rad)

//////////////[Private Global Variables]////////////////////////////////////////////////////////////

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void nfCalcPosition(RobotGlobalData *sys)
*
* Calculates the change in position of the robot from the number of encoder pulses read from each
* wheel.
*
* Inputs:
* RobotGlobalData *sys:
*	A pointer to the global data structure
*
* Returns:
* none
*
*/
void nfCalcPosition(RobotGlobalData *sys)
{
	float dTheta = 0;				//delta angle of each wheel in radians
	float r = 0, dx = 0, dy = 0;

	// calculate the new position
	//Calculate the change in heading
	dTheta = (float)(sys->pos.rightPulses - sys->pos.leftPulses)*PULSE_ANG;
	
	//Calculate the change in position
	if(sys->pos.rightPulses != sys->pos.leftPulses)
	{
		//Calculate the radius of the curvature of the motion
		r	= (TRACK_WIDTH_H*(sys->pos.rightPulses + sys->pos.leftPulses))
			/(float)(sys->pos.rightPulses - sys->pos.leftPulses);
		dx	= r*(sin(sys->pos.heading + dTheta) - sin(sys->pos.heading));
		dy	= r*(cos(sys->pos.heading) - cos(sys->pos.heading - dTheta));
	} else {
		dx	= PULSE_DIST*sys->pos.rightPulses*cos(sys->pos.heading);
		dy	= PULSE_DIST*sys->pos.rightPulses*sin(sys->pos.heading);
	}
	
	// add pulses to the totals
	sys->pos.leftTotal += sys->pos.leftPulses;
	sys->pos.rightTotal += sys->pos.rightPulses;
	
	//Update position structure.
	sys->pos.heading += dTheta;
	sys->pos.x += dx;
	sys->pos.y += dy;
	
	/***Move this to a comunication Module***/
	// display the new position (convert heading to degrees)
	char str[100]; // serial output string
	sprintf(str, "POS,%6.3f,%6.3f,%6.1f,%1d,%1d,%4d,%4d,%4i\r\n",
		sys->pos.x, sys->pos.y, sys->pos.heading * 180. / M_PI,	sys->pos.leftPulses,
		sys->pos.rightPulses, sys->pos.leftTotal, sys->pos.rightTotal, sys->timeStamp);
	OutputString(str);	
}

/*
* Function: 
* void nfUpdateNavigationData(RobotGlobalData *sys)
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
void nfUpdateNavigationData(RobotGlobalData *sys)
{
	//Update the system time stamp from the timer driver module. This provides a level of
	//abstraction from the hardware drivers.
	sys->timeStamp = systemTimestamp;
	
	//Get the wheel encoder pulse counts from the motor driver module
	getEncPulses(&sys->pos.leftPulses, &sys->pos.rightPulses);
	
	//If movement has occurred, re-calculate position
	if(sys->pos.leftPulses || sys->pos.rightPulses)
	{
		//Calculate position of robot.
		nfCalcPosition(sys);
	}
}

/*
* Function:
* float nfGetDistTravelled(int8_t pulses)
*
* Will return the distance a wheel has traveled based on how many pulses have occurred. Distance
* constants are set in the defines in the motor_driver module
*
* Inputs:
* int8_t pulses
*	How many pulses to calculate the distance for
*
* Returns:
* A float containing the distance in metres
*
*/
float nfGetDistTravelled(int8_t pulses)
{
	return pulses*PULSE_DIST;
}