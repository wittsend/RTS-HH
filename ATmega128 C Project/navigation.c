/*
* navigation.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 15/03/2018 10:37:41 AM
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* This module retrieves raw navigation data from the motor driver and calculates the position of 
* the robot. Additionally, it retrieves and stored the system timestamp from the timer module and
* stores it in the Robot Global Structure, and it also provides functions for converting and working
* with angles.
*
* More Info:
* Atmel ATmega128 Datasheet:http://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* static void nfCalcPosition(RobotGlobalData *sys)
*
* void nfUpdateNavigationData(RobotGlobalData *sys);
* float nfWrapAngle(float angleDeg);
* float nfWrapAngleRad(float angleRad);
* float nfDeg2Rad(float deg)
* float nfRad2Deg(float rad)
* void nfGetDist(float x1, float y1, float x2, float y2, float *heading, float *distance)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"
#include <math.h>			//Trig functions
#include "timer.h"			//For getting the system time stamp.
#include "uart_driver.h"	//For outputting position data to PC
#include "motor_driver.h"	//For reading from the wheel encoders
#include "uart_driver.h"	//For communicating with the PC
#include "navigation.h"

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
//Robot dimensions
#define TRACK_WIDTH			0.10640		//Track width of robots wheels (m)
//#define TRACK_WIDTH_H		0.0532		//Half track width (m)
#define TRACK_WIDTH_H		(TRACK_WIDTH/2)//Half track width (m)
#define PULSE_DIST			0.007514	//Distance travelled in one encoder pulse (m)
//#define PULSE_ANG			0.06780		//Angle rotated in one pulse (PULSE_DIST/TRACK_WIDTH) (rad)
#define PULSE_ANG			(PULSE_DIST/TRACK_WIDTH)//Angle rotated in one pulse 
													//(PULSE_DIST/TRACK_WIDTH) (rad)

//////////////[Private Global Variables]////////////////////////////////////////////////////////////

//////////////[Private Functions]///////////////////////////////////////////////////////////////////
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
static void nfCalcPosition(RobotGlobalData *sys)
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
		dy	= r*(cos(sys->pos.heading) - cos(sys->pos.heading + dTheta));
	} else {
		dx	= PULSE_DIST*sys->pos.rightPulses*cos(sys->pos.heading);
		dy	= PULSE_DIST*sys->pos.rightPulses*sin(sys->pos.heading);
	}
	
	// add pulses to the totals
	sys->pos.leftTotal += sys->pos.leftPulses;
	sys->pos.rightTotal += sys->pos.rightPulses;
	
	//Update position structure.
	sys->pos.heading += dTheta;
	sys->pos.heading = nfWrapAngleRad(sys->pos.heading);
	sys->pos.x += dx;
	sys->pos.y += dy;
	
	// display the new position (convert heading to degrees)
	char str[100]; // serial output string
	sprintf(str, "POS,%6.3f,%6.3f,%6.1f,%1d,%1d,%4d,%4d,%4i\r\n",
		sys->pos.x, sys->pos.y, sys->pos.heading * 180. / M_PI,	sys->pos.leftPulses,
		sys->pos.rightPulses, sys->pos.leftTotal, sys->pos.rightTotal, (unsigned int)
		sys->timeStamp);
	
	uartOutputString(str);	
}

//////////////[Public Functions]////////////////////////////////////////////////////////////////////
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
	sys->timeStamp = get_ms();
	
	//Get the wheel encoder pulse counts from the motor driver module (if any)
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
* float nfWrapAngle(float angleDeg)
*
* Will take any angle in degrees and convert it to its equivalent value between -180 and 180 degrees
*
* Inputs:
* float angleDeg
*   Angle to wrap (in degrees)
*
* Returns:
* Wrapped equivalent of the given angle
*
*/
float nfWrapAngle(float angleDeg)
{
	while(angleDeg > 180.0)
		angleDeg -= 360.0;
	while(angleDeg < -179.99)
		angleDeg += 360.0;
	return angleDeg;
}

/*
* Function:
* float nfWrapAngle(float angleDeg)
*
* Will take any angle in radians and convert it to its equivalent value between -PI and PI radians
*
* Inputs:
* float angleRad
*   Angle to wrap (in radians)
*
* Returns:
* Wrapped equivalent of the given angle
*
*/
float nfWrapAngleRad(float angleRad)
{
	while(angleRad > M_PI)
		angleRad -= (2*M_PI);
	while(angleRad < (-1*M_PI))
		angleRad += (2*M_PI);
	return angleRad;
}

/*
* Function:
* float nfDeg2Rad(float deg)
*
* Converts an angle from degrees to radians
*
* Inputs:
* float deg
*   Angle to convert (in degrees)
*
* Returns:
* The equivalent angle in radians
*
*/
float nfDeg2Rad(float deg)
{
	return deg*M_PI/180.;
}

/*
* Function:
* float nfRad2Deg(float rad)
*
* Converts an angle from radians to degrees
*
* Inputs:
* float rad
*   Angle to convert (in radians)
*
* Returns:
* The equivalent angle in degrees
*
*/
float nfRad2Deg(float rad)
{
	return rad*180./M_PI;
}

/*
* Function:
* void nfGetDist(float x1, float y1, float x2, float y2, float *heading, float *distance)
*
* Calculates the distance and heading between two points in space (m)
*
* Inputs:
* float x1, y1:
*	The cordinates of the first point
* float x2, y2:
*	The coordinates of the second point
* float *heading:
*	A pointer where the relative heading between the two points will be returned to
* float *distance:
*	A pointer where the distance will be returned.
*
* Returns:
* none
*
*/
void nfGetDist(float x1, float y1, float x2, float y2, float *heading, float *distance)
{
	float diffX = (x2 - x1);
	float diffY = (y2 - y1);
	*distance = sqrt(diffX*diffX + diffY*diffY);
	
	*heading = nfWrapAngleRad(atan2(diffY, diffX));
}