/*
* pid_functions.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 19/03/2018 6:50:31 PM
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* Provides functions for controlling the robot with PID control
*
* More Info:
* Atmel ATmega128 Datasheet:http://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* float pidRotateToHeading(float heading, RobotGlobalData *sys);
* float pidDriveToHeading(float speed, float heading, RobotGlobalData *sys);
* float pidGoToPosition(float speed, float x, float y, RobotGlobalData *sys);
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"
#include "motor_driver.h"
#include "navigation.h"
#include <math.h>
#include "uart_driver.h"			//Debug strings
#include "pid_functions.h"

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////

//#define MF_HEADING_ERR	0.008727	//Error angle allowed before the maneuver is deemed complete
#define MF_HEADING_ERR	0.1745		//Error angle allowed before the maneuver is deemed complete
#define MF_DIST_ERR		0.05		//Error distance threshold

//Rotate to heading PID function constants
#define RTH_KP			300			//Proportional error constant
#define RTH_KI			400			//Integral error constant
#define RTH_KD			400			//Derivative error constant
#define RTH_IERR_MAX	0.5236		//Maximum value of the integral error

//Drive to heading PID function constants
#define DTH_KP			300			//Proportional error constant
#define DTH_KI			400			//Integral error constant
#define DTH_KD			400			//Derivative error constant
#define DTH_IERR_MAX	0.5236		//Maximum value of the integral error

//////////////[Private Global Variables]////////////////////////////////////////////////////////////

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function: 
* pidRotateToHeading(float heading, RobotGlobalData *sys)
*
* Allows the robot to rotate on the spot to face the desired heading.
*
* Inputs:
* float heading:
*	The target heading in radians
* RobotGlobalData *sys
*	A pointer to the robot global data structure
*
* Returns:
* Returns the proportional error while the maneuvre is in progress, otherwise returns a 0 when
* complete
*
*/
float pidRotateToHeading(float heading, RobotGlobalData *sys)
{
	//Set up variables
	float pErr;						//Proportional (signed) error
	static float pErrOld = 0;		//Old proportional Error
	static float iErr = 0;			//Integral Error_
	float dErr;						//Derivative error
	float motorSpeed;				//Stores motorSpeed calculated by PID sum

	
	//Make sure heading is in range (-pi to pi)
	heading = nfWrapAngleRad(heading);
	pErr = heading - sys->pos.heading;				//Signed Error

	//Force the P controller to always take the shortest path to the destination.
	//For example if the robot was currently facing at -120 degrees and the target was 130 degrees,
	//instead of going right around from -120 to 130, it will go to -180 and down to 130.
	if(pErr > M_PI)
		pErr -= (2*M_PI);
	if(pErr < (-1*M_PI))
		pErr += (2*M_PI);
	
	//Calculate proportional error values		
	iErr += pErr;									//Integral error
	dErr = pErr - pErrOld;							//Derivative error
	
	//Store pErr for calculating dErr next time around.
	pErrOld = pErr;
	
	//Limit the size of iErr
	iErr = capToRangeFlt(iErr, -RTH_IERR_MAX, RTH_IERR_MAX);
	
	//If motorSpeed ends up being out of range, then dial it back
	motorSpeed = capToRangeFlt((RTH_KP*pErr + RTH_KI*iErr + RTH_KD*dErr), -1023, 1023);
	moveRobot(motorSpeed, 1023.);
	//If error value is less than the value set in MF_HEADING_ERR then exit with a 0 (indicating
	//that the maneuver is complete
	
	if((fabs(pErr) < MF_HEADING_ERR))
	{
		motorStop();
		pErrOld = 0;			//Clear the static vars so they don't interfere next time we call this
								//function
		iErr = 0;
		return 0;
	} else {
		//Keeping the turn ratio at 1023 makes the robot rotate on the spot.
		return pErr;	//If not, return pErr
	}
}

/*
* Function:
* pidDriveToHeading(float heading, RobotGlobalData *sys)
*
* Allows the robot to drive a path at a given speed towards the given heading.
*
* Inputs:
* float speed:
*	A PWM speed value between -1023 and 1023 that the robot will drive at
* float heading:
*	The target heading in radians
* RobotGlobalData *sys
*	A pointer to the robot global data structure
*
* Returns:
* Returns the proportional error while the maneuvre is in progress, otherwise returns a 0 when
* the robot is on track.
*
*/
float pidDriveToHeading(float speed, float heading, RobotGlobalData *sys)
{
	char debugString[100];
	//Set up variables
	float pErr;						//Proportional (signed) error
	static float pErrOld = 0;		//Old proportional Error
	static float iErr = 0;			//Integral Error
	float dErr;						//Derivative error
	float turnRatio;				//Stores turnRatio calculated by PID sum
	
	//Make sure heading is in range (-pi to pi)
	heading = nfWrapAngleRad(heading);
	
	//Calculate proportional error values
	pErr = heading - sys->pos.heading;				//Signed Error

	//Force the P controller to always take the shortest path to the destination.
	//For example if the robot was currently facing at -120 degrees and the target was 130 degrees,
	//instead of going right around from -120 to 130, it will go to -180 and down to 130.
	if(pErr > M_PI)
		pErr -= (2*M_PI);
	if(pErr < (-1*M_PI))
		pErr += (2*M_PI);

	iErr += pErr;									//Integral error
	dErr = pErr - pErrOld;							//Derivative error
	

	//Store pErr for calculating dErr next time around.
	pErrOld = pErr;
	
	//Limit the size of iErr
	iErr = capToRangeFlt(iErr, -DTH_IERR_MAX, DTH_IERR_MAX);
	
	//If turnRatio ends up being out of range, then dial it back
	turnRatio = capToRangeFlt((DTH_KP*pErr + DTH_KI*iErr + DTH_KD*dErr), -1023., 1023.);

	moveRobot(speed, turnRatio);
	return pErr;	//If not, return pErr
}

/*
* Function:
* pidGoToPosition(float speed, float x, float y, RobotGlobalData *sys)
*
* Will drive the robot to an x y position in 2D space
*
* Inputs:
* float speed:
*	A PWM speed value between 0 and 1023 that the robot will drive at
* float x:
*	The target x position in metres
* float y:
*	The target y position in metres
* RobotGlobalData *sys
*	A pointer to the robot global data structure
*
* Returns:
* The state that this function is in. If returns a 0, then the maneuvre is complete.
*
*/
float pidGoToPosition(float speed, float x, float y, RobotGlobalData *sys)
{
	char debugString[100];
	float distance;
	float errorHeading = 0;
	static float heading;
	
	switch(sys->state.gtp)
	{
		//Calculate the distance and heading to the target from the current position.
		//If rotation is necessary, then switch to the turn state. Else, if the distance is greater
		//than the threshold, drive forward to reduce it, otherwise just exit the function with 0.
		case GTP_START:
			//Calculate the heading and distance to the target.
			nfGetDist(sys->pos.x, sys->pos.y, x, y, &heading, &distance);
			
			sprintf(debugString, "GTP_START, Dist:%1.2fm, Heading:%1.2f deg\r\n", distance, nfRad2Deg(heading));
			uartOutputString(debugString);
			
			//If we aren't close enough to the target
			if(distance > MF_DIST_ERR)
			{
				sprintf(debugString, "Performing Turn...\r\n");
				uartOutputString(debugString);
				sys->state.gtp = GTP_TURN;	//Start turning
			} else {
				sprintf(debugString, "Close enough already.\r\n");
				uartOutputString(debugString);
				return sys->state.gtp;		//If we are close enough, do no more
			}
			break;
		
		//If the robot is not facing the target, then turn to face it. Once done, move to the DRIVE
		//state.
		case GTP_TURN:
			if(!pidRotateToHeading(heading, sys))
				sprintf(debugString, "Turn complete, driving forward...\r\n");
				uartOutputString(debugString);
				sys->state.gtp = GTP_DRIVE;
			break;
		
		//Drive towards the target, keeping an eye on the distance.	
		case GTP_DRIVE:
			//Calculate the heading and distance to the target.
			nfGetDist(sys->pos.x, sys->pos.y, x, y, &heading, &distance);
			errorHeading = pidDriveToHeading(distance*2048, heading, sys);
			
			sprintf(debugString, "Distance to go: %1.2fm Err Heading: %1.2fdeg\r\n", distance, nfRad2Deg(errorHeading));
			uartOutputString(debugString);
			sprintf(debugString, "Robot Heading: %1.2fdeg\r\n", nfRad2Deg(sys->pos.heading));
			uartOutputString(debugString);
			
			if(distance < MF_DIST_ERR)
			{
				sprintf(debugString, "Destination reached.\r\n");
				uartOutputString(debugString);
				motorStop();
				sys->state.gtp = GTP_FINISHED;
			}
			break;
			
		case GTP_FINISHED:
			distance = 0;
			heading = 0;
			sys->state.gtp = GTP_START;
			break;
	}
	return sys->state.gtp;
}