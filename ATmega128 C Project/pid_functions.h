/*
* pid_functions.h
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 19/03/2018 6:50:19 PM
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* Header file for PID functions
*
* More Info:
* Atmel ATmega128 Datasheet:http://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void funcName(void)
*
*/

#ifndef PID_FUNCTIONS_H_
#define PID_FUNCTIONS_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////

//////////////[Public Defines]//////////////////////////////////////////////////////////////////////

//////////////[External Global Variables]///////////////////////////////////////////////////////////

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
float pidRotateToHeading(float heading, RobotGlobalData *sys);

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
float pidDriveToHeading(float speed, float heading, RobotGlobalData *sys);

float pidGoToPosition(float speed, float x, float y, RobotGlobalData *sys);
#endif /* PID_FUNCTIONS_H_ */