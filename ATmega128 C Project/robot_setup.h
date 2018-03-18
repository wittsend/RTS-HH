/*
* robot_setup.h
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/03/2018 9:00:28 PM
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
* void funcName(void)
*
*/

#ifndef ROBOT_SETUP_H_
#define ROBOT_SETUP_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include <avr/io.h>		//Hardware specific register defines
#include <stdint.h>		//Allows for specific integer variable sizes

//////////////[Public Defines]//////////////////////////////////////////////////////////////////////
typedef enum MainStates
{
	MAIN_IDLE,
	MAIN_GO_TO_POS
} MainStates;

typedef enum GoToPosStates
{
	GTP_TURN,
	GTP_DRIVE;
} GoToPosStates;

//A structure that holds state machine states for the system
typedef struct StateMachines
{
	MainStates main,
	GoToPosStates gtp;
} StateMachines;

//A structure that holds information about the position and orientation of the robot.
typedef struct PositionData
{
	float x;				//X position of the robot (m)
	float y;				//Y position of the robot (m)
	float heading;			//Heading of the robot (Radians?)
	int8_t leftPulses;		//Number of pulses from left wheel
	int8_t rightPulses;		//Number of pulses from right wheel
	int leftTotal;			//Total left pulses
	int rightTotal;			//Total right pulses
	//uint8_t pollEnabled;	//Whether or not to poll the navigation data
	//uint8_t pollInterval;	//The interval at which to poll
} PositionData;

//The root structure of the global data tree. 
typedef struct RobotGlobalData
{
	StateMachines state,
	PositionData pos;
	uint32_t timeStamp;
} RobotGlobalData;

//////////////[External Global Variables]///////////////////////////////////////////////////////////
extern RobotGlobalData sys;

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
void robotSetup(void);


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
*/
int32_t capToRangeInt(int32_t valueToCap, int32_t minimumVal, int32_t maximumVal);

float capToRangeFlt(float valueToCap, float minimumVal, float maximumVal);

#endif /* ROBOT_SETUP_H_ */