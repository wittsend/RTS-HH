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
typedef struct RobotGlobalData
{
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