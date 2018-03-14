/*
* navigation.h
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 15/03/2018 10:37:54 AM
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

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////

//////////////[Public Defines]//////////////////////////////////////////////////////////////////////

//////////////[External Global Variables]///////////////////////////////////////////////////////////

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
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
void updateNavigationData(RobotGlobalData *sys);

#endif /* NAVIGATION_H_ */