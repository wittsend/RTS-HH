/*
* navigation.h
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 15/03/2018 10:37:54 AM
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* Header file for the navigation functions module.
*
* More Info:
* Atmel ATmega128 Datasheet:http://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void nfUpdateNavigationData(RobotGlobalData *sys);
* float nfWrapAngle(float angleDeg);
* float nfWrapAngleRad(float angleRad)
* float nfDeg2Rad(float deg)
* float nfRad2Deg(float rad)
* void nfGetDist(float x1, float y1, float x2, float y2, float *heading, float *distance)
*
*/

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////

//////////////[Public Defines]//////////////////////////////////////////////////////////////////////

//////////////[External Global Variables]///////////////////////////////////////////////////////////

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
void nfUpdateNavigationData(RobotGlobalData *sys);

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
float nfWrapAngle(float angleDeg);

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
float nfWrapAngleRad(float angleRad);

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
float nfDeg2Rad(float deg);

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
float nfRad2Deg(float rad);

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
void nfGetDist(float x1, float y1, float x2, float y2, float *heading, float *distance);

#endif /* NAVIGATION_H_ */