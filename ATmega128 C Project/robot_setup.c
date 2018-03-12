/*
* robot_setup.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/03/2018 9:00:13 PM
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

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"

#include <avr/interrupt.h>

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////

//////////////[Private Global Variables]////////////////////////////////////////////////////////////

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function: 
* [function declaration]
*
* [brief purpose of function]
*
* Inputs:
* [input arguments and any relevant explanation]
*
* Returns:
* [return values and any relevant explanation]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void setup(void)
{
	sei();
}

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
* Implementation:
* If valueToCap is greater than maximumVal, then make it equal maximumValue, otherwise if
* valueToCap is less than minimumValue then make it equal minimum value.
*
*/
int32_t capToRangeInt(int32_t valueToCap, int32_t minimumVal, int32_t maximumVal)
{
	if(valueToCap > maximumVal)
		valueToCap = maximumVal;
	if(valueToCap < minimumVal)
		valueToCap = minimumVal;
	return valueToCap;
}

float capToRangeFlt(float valueToCap, float minimumVal, float maximumVal)
{
	if(valueToCap > maximumVal)
		valueToCap = maximumVal;
	if(valueToCap < minimumVal)
		valueToCap = minimumVal;
	return valueToCap;
}