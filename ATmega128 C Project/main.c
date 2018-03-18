/*
* main.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/03/2018 4:20:03 PM
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* The main file for Assignment 1
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
#include <stdio.h>		//sprintf



#include "navigation.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////



//////////////[Private Global Variables]///////////////////////////////////////////////////////////
extern RobotGlobalData sys;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////

int main (void)
{
	//char str[40]; // serial output string

	robotSetup(); // initialise ATmega128

	//// display started on serial port
	//OutputString("\r\n**************\r\n");
	//OutputString("ROBOT POSITION\r\n");
	//// display the wheel pwm settings
	////sprintf(str, "SPEED L=%d R=%d\r\n", LEFT_SPEED, RIGHT_SPEED);
	//OutputString(str);

	while (1) // loop forever
	{
		
		//Operate(&pos); // calculate position from wheel counters
		nfUpdateNavigationData(&sys);
	}
}
