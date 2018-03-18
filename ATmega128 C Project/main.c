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
* int main(void)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"
#include "navigation.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////

//////////////[Private Global Variables]///////////////////////////////////////////////////////////
extern RobotGlobalData sys;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
int main(void)
{
	robotSetup(); // initialise ATmega128
	while (1) // loop forever
	{
		switch(sys.state.main)
		{
			case MAIN_IDLE:
				//Do nothing state
				break;
				
			case MAIN_GO_TO_POS:
				break;
		}
		
		nfUpdateNavigationData(&sys);
	}
}
