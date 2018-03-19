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
#include "motor_driver.h"
#include "navigation.h"
#include <math.h>			//M_PI
#include <stdlib.h>			//abs()

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////


//////////////[Private Global Variables]///////////////////////////////////////////////////////////
extern RobotGlobalData sys;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
float pidRotateToHeading(float heading, RobotGlobalData *sys);

int main(void)
{
	robotSetup(); // initialise ATmega128
	
	uint32_t nextPIDUpdate = 100;
	
	while (1) // loop forever
	{
		switch(sys.state.main)
		{
			case M_IDLE:
				//Do nothing state
				motorStop();
				break;
				
			case M_GO_TO_POS:
				if(sys.timeStamp >= nextPIDUpdate)
				{
					nextPIDUpdate = sys.timeStamp + 100;
					if(!pidRotateToHeading(M_PI_2, &sys)) sys.state.main = M_IDLE;
				}
				
				//moveRobot(-256, 1023);
				//sys.state.main = M_IDLE;
				break;
		}
		
		nfUpdateNavigationData(&sys);
	}
}





