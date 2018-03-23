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
#include "pid_functions.h"
#include <math.h>			//M_PI
#include <stdlib.h>			//abs()

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
#define PID_UPDATE_RATE	100

//////////////[Private Global Variables]///////////////////////////////////////////////////////////
extern RobotGlobalData sys;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
float pidRotateToHeading(float heading, RobotGlobalData *sys);

int main(void)
{
	static uint32_t nextPIDUpdate = PID_UPDATE_RATE;
	robotSetup(); // initialise ATmega128
	
	
	//char debugString()
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
					nextPIDUpdate = sys.timeStamp + PID_UPDATE_RATE;
					//if(!pidGoToPosition(1023, 0.25, 0.25, &sys)) sys.state.main = M_IDLE;
					//if(!pidRotateToHeading(nfDeg2Rad(180), &sys)) sys.state.main = M_IDLE;
					//pidDriveToHeading(1023., nfDeg2Rad(0), &sys);
					if(sys.timeStamp > 5000) sys.state.main = M_IDLE;	//Timeout after 10 seconds
					
					//pidDriveToHeading(1023, 45, &sys);
					moveRobot(1023, -512);
				
					//sys.state.main = M_IDLE;
					//motorStop();
					break;
				}
		}
		
		nfUpdateNavigationData(&sys);
	}
}





