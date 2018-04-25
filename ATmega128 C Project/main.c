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
#include "remote_control.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////

//////////////[Private Global Variables]////////////////////////////////////////////////////////////
extern RobotGlobalData sys;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
int main(void)
{
	robotSetup(); // initialise ATmega128
	
	while (1) // loop forever
	{
		switch(sys.state.main)
		{
			//Do nothing state
			case M_IDLE:
				motorStop();
				break;
			
			//This state will check to see if the appropriate (100ms) has elapsed since it as last
			//run. If is has, the state will perform another PID controlled motion function. In this
			//case the robot will turn and drive to a hard-coded position relative to the robot's
			//starting position.
			case M_GO_TO_POS:
				if(sys.timeStamp >= sys.pidNextPIDUpdate)
				{
					sys.pidNextPIDUpdate = sys.timeStamp + sys.pidUpdateInterval;
					if(!pidGoToPosition(1023, sys.rc.x, sys.rc.y, &sys)) sys.state.main = M_IDLE;
				}
		}
		
		//Retrieve encoder data from the wheels and recalculate the robot's position
		nfUpdateNavigationData(&sys);
		
		//Check for command from the PC, and if a new one has been received, then execute:
		rcExecuteCommand(&sys);
	}
}

