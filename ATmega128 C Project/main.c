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

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////

//////////////[Private Global Variables]////////////////////////////////////////////////////////////
extern RobotGlobalData sys;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
float pidRotateToHeading(float heading, RobotGlobalData *sys);

int main(void)
{
	robotSetup(); // initialise ATmega128
	
	uint8_t moveState = 1;	//This is the state of the state machine within M_GO_TO_POS that allows
							//the robot to drive in a square.
	
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
					switch(moveState)
					{						
						case 1:
							if(!pidGoToPosition(1023, 0.6, 0, &sys)) moveState = 2;
							break;
						
						case 2:
							if(!pidGoToPosition(1023, 0	, 0, &sys)) moveState = 3;
							break;

						case 3:
							//if(!pidGoToPosition(1023, 0, 0.6, &sys)) moveState = 4;
							sys.state.main = M_IDLE;
							break;

						case 4:
							if(!pidGoToPosition(1023, 0, 0, &sys)) sys.state.main = M_IDLE;
							break;
					}
					break;
				}
		}
		
		//Retrieve encoder data from the wheels and recalculate the robot's position
		nfUpdateNavigationData(&sys);
	}
}

