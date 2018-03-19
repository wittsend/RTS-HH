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
#define MF_HEADING_ERR	0.008727
#define RTH_KP			250.
#define RTH_KD			245

//////////////[Private Global Variables]///////////////////////////////////////////////////////////
extern RobotGlobalData sys;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
float mfRotateToHeading(float heading, RobotGlobalData *sys);

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
				//moveRobot(1023., -1023.);
				motorStop();
				break;
				
			case M_GO_TO_POS:
				if(sys.timeStamp > nextPIDUpdate)
				{
					nextPIDUpdate = sys.timeStamp + 100;
					if(mfRotateToHeading(M_PI_2, &sys) == 0.) sys.state.main = M_IDLE;
				}
				
				//moveRobot(-256, 1023);
				//sys.state.main = M_IDLE;
				break;
		}
		
		nfUpdateNavigationData(&sys);
	}
}


float mfRotateToHeading(float heading, RobotGlobalData *sys)
{
	static float pErr;				//Proportional (signed) error
	static float pErrOld;			//Old proportional Error
	static uint32_t timeOld = 0;	//Time of the last update
	
	float dErr;
	float dTime;
	float motorSpeed;				//Stores motorSpeed calculated by PID sum
	
	//Make sure heading is in range (-180 to 180)
	heading = nfWrapAngleRad(heading);
	
	//Calculate proportional error values
	pErr = heading - sys->pos.heading;				//Signed Error
	
	dErr = pErr - pErrOld;
	
	pErrOld = pErr;
	
	dTime = sys->timeStamp - dTime;
	timeOld = sys->timeStamp;
	
	//Force the P controller to always take the shortest path to the destination.
	//For example if the robot was currently facing at -120 degrees and the target was 130 degrees,
	//instead of going right around from -120 to 130, it will go to -180 and down to 130.
	if(pErr > M_PI)
		pErr -= (2*M_PI);
	if(pErr < (-1*M_PI))
		pErr += (2*M_PI);
	
	//If motorSpeed ends up being out of range, then dial it back
	motorSpeed = (RTH_KP*pErr + RTH_KD*dErr);
	motorSpeed = capToRangeFlt(motorSpeed, -1023, 1023);

	//If error is less than 0.5 deg and delta yaw is less than 0.5 degrees per second then we can
	//stop
	if((fabs(pErr) < MF_HEADING_ERR))
	{
		motorStop();
		pErr = 0;			//Clear the static vars so they don't interfere next time we call this
							//function
		pErrOld	= 0;
		return 0;
	} else {
		moveRobot(motorSpeed, 1023.);
		return pErr;	//If not, return pErr
	}
}


