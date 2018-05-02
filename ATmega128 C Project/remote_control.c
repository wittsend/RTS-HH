/*
* remote_control.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/04/2018 07:33:12
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* Looks for commands sent from the PC and has the robot act on them.
*
* More Info:
* Atmel ATmega128 Datasheet:http://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void rcExecuteCommand(RobotGlobalData *sys)
* uint8_t rcGetCommand(RemoteCommandData *pCommand)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"
#include <string.h>
#include <stdio.h>

#include "uart_driver.h"
#include "remote_control.h"

//////////////[Private Functions]///////////////////////////////////////////////////////////////////
/*
* Function:
* uint8_t rcGetCommand(RemoteCommandData *pCommand)
*
* If a command has been received on the UART, will extract the command and it's associated data
* and store it in the global remote control data structure.
*
* Inputs:
* RemoteCommandData *pCommand:
*	Pointer to the command data structure to store the command. (This is embedded into the "sys"
*	structure).
*
* Returns:
*	A 1 if a new command has been received on the UART
*
*/
static uint8_t rcGetCommand(RemoteCommandData *pCommand)
{
	uint8_t cmdLen = 0;							//Stores the address to the command string
	const char *pCmdData = uart0GetCmd(&cmdLen);//See if a new command has been received on the UART
	
	pCommand->newCmd = 0;
	
	//If the command length is greater than 0
	if(cmdLen)
	{
		//Create the necessary vars on the stack
		uint8_t readError = 0;//If the number of parameters returned does not match what is expected
		int n;					//The number of parameters found in a string
		float f1, f2;			//The values of the extracted parameters
		char str[50];			//A string that holds the command string

		//If we have a GO command
		if (strncmp(pCmdData, "GO", 2) == 0)
		{
			n = sscanf(pCmdData + 3, "%f,%f", &f1, &f2);
			if (n == 2)							//If two floats have been found.
			{
				pCommand->cmd = RC_CMD_GO;
				pCommand->x = f1;
				pCommand->y = f2;
				sprintf(str, "CMD GO %.3f,%.3f\r\n", f1, f2);
			} else {
				readError = 1;
			}
		}

		//If we have a STOP command
		if (strncmp(pCmdData, "STOP", 4) == 0)
		{
			pCommand->cmd = RC_CMD_STOP;
			pCommand->x = 0;
			pCommand->y = 0;
			sprintf(str, "CMD STOP\r\n");
		}
		
		//If there was an error in the command string.
		if(readError)
		{
			sprintf(str, "CMD ERROR\r\n");
			pCommand->newCmd = 0;
		} else {
			pCommand->newCmd = 1;
		}
		
		//Transmit message back to the PC.
		uart0OutputString(str);
	}
	
	return pCommand->newCmd;
}

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function: 
* void rcExecuteCommand(RobotGlobalData *sys)
*
* Checks for a new command from the PC, and if one has been sent, then set the main robot state
* accordingly to execute it.
*
* Inputs:
* RobotGlobalData *sys:
*	Pointer to the global data structure
*
* Returns:
* none
*
*/
void rcExecuteCommand(RobotGlobalData *sys)
{
	//Look for a new command on the UART
	uint8_t newCmd = rcGetCommand(&(sys->rc));
	
	if(newCmd)
	{
		//Set the main state to execute the received command.
		switch(sys->rc.cmd)
		{
			case RC_CMD_GO:
				sys->state.main = M_GO_TO_POS;
				break;
				
			case RC_CMD_STOP:
				sys->state.main = M_IDLE;
				break;
				
			case RC_CMD_NONE:
				break;	
		}
	}
	return;
}
