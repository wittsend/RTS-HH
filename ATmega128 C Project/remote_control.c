/*
* remote_control.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/04/2018 07:33:12
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
#include <string.h>
#include <stdio.h>

#include "uart_driver.h"
#include "remote_control.h"

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
void rcExecuteCommand(RobotGlobalData *sys)
{
	uint8_t newCmd = rcGetCommand(&(sys->rc));
	return;
}


uint8_t rcGetCommand(RemoteCommandData *pCommand)
{
	uint8_t cmdLen = 0;							//Stores the address to the command string
	const char *pCmdData = uart0GetCmd(&cmdLen);	//See if a new command has been received on the UART
	
	pCommand->newCmd = 0;
	
	if(cmdLen)
	{
		uint8_t readError = 0;
		int n;
		float f1, f2;
		char str[50];

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
		
		if(readError) 
		{
			sprintf(str, "CMD ERROR\r\n");
			pCommand->newCmd = 0;
		} else {
			pCommand->newCmd = 1;
		}
		
		uart0OutputString(str);
	}
	
	return pCommand->newCmd;
}