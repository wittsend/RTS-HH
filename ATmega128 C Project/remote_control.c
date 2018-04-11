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
void rcExecuteCommand(void)
{
	RemoteCommandData cmd;
	uint8_t newCmd = rcGetCommand(&cmd)
	return;
}

uint8_t rcGetCommand(RemoteCommandData *pCommand)
{
	uint16_t pCmdData;
	uint8_t newCmd = uart0GetCmd(pCmdData);
	
	//if(newCmd)
		return pCmdData;
	//else
	//	return 0;
}