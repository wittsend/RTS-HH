/*
* remote_control.h
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/04/2018 07:32:57
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

#ifndef REMOTE_CONTROL_H_
#define REMOTE_CONTROL_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////

//////////////[Public Defines]//////////////////////////////////////////////////////////////////////

//////////////[External Global Variables]///////////////////////////////////////////////////////////
//Valid commands enumeration
typedef enum RemoteControlCommands
{
	RC_CMD_GO,
	RC_CMD_STOP
} RemoteControlCommands;

//The received command data structure
typedef struct RemoteCommandData
{
	RemoteControlCommands cmd;	//Type of command
	uint8_t newCmd;				//Whether this is a new command or not
	float x;					//X parameter of command
	float y;					//Y parameter of command
} RemoteCommandData;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////

uint8_t rcGetCommand(RemoteCommandData *pCommand);



#endif /* REMOTE_CONTROL_H_ */