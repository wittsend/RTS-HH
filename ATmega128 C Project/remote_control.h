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


//////////////[Functions]///////////////////////////////////////////////////////////////////////////
void rcExecuteCommand(RobotGlobalData *sys);

uint8_t rcGetCommand(RemoteCommandData *pCommand);



#endif /* REMOTE_CONTROL_H_ */