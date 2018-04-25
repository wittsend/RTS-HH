/*
* remote_control.h
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/04/2018 07:32:57
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* Header for the remote control module.
*
* More Info:
* Atmel ATmega128 Datasheet:http://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void rcExecuteCommand(RobotGlobalData *sys)
*
*/

#ifndef REMOTE_CONTROL_H_
#define REMOTE_CONTROL_H_

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
void rcExecuteCommand(RobotGlobalData *sys);

#endif /* REMOTE_CONTROL_H_ */