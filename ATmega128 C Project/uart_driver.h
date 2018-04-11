/*
* uart_driver.h
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/03/2018 4:31:38 PM
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* Header file for the UART communication driver
*
* More Info:
* Atmel ATmega128 Datasheet:http://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void uart0Init(void);
* uint8_t uart0OutputString(char* str);
*
*/

#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include <stdio.h>			//sprintf will most likely be needed for outputting strings

//////////////[Public Defines]//////////////////////////////////////////////////////////////////////

//////////////[External Global Variables]///////////////////////////////////////////////////////////
extern volatile unsigned char uartRxCmdRcv;			//Command received flag

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
void uart0Init(void);
char *uart0GetCmd(uint8_t *cmdLen);
uint8_t uart0OutputString(char* str);

#endif /* UART_DRIVER_H_ */