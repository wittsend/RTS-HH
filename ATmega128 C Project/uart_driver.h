/*
* uart_driver.h
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/03/2018 4:31:38 PM
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

#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////

//////////////[Public Defines]//////////////////////////////////////////////////////////////////////
#define UART_USE_INTS				1		//Allow the UART to use interrupts for transmission
#define UART_BUF_MAX				1000	//serial output buffer size

//////////////[External Global Variables]///////////////////////////////////////////////////////////

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
void OutputString(char* str);

#endif /* UART_DRIVER_H_ */