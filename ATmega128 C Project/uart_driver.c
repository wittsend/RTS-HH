/*
* uart_driver.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/03/2018 4:31:29 PM
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
#include "uart_driver.h"

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////


//////////////[Private Global Variables]////////////////////////////////////////////////////////////
#if UART_USE_INTS == 1
// serial port 0 transmit buffer
volatile char buffer[UART_BUF_MAX];			//buffer (queue) data
volatile unsigned int head, tail, count;	//buffer (queue) indexes
#endif

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
#if UART_USE_INTS == 1

/*
* Function:
* void OutputString(char* str)
*
* Allows the transmission of strings via UART WITH the use of interrupts
*
* Inputs:
* char* str:
*	Pointer to a string of characters to transmit out the UART
*
* Returns:
* none
*
*/
void OutputString(char* str)
{
	int length = strlen(str);
	UCSR0B &= ~(1 << UDRIE0); // disable serial port 0 UDRE interrupt
	// check for too many chars
	if (count + length >= UART_BUF_MAX)
	{
		UCSR0B |= (1 << UDRIE0); // enable serial port 0 UDRE interrupt
		return;
	}
	// write the characters into the buffer
	for (int n = 0; n < length; n++)
	{
		buffer[tail] = str[n];
		tail++;
		if (tail >= UART_BUF_MAX)
		{
			tail = 0;
		}
	}
	count += length;
	UCSR0B |= (1 << UDRIE0); // enable serial port 0 UDRE interrupt
}

/*
* Function:
* ISR(USART0_UDRE_vect)
*
* Interrupt service routine for UART0 data ready register
*
* Inputs:
* none
*
* Returns:
* none
*
*/
ISR(USART0_UDRE_vect)
{
	if (count > 0) // if there are more characters
	{
		UDR0 = buffer[head]; // transmit the next character
		// adjust the buffer variables
		head++;
		if (head > UART_BUF_MAX)
		{
			head = 0;
		}
		count--;
	}
	if (count == 0) // if there are no more characters
	{
		UCSR0B &= ~(1 << UDRIE0); // then disable serial port 0 UDRE interrupt
	}
}

#else	//If not using interrupts

/*
* Function:
* void OutputString(char* str)
*
* Allows the transmission of strings via UART WITHOUT the use of interrupts
*
* Inputs:
* char* str:
*	Pointer to a string of characters to transmit out the UART
*
* Returns:
* none
*
*/
void OutputString(char* str)
{
	int length = strlen(str);
	// for each character in the string
	for (int n = 0; n < length; n++)
	{
		// wait while the serial port is busy
		while (!(UCSR0A & (1 << UDRE0)));
		// transmit the character
		UDR0 = str[n];
	}
}
#endif