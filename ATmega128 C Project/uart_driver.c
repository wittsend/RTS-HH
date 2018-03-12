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
* void uart0Init(void);
* void OutputString(char* str);
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include <avr/io.h>
#include "uart_driver.h"

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
#define UART0_RXCIE		0x00		//Receive complete interrupt enable
#define UART0_TXCIE		0x00		//Transmit complete interrupt enable
#define UART0_UDRIE		UART_USE_INTS//Data register empty interrupt enable
#define UART0_RXEN		0x01		//Receive enable setting
#define UART0_TXEN		0x01		//Transmit enable setting
#define UART0_UCSZ		0x03		//Character size (8-Bit)
#define UART0_UMSEL		0x00		//Mode select (Async)
#define UART0_UPM		0x00		//Parity mode (None)
#define UART0_USBS		0x00		//Stop bit select (1-bit)
#define UART0_UCPOL		0x00		//Clock Polarity (NA)
// 9600 bps = 51, 19200 bps = 25, 38400 bps = 12
#define UART0_UBRR		12			//Baud rate setting (38400bps)

//////////////[Private Global Variables]////////////////////////////////////////////////////////////
#if UART_USE_INTS == 1
// serial port 0 transmit buffer
volatile char buffer[UART_BUF_MAX];			//buffer (queue) data
volatile unsigned int head, tail, count;	//buffer (queue) indexes
#endif

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void uart0Init(void)
*
* Initialises the UART interface for communication with a PC
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void uart0Init(void)
{
	//Load the UART registers with the values defined above
	UCSR0B 
	=	((UART0_RXCIE<<7)&0x01)
	|	((UART0_TXCIE<<6)&0x01)
	|	((UART0_UDRIE<<5)&0x01)
	|	((UART0_RXEN<<4)&0x01)
	|	((UART0_TXEN<<3)&0x01)
	|	((UART0_UCSZ<<2)&0x04);
	
	UCSR0C
	=	((UART0_UMSEL<<6)&0x01)
	|	((UART0_UPM<<4)&0x03)
	|	((UART0_USBS<<3)&0x01)
	|	((UART0_UCSZ<<1)&0x03)
	|	((UART0_UCPOL<<0)&0x01);
	
	UBRR0H = (UART0_UBRR & 0xFF00); 
	UBRR0L = (UART0_UBRR & 0x00FF);
	
	#if UART_USE_INTS == 1
	//Initialise serial output buffer
	head = 0;
	tail = 0;
	count = 0;
	#endif
}

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