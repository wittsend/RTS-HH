/*
* uart_driver.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/03/2018 4:31:29 PM
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* Initialises the serial communication hardware and provides functions for using it.
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

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"
#include <avr/interrupt.h>
#include <string.h>
#include "uart_driver.h"

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
#define UART_USE_INTS	1			//Allow the UART to use interrupts for transmission
#define UART_TXBUF_MAX	1024		//serial output buffer size
#define UART_RXBUF_MAX	20			//Serial input buffer size

#define UART0_RXCIE		0x01		//Receive complete interrupt enable
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

#define UDREIntDisable	UCSR0B &= ~(1 << UDRIE0)
#define UDREIntEnable	UCSR0B |= (1 << UDRIE0)
#define RXIntDisable	UCSR0B &= ~(1 << RXCIE0)
#define RXIntEnable		UCSR0B |= (1 << RXCIE0)

//////////////[Private Global Variables]////////////////////////////////////////////////////////////
#if UART_USE_INTS == 1
// serial port 0 transmit buffer
volatile char txBuffer[UART_TXBUF_MAX];				//buffer (queue) data
volatile unsigned int txHead, txTail, txCount;		//buffer (queue) indexes
#endif
//Serial port 0 Receive buffer
volatile unsigned char uartRxCmdRcv = 0;			//Command received flag
char uartCommand[UART_RXBUF_MAX];					//last received command string

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
	=	((UART0_RXCIE&0x01)<<7)
	|	((UART0_TXCIE&0x01)<<6)
	|	((UART0_UDRIE&0x01)<<5)
	|	((UART0_RXEN&0x01)<<4)
	|	((UART0_TXEN&0x01)<<3)
	|	((UART0_UCSZ&0x04)<<2);
	
	UCSR0C
	=	((UART0_UMSEL&0x01)<<6)
	|	((UART0_UPM&0x03)<<4)
	|	((UART0_USBS&0x01)<<3)
	|	((UART0_UCSZ&0x03)<<1)
	|	((UART0_UCPOL&0x01)<<0);
	
	UBRR0H = ((UART0_UBRR & 0xFF00)>>8); 
	UBRR0L = (UART0_UBRR & 0x00FF);
	
	#if UART_USE_INTS == 1
	//Initialise serial output buffer
	txHead = 0;
	txTail = 0;
	txCount = 0;
	#endif
}

char *uart0GetCmd(uint8_t *cmdLen)
{
	if(uartRxCmdRcv)
	{
		*cmdLen = UART_RXBUF_MAX;	//Return the length of the receive buffer as a ref param
		uartRxCmdRcv = 0;			//Reset the command received flag, as this command will be read
		return uartCommand;			//Return the address to the receive buffer
	}
	*cmdLen = 0;
	return 0;
}

#if UART_USE_INTS == 1
/*
* Function:
* uint8_t uart0OutputString(char* str)
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
uint8_t uart0OutputString(char* str)
{
	int length = strlen(str);
	UDREIntDisable;				// disable serial port 0 UDRE interrupt
	
	// check for too many chars
	if (txCount + length >= UART_TXBUF_MAX)
	{
		UDREIntEnable;			//enable serial port 0 UDRE interrupt
		return 1;				//1 indicates error
	}
	
	// write the characters into the buffer
	for (int n = 0; n < length; n++)
	{
		txBuffer[txTail] = str[n];
		txTail++;
		if (txTail >= UART_TXBUF_MAX)
			txTail = 0;
	}
	
	txCount += length;
	UDREIntEnable;				// enable serial port 0 UDRE interrupt
	
	return 0;
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
	if (txCount > 0) // if there are more characters
	{
		UDR0 = txBuffer[txHead]; // transmit the next character
		// adjust the buffer variables
		txHead++;
		if (txHead > UART_TXBUF_MAX)
		{
			txHead = 0;
		}
		txCount--;
	}
	
	if (txCount == 0) // if there are no more characters
		UDREIntDisable; // then disable serial port 0 UDRE interrupt
}

/*
* Function:
* ISR(USART0_RX_vect)
*
* Interrupt service routine for UART0 Receive complete
*
* Inputs:
* none
*
* Returns:
* none
*
*/
ISR(USART0_RX_vect)
{
	char ch;
	static char rxBuffer[UART_RXBUF_MAX];	// receiving command
	static int recIndex = 0;				// number of command chars
	ch = UDR0;
	if (ch >= ' ' && ch <= '~' && recIndex < UART_RXBUF_MAX - 1)
	{
		rxBuffer[recIndex] = ch;
		recIndex++;
	}
	if (ch == '\r')
	{
		rxBuffer[recIndex] = 0;
		if (!uartRxCmdRcv)	//If the last command has not been processed, then do not load new cmd.
		{
			strcpy((char*)uartCommand, rxBuffer);
			uartRxCmdRcv = 1;
		}
		recIndex = 0;
	}
}

#else	//If not using interrupts

/*
* Function:
* uint8_t uart0OutputString(char* str)
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
uint8_t uart0OutputString(char* str)
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