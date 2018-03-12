/*
 * AssignmentIntroIncomplete.c
 *
 * Created: 27/02/2018 2:56:57 p.m.
 * Author : jcollins
 * Description:
 *     Robot counts and displays the wheel pulse counts.
 *     Does not use serial interrupts.
 *     Written for Atmel Studio.
 */

/*******************************************************************************
* INCLUDES
*******************************************************************************/
#define F_CPU 8000000 // crystal frequency for delay.h
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>

/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
#define LOOP_DELAY 200 // milliseconds

/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void Setup(void); // ATmega128 initialisation for this program
void Operate(void); // operation repeated in the main loop
void OutputString(char* str); // transmit a string out serial port 0 to PC

/*******************************************************************************
* GLOBAL VARIABLES
*******************************************************************************/
volatile int leftPulseCount, rightPulseCount; // wheel pulse counts

/*******************************************************************************
* MAIN FUNCTION
*******************************************************************************/
int main (void)
{
    // initialise ATmega128
	Setup();
	// display starting message
	OutputString("INTRODUCTORY ASSIGNMENT\r\n");
	OutputString("STARTING MOTORS\r\n");
	// both motors forward: make PORTA bits 6 and 7 both = 0
	// ADD CODE
    // both motors on: make DDRB bits 5 and 6 both = 0
	// ADD CODE
	// turn the motors on
	while (1) // loop forever
	{
        // wait for a constant time
        _delay_ms(LOOP_DELAY);
        // perform the program operation
		Operate();
	}
}

/*******************************************************************************
* OTHER FUNCTIONS
*******************************************************************************/
// initialise ATmega128
void Setup(void)
{
    // SET UP DIGITAL INPUT/OUTPUT
    // enable motor direction outputs: make DDRA bits 6 and 7 both = 1
    DDRA |=	0xC0;
    // both motors forward: make PORTA bits 6 and 7 both = 0
	//PORTA &= ~(0xC0);	//FWD
	PORTA |= (0xC0);	//RWD
    // enable motor outputs: make DDRB bits 5 and 6 both = 1
    DDRB |=	0x60;
    // both motors off: make PORTB bits 5 and 6 both = 0
	//PORTB &= ~(0x60);	//OFF
	PORTB |= (0x60);	//ON
    // enable the wheel pulse generator electronics:
    // make DDRC bit 3 = 1 and PORTC bit 3 = 1
	DDRC |= 0x08;
	PORTC |= 0x08;
	// SET UP THE SERIAL PORT
	UCSR0A = 0; // do not need to set any bits in this register
	UCSR0B = (1 << RXEN) | (1 << TXEN); // enable receive and transmit
	UCSR0C = (1 << UCSZ1) | (1 << UCSZ0); // 8 data bits, no parity, 1 stop
	UBRR0H = 0; // 9600 bps = 51, 19200 bps = 25, 38400 bps = 12
	UBRR0L = 12;
	// wait 500ms for the electronic hardware to settle
	_delay_ms(500);
	// enable external interrupts for the wheel pulses (INT0, INT1)
	// use rising edges of wheel pulses
	EICRA = (1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00);
	EIFR = (1 << INTF1) | (1 << INTF0); // clear interrupt flags
	EIMSK = (1 << INT1) | (1 << INT0); // enable INT0, INT1
    // enable interrupts last
	sei();
}

// program operation called from the main loop
void Operate(void)
{
    int leftCount, rightCount; // copies of the latest pulse counts
	static int leftTotal = 0, rightTotal = 0; // total pulses from each wheel
    char str[20]; // string for the message to the PC
    // get the latest pulse counts
    cli(); // disable interrupts, temporarily stop counting
    leftCount = leftPulseCount; // copy the left pulse count
    leftPulseCount = 0; // set the left counter to zero
    rightCount = rightPulseCount; // copy the right pulse count
    rightPulseCount = 0; // set the right counter to zero
    sei(); // enable interrupts, start counting again
	// add the latest pulse counts to the totals
	leftTotal += leftCount;
	rightTotal += rightCount;
	// construct the message for the PC
	sprintf(str, "%3d,%3d,%6d,%6d\r\n", leftCount, rightCount, leftTotal, rightTotal);
    // send the counts to the PC to display
	OutputString(str);
}

// transmit message to PC using the serial port
void OutputString(char* message)
{
    // get the message length
	int length = strlen(message);
	// for each character in the message string
	for (int n = 0; n < length; n++)
	{
		// wait while the serial port is busy
		while (!(UCSR0A & (1 << UDRE0)));
		// transmit the character
		UDR0 = message[n];
	}
}

/*******************************************************************************
* INTERRUPT FUNCTIONS
*******************************************************************************/
// left wheel pulse counter
ISR(INT0_vect)
{
	// increment the left pulse counter
	if(PORTA & (0x40))
		leftPulseCount--;
	else
		leftPulseCount++;
}

// right wheel pulse counter
ISR(INT1_vect)
{
	// increment the right pulse counter
	if(PORTA & (0x80))
		rightPulseCount--;
	else
		rightPulseCount++;

}