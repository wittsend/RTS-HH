/*
* main.h
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/03/2018 4:20:03 PM
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
#define F_CPU 8000000 // crystal frequency for delay.h
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////

#define LOOP_DELAY			200 // milliseconds





//Robot dimensions
#define TRACK_WIDTH			0.10640	//Track width of robots wheels (m)
#define TRACK_WIDTH_P		14.9564	//Track width in pulses
#define PPR					24.		//Pulses per revolution
#define PULSE_DIST			0.007114//Distance travelled in one pulse (m)



//////////////[Private Global Variables]///////////////////////////////////////////////////////////
struct Position
{
	float x; // x and y position coordinates in metres
	float y;
	float h; // heading angle in radians, anti-clockwise from the x-axis
};




//////////////[Functions]///////////////////////////////////////////////////////////////////////////
void Setup(void); // ATmega128 initialisation for this program
// calculate robot position from wheel pulse counts
void Operate(struct Position *ppos);

/*******************************************************************************
* MAIN FUNCTION
*******************************************************************************/
int main (void)
{
	char str[40]; // serial output string
	struct Position pos; // current position from CalcPosition
	Setup(); // initialise ATmega128
	// set initial robot location at origin
	pos.x = 0.;
	pos.y = 0.;
	pos.h = 0.;
	// display started on serial port
	OutputString("\r\n**************\r\n");
	OutputString("ROBOT POSITION\r\n");
	// display the wheel pwm settings
	sprintf(str, "SPEED L=%d R=%d\r\n", LEFT_SPEED, RIGHT_SPEED);
	OutputString(str);
	OCR1A = LEFT_SPEED; // set the motor speeds
	OCR1B = RIGHT_SPEED;
	while (1) // loop forever
	{
		_delay_ms(LOOP_DELAY); // wait for a constant time
		Operate(&pos); // calculate position from wheel counters
	}
}

/*******************************************************************************
* OTHER FUNCTIONS
*******************************************************************************/
void Setup(void) // ATmega128 setup
{

	

	

	// wait 500ms for the electronic hardware to settle
	_delay_ms(500);
	// enable external interrupts for the wheel pulses (INT0, INT1)
	// use rising edges of wheel pulses


	sei(); // enable interrupts last
}

void Operate(struct Position *ppos) // calculate the robot position
{
	int leftCount, rightCount;		// number of wheel pulses
	float dTheta = 0;				//delta angle of each wheel in radians
	float r = 0, dx = 0, dy = 0;
	
	
	static int leftTotal = 0, rightTotal = 0, timeTotal = 0;
	char str[100]; // serial output string
	// get the pulse counts
	cli();
	leftCount = leftPulseCount; // copy the pulse counts
	rightCount = rightPulseCount;
	leftPulseCount = 0; // reset the pulse counts to zero
	rightPulseCount = 0;
	sei();
	// increment the time counter
	timeTotal++;
	// if there were any pulses
	// ADD CODE
	if (leftCount || rightCount)
	{
		// calculate the new position
		
		//Calculate the change in heading
		dTheta = (float)(rightCount - leftCount)*PULSE_DIST/TRACK_WIDTH;

		//Calculate the change in position
		if(rightCount != leftCount)
		{
			//Calculate the radius of the curvature of the motion
			r = (TRACK_WIDTH/2*((float)rightCount + (float)leftCount)*PULSE_DIST)
			/(((float)rightCount - (float)leftCount)*PULSE_DIST);
			dx = r*(sin(ppos->h + dTheta) - sin(ppos->h));
			dy = r*(cos(dTheta) - cos(dTheta - ppos->h));
			} else {
			dx = PULSE_DIST*rightCount*cos(ppos->h);
			dy = PULSE_DIST*rightCount*sin(ppos->h);
		}
		
		// add pulses to the totals
		leftTotal += leftCount;
		rightTotal += rightCount;
		
		//Update position structure.
		ppos->h += dTheta;
		ppos->x += dx;
		ppos->y += dy;
		
		// display the new position (convert heading to degrees)
		sprintf(str, "POS,%6.3f,%6.3f,%6.1f,%1d,%1d,%4d,%4d,%4d\r\n",
		ppos->x, ppos->y, ppos->h * 180. / M_PI,
		leftCount, rightCount, leftTotal, rightTotal, timeTotal);
		OutputString(str);
	}
}
