/*
* timer.h
*
* Author : Matthew Witt
* Created: 6/08/2017 1:23:27 PM
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* Sets up the timer for 1ms interrupts. Modified for Hedgehog robots
*
* More Info:
* Atmel ATmega128 Datasheet:http://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void timer3Init(void)
* ISR(TIMER3_COMPA_vect)
*
*/
 
#ifndef TIMER_H_
#define TIMER_H_

///////////////[Global Vars]////////////////////////////////////////////////////////////////////////
extern uint32_t systemTimestamp;	//Timestamp that stores the number of ms since powerup.

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void timer3Init(void)
*
* Initializes timer3
* Used to time events with a 1ms interrupt on RC compare match
* Sets timr0 CLK speed to 12.5MHz for camera
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void timer3Init(void);


#endif /* TIMER_H_ */