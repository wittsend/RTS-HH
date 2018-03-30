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
* int delay_ms(uint16_t period_ms);
* uint8_t nbdelay_ms(uint16_t period_ms);
* int get_ms(uint32_t *timestamp);
* ISR(TIMER3_COMPA_vect)
*
*/
 
#ifndef TIMER_H_
#define TIMER_H_

//////////////[Global Vars]/////////////////////////////////////////////////////////////////////////

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

/*
* Function: int delay_ms(uint16_t period_ms)
*
* Halts execution for desired number of milliseconds.
*
* Inputs:
* period_ms is the number of milliseconds to wait
*
* Returns:
* Always returns 0
*
*/
int delay_ms(uint16_t period_ms);

/*
* Function:
* uint8_t nbdelay_ms(uint16_t period_ms)
*
* Non blocking delay.
*
* Inputs:
* uint32_t period_ms
*   Delay in ms
*
* Returns:
* 0 when time is up, otherwise 1
*
*/
uint8_t nbdelay_ms(uint16_t period_ms);

/*
* Function: int get_ms(uint32_t *timestamp)
*
* Outputs the system uptime generated from Timer 3.
*
* Inputs:
* none
*
* Returns:
* The current system timestamp in milliseconds.
*
* Implementation:
* Disable the timer compare interrupt, then retrieve the millisecond value stored in
* systemTimestamp. Finally, enable the timer interrupt again and return the time stamp value.
*
*/
uint32_t get_ms();

#endif /* TIMER_H_ */