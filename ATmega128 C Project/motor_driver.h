/*
* motor_driver.h
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 12/03/2018 6:44:38 PM
*
* Project Repository: https://github.com/wittsend/RTS-HH
*
* Header file for the Hedgehogs motor driver module
*
* More Info:
* Atmel ATmega128 Datasheet:http://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void motorInit(void);
* void motorLeftDrive(int16_t speed);
* void motorRightDrive(int16_t speed);
* void motorStop(void);
* uint8_t moveRobot(float speed, float turnRatio);
* int8_t getLeftEncPulses(void);
* int8_t getRightEncPulses(void);
*
*/

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////

//////////////[Public Defines]//////////////////////////////////////////////////////////////////////
#define MOTOR_PPR				24.			//Pulses per revolution
#define MOTOR_PULSE_DIST		0.007114	//Distance travelled in one pulse (m)

//////////////[External Global Variables]///////////////////////////////////////////////////////////

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void motorInit(void)
*
* Initialises all hardware required to drive the motors on the Hedgehog
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void motorInit(void);

/*
* Function:
* void motorLeftDrive(int16_t speed)
*
* Drives the left motor at the given speed and direction
*
* Inputs:
* int16_t speed:
*	An integer between -1023 and 1023 that indicates the PWM speed to drive the motor at (1023 is
*	maximum). A negative number drives the motor in reverse.
*
* Returns:
* none
*
*/
void motorLeftDrive(int16_t speed);

/*
* Function:
* void motorRightDrive(int16_t speed)
*
* Drives the right motor at the given speed and direction
*
* Inputs:
* int16_t speed:
*	An integer between -1023 and 1023 that indicates the PWM speed to drive the motor at (1023 is
*	maximum). A negative number drives the motor in reverse.
*
* Returns:
* none
*
*/
void motorRightDrive(int16_t speed);

/*
* Function:
* motorStop(void)
*
* Stops all motors
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void motorStop(void);

/*
* Function:
* uint8_t moveRobot(float speed, float turnRatio)
*
* A function to move the robot in a given direction
*
* Inputs:
* float speed:
*	A floating point value between -1023 and 1023 that indicates how fast the motors should be
*	driven. Negative numbers will attempt to drive the robot backwards.
* float turnRatio:
*   The ratio of rotation to be applied to the motion (-1023 to 1023). if turnRatio is 0, then
*   robot just drives straight at 'speed'. -1023 will have robot rotating CCW on the spot at
*   'speed'. 512 would be half and half driving forward with a CW rotational element applied. If
*   both speed and turnRatio are negative, then robot will rotate in CW (-1*-1) = 1
*
* Returns:
* Returns 0 on exit
*
*/
uint8_t moveRobot(float speed, float turnRatio);

/*
* Function:
* int8_t getRightEncPulses(void)
*
* Returns the number of pulses that have occurred since the last time the encoder was checked
*
* Inputs:
* none
*
* Returns:
* Returns the number of pulses that have occurred on the right wheel since this function was last
* called
*
*/
int8_t getLeftEncPulses(void);

/*
* Function:
* int8_t getRightEncPulses(void)
*
* Returns the number of pulses that have occurred since the last time the encoder was checked
*
* Inputs:
* none
*
* Returns:
* Returns the number of pulses that have occurred on the left wheel since this function was last
* called
*
*/
int8_t getRightEncPulses(void);

#endif /* MOTOR_DRIVER_H_ */