//============================================================================================================
/** \file motor.h
 *	This file contains a header for an ATtiny2313 motor driver that outputs to a L293D motor driver chip. The
 *	ATtiny2313 will output a PWM signal and two directional signals to determine the H-bridge inputs.
 *
 *  Revised:
 *    \li 04-09-2011 JV	Original file
 *
 *  License:
 *	This file released under the Lesser GNU Public License, version 2. This program
 *	is intended for educational use only, but it is not limited thereto. 
 */
//============================================================================================================
/// This define prevents this .h file from being included more than once in a .cc file
#ifndef _MOTOR_H_
#define _MOTOR_H_

//============================================================================================================
/* Definitions */

#define MOTOR_PORT	PORTB		///< Name of port connected to motor output pins
#define MOTOR_DDR   DDRB			///< Data direction register for motor output pins
#define PIN_PWM	PINB2		///< PWM output pin
#define PIN_INA	PINB1		///< Direction input A
#define PIN_INB	PINB0		///< Direction input B

//============================================================================================================
/* Class Definition */

//-------------------------------------------------------------------------------------
/** This class sets up a motor class for the ATtiny 2313
 */

class motor
{
	public:
		/// The constructor sets up the port with the given baud rate and port number.
		motor (void);

		/// This method performs an emergency stop
		void stop (void);
		
		/// This method sets the motor to spin in direction 0
		void d0 (void);
		
		/// This method sets the motor to spin in direction 1
		void d1 (void);
		
		/// This method outputs a PWM value at a given direction
		void output (unsigned char);
};

//============================================================================================================

#endif


