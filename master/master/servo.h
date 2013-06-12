//*************************************************************************************
/** \file servo.h
 *    This file contains a class which runs the servo. 
 *
 *  Revisions:
 *    \li 02-24-2012 JV  Created file
 *
 *  License:
 *    This file released under the Lesser GNU Public License, version 2. This program
 *    is intended for educational use only, but its use is not limited thereto. 
 */
//*************************************************************************************

#ifndef _SERVO_H_
#define _SERVO_H_						///< Prevents multiple inclusion of file

#define SERVO_DDR	DDRD
#define SERVO_PORT	PORTD
#define SERVO_PIN1	PIND5
#define	SERVO_PIN2	PIND4


//--------------------------------------------------------------------------------------
/** This class operates two servos using Timer Counter 1 
 */

class servo
{
	protected:
		unsigned char which_motor;			///< Is this object for motor 1 or 2?

	public:
		/// This constructor creates a motor controller object.
		servo (unsigned char);

		/// This method sets the OCR value to output to the servo
		void output (unsigned char);
};

#endif // _SERVO_H_
