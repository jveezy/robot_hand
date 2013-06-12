//*************************************************************************************
/** \file servo.cpp
 *    This file contains a class which runs the motor driver on an ME405 board, 
 *    version 0.60+. This version has two VNH3SP30 motor drivers controlled by the
 *    ATmega128 microcontroller. Please see the class definition for class motor405
 *    for a list of which pins are connected to which signals. 
 *
 *  Revisions:
 *    \li 02-24-2012 JV  Created file
 *
 *  License:
 *    This file released under the Lesser GNU Public License, version 2. This program
 *    is intended for educational use only, but its use is not limited thereto. 
 */
//*************************************************************************************

#include <avr/io.h>
#include "servo.h"


//--------------------------------------------------------------------------------------
/** This constructor enables the servo driver 
 *  @param servo_number The number of the motor to be controlled, motor 1 or motor 2
 */

servo::servo (unsigned char servo_number)
{
	static bool timer_set_up = false;		// Set up timer only on first call

	which_motor = servo_number;				// Save the number of the servo to control

	// If nobody has yet set up the timer which both motors share, set it up now
	if (!timer_set_up)
	{
		// Set up Timer 1 in 16-bit fast PWM mode with the 
		//prescaler at I/O clock / 8, with the PWM in normal polarity
		TCCR1A = (1 << WGM11);
		TCCR1B = (1 << CS11) | (1 << WGM12) | (1 << WGM13);
		ICR1 = 49999;

		// The block above should only be called once when the first PWM is set up
		timer_set_up = true;
	}

	// If motor 1 is being set up, use Port C for mode and OC1B = Port B pin 6 for PWM
	if (which_motor == 1)
	{
		// Set output compare for fast PWM, normal polarity
		TCCR1A |= (1 << COM1A1);

		// Set up the servo pin as an output
		SERVO_DDR |= (1 << SERVO_PIN1);

		// Set the duty cycle to zero, also for safety
		OCR1AH = 0;
		OCR1AL = 0;
	}
	// If motor 2 is being set up, use Port D for mode and OC1A = Port B pin 5 for PWM
	else if (which_motor == 2)
	{
		// Set output compare for fast PWM, normal polarity
		TCCR1A |= (1 << COM1B1);

		// Set up the servo pin as an output
		SERVO_DDR |= (1 << SERVO_PIN2);

		// Set the duty cycle to zero, also for safety
		OCR1BH = 0;
		OCR1BL = 0;
	}
	else
	{
		return;
		// An invalid motor number has been chosen -- we can't do anything
	}
}

//--------------------------------------------------------------------------------------
/** This method outputs a servo signal to the servo based on an input angle
 *  @param angle The angle the servo should turn
 */

void servo::output (unsigned char angle)
{
	unsigned short int new_ocr = (unsigned short int) ((unsigned long int) 2500 * angle / 180) + 2500;
	
	if (which_motor == 1)
	{
		OCR1A = new_ocr;
	}
	else if (which_motor == 2)
	{
		OCR1B = new_ocr;
	}
}
