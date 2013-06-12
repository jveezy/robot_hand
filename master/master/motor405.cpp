//*************************************************************************************
/** \file motor405.cpp
 *    This file contains a class which runs the motor driver on an ME405 board, 
 *    version 0.60+. This version has two VNH3SP30 motor drivers controlled by the
 *    ATmega128 microcontroller. Please see the class definition for class motor405
 *    for a list of which pins are connected to which signals. 
 *
 *  Revisions:
 *    \li 01-17-2008 JRR Created file
 *    \li 03-01-2008 JRR Fixed bug in brake() which prevented proper braking
 *    \li 05-15-2008 JRR Version for dual-driver ME405 board written
 *    \li 01-18-2009 JRR Reformatted a bit
 *    \li 01-15-2008 JRR Changed to new file/directory layout with ./lib and *.cpp
 *
 *  License:
 *    This file released under the Lesser GNU Public License, version 2. This program
 *    is intended for educational use only, but its use is not limited thereto. 
 */
//*************************************************************************************

#include <avr/io.h>
#include "motor405.h"


//--------------------------------------------------------------------------------------
/** This constructor enables the ME405 board's motor driver chip and sets the duty 
 *  cycle to zero in braking mode.  On the ME405 board, the driver is connected as 
 *  shown in the header file motor405.h. 
 *  @param motor_number The number of the motor to be controlled, motor 1 or motor 2
 */

motor405::motor405 (unsigned char motor_number)
{
	static bool timer_set_up = false;		// Set up timer only on first call

	which_motor = motor_number;				// Save the number of the motor to control

	// If nobody has yet set up the timer which both motors share, set it up now
	if (!timer_set_up)
	{
		// Set up Timer 1 in 8-bit fast PWM mode with the prescaler at I/O clock / 64, 
		// with the PWM in normal polarity (meaning motor on or brake on is logic 1)
		TCCR1A = (1 << WGM10);
		TCCR1B = (1 << CS11) | (1 << CS10) | (1 << WGM12);

		// The block above should only be called once when the first PWM is set up
		timer_set_up = true;
	}

	// Set the INA and INB bits to put the motor into braking mode for now; this is
	// generally the safest setting to use as a default
	brake ();

	// If motor 1 is being set up, use Port C for mode and OC1B = Port B pin 6 for PWM
	if (which_motor == 1)
	{
		// Set output compare for fast PWM, normal polarity
		TCCR1A |= (1 << COM1B1);

		// Set up the microcontroller pins connected to INA and INB as outputs
		M405_DDR_1 |= (M405_INA_1 | M405_INB_1);

		// Set the enable/diagnostic line as an input with the pullup turned on
		M405_DDR_1 &= ~M405_DIAG_1;
		M405_PORT_1 |= M405_DIAG_1;

		// Set up the PWM bit as an output; it's on Port B pin 6
		M405_PWM_D1 |= M405_PWM_B1;

		// Set the duty cycle to zero, also for safety
		OCR1BH = 0;
		OCR1BL = 0;
	}
	// If motor 2 is being set up, use Port D for mode and OC1A = Port B pin 5 for PWM
	else if (which_motor == 2)
	{
		// Set output compare for fast PWM, normal polarity
		TCCR1A |= (1 << COM1A1);

		// Set up the microcontroller pins connected to INA and INB as outputs
		M405_DDR_2 |= (M405_INA_2 | M405_INB_2);

		// Set the enable/diagnostic line as an input with the pullup turned on
		M405_DDR_2 &= ~M405_DIAG_2;
		M405_PORT_2 |= M405_DIAG_2;

		// Set up the PWM bit as an output; it's on Port B pin 5
		M405_PWM_D2 |= M405_PWM_B2;

		// Set the duty cycle to zero, also for safety
		OCR1AH = 0;
		OCR1AL = 0;
	}
	else
	{
		return;
		// An invalid motor number has been chosen -- we can't do anything
	}
}


//--------------------------------------------------------------------------------------
/** This method sets the motor driver chip's mode bits to cause the driver to push 
 *  current through the motor so as to cause torque in the "clockwise" direction. The
 *  definition of "clockwise" is rather arbitrary, depending on whether one is looking
 *  at the motor from one end or the other, so the user must decide if clockwise means
 *  forwards, or backwards, or whatever. 
 */

void motor405::clockwise (void)
{
	if (which_motor == 1)
	{
		M405_PORT_1 |= M405_INA_1;
		M405_PORT_1 &= ~M405_INB_1;
	}
	else if (which_motor == 2)
	{
		M405_PORT_2 |= M405_INA_2;
		M405_PORT_2 &= ~M405_INB_2;
	}
}


//--------------------------------------------------------------------------------------
/** This method sets the motor driver chip's mode bits to cause the driver to push 
 *  current through the motor so as to cause torque in the "counterclockwise" direction. 
 *  The definition of "counterclockwise" is arbitrary, depending on whether one is 
 *  looking at the motor from one end or the other, so the user must decide if clockwise
 *  means backwards, or forwards, or whatever. 
 */

void motor405::counterclockwise (void)
{
	if (which_motor == 1)
	{
		M405_PORT_1 |= M405_INB_1;
		M405_PORT_1 &= ~M405_INA_1;
	}
	else if (which_motor == 2)
	{
		M405_PORT_2 |= M405_INB_2;
		M405_PORT_2 &= ~M405_INA_2;
	}
}


//--------------------------------------------------------------------------------------
/** This method puts the motor driver chip into braking mode by setting both the INA 
 *  and INB pins to logic 0. How strongly the braking action takes place is controlled
 *  by the PWM signal; unlike many chips, this one has to be turned on to brake. 
 */

void motor405::brake (void)
{
	if (which_motor == 1)
	{
		M405_PORT_1 &= ~(M405_INA_1 | M405_INB_1);
	}
	else if (which_motor == 2)
	{
		M405_PORT_2 &= ~(M405_INA_2 | M405_INB_2);
	}
}


//--------------------------------------------------------------------------------------
/** This method sets the duty cycle of the motor driver. It simply puts its argument
 *  into the output compare register for Timer 1, which is the duty cycle register when
 *  the timer is in 8-bit fast PWM mode. 
 *  @param new_duty A new value for the motor duty cycle
 */

void motor405::set_duty_cycle (unsigned char new_duty)
{
	if (which_motor == 1)
	{
		OCR1AL = new_duty;
	}
	else if (which_motor == 2)
	{
		OCR1BL = new_duty;
	}
}
