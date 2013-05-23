//============================================================================================================
/** \file motor.h
 *	This file contains program for an ATtiny2313 motor driver that outputs to a L293D motor driver chip. The
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

#include <avr/io.h>
#include "motor.h"

//--------------------------------------------------------------------------------------
/** This constructor sets up the ATtiny2313 for motor driving.
 */

motor::motor (void)
{
	// Set up registers for Output Compare on OC0A
	
		// Set up Timer/Counter Control Register A
		TCCR0A = (1 << COM0A1);	// Clear OC0A on Compare Match, set OC0A at timer max
		TCCR0A |= (1 << WGM01);	// Fast PWM Mode. Count from 0 to 255
		TCCR0A |= (1 << WGM00);	
	
		// Set up Timer/Counter Control Register B
		TCCR0B = (1 << CS02);	// Divide clock by prescaler 256
	
		// Clear Output Compare Register
		OCR0A = 0;	// Clear motor output pwm register
		
	// Set up pins for output
	
		// Set data directions
		MOTOR_DDR |= (1 << PIN_PWM);	// Set all three pins to outputs
		MOTOR_DDR |= (1 << PIN_INA);
		MOTOR_DDR |= (1 << PIN_INB);
		
		// Set both directional pins off
		MOTOR_PORT &= ~(1 << PIN_INA);
		MOTOR_PORT &= ~(1 << PIN_INB);
}

void motor::stop (void)
{
	// Both pins high
	MOTOR_PORT |= (1 << PIN_INA);
	MOTOR_PORT |= (1 << PIN_INB); 
}

void motor::d0 (void)
{
	// A high; B low
	MOTOR_PORT |= (1 << PIN_INA);
	MOTOR_PORT &= ~(1 << PIN_INB);
}

void motor::d1 (void)
{
	// A low; B high
	MOTOR_PORT &= ~(1 << PIN_INA);
	MOTOR_PORT |= (1 << PIN_INB);
}

void motor::output (unsigned char duty_cycle)
{
	// Set duty cycle
	OCR0A = duty_cycle;
}

