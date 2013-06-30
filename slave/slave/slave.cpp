//============================================================================================================
/** \file controller.cc
 *	This file contains a program for an ATtiny2313 chip to control a single motor using proportional control.
 *	It includes interrupt service routines for monitoring the two quadrature encoder channels and PWM output
 *	to a motor driver chip. It also includes serial communication code to communicate with another
 *	microcontroller or a computer.
 *
 *  Revisions
 *	\li  04-02-2011	JV	Original file for four channel quadrature decoder
 *	\li	04-05-2011	JV	File changed to include complete motor control
 *	\li	04-12-2011	JV	Motor control classes tested
 *
 *  License:
 *	This file released under the Lesser GNU Public License, version 2. This program
 *	is intended for educational use only, but it is not limited thereto. 
 */
//============================================================================================================
/* Includes */

// Standard Libraries

#include <avr/io.h>			// AVR device-specific input/output definitions
#include <avr/interrupt.h>	// AVR interrupt code
	
// Header Files

#include "motor.h"			// Motor Object
#include "serial.h"			// Serial Object
#include "angles.h"			// Angle Configuration

//============================================================================================================
/* Definitions */

#define INTERRUPT_DDR	DDRD
#define INTERRUPT_PORT	PORTD
#define INTERRUPT_PINR	PIND
#define PIN_INT0		PIND2
#define PIN_INT1		PIND3

#define CLOCKWISE		count++
#define COUNTERCLOCKWISE	count--
#define ENCODER_ERROR	errors++


//============================================================================================================
/* Variable Definitions and Initialization */

	// Serial Port
	char 			character_in;		// Incoming character read by serial port
	char				throwaway;		// Throwaway character from when other chips are talked to
	unsigned char		count_input;		// Incoming count set point from master chip

	// Encoder Reading
	unsigned short int	count = 1;			// Encoder count
	unsigned char		count_8bit = 1;		// Top 8 bits of the 10 bit encoder count
	unsigned char		current_reading = 0;	// Current encoder quadrature reading
	unsigned char		previous_reading = 0;	// Last encoder quadrature reading
	unsigned char		errors = 0;			// Number of encoder errors
	
	// Configuration
	unsigned char		kp_array[10];
	unsigned char		ki_array[10];
	unsigned char		kd_array[10];
	unsigned char		set_point_angles[10];

	// Control Loop
	unsigned short int	desired_count;		// Desired encoder count
	short int			control_error;		// Difference between encoder_count and desired_count
	unsigned char		kp;				// Proportional gain
	unsigned char		ki;				// Integral gain
	unsigned char		kd;				// Derivative gain
	long int			motor_output;		// PWM value to output to the motor
	unsigned char		set_point = 1;		// Set point (1-5) for motor position
	unsigned char		motor_number;		// '1'-'0' identification of which motor number

	// State Transition Logic
	unsigned char 		state_motor = 0;	// Next state to jump into in motor task
	unsigned char		state_data = 0;	// Next state to jump into for data task
	
	// Flags
	bool				flag_enable = false;	// Motor output enable
	bool				flag_calibrate = false;	// Encoder calibration flag

	// Miscellaneous
	unsigned long		i = 0;			// Dummy counter
	unsigned char		i_angle = 0;	// Angle count
	
	// Objects
	motor mtr;
	serial sport;


//============================================================================================================
/* State-Transition Logic Tasks */

// Motor Task

	unsigned char motor_task(unsigned char state_motor, motor* the_motor)
	{
		mtr = *the_motor;
		
		switch(state_motor)
		{
			case(0):		// Check Flags
				if (!(flag_enable))	// If motor stop command issued
				{
					state_motor = 1;	// go to state 1
					break;
				}
				if (flag_enable)	// If motor go command issued (or no overriding commands issued)
				{
					state_motor = 2;	// go to state 2
					break;
				}
				break;
			case(1):		// Stop Motor
				mtr.stop();		// Activate brake
				state_motor = 0;	// go to state 0
				break;
			case(2):		// Calculate Motor Output
			
				// Calculate control loop error
				control_error = count - desired_count;
					
				// Calculate value and trim to 0-255 range
				if (control_error > 1)
					motor_output = (long) (kp * control_error * 255) / 768;
				else if (control_error < -1)
					motor_output = (long) (0xFF * control_error * 255) / 768;
				else
					motor_output = 0;
				if (motor_output > 255)
					motor_output = 255;
				
				state_motor = 3;	// go to state 3
				break;
			case(3):		// Output to Motor
				// Set direction
				if (control_error > 0)
				{
					mtr.d1();
					mtr.output( (unsigned char) motor_output);
				}
				else if (control_error < 0)
				{
					mtr.d0();
					mtr.output( (unsigned char) motor_output);
				}
				else
					mtr.stop();
					
				state_motor = 0;	// Always return to state 1
				break;
			default:
				state_motor = 0;
				break;
		}
		return(state_motor);
	}
	
// Data Task

	unsigned char data_task(unsigned char state_data, serial* serial_port, motor* the_motor)
	{
		sport = *serial_port;
		mtr = *the_motor;
		
		switch(state_data)
		{
			case(0):		// Check for Character
				if(sport.check_for_char())	
				{	
					state_data = 1;	// If character received go to state 1
				}
				else					
				{
					state_data = 0;	// If not remain in state 0
				}
				break;
			case(1):		// Process Character
					
				// Interpret character
				switch(character_in)
				{
					// a,b,c,d,e define set points
					case('a'):
					case('b'):
					case('c'):
					case('d'):
					case('e'):
						switch(character_in)
						{
							case('a'):						
								set_point = 1;
								break;
							case('b'):
								set_point = 2;
								break;
							case('c'):
								set_point = 3;
								break;
							case('d'):
								set_point = 4;
								break;
							case('e'):
								set_point = 5;
								break;
							default:
								break;
						}
						//sport.send('A');		// Confirm command reception
						state_data = 6;
						break;
					// S,G disable and enable the motor
					case('S'):	// Stop Motor
						flag_enable = false;	// Disable motor
						state_data = 1;		// Go to state 1
						sport.send('s');		// Confirm command reception
						break;
					case('G'):	// Go (enable motor)
						flag_enable = true;		// Enable motor
						state_data = 1;			// Go to state 1
						sport.send('g');		// Confirm command reception
						break;
					// C clears the encoder count to calibrate the motor position
					case('C'):	// Calibrate
						flag_calibrate = !flag_calibrate;	// Toggle calibration flag
						state_data = 5;						// Go to state 5
						sport.send('c');					// Confirm command reception
						break;
					// Q queries to identify whether the motor is done moving to the correct position
					case('Q'):	// Position Query from master chip
						state_data = 2;
						break;
					case('1'):	// Identify motor
					case('2'):
					case('3'):
					case('4'):
					case('5'):
					case('6'):
					case('7'):
					case('8'):
					case('9'):
						motor_number = character_in - 0x30;
						state_data = 3;
						break;
					case('0'):
						motor_number = 10;
						state_data = 3;
						break;
					case('E'):	// Encoder Query
						state_data = 4;
						break;
					default:
						state_data = 1;	// Return to state 1 if character is unclear
						break;
				}
				break;
			case(2):		// Position Query
				state_data = 1;
				break;
			case(3):		// Motor Identification and data loading
				// Load angle data
				for (i_angle = 0; i < 5; i++)
				{
					set_point_angles[i_angle] = angles[i_angle][motor_number-1];
				}
				
				// Load gain data
				kp = kp_array[motor_number-1];
				
				// Send confirmation back to master
				sport.send('!');
				
				state_data = 1;
				break;
			case(4):		// Respond to Encoder Query
				count_8bit = (unsigned char) (count << 2);
				sport.send(count_8bit);
				break;
			case(5):		// Calibrate
				if (!flag_calibrate)	// If the calibration flag has been turned off
				{
					count = 1;	// Clear count
				}
				state_data = 1;	// Always return to state 1
				break;
			case(6):		// New set point
				desired_count = set_point_angles[set_point-1];
				state_data = 1;
				break;
			default:
				state_data = 1;
				break;
		}
		return(state_data);
	}

//============================================================================================================
/* Main Function */

// Initialize variables


int main(void)
{

// Setup

	// Create objects
	motor mtr;	// Create motor
	serial sport;	// Create serial port
	
	// Setup Encoder Data Directions
	INTERRUPT_DDR &= ~(1 << PIN_INT0);	// Input
	INTERRUPT_DDR &= ~(1 << PIN_INT1);	// Input
	
	// Enable interrupts on INTO, INT1, and PCINT2
	
		// Enable interrupt on both rising and falling edges for both pins
		MCUCR |= (1 << ISC10);	// ISC11 = 0, ISC10 = 1
		MCUCR |= (1 << ISC00);	// ISC01 = 0, ISC00 = 1
		
		// Enable interrupts on INTO, INT1, and PCIE
		GIMSK |= (1 << INT0);
		GIMSK |= (1 << INT1);
		GIMSK |= (1 << PCIE);
		
		// Enable interrupts on PCINT2
		PCMSK |= (1 << PCINT2);	// Write 1 to PCINT2 bit of PCMSK register
	
	// Turn on interrupts
	sei();
	
	// Motor Data
	for (i = 1; i<10; i++)
	{
		kp_array[i] = 4;
	}

// Loop

	while(true)	// loop forever between these two tasks
	{		
		state_motor = motor_task(state_motor, &mtr);
		state_data = data_task(state_data, &sport, &mtr);
	}	
	return(0);
}

//============================================================================================================
/* Interrupt Service Routines */


// Interrupt for Encoder Channel A
ISR(INT0_vect)
{
	// Store last reading to old variable
	previous_reading = current_reading;
	
	// Take in new reading
	current_reading = ((PIND & 0b00000100) >> 1) | ((PIND & 0b00001000) >> 3);	// (A << 1) | B

	// Evaluate Reading
	switch(current_reading)
	{
		case(0):				// Current value == 00
			switch(previous_reading)
			{
				case(1):		// 01 to 00
					CLOCKWISE ;
					break;
				case(2):		// 10 to 00
					COUNTERCLOCKWISE ;
					break;
				default:		// 11 to 00
					ENCODER_ERROR ;
					break;
			}
			break;
		case(1):				// Current value == 01
			switch(previous_reading)
			{
				case(3):		// 11 to 01
					CLOCKWISE ;
					break;
				case(0):		// 00 to 01
					COUNTERCLOCKWISE ;
					break;
				default:		// 10 to 01
					ENCODER_ERROR ;
					break;
			}
			break;
		case(2):				// Current value == 10
			switch(previous_reading)
			{
				case(0):		// 00 to 10
					CLOCKWISE ;
					break;
				case(3):		// 11 to 10
					COUNTERCLOCKWISE ;
					break;
				default:		// 01 to 10
					ENCODER_ERROR ;
					break;
			}
			break;
		case(3):				// Current value == 11
			switch(previous_reading)
			{
				case(2):		// 10 to 11
					CLOCKWISE ;
					break;
				case(1):		// 01 to 11
					COUNTERCLOCKWISE ;
					break;
				default:		// 00 to 11
					ENCODER_ERROR ;
					break;
			}
			break;
		default:
			break;
	}
}

// Interrupt for Encoder Channel B
ISR(INT1_vect, ISR_ALIASOF(INT0_vect));	// Duplicate code from encoder channel A


