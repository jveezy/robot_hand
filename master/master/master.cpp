//======================================================================================
/** \file mototest.cpp
 *    This file contains a program to test the ME 405 board's motor driver. It makes a
 *    motor move at speeds determined by the value read from a potentiometer connected 
 *    to the AVR's analog inputs. 
 *
 *  Revisions
 *    \li 01-05-2008 JRR Original file
 *    \li 02-03-2008 JRR Various cleanup, tested on new ME 405 boards
 *    \li 01-15-2008 JRR Changed to new file/directory layout with ./lib and *.cpp
 *    \li 01-19-2011 JRR Updated calls to newer version of stl_task constructor
 *
 *  License:
 *    This file released under the Lesser GNU Public License, version 2. This program
 *    is intended for educational use only, but it is not limited thereto. 
 */
//======================================================================================

											// System headers included with < >
#include <stdlib.h>							// Standard C library
#include <avr/io.h>							// Input-output ports, special registers
#include <avr/interrupt.h>					// Interrupt handling functions

											// User written headers included with " "
#include "lib/queue.h"						// Queue class used for character buffer
#include "servo.h"							// Servo class
#include "slave_picker.h"					// The class that sets the multiplexer pins
#include "character.h"
#include "character_database.h"				// The class that stores all character info
#include "lib/rs232int.h"					// Serial port header
#include "lib/global_debug.h"				// Header for serial debugging port
#include "motor.h"							// Class containing all motors
#include "lib/stl_timer.h"					// Microsecond-resolution timer
#include "lib/stl_task.h"					// Base class for all task classes
#include "task_output.h"					// The task that outputs all commands to the motor controllers
#include "task_user.h"						// The task that listens to the user


//--------------------------------------------------------------------------------------
/** The main function is the "entry point" of every C program, the one which runs first
 *  (after standard setup code has finished). For mechatronics programs, main() runs an
 *  infinite loop and never exits. 
 */

int main ()
{
	volatile unsigned int dummy = 0;		// Delay loop kind of counter
	char time_string[16];					// Character buffer holds printable time
	char input_char;						// A character typed by the user
	unsigned char motor_duty = 0;			// Duty cycle to send to motor
	bool going_clockwise = true;			// Which way is the motor going? 

	// Create objects

			// Create a serial port object. Messages will be printed to this port, which
			// should be hooked up to a dumb terminal program like minicom on a PC
			rs232 sport_slave (9600, 1);
			rs232 sport_comp (9600, 0);
			set_glob_debug_port (&sport_comp);

			// Create a slave picker
			slave_picker the_slave_picker;
			
			// Create a character database
			character_database char_dbase;
			
			// Servos and motor databases
			servo servo_top(1);
			servo servo_bottom(2);
			motor the_motors (&sport_slave,&the_slave_picker,&servo_top,&servo_bottom);

			// Create a microsecond-resolution timer
			task_timer the_timer;

	// Create and set up tasks
			
			// Create a time stamp which holds the interval between runs of the motor task
			// The time stamp is initialized with a number of seconds, then microseconds
			time_stamp interval_time (0, 10000);

			// Set the interval to 20ms
			interval_time.set_time (0, 10000);
			task_output output_task (the_timer, interval_time, &sport_comp, &sport_slave, &the_slave_picker, &the_motors);

			// Set the interval a bit slower for the user interface task
			interval_time.set_time (0, 25000);

			// Create a task to read commands from the keyboard
			task_user user_task (the_timer, interval_time, &sport_comp, &sport_slave, &the_slave_picker, &output_task);
			
			// Turn on interrupt processing so the timer can work
			sei ();

	// Run the main scheduling loop, in which the tasks are continuously scheduled.
	// This program currently uses very simple "round robin" scheduling in which the
	// tasks are simply called in order. More sophisticated scheduling strategies
	// will be used in other more sophisticated programs
	while (true)
	{
		output_task.schedule ();
		user_task.schedule ();
	}

	return (0);
}

