//*************************************************************************************
/** \file task_output.cpp
 *    This file contains a task class for running a user interface for the motor
 *    controller demonstration. It has a single-state task which just reads the
 *    serial port to see if the user typed anything, and acts if s/he has done so. 
 *
 *  Revisions:
 *    \li 02-06-2008 JRR Original file
 *    \li 05-15-2008 JRR Modified to work with two motor drivers rather than one
 *    \li 03-08-2009 JRR Added code to test A/D converter
 *    \li 01-19-2011 JRR Updated calls to newer version of stl_task constructor
 *
 *  License:
 *    This file released under the Lesser GNU Public License, version 2. This program
 *    is intended for educational use only, but it is not limited thereto. 
 */
//*************************************************************************************


#include <stdlib.h>
#include <avr/io.h>
#include "lib/rs232int.h"
#include "lib/stl_timer.h"
#include "lib/stl_task.h"
#include "servo.h"
#include "slave_picker.h"			// The class that sets the multiplexer pins
#include "motor.h"
#include "task_output.h"
#include "lib/global_debug.h"


//-------------------------------------------------------------------------------------
/** This constructor creates a user interface task object. It checks if the user has
 *  typed a meaningful command at the serial port and if so tells the motor controller
 *  what to do. 
 *  @param a_timer   A reference to the real-time measuring timer for tasks
 *  @param t_stamp   A timestamp which contains the time between runs of this task
 *  @param p_mo_task A pointer to a motor control task to be ordered around
 *  @param p_timer   A pointer to the main real-time clock object in use
 *  @param p_a_to_d  A pointer to the A/D converter which measures voltages
 *  @param p_ser	 A pointer to a serial device for sending and receiving messages
 */

task_output::task_output (task_timer& a_timer, time_stamp& t_stamp, base_text_serial* p_ser_comp, base_text_serial* p_ser_slave, slave_picker* p_slave_picker, motor* p_the_motors ) 
	: stl_task (a_timer, t_stamp)
{
	
	// Assign pointers
	p_serial_comp = p_ser_comp;
	p_serial_slave = p_ser_slave;
	p_slave_chooser = p_slave_picker;
	p_motors = p_the_motors;
	
	for(unsigned char i = 0; i < 8; i++)
	{
		finger_configuration[i] = 0;
	}
	
	for(unsigned char i = 1; i < 14; i++)
	{
		output[i] = 0;
	}
	
}

//-------------------------------------------------------------------------------------
/** This is the function which runs when it is called by the task scheduler. It causes
 *  the motor to move back and forth, having several states to cause such motion. 
 *  @param state The state of the task when this run method begins running
 *  @return The state to which the task will transition, or STL_NO_TRANSITION if no
 *	  transition is called for at this time
 */

char task_output::run (char state)
{
	switch(state)
	{
		// Wait for output change
		case(0):		
				if(flag_output_change)
				{
					flag_output_change = false;
					return(1);	// Go to state 1 (collect outputs)
				}
				break;
				
		// Process outputs
		case(1):		
				// Pinky (finger configuration 0, motor 6)
				switch(finger_configuration[0])
				{
					case(STRAIGHT):
						output[6] = '0';
						break;
					case(CURL):
						output[6] = '2';
						break;
					case(CLENCH):
						output[6] = '4';
						break;
					default:
						GLOB_DEBUG("Error");
						break;	
				}
				
				// Ring (finger configuration 1, motor 5)
				switch(finger_configuration[1])
				{
					case(STRAIGHT):
						output[5] = '0';
						break;
					case(CURL):
						output[5] = '2';
						break;
					case(CLENCH):
						output[5] = '4';
						break;
					default:
						GLOB_DEBUG("Error");
						break;	
				}
				
				// Middle (finger configuration 2, motors 3 and 4)
				switch(finger_configuration[2])
				{
					case(STRAIGHT):
						output[4] = '0';
						output[3] = '0';
						break;
					case(CURL):
						output[4] = '2';
						output[3] = '2';
						break;
					case(CLENCH):
						output[4] = '4';
						output[3] = '4';
						break;
					case(VERTICAL_CLENCH):
						output[4] = '4';
						output[3] = '0';
						break;
					case(HORIZONTAL_STRAIGHT):
						output[4] = '0';
						output[3] = '4';
						break;
					default:
						GLOB_DEBUG("Error");
						break;	
				}
				
				// Index (finger configuration 3, motors 1 and 2)
				switch(finger_configuration[3])
				{
					case(STRAIGHT):
						output[2] = '0';
						output[1] = '0';
						break;
					case(CURL):
						output[2] = '2';
						output[1] = '2';
						break;
					case(CLENCH):
						output[2] = '4';
						output[1] = '4';
						break;
					case(VERTICAL_CLENCH):
						output[2] = '0';
						output[1] = '4';
						break;
					case(HORIZONTAL_STRAIGHT):
						output[2] = '4';
						output[1] = '0';
						break;
					case(SLANTED_STRAIGHT):
						output[2] = '0';
						output[1] = '2';
						break;
					default:
						GLOB_DEBUG("Error");
						break;	
				}
				
				// Index spread (finger configuration 4, motor 11)
				switch(finger_configuration[4])
				{
					case(0):
						output[11] = '0';
						break;
					case(1):
						output[11] = '1';
						break;
					default:
						GLOB_DEBUG("Error");
						break;	
				}
				
				// Thumb (finger configuration 5, motors 7, 8, 9, and 10)
				switch(finger_configuration[5])
				{
					case(FLAT_SIDE):
						output[10] = '0';
						output[9] = '0';
						output[8] = '0';
						output[7] = '0';
						break;
					case(OUT_SIDE):
						output[10] = '0';
						output[9] = '4';
						output[8] = '0';
						output[7] = '0';
						break;
					case(FOLD_FLAT):
						output[10] = '3';
						output[9] = '1';
						output[8] = '3';
						output[7] = '0';
						break;
					case(FOLD_STRAIGHT_OUT):
						output[10] = '4';
						output[9] = '4';
						output[8] = '0';
						output[7] = '0';
						break;
					case(FOLD_STRAIGHT_UP):
						output[10] = '4';
						output[9] = '0';
						output[8] = '0';
						output[7] = '0';
						break;
					case(THUMB_CURL):
						output[10] = '4';
						output[9] = '2';
						output[8] = '1';
						output[7] = '2';
						break;
					default:
						GLOB_DEBUG("Error");
						break;	
				}
				
				// Wrist flexion/extension (finger configuration 6, motor 12)
				output[12] = finger_configuration[6];
				
				// Wrist supination/pronation
				output[13] = finger_configuration[7];
				
				return(2);	// Go to state 2 (output) when done
				break;
				
		// Output to motor controllers
		case(2):		
				for(unsigned char i = 1; i < 14; i++)
				{
					p_motors->output(i,output[i]);
				}
				return(0);	// Return to state 0 (wait) when done
				break;
		
		default:
				break;
	}
	// If we get here, no transition is called for
	return (STL_NO_TRANSITION);
}

void task_output::change_output(unsigned char finger, unsigned char configuration)
{
	finger_configuration[finger] = configuration;
	flag_output_change = true;
}