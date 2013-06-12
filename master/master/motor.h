//*************************************************************************************
/** \file motor.h
 *	  This file contains a task class for running a user interface for the motor
 *	  controller demonstration. It has a single-state task which just reads the
 *	  serial port to see if the user typed anything, and acts if s/he has done so. 
 *
 *  Revisions:
 *	  \li 02-06-2008 JRR Original file
 *	  \li 05-15-2008 JRR Modified to work with two motor drivers rather than one
 *	  \li 03-08-2009 JRR Added code to test A/D converter
 *    \li 01-19-2011 JRR Updated calls to newer version of stl_task constructor
 *
 *  License:
 *	This file released under the Lesser GNU Public License, version 2. This program
 *	is intended for educational use only, but it is not limited thereto. 
 */
//*************************************************************************************

#include "servo.h"

#ifndef _MOTOR_H_
#define _MOTOR_H_

#define MOTOR_SWITCH_DDR	DDRD
#define MOTOR_SWITCH_PORT	PORTD
#define MOTOR_SWITCH_PIN	PIND6

//-------------------------------------------------------------------------------------
/** This class contains a task which moves a motorized lever back and forth. 
 *  WARNING:  This task uses an older version of parent class stl_task, and its 
 *            constructor parameters are out of date. Use it as an example, but
 *            do not attempt to just copy the parameters. 
 */

class motor
{
	public:
		
		slave_picker* p_slave_chooser;
		base_text_serial* p_serial_slave;
		servo* p_servo_top;
		servo* p_servo_bottom;
		
		// The constructor creates a new task object
		motor (base_text_serial*, slave_picker*, servo*, servo*);

		// The run method is where the task actually performs its function
		void output (unsigned char, unsigned char);
};

#endif
