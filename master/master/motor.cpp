//*************************************************************************************
/** \file motor.cpp
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
#include "lib/base_text_serial.h"
#include "slave_picker.h"
#include "servo.h"							// Driver for servo motors
#include "motor.h"
#include "lib/global_debug.h"

//-------------------------------------------------------------------------------------
/** This constructor creates a slave_picker object. It outputs the correct pins to
 *  choose multiplexer outputs to make sure the master is communicating with the right
 *  slave chips.
 */

motor::motor (base_text_serial* p_serialport_slave, slave_picker* p_slave_picker, servo* p_servotop, servo* p_servobottom,base_text_serial* p_serialport_comp)
{
	p_slave_chooser = p_slave_picker;
	p_serial_slave = p_serialport_slave;
	p_serial_comp = p_serialport_comp;
	p_servo_top = p_servotop;
	p_servo_bottom = p_servobottom;
	
	MOTOR_SWITCH_DDR |= (1 << MOTOR_SWITCH_PIN);
	MOTOR_SWITCH_PORT &= ~(1 << MOTOR_SWITCH_PIN);
}


//-------------------------------------------------------------------------------------
/** This is the function which runs when it is called by the task scheduler. It causes
 *  the motor to move back and forth, having several states to cause such motion. 
 *  @param pinnumber The output pin on the multiplexers
 *  @return Nothing
 */

void motor::output (unsigned char motornumber, unsigned char charout)
{
	motornum = motornumber;
	*p_serial_comp << "Select motor " << ascii << motornum << dec << endl;
	output_value = charout;
	
	if (motornum <= 10)
	{
		p_slave_chooser->choose(motornum);
		*p_serial_slave << output_value;
	}
	else if (motornum == 11)
	{
		if(output_value == 1)
		{
			MOTOR_SWITCH_PORT |= (1 << MOTOR_SWITCH_PIN);
		}
		else if(output_value == 0)
		{
			MOTOR_SWITCH_PORT &= ~(1 << MOTOR_SWITCH_PIN);
		}
	}
	else if (motornum == 12)
	{
		p_servo_top->output(output_value);
	}
	else if (motornum == 13)
	{
		p_servo_bottom->output(output_value);
	}
}

