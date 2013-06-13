//*************************************************************************************
/** \file task_output.h
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

#include "motor.h"

#ifndef	_TASK_OUTPUT_H_
#define	_TASK_OUTPUT_H_

#define MAX_SENTENCE_SIZE	255


#define KEY_ESCAPE			0x1B
#define KEY_ENTER			0x0D
#define KEY_BACKSPACE		0x08

/*#define STRAIGHT			0
#define CURL				1
#define CLENCH				2
#define VERTICAL_CLENCH		3
#define HORIZONTAL_STRAIGHT	4
#define SLANTED_STRAIGHT	5

#define FLAT_SIDE			0
#define	OUT_SIDE			1
#define FOLD_FLAT			2
#define FOLD_STRAIGHT_OUT	3
#define FOLD_STRAIGHT_UP	4
#define THUMB_CURL			5*/

//-------------------------------------------------------------------------------------
/** This class contains a task which moves a motorized lever back and forth. 
 *  WARNING:  This task uses an older version of parent class stl_task, and its 
 *            constructor parameters are out of date. Use it as an example, but
 *            do not attempt to just copy the parameters. 
 */

class task_output : public stl_task
{
	protected:

		base_text_serial* 	p_serial_comp;			///< Pointer to serial device for computer
		base_text_serial* 	p_serial_slave;			///< Pointer to serial device for slave
		slave_picker* 		p_slave_chooser;		///< Pointer to slave picker for mux pins
		motor*				p_motors;				///< Pointer to all the motors
		                                                                               
		
		unsigned char		finger_configuration[8];
		unsigned char		output[14];
		bool				flag_output_change;
		unsigned char		input_character;
		unsigned char		character_to_output;
		bool				flag_interference_thumb;
		bool				flag_interference_index;
		bool				flag_interference_middle;
		bool				flag_interference_ring;
		bool				flag_interference_pinky;
		unsigned char		character_step;

	public:
		// The constructor creates a new task object
		task_output (task_timer&, time_stamp&, base_text_serial*, base_text_serial*, slave_picker*, motor*);

		// The run method is where the task actually performs its function
		char run (char);
		
		// Change Output Values
		void change_output(unsigned char, unsigned char);

		void set_new_character(unsigned char);
		
		void stop_motor (unsigned char);
		void start_motor (unsigned char);
		bool query_motor (unsigned char);
		void init_motor (unsigned char);
		void set_motor (unsigned char);
};

#endif
