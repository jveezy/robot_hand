//*************************************************************************************
/** \file task_user.h
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


#include "lib/stl_timer.h"

#ifndef	_TASK_USER_H_
#define	_TASK_USER_H_

#define MAX_SENTENCE_SIZE	255

//-------------------------------------------------------------------------------------
/** This class contains a task which moves a motorized lever back and forth. 
 *  WARNING:  This task uses an older version of parent class stl_task, and its 
 *            constructor parameters are out of date. Use it as an example, but
 *            do not attempt to just copy the parameters. 
 */

class task_user : public stl_task
{
	protected:

		base_text_serial* 	p_serial_comp;			///< Pointer to serial device for computer
		base_text_serial* 	p_serial_slave;			///< Pointer to serial device for slave
		slave_picker* 		p_slave_chooser;		///< Pointer to slave picker for mux pins
		character_database* p_character_database;	///< Pointer to the character database
		task_output*		p_task_output;			///< Pointer to the output task
		
		unsigned char 		input_character;		///< Input character from serial device
		bool 				flag_message_printed;	///< Boolean to prevent messages from being printed repeatedly
		char				backspace;				///< Backspace variable
		unsigned char		index;					///< Index for character array
		unsigned char		character_to_output;	///< Placeholder for character to output
		unsigned char		character_to_test;		///< Placeholder for character to test
		unsigned char		steps;					///< Number of steps in a character
		unsigned char		current_step;			///< Current gesture output step
		unsigned char		output_delay;			///< Number of times to skip displaying a character
		unsigned char		current_delay;			///< Remaining number of delay counts
		unsigned char		output_configuration;	///< Finger configuration to output to output task
		
		bool				flag_period;			///< Flag to indicate a period character
		bool				flag_comma;				///< Flag to indicate a comma character
		bool				flag_space;				///< Flag to indicate a space character
		bool				flag_delay_countdown;	///< Flag to indicate the output delay countdown has begun
		
		queue<char, unsigned char, MAX_SENTENCE_SIZE> character_buffer;	///< Character buffer
		
		unsigned char		i_motor;
		

	public:
		// The constructor creates a new task object
		task_user (task_timer&, time_stamp&, base_text_serial*, base_text_serial*, slave_picker*, character_database*, task_output*);

		// The run method is where the task actually performs its function
		char run (char);
		
		void stop_motor (unsigned char);
		void start_motor (unsigned char);
		bool query_motor (unsigned char);
		void init_motor (unsigned char);
		void set_motor (unsigned char);

};

#endif
