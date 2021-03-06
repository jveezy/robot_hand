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

//#include "motor.h"
#include "servo.h"

#ifndef	_TASK_OUTPUT_H_
#define	_TASK_OUTPUT_H_

#define MAX_SENTENCE_SIZE	255

#define KEY_ESCAPE			0x1B
#define KEY_ENTER			0x0D
#define KEY_BACKSPACE		0x08

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
		//motor*				p_motors;			///< Pointer to all the motors
		servo*				p_servo_top;			///< Pointer to the top servo
		servo*				p_servo_bottom;			///< Pointer to the bottom servo
		                                                                               
		
		unsigned char		finger_configuration[8];
		unsigned char		output[14];
		bool				flag_output_change;
		unsigned char		input_character;
		unsigned char		character_to_output;
		unsigned char		motor_to_stop;
		unsigned char		motor_to_start;
		unsigned char		motor_to_init;
		bool				flag_interference_thumb;
		bool				flag_interference_index;
		bool				flag_interference_middle;
		bool				flag_interference_ring;
		bool				flag_interference_pinky;
		bool				flag_motors_enabled;
		bool				flag_ready_to_output;
		bool				flag_stop_motors;
		bool				flag_start_motors;
		bool				flag_init_motors;
		unsigned char		character_step;
		unsigned char		i;

	public:
		// The constructor creates a new task object
		task_output (task_timer&, time_stamp&, base_text_serial*, base_text_serial*, slave_picker*, servo*, servo*);

		// The run method is where the task actually performs its function
		char run (char);

		void set_new_character(unsigned char);
		
		void stop_motor (void);
		void start_motor (void);
		bool motors_enabled(void);
		bool query_motor (unsigned char);
		void init_motor (void);
		//void set_motor (unsigned char);
		void output_to_motor(unsigned char, unsigned char);
		bool ready_to_output(void);
		
		void open_thumb(void);
		void open_index(void);
		void open_middle(void);
		void open_ring(void);
		void open_pinky(void);
		
		void thumb_flat_up(void);
		void thumb_fold_up(void);
		void thumb_fold_in(void);
		void thumb_fold_out(void);
		void thumb_stretch(void);
		void thumb_curl(void);
		
		void index_stretch(void);
		void index_curl(void);
		void index_clench(void);
		void index_vert_clench(void);
		void index_cross(void);
		void index_u(void);
		void index_fold(void);
		
		void middle_stretch(void);
		void middle_curl(void);
		void middle_clench(void);
		void middle_vert_clench(void);
		void middle_fold(void);
		
		void ring_stretch(void);
		void ring_curl(void);
		void ring_clench(void);
		
		void pinky_stretch(void);
		void pinky_curl(void);
		void pinky_clench(void);
		
		void wrist_default(void);
		void wrist_bent(void);
		void wrist_bent_and_twisted(void);
		void wrist_twisted(void);
		void wrist_z1(void);
		void wrist_z2(void);
		void wrist_z3(void);
};

#endif
