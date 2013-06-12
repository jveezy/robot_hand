//*************************************************************************************
/** \file character.h
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

#ifndef _CHARACTER_H_
#define _CHARACTER_H_


//-------------------------------------------------------------------------------------
/** This class contains a task which moves a motorized lever back and forth. 
 *  WARNING:  This task uses an older version of parent class stl_task, and its 
 *            constructor parameters are out of date. Use it as an example, but
 *            do not attempt to just copy the parameters. 
 */

class character
{
	protected:
		
		unsigned char letter;	///< Matrix that stores the number of gestures required for each letter
		unsigned char steps;
		
		unsigned char pinky[4];
		unsigned char ring[4];
		unsigned char middle[4];
		unsigned char index[4];
		unsigned char index_spread[4];
		unsigned char thumb[4];
		unsigned char wrist_top[4];
		unsigned char wrist_bottom[4];

	public:
		// The constructor creates a new task object
		character (void);
		
		// Sets the letter and number of steps
		void set_letter_and_steps(unsigned char, unsigned char);

		// The run method is where the task actually performs its function
		void set_config (unsigned char, unsigned char, unsigned char, unsigned char, unsigned char, 
						 unsigned char, unsigned char, unsigned char, unsigned char);
		
		// Retrieve letter
		unsigned char get_letter(void);
		
		// Get number of steps
		unsigned char get_steps(void);
		
		// Get finger configuration number
		unsigned char get_config(unsigned char, unsigned char);

};

#endif
