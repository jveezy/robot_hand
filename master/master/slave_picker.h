//*************************************************************************************
/** \file slave_picker.h
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

#include "lib/base_text_serial.h"

#ifndef _SLAVE_PICKER_H_
#define _SLAVE_PICKER_H_

//-------------------------------------------------------------------------------------
/** This class contains a task which moves a motorized lever back and forth. 
 *  WARNING:  This task uses an older version of parent class stl_task, and its 
 *            constructor parameters are out of date. Use it as an example, but
 *            do not attempt to just copy the parameters. 
 */

class slave_picker
{
	protected:
		unsigned char		pinarray[4];	// Create pin array
		base_text_serial* 	p_serial_comp;			///< Pointer to serial device for computer

	public:
		// The constructor creates a new task object
		slave_picker (base_text_serial*);

		// The run method is where the task actually performs its function
		void choose (unsigned char);
};

#endif
