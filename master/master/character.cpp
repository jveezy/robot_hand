//*************************************************************************************
/** \file character.cpp
 *    This file contains a character class for running a user interface for the motor
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
#include "character.h"

//-------------------------------------------------------------------------------------
/** This constructor creates a slave_picker object. It outputs the correct pins to
 *  choose multiplexer outputs to make sure the master is communicating with the right
 *  slave chips.
 */

character::character (void)
{	
	// Initialize all configurations to zero
	for(unsigned char i = 0; i < 4; i++)
	{
		pinky[i] = 0;
		ring[i] = 0;
		middle[i] = 0;
		index[i] = 0;
		index_spread[i] = 0;
		thumb[i] = 0;
		wrist_top[i] = 0;
		wrist_bottom[i] = 0;
	}
}

void character::set_letter_and_steps (unsigned char input_letter, unsigned char num_steps)
{
	letter = input_letter;
	steps = num_steps;
}

/** This is the function which runs when it is called by the task scheduler. It causes
 *  the motor to move back and forth, having several states to cause such motion. 
 *  @param pinnumber The output pin on the multiplexers
 *  @return Number of gestures for this character
 */

void character::set_config(unsigned char step, unsigned char pinky_conf, unsigned char ring_conf, unsigned char middle_conf, 
						   unsigned char index_conf, unsigned char index_spread_conf, unsigned char thumb_conf, 
						   unsigned char wrist_top_conf, unsigned char wrist_bottom_conf)
{
	pinky[step] = pinky_conf;
	ring[step] = ring_conf;
	middle[step] = middle_conf;
	index[step] = index_conf;
	index_spread[step] = index_spread_conf;
	thumb[step] = thumb_conf;
	wrist_top[step] = wrist_top_conf;
	wrist_bottom[step] = wrist_bottom_conf;
}

/** This is the function which runs when it is called by the task scheduler. It causes
 *  the motor to move back and forth, having several states to cause such motion. 
 *  @param pinnumber The output pin on the multiplexers
 *  @return Number of gestures for this character
 */

unsigned char character::get_letter(void)
{
	return(letter);
}

/** This is the function which runs when it is called by the task scheduler. It causes
 *  the motor to move back and forth, having several states to cause such motion. 
 *  @param pinnumber The output pin on the multiplexers
 *  @return Number of gestures for this character
 */

unsigned char character::get_steps(void)
{
	return(steps);
}

/** This is the function which runs when it is called by the task scheduler. It causes
 *  the motor to move back and forth, having several states to cause such motion. 
 *  @param pinnumber The output pin on the multiplexers
 *  @return Number of gestures for this character
 */

unsigned char character::get_config(unsigned char finger, unsigned char step)
{
	switch(finger)
	{
		case(0):	// Pinky
			return(pinky[step]);
			break;
		case(1):	// Ring
			return(ring[step]);
			break;
		case(2):	// Middle
			return(middle[step]);
			break;
		case(3):	// Index
			return(index[step]);
			break;
		case(4):	// Index spread
			return(index_spread[step]);
			break;
		case(5):	// Thumb
			return(thumb[step]);
			break;
		case(6):	// Wrist Top
			return(wrist_top[step]);
			break;
		case(7):	// Wrist Bottom
			return(wrist_bottom[step]);
			break;
	}
}

