//*************************************************************************************
/** \file slave_picker.cpp
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
#include "slave_picker.h"
#include "lib/global_debug.h"

//-------------------------------------------------------------------------------------
/** This constructor creates a slave_picker object. It outputs the correct pins to
 *  choose multiplexer outputs to make sure the master is communicating with the right
 *  slave chips.
 */

slave_picker::slave_picker (void)
{
	// Clear array
	for(unsigned char i = 0; i < 4; i++)
	{
		pinarray[i] = 0;
	}
	
	DDRA = 0xFF;	// Set all pins in register A to be outputs
}


//-------------------------------------------------------------------------------------
/** This is the function which runs when it is called by the task scheduler. It causes
 *  the motor to move back and forth, having several states to cause such motion. 
 *  @param pinnumber The output pin on the multiplexers
 *  @return Nothing
 */

void slave_picker::choose (unsigned char pinnumber)
{
	GLOB_DEBUG (pinnumber);
	
	// Split number into individual bits
	for(unsigned char i = 0; i < 4; i++)
	{
		pinarray[i] = (0b00000001 << i) & pinnumber;
	}
	
	// Output to pins
	if(pinarray[0])
	{
		PORTA |= (1 << PINA0);
		PORTA |= (1 << PINA4);
	}
	else
	{
		PORTA &= ~(1 << PINA0);
		PORTA &= ~(1 << PINA4);
	}
	
	if(pinarray[1])
	{
		PORTA |= (1 << PINA1);
		PORTA |= (1 << PINA5);
	}
	else
	{
		PORTA &= ~(1 << PINA1);
		PORTA &= ~(1 << PINA5);
	}
	
	if(pinarray[2])
	{
		PORTA |= (1 << PINA2);
		PORTA |= (1 << PINA6);
	}
	else
	{
		PORTA &= ~(1 << PINA2);
		PORTA &= ~(1 << PINA6);
	}
	
	if(pinarray[3])
	{
		PORTA |= (1 << PINA3);
		PORTA |= (1 << PINA7);
	}
	else
	{
		PORTA &= ~(1 << PINA3);
		PORTA &= ~(1 << PINA7);
	}

}

