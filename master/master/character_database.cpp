//*************************************************************************************
/** \file character_database.cpp
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
#include "character.h"
#include "character_database.h"

//-------------------------------------------------------------------------------------
/** This constructor creates a slave_picker object. It outputs the correct pins to
 *  choose multiplexer outputs to make sure the master is communicating with the right
 *  slave chips.
 */

character_database::character_database ()
{
	character_array = new character[36];
	
	create_all_characters();
}

void character_database::create_all_characters(void)
{
	// Create characters
			character_array[0].set_letter_and_steps('0',1);
			character_array[1].set_letter_and_steps('1',1);
			character_array[2].set_letter_and_steps('2',1);
			character_array[3].set_letter_and_steps('3',1);
			character_array[4].set_letter_and_steps('4',2);
			character_array[5].set_letter_and_steps('5',1);
			character_array[6].set_letter_and_steps('6',2);
			character_array[7].set_letter_and_steps('7',2);
			character_array[8].set_letter_and_steps('8',2);
			character_array[9].set_letter_and_steps('9',1);
			character_array[10].set_letter_and_steps('A',1);
			character_array[11].set_letter_and_steps('B',2);
			character_array[12].set_letter_and_steps('C',1);
			character_array[13].set_letter_and_steps('D',1);
			character_array[14].set_letter_and_steps('E',2);
			character_array[15].set_letter_and_steps('F',1);
			character_array[16].set_letter_and_steps('G',1);
			character_array[17].set_letter_and_steps('H',1);
			character_array[18].set_letter_and_steps('I',1);
			character_array[19].set_letter_and_steps('J',4);
			character_array[20].set_letter_and_steps('K',2);
			character_array[21].set_letter_and_steps('L',1);
			character_array[22].set_letter_and_steps('M',2);
			character_array[23].set_letter_and_steps('N',2);
			character_array[24].set_letter_and_steps('O',1);
			character_array[25].set_letter_and_steps('P',1);
			character_array[26].set_letter_and_steps('Q',1);
			character_array[27].set_letter_and_steps('R',2);
			character_array[28].set_letter_and_steps('S',2);
			character_array[29].set_letter_and_steps('T',3);
			character_array[30].set_letter_and_steps('U',1);
			character_array[31].set_letter_and_steps('V',1);
			character_array[32].set_letter_and_steps('W',1);
			character_array[33].set_letter_and_steps('X',1);
			character_array[34].set_letter_and_steps('Y',1);
			character_array[35].set_letter_and_steps('Z',4);
	// Populate with correct values
			character_array[0].set_config(0,1,1,1,1,0,5,0,0);
			character_array[1].set_config(0,2,2,2,0,0,0,0,0);
			character_array[2].set_config(0,2,2,0,2,1,0,0,0);
			character_array[3].set_config(0,2,2,0,0,1,0,0,0);
			character_array[4].set_config(0,0,0,0,0,0,3,0,0);
			character_array[4].set_config(1,0,0,0,0,0,2,0,0);
			character_array[5].set_config(0,0,0,0,0,0,0,0,0);
			character_array[6].set_config(0,2,0,0,0,0,3,0,0);
			character_array[6].set_config(1,2,0,0,0,0,2,0,0);
			character_array[7].set_config(0,0,2,0,0,1,3,0,0);
			character_array[7].set_config(1,0,2,0,0,1,3,0,0);
			character_array[8].set_config(0,0,0,2,0,0,3,0,0);
			character_array[8].set_config(1,0,0,2,0,0,2,0,0);
			character_array[9].set_config(0,0,0,0,2,0,0,0,0);
			character_array[10].set_config(0,2,2,2,2,0,0,0,0);
			character_array[11].set_config(0,0,0,0,0,0,3,0,0);
			character_array[11].set_config(1,0,0,0,0,0,2,0,0);
			character_array[12].set_config(0,1,1,1,1,0,3,0,0);
			character_array[13].set_config(0,1,1,1,1,0,3,0,0);
			character_array[14].set_config(0,1,1,1,1,0,3,0,0);
			character_array[14].set_config(1,1,1,1,1,0,2,0,0);
			character_array[15].set_config(0,0,0,0,3,0,0,0,0);
			character_array[16].set_config(0,2,2,2,4,0,0,0,0);
			character_array[17].set_config(0,2,2,4,4,0,0,0,0);
			character_array[18].set_config(0,0,2,2,2,0,0,0,0);
			character_array[19].set_config(0,0,2,2,2,0,0,0,0);
			character_array[19].set_config(1,0,2,2,2,0,0,90,0);
			character_array[19].set_config(2,0,2,2,2,0,0,90,45);
			character_array[19].set_config(3,0,2,2,2,0,0,0,45);
			character_array[20].set_config(0,2,2,0,0,1,0,0,0);
			character_array[20].set_config(1,2,2,0,0,1,4,0,0);
			character_array[21].set_config(0,2,2,2,0,0,1,0,0);
			character_array[22].set_config(0,2,1,3,3,0,3,90,0);
			character_array[22].set_config(1,2,1,3,3,0,2,90,0);
			character_array[23].set_config(0,2,2,3,3,0,3,90,0);
			character_array[23].set_config(1,2,2,3,3,0,2,90,0);
			character_array[24].set_config(0,1,1,1,1,0,3,0,0);
			character_array[25].set_config(0,2,2,4,0,0,3,0,0);
			character_array[26].set_config(0,2,2,2,4,0,3,0,0);
			character_array[27].set_config(0,2,2,0,5,0,0,0,0);
			character_array[27].set_config(0,2,2,0,5,0,0,0,0);
			character_array[28].set_config(0,2,2,2,2,0,1,0,0);
			character_array[28].set_config(0,2,2,2,2,0,2,0,0);
			character_array[29].set_config(0,2,2,2,0,1,0,0,0);
			character_array[29].set_config(0,2,2,2,0,1,4,0,0);
			character_array[29].set_config(0,2,2,2,3,1,4,0,0);
			character_array[30].set_config(0,2,2,0,3,0,0,0,0);
			character_array[31].set_config(0,2,2,0,0,1,0,0,0);
			character_array[32].set_config(0,2,0,0,0,0,0,0,0);
			character_array[33].set_config(0,2,2,2,0,0,0,0,0);
			character_array[34].set_config(0,2,2,2,1,0,1,0,0);
			character_array[35].set_config(0,2,2,2,0,0,0,0,0);
			character_array[35].set_config(1,2,2,2,0,0,0,0,45);
			character_array[35].set_config(2,2,2,2,0,0,0,90,0);
			character_array[35].set_config(3,2,2,2,0,0,0,90,45);
}

//-------------------------------------------------------------------------------------
/** This is the function which runs when it is called by the task scheduler. It causes
 *  the motor to move back and forth, having several states to cause such motion. 
 *  @param pinnumber The output pin on the multiplexers
 *  @return Number of gestures for this character
 */

unsigned char character_database::get_index (unsigned char input_character)
{
	unsigned char index = 0;
	
	// Determine which row in the array matches the character
	if ( (input_character >= '0')&&(input_character <= '9') )	// Check if it's a number
	{
		index = input_character - 0x30;	// Subtract hex 30 to convert from ascii to decimal
	}
	else if ( (input_character >= 'A')&&(input_character <= 'Z' ) )	// Check if it's a letter
	{
		index = input_character - 'A' + 10;	// Subtract 0x41 to shift down to 0. Add 10 to align with gesture_count array.
	}
	else if (input_character == ',')
	{
		index = 36;
	}
	else if (input_character == '.')
	{
		index = 37;
	}
	else if (input_character == ' ')
	{
		index = 38;
	}

	// Grab count value using index
	return(index);
}


/** This is the function which runs when it is called by the task scheduler. It causes
 *  the motor to move back and forth, having several states to cause such motion. 
 *  @param pinnumber The output pin on the multiplexers
 *  @return Number of gestures for this character
 */

/*void character_database::initialize_gesture_count (void)
{
	// Fill in character column of gesture count array
	
			// Fill in 0-9
			for (unsigned char i = 0; i < 10; i++)
			{
				gesture_count[i][0] = 0x30 + i;	// Character 0 + i
			}
			
			// Fill in letters
			for (unsigned char i = 0; i < 26; i++)
			{
				gesture_count[i+10][0] = 0x41 + i;	// Letter A + i
			}
			
			// Comma, period, space
			gesture_count[36][0] = ',';
			gesture_count[37][0] = '.';
			gesture_count[38][0] = ' ';
	// Fill in count column of gesture count array
	
			// Fill in all 1s for default
			for (unsigned char i = 0; i < NUM_CHARACTERS; i++)
			{
				gesture_count[i][1] = 1;
			}
			
			// Add exceptions
			gesture_count[4][1] = 2;	// 4
			gesture_count[6][1] = 2;	// 6
			gesture_count[7][1] = 2;	// 7
			gesture_count[8][1] = 2;	// 8
			gesture_count[11][1] = 2;	// B
			gesture_count[14][1] = 2;	// E
			gesture_count[19][1] = 4;	// J
			gesture_count[22][1] = 2;	// M
			gesture_count[23][1] = 2;	// N
			gesture_count[27][1] = 2;	// R
			gesture_count[28][1] = 2;	// S
			gesture_count[29][1] = 3;	// T
			gesture_count[35][1] = 4;	// Z
}*/







