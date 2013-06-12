//*************************************************************************************
/** \file task_user.cpp
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
#include "lib/rs232int.h"
#include "lib/stl_timer.h"
#include "lib/stl_task.h"
#include "slave_picker.h"			// The class that sets the multiplexer pins
#include "lib/queue.h"
#include "character.h"				// The class that stores character info
#include "character_database.h"		
#include "task_output.h"
#include "task_user.h"
#include "lib/global_debug.h"


//-------------------------------------------------------------------------------------
/** This constructor creates a user interface task object. It checks if the user has
 *  typed a meaningful command at the serial port and if so tells the motor controller
 *  what to do. 
 *  @param a_timer   A reference to the real-time measuring timer for tasks
 *  @param t_stamp   A timestamp which contains the time between runs of this task
 *  @param p_mo_task A pointer to a motor control task to be ordered around
 *  @param p_timer   A pointer to the main real-time clock object in use
 *  @param p_a_to_d  A pointer to the A/D converter which measures voltages
 *  @param p_ser	 A pointer to a serial device for sending and receiving messages
 */

task_user::task_user (task_timer& a_timer, time_stamp& t_stamp, base_text_serial* p_ser_comp, 
					  base_text_serial* p_ser_slave, slave_picker* p_slave_picker, 
					  character_database* p_char_dbase, task_output* p_output_task ) 
	: stl_task (a_timer, t_stamp)
{
	flag_message_printed = false;	// Clear message_printed flag
	
	// Assign pointers
	p_serial_comp = p_ser_comp;
	p_serial_slave = p_ser_slave;
	p_slave_chooser = p_slave_picker;
	p_character_database = p_char_dbase;
	p_task_output = p_output_task;
	
	character_buffer.flush();	// Flush character buffer
	
	backspace = 0x08;			// Backspace character for printing
	
}

//-------------------------------------------------------------------------------------
/** This is the function which runs when it is called by the task scheduler. It causes
 *  the motor to move back and forth, having several states to cause such motion. 
 *  @param state The state of the task when this run method begins running
 *  @return The state to which the task will transition, or STL_NO_TRANSITION if no
 *	  transition is called for at this time
 */

char task_user::run (char state)
{
	switch(state)
	{
		// Home screen
		case(0):
				if(!flag_message_printed)
				{
					*p_serial_comp << endl << endl << "Robotic Fingerspelling Hand" << endl << endl;
					*p_serial_comp << endl << "ESC Stop Motors" << endl << "C   Calibrate" << endl << "ENT Enter Sentence" << endl;
					flag_message_printed = true;
				}
				if(p_serial_comp->check_for_char())
				{
					flag_message_printed = false;
					input_character = p_serial_comp->getchar();
					switch(input_character)
					{
						case(0x1B):		// Escape
							return(1);	// Go to state 1 (stop motors)
							break;
						case('c'):		// Capital or lowercase C entered
						case('C'):
							return(2);	// Go to state 2 (calibration menu)
							break;
						case(0x0D):		// Enter
							return(4);	// Go to state 4 (enter sentence)
							break;
						default:
							break;
					}
				}
				return(STL_NO_TRANSITION);
				break;
		// Stop motors
		case(1):
				for(i_motor = 1; i_motor < 11; i_motor++)
				{
					stop_motor(i_motor);
				}
				*p_serial_comp << endl << "State 1: Motors Stopped" << endl;
				return(0);	// return to state 0 (home screen)
				break;
		// Print calibration messages
		case(2):
				*p_serial_comp << endl << "Calibrate which motor?" << endl << endl << "1 - M1" << endl << "2 - M2"
				<< endl << "3 - M3" << endl << "4 - M4" << endl << "5 - M5" << endl << "6 - M6" << endl << "7 - M7"
				<< endl << "8 - M8" << endl << "9 - M9" << endl << "0 - M10" << endl << "ESC Cancel" << endl;
				return(3);	// return to state 0 (home screen)
				break;
		// Perform calibration
		case(3):
				if(p_serial_comp->check_for_char())
				{
					input_character = p_serial_comp->getchar();		// Collect character
					
					if( (input_character >= 0x31) && (input_character <= 0x39) )
					{
						// Subtract 0x30 from input character to get decimal value
						input_character = input_character - 0x31;
						
						// Set multiplexer to the proper pin
						p_slave_chooser->choose(input_character);
						
						// Output backspace character to clear encoder count in slave
						if(p_serial_slave->ready_to_send())
						{
							*p_serial_slave << 0x1B;
						}
						return(2);		// Go back to state 2 (calibrate menu)
					}
					else if ( input_character == '0' )
					{
						// Choose multiplexer channel 10-1
						p_slave_chooser->choose(10);
						
						// Output backspace character to clear encoder count in slave
						if(p_serial_slave->ready_to_send())
						{
							*p_serial_slave << 'c';
						}
						return(2);		// Go back to state 2 (calibrate menu)
					}
					else if (input_character == 0x1B)	// Escape
					{
						return(0);		// Go back to state 0 (home screen)
					}
					else
					{			break;
						*p_serial_comp << endl << "Invalid character" << endl;
						return(2);	// Reprint message and remain in loop until valid character received
					}
				}
				break;
		// Collect characters
		case(4):
				// Print instructions and flush the character buffer the first time through.
				if(!flag_message_printed)
				{
					*p_serial_comp << endl << "Input sentence. Letters, numbers, commas, periods, and spaces only. 255 characters max."
					<< endl << "Enter when done. Escape to quit." << endl << "> ";
					flag_message_printed = true;
					if(!character_buffer.is_empty())	// If character buffer is NOT empty
					{
						character_buffer.flush();		// flush character buffer
					}
				}
				
				// Collect characters
				if(p_serial_comp->check_for_char())		// Check if character received
				{
					input_character = p_serial_comp->getchar();		// collect character if so.
					
					// Process character
					
					// Is it a number (between hex 30 and 39), capital letter (between hex 41 and 5A), space, comma, or period?
					// If so it can be stored directly
					if( ( (input_character >= '0')&&(input_character <= '9') )||( (input_character >= 'A')&&(input_character <= 'Z' ) )||(input_character == ' ')||(input_character == ',')||(input_character == '.') )	
					{
						if (character_buffer.num_items() <= MAX_SENTENCE_SIZE )	// If it is and there's room in the character buffer
						{
							*p_serial_comp << ascii << input_character;						// Echo character to the screen
							character_buffer.put(input_character);					// store it directly into the character buffer
						}
						else														// If the character buffer is full, scream at the user.
						{
							*p_serial_comp << endl << "TOO MANY CHARACTERS" << endl;
						}
					}
					// If not a number or capital letter, is it a lowercase letter (between hex 61 and 7A)?
					else if ( (input_character >= 'a')&&(input_character <= 'z') )
					{
						if (character_buffer.num_items() <= MAX_SENTENCE_SIZE )	// If it is and there's room in the character buffer
						{
							
							input_character = input_character - ('a' - 'A');		// convert it to a capital letter first.
							character_buffer.put(input_character);					// Store it in the character buffer
							*p_serial_comp << ascii << input_character;						// Echo character to the screen
						}
						else														// If the character buffer is full, scream at the user.
						{
							*p_serial_comp << endl << "TOO MANY CHARACTERS" << endl;
						}
					}
					// If backspace (hex 08)
					else if (input_character == 0x08)
					{
						*p_serial_comp << ascii << backspace << ' ' << backspace;		// Backspace, space, backspace to step back, erase, and move the cursor back.
						character_buffer.delete_one();				// Delete the last character stored in the buffer.
					}
					// If question mark or exclamation point, store as period.
					else if ( (input_character == '?')||(input_character == '!') )
					{
						*p_serial_comp << ascii << input_character;						// Echo character to the screen
						input_character = '.';									// convert it to a period first
						character_buffer.put(input_character);					// Store it in the character buffer
					}
					// If Enter is pressed, user is done.
					else if (input_character == 0x0D)
					{
						*p_serial_comp << endl << "Parsing sentence." << endl;
						flag_message_printed = false;
						return(5);	// Go to state 5
					}
					// If any other characters are pressed
					else
					{
						// Don't echo or store anything
						return(STL_NO_TRANSITION);
					}
					
				}
				
				return(STL_NO_TRANSITION);	// Don't leave the state until Enter is pressed.
				break;
		// Prepare one character for output
		case(5):
				/* *p_serial_comp << endl;
				for (unsigned char i = 0; i < character_buffer.num_items(); i++)
				{
					*p_serial_comp << character_buffer[i];
					*p_serial_comp << ascii << p_character_database->character_array[p_character_database->get_index(character_buffer[i])].get_steps();
					*p_serial_comp << " ";
				}*/
				if(character_buffer.num_items())	// If character buffer is not empty
				{
					character_to_output = character_buffer.get();						// Retrieve the character
					
					// If it's not a pause character, collect information.
					if ((character_to_output != '.')||(character_to_output != ',')||(character_to_output != ' '))
					{
						index = p_character_database -> get_index(character_to_output);		// Locate the character in the array
						steps = p_character_database -> character_array[index].get_steps();	// Get the number of steps from the character database
					}
					
					// If it's a pause character, set the proper flag.
					else
					{
						switch(character_to_output)
						{
							case('.'):
								flag_period = true;
								break;
							case(','):
								flag_comma = true;
								break;
							case(' '):
								flag_space = true;
								break;
							default:
								break;
						}
					}
					return(6);															// Now go to state 6 to figure out timing
				}
				else
				{
					return(9);	// Printing is done. Go to state 9 to print the end message.
				}
				break;
		// Timing calculations
		case(6):
				if(flag_comma || flag_space || flag_period )
				{
					if(flag_comma)
					{
						output_delay = 60;
					}
					else if(flag_space)
					{
						output_delay = 40;
					}
					else
					{
						output_delay = 80;
					}
				}
				else
				{
					output_delay = 40 / steps;
				}
				current_delay = 0;
				current_step = 0;
				return(7);	// Go to state 7 to start outputting
				break;
		// Delay
		case(7):
				if(current_delay == output_delay)	// Wait until delay counter increments up to desired value
				{
					current_delay = 0;			// Clear delay count
					return(8);					// Now go to state 8 to output data
				}
				else
				{
					current_delay++;
					return(STL_NO_TRANSITION);	// Keep delaying until ready
				}
				break;
		// Retrieve and set gesture output values	
		case(8):
				// Pinky
				output_configuration = p_character_database -> character_array[index].get_config(0,current_step);	// Retrieve from database
				p_task_output -> change_output (0 , output_configuration);					// Send to output task
				
				// Ring
				output_configuration = p_character_database -> character_array[index].get_config(1,current_step);	// Retrieve from database
				p_task_output -> change_output (1 , output_configuration);					// Send to output task
				
				// Middle
				output_configuration = p_character_database -> character_array[index].get_config(2,current_step);	// Retrieve from database
				p_task_output -> change_output (2 , output_configuration);					// Send to output task
				
				// Index
				output_configuration = p_character_database -> character_array[index].get_config(3,current_step);	// Retrieve from database
				p_task_output -> change_output (3 , output_configuration);					// Send to output task
				
				// Index spread
				output_configuration = p_character_database -> character_array[index].get_config(4,current_step);	// Retrieve from database
				p_task_output -> change_output (4 , output_configuration);					// Send to output task
				
				// Thumb
				output_configuration = p_character_database -> character_array[index].get_config(5,current_step);	// Retrieve from database
				p_task_output -> change_output (5 , output_configuration);					// Send to output task
				
				// Wrist extension/flexion
				output_configuration = p_character_database -> character_array[index].get_config(6,current_step);	// Retrieve from database
				p_task_output -> change_output (6 , output_configuration);					// Send to output task
				
				// Wrist supination/pronation
				output_configuration = p_character_database -> character_array[index].get_config(7,current_step);	// Retrieve from database
				p_task_output -> change_output (7 , output_configuration);					// Send to output task
				
				*p_serial_comp << ascii << endl << "Letter " << character_to_output << " step " << (current_step+0x30) << endl;
				
				current_step++;				// Increment step count now that this step is done.
				
				if(current_step == steps)	// If current step = step
				{
					return(5);	// All steps for this character are done. Go back to state 5 to get the next character.
				}
				else
				{
					return(7);	// If not. Go back to the delay state.
				}
				break;
		// Done
		case(9):
				*p_serial_comp << endl << "Message done. Returning to message prompt." << endl;
				return(4);	// Return to message prompt
				break;
			
	}
	// If we get here, no transition is called for
	return (STL_NO_TRANSITION);
}

void task_user::stop_motor(unsigned char motornum)
{
	p_slave_chooser->choose(motornum);
	if(p_serial_slave->ready_to_send())
	{
		*p_serial_slave << "S";
		input_character = p_serial_comp->getchar();		// Wait for response
		if (input_character != 's')
		{
			*p_serial_comp << endl << "Motor stop error " << motornum << endl;
		}
	}
}

void task_user::start_motor(unsigned char motornum)
{
	p_slave_chooser->choose(motornum);
	if(p_serial_slave->ready_to_send())
	{
		*p_serial_slave << "G";
		input_character = p_serial_comp->getchar();		// Wait for response
		if (input_character != 'g')
		{
			*p_serial_comp << endl << "Motor enable error " << motornum << endl;
		}
	}
} 

bool task_user::query_motor(unsigned char motornum)
{
	p_slave_chooser->choose(motornum);
	if(p_serial_slave->ready_to_send())
	{
		*p_serial_slave << "Q";
		input_character = p_serial_comp->getchar();		// Wait for response
		if (input_character == 'Q')
		{
			return(false);
		}
		else if (input_character == 'q')
		{
			return(true);
		}
		else
			*p_serial_comp << endl << "Motor query error " << motornum << endl;
	}
}

void task_user::init_motor(unsigned char motornum)
{
	p_slave_chooser->choose(motornum);
	if(p_serial_slave->ready_to_send())
	{
		if (motornum > 0 && motornum < 10)
			character_to_output = motornum + 0x30;
		else if (motornum == 10)
			character_to_output = '0';
		else
			*p_serial_comp << endl << "Motor conf error " << motornum << endl;
		*p_serial_slave << character_to_output;
		input_character = p_serial_comp->getchar();		// Wait for response
		if (input_character != '!')
		{
			*p_serial_comp << endl << "Motor conf error " << motornum << endl;
		}
	}
}

void task_user::set_motor(unsigned char motornum, unsigned char setpoint)
{
	p_slave_chooser->choose(motornum);
	if(p_serial_slave->ready_to_send())
	{
		*p_serial_slave << setpoint;
		input_character = p_serial_comp->getchar();		// Wait for response
		if (input_character != 'A')
		{
			*p_serial_comp << endl << "Motor set error " << motornum << endl;
		}
	}
	
}
