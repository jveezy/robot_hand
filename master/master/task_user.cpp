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
					  task_output* p_output_task ) 
	: stl_task (a_timer, t_stamp)
{
	flag_message_printed = false;	// Clear message_printed flag
	
	// Assign pointers
	p_serial_comp = p_ser_comp;
	p_serial_slave = p_ser_slave;
	p_slave_chooser = p_slave_picker;
	//p_character_database = p_char_dbase;
	p_task_output = p_output_task;
	
	character_buffer.flush();	// Flush character buffer
	
	backspace = 0x08;			// Backspace character for printing
	
	*p_serial_comp << endl << "User task initialized" << endl;
	
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
	//*p_serial_comp << endl << "Use State " << state << endl;
	switch(state)
	{
		// Home screen
		case(0):
			if(!flag_message_printed)
			{
				*p_serial_comp << endl << endl << "Robotic Fingerspelling Hand" << endl << endl;
				*p_serial_comp <<	endl << "ESC Stop Motors" << 
									endl << "C   Calibrate" << 
									endl << "ENT Enter Sentence" << 
									endl << "E   Encoder Query" << 
									endl << "M   Manual Mode" << endl ;
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
					case('E'):
					case('e'):
						return(10);	// Go to state 10 (Encoder query)
						break;
					case('M'):
					case('m'):
						return(13);	// Go to state 13 (Manual mode)
						break;
					default:
						*p_serial_comp << endl << "Invalid command" << endl;
						break;
				}
			}
			//*p_serial_comp << endl << "User task state 0" << endl;
			return(STL_NO_TRANSITION);
			break;
		// Stop motors
		case(1):
			*p_serial_comp << endl << "Sending stop command" << endl;
			p_task_output->stop_motor();
			return(0);	// return to state 0 (home screen)
			break;
		// Print calibration messages
		case(2):
			*p_serial_comp << endl << "Calibrate which motor?" << endl << endl << "1 - M1" << endl << "2 - M2"
			<< endl << "3 - M3" << endl << "4 - M4" << endl << "5 - M5" << endl << "6 - M6" << endl << "7 - M7"
			<< endl << "8 - M8" << endl << "9 - M9" << endl << "0 - M10" << endl << "ESC Cancel" << endl;
			return(3);	// Process input in state 3
			break;
		// Perform calibration
		case(3):
			if(p_serial_comp->check_for_char())
			{
				input_character = p_serial_comp->getchar();		// Collect character
				
				// Clear character buffer
				while(p_serial_slave ->check_for_char())
				{
					p_serial_slave->getchar();
				}
				
				if( (input_character >= 0x31) && (input_character <= 0x39) )
				{
					// Subtract 0x30 from input character to get decimal value
					input_character = input_character - 0x30;
						
					// Set multiplexer to the proper pin
					p_slave_chooser->choose(input_character);
						
					// Output C character to clear encoder count in slave
					if(p_serial_slave->ready_to_send())
					{
						*p_serial_slave << 'C';
					}
					return(16);		// Wait for response in state 16
				}
				else if ( input_character == '0' )
				{
					// Choose multiplexer channel 10
					p_slave_chooser->choose(10);
						
					// Output C character to clear encoder count in slave
					if(p_serial_slave->ready_to_send())
					{
						*p_serial_slave << 'C';
					}
					return(16);		// Wait for response in state 16
				}
				else if (input_character == 0x1B)	// Escape
				{
					return(0);		// Go back to state 0 (home screen)
				}
				else
				{
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
						*p_serial_comp << ascii << input_character << numeric;	// Echo character to the screen
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
						*p_serial_comp << ascii << input_character << numeric;	// Echo character to the screen
					}
					else														// If the character buffer is full, scream at the user.
					{
						*p_serial_comp << endl << "TOO MANY CHARACTERS" << endl;
					}
				}
				// If backspace (hex 08)
				else if (input_character == 0x08)
				{
					*p_serial_comp << ascii << backspace << " " << backspace << numeric;	// Backspace, space, backspace to step back, erase, and move the cursor back.
					character_buffer.delete_one();				// Delete the last character stored in the buffer.
				}
				// If question mark or exclamation point, store as period.
				else if ( (input_character == '?')||(input_character == '!') )
				{
					*p_serial_comp << ascii << input_character << numeric;	// Echo character to the screen
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
				// If ESC is pressed, user is quitting
				else if (input_character == 0x1B)
				{
					*p_serial_comp << endl << "Quitting" << endl;
					flag_message_printed = false;
					return(0);	// Go to state 5
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
			if(character_buffer.num_items())	// If character buffer is not empty
			{
				character_to_output = character_buffer.get();						// Retrieve the character
					
				// If it's not a pause character, collect information.
				if ((character_to_output != '.')||(character_to_output != ',')||(character_to_output != ' '))
				{
					flag_outputting_letter = true;
				}
				// If it's a pause character, set the proper flag.
				else
				{
					flag_outputting_letter = false;
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
				return(6);	// Now go to state 6 to figure out timing
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
				output_delay = 20;
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
		// Output values
		case(8):
			// Enable motors if they're disabled
			if (!(p_task_output -> motors_enabled()))
			{
				p_task_output -> init_motor();
				p_task_output -> start_motor();
			}
			
			// Output the character
			if (p_task_output -> ready_to_output())
			{	
				p_task_output -> set_new_character(character_to_output);
				return(5);	// Output next letter
			}
			else
			{
				return(STL_NO_TRANSITION);	// Wait
			}
			break;
		// Done
		case(9):
			if (p_task_output -> ready_to_output() && flag_outputting_letter == true)
			{
				*p_serial_comp << endl << "Message done. Returning to message prompt." << endl;
				flag_outputting_letter = false;
				return(4);	// Return to message prompt
			}
			else
			{
				return(STL_NO_TRANSITION);	// Wait
			}
			break;
		// Encoder Reading Prompt
		case(10):
			*p_serial_comp << endl << "Read which encoder?" << endl << endl << "1 - M1" << endl << "2 - M2"
			<< endl << "3 - M3" << endl << "4 - M4" << endl << "5 - M5" << endl << "6 - M6" << endl << "7 - M7"
			<< endl << "8 - M8" << endl << "9 - M9" << endl << "0 - M10" << endl << "ESC Cancel" << endl;
			return(11);
			break;
		// Encoder Reading Processing
		case(11):
			if(p_serial_comp->check_for_char())
			{
				input_character = p_serial_comp->getchar();		// Collect character
				
				// Clear slave character buffer
				while(p_serial_slave ->check_for_char())
				{
					p_serial_slave->getchar();
				}
				
				if( (input_character >= 0x31) && (input_character <= 0x39) )
				{
					// Subtract 0x30 from input character to get decimal value
					input_character = input_character - 0x30;
					
					// Set multiplexer to the proper pin
					p_slave_chooser->choose(input_character);
					
					// Output E character to trigger encoder count return
					if(p_serial_slave->ready_to_send())
					{
						*p_serial_slave << 'E';
					}
					return(12);		// Go back to state 10 (encoder prompt)
				}
				else if ( input_character == '0' )
				{
					// Choose multiplexer channel 10
					p_slave_chooser->choose(10);
					
					// Output E character to trigger encoder count return
					if(p_serial_slave->ready_to_send())
					{
						*p_serial_slave << 'E';
					}
					return(12);		// Go back to state 10 (encoder prompt)
				}
				else if (input_character == 0x1B)	// Escape
				{
					return(0);		// Go back to state 0 (home screen)
				}
				else
				{			
					*p_serial_comp << endl << "Invalid character" << endl;
					return(10);		// Go back to state 10 (encoder prompt)
				}
			}
			break;
		// Wait for encoder count return
		case(12):
			if(p_serial_slave->check_for_char())
			{
				encoder_reading = p_serial_slave -> getchar();
				encoder_reading = encoder_reading*4;	// Convert from 8 bit truncated count to 10 bit count
				*p_serial_comp << endl << "Encoder reading: " << numeric << encoder_reading << endl;
				return(10);	// Return to encoder prompt
			}
			else if (p_serial_comp ->check_for_char())
			{
				input_character = p_serial_comp->getchar();
				if (input_character == 0x1B)
				{
					*p_serial_comp << endl << "Reading cancelled" << endl;
					return(10);
				}
			}
			else
			{
				return(STL_NO_TRANSITION);
			}
			break;
		// Manual Mode Prompt
		case(13):
			*p_serial_comp << endl << "Control which motor?" << endl << endl << "1 - M1" << endl << "2 - M2"
			<< endl << "3 - M3" << endl << "4 - M4" << endl << "5 - M5" << endl << "6 - M6" << endl << "7 - M7"
			<< endl << "8 - M8" << endl << "9 - M9" << endl << "0 - M10" << endl << "ESC Cancel" << endl;
			return(14);
			break;
		// Manual Mode Processing
		case(14):
			if(p_serial_comp->check_for_char())
			{
				input_character = p_serial_comp -> getchar();		// Collect character
				
				// Clear slave character buffer
				while(p_serial_slave ->check_for_char())
				{
					p_serial_slave->getchar();
				}
				
				if( (input_character >= 0x31) && (input_character <= 0x39) )
				{
					// Subtract 0x30 from input character to get decimal value
					input_character = input_character - 0x30;
					
					// Set multiplexer to the proper pin
					p_slave_chooser->choose(input_character);
				}
				else if ( input_character == '0' )
				{
					// Choose multiplexer channel 10
					p_slave_chooser->choose(10);
				}
				else if (input_character == 0x1B)	// Escape
				{
					return(0);		// Go back to state 0 (home screen)
				}
				else
				{
					*p_serial_comp << endl << "Invalid character" << endl;
					return(13);		// Go back to state 13 (encoder prompt)
				}
				*p_serial_comp << endl << "Input command. ESC to exit." << endl;
				return(15);
			}
			break;
		case(15):
			// Command Prompt
			if(p_serial_comp -> check_for_char())
			{	
				input_character = p_serial_comp -> getchar();
				
				if (input_character != 0x1B)
				{
					if(p_serial_slave -> ready_to_send())
					{
						*p_serial_slave << input_character;
						*p_serial_comp << endl << "Sent " << ascii << input_character << numeric << " to motor." << endl;
					}
				}
				else
				{
					return(13);	// Return to motor list if ESC pressed
				}
				return(STL_NO_TRANSITION);		// Stay here unless ESC pressed
			}
			return(STL_NO_TRANSITION);
			break;
		// Wait for calibration response
		case(16):
			if (p_serial_slave -> check_for_char())
			{
				input_character = p_serial_slave -> getchar();
				if (input_character == 'c')
				{
					*p_serial_comp << endl << "Calibration successful." << endl;
				}
				else
				{
					*p_serial_comp << endl << "Calibration failed." << endl;
				}
			}
			else if (p_serial_comp -> check_for_char())
			{
				input_character = p_serial_comp -> getchar();
				if (input_character == 0x1B)
				{
					*p_serial_comp << endl << "Calibration cancelled" << endl;
					return(2);
				}
			}
			else
			{
				return (STL_NO_TRANSITION);
			}
			break;
		default:
			break;
	}
	// If we get here, no transition is called for
	return (STL_NO_TRANSITION);
};
