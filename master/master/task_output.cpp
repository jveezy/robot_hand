//*************************************************************************************
/** \file task_output.cpp
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
#include "servo.h"
#include "slave_picker.h"			// The class that sets the multiplexer pins
//#include "motor.h"
#include "task_output.h"
#include "lib/global_debug.h"

#define MOTOR_SWITCH_DDR	DDRD
#define MOTOR_SWITCH_PORT	PORTD
#define MOTOR_SWITCH_PIN	PIND6


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

task_output::task_output (task_timer& a_timer, time_stamp& t_stamp, base_text_serial* p_ser_comp, base_text_serial* p_ser_slave, slave_picker* p_slave_picker, servo* p_servotop, servo* p_servobottom) 
	: stl_task (a_timer, t_stamp)
{
	
	// Assign pointers
	p_serial_comp = p_ser_comp;
	p_serial_slave = p_ser_slave;
	p_slave_chooser = p_slave_picker;
	//p_motors = p_the_motors;
	p_servo_top = p_servotop;
	p_servo_bottom = p_servobottom;
	
	for (i = 0; i < 8; i++)
	{
		finger_configuration[i] = 0;
	}
	
	for (i = 1; i < 14; i++)
	{
		output[i] = 0;
	}
	
	// Initialize variables
	flag_interference_thumb = false;
	flag_interference_index = false;
	flag_interference_middle = false;
	flag_interference_ring = false;
	flag_interference_pinky = false;
	flag_stop_motors = false;
	flag_start_motors = false;
	flag_init_motors = false;
	character_step = 1;
	motor_to_init = 1;
	motor_to_start = 1;
	motor_to_stop = 1;
	
	MOTOR_SWITCH_DDR |= (1 << MOTOR_SWITCH_PIN);
	MOTOR_SWITCH_PORT &= ~(1 << MOTOR_SWITCH_PIN);
	
	*p_serial_comp << endl << "Output task initialized." << endl;
	
}

//-------------------------------------------------------------------------------------
/** This is the function which runs when it is called by the task scheduler. It causes
 *  the motor to move back and forth, having several states to cause such motion. 
 *  @param state The state of the task when this run method begins running
 *  @return The state to which the task will transition, or STL_NO_TRANSITION if no
 *	  transition is called for at this time
 */

char task_output::run (char state)
{
	//*p_serial_comp << endl << "Out State " << state << endl;
	
	switch(state)
	{
		// Wait for output change
		case(0):
			if (flag_stop_motors)
			{
				return(3);
			}
			else if (flag_start_motors)
			{
				return(5);
			}
			else if(flag_init_motors)
			{
				return(7);
			}
			else if(flag_output_change)
			{
				flag_output_change = false;
				return(1);	// Go to state 1 (Check for interferences)
			}
			else
			{
				flag_ready_to_output = true;
				return(STL_NO_TRANSITION);				
			}
			break;
		// Check for interferences		
		case(1):
			flag_ready_to_output = false;
			if(flag_interference_thumb)
			{
				open_thumb();
				flag_interference_thumb = false;
			}
			if(flag_interference_index)
			{
				open_index();
				flag_interference_index = false;
			}
			if(flag_interference_middle)
			{
				open_middle();
				flag_interference_middle = false;
			}
			if(flag_interference_ring)
			{
				open_ring();
				flag_interference_ring = false;
			}
			if(flag_interference_pinky)
			{
				open_pinky();
				flag_interference_pinky = false;
			}
			return(2);
			break;	
		// Process outputs
		case(2):	
			if (!flag_motors_enabled)
			{
				flag_motors_enabled = true;
			}	
			// Parse character
			switch(character_to_output)
			{
				case('0'):
				case('O'):
				case('o'):
					thumb_curl();
					index_curl();
					middle_curl();
					ring_curl();
					pinky_curl();
					wrist_default();
					return(0);
					break;
				case('1'):
					thumb_flat_up();
					index_stretch();
					middle_clench();
					ring_clench();
					pinky_clench();
					wrist_default();
					return(0);
					break;
				case('2'):
					thumb_flat_up();
					index_stretch();
					middle_stretch();
					ring_clench();
					pinky_clench();
					wrist_default();
					return(0);
					break;
				case('3'):
					thumb_stretch();
					index_stretch();
					middle_stretch();
					ring_clench();
					pinky_clench();
					wrist_default();
					return(0);
					break;
				case('4'):
				case('b'):
				case('B'):
					switch(character_step)
					{
						case(1):
							thumb_fold_out();
							index_stretch();
							middle_stretch();
							ring_stretch();
							pinky_stretch();
							wrist_default();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
							thumb_fold_in();
							flag_interference_thumb = true;
							character_step = 1;
							return(0);
						default:
							*p_serial_comp << endl << "Error character " << character_to_output << " step " << character_step << endl;
					}
					break;
				case('5'):
					thumb_stretch();
					index_stretch();
					middle_stretch();
					ring_stretch();
					pinky_stretch();
					wrist_default();
					return(0);
					break;
				case('6'):
				case('W'):
				case('w'):
					switch(character_step)
					{
						case(1):
							thumb_fold_out();
							index_stretch();
							middle_stretch();
							ring_stretch();
							pinky_clench();
							wrist_default();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
							thumb_fold_in();
							flag_interference_thumb = true;
							character_step = 1;
							return(0);
						default:
							*p_serial_comp << endl << "Error character " << character_to_output << " step " << character_step << endl;
					}
					break;
				case('7'):
					switch(character_step)
					{
						case(1):
							thumb_fold_out();
							index_stretch();
							middle_stretch();
							ring_clench();
							pinky_stretch();
							wrist_default();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
							thumb_fold_in();
							flag_interference_thumb = true;
							character_step = 1;
							return(0);
						default:
							*p_serial_comp << endl << "Error character " << character_to_output << " step " << character_step << endl;
					}
					break;
				case('8'):
					switch(character_step)
					{
						case(1):
							thumb_fold_out();
							index_stretch();
							middle_clench();
							ring_stretch();
							pinky_stretch();
							wrist_default();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
							thumb_fold_in();
							flag_interference_thumb = true;
							character_step = 1;
							return(0);
						default:
							*p_serial_comp << endl << "Error character " << character_to_output << " step " << character_step << endl;
					}
					break;
				case('9'):
					thumb_flat_up();
					index_clench();
					middle_stretch();
					ring_stretch();
					pinky_stretch();
					wrist_default();
					return(0);
					break;
				case('A'):
				case('a'):
					thumb_flat_up();
					index_clench();
					middle_clench();
					ring_clench();
					pinky_clench();
					wrist_default();
					return(0);
					break;
				case('C'):
				case('c'):
					thumb_fold_out();
					index_curl();
					middle_curl();
					ring_curl();
					pinky_clench();
					wrist_default();
					return(0);
					break;
				case('D'):
				case('d'):
					thumb_curl();
					index_stretch();
					middle_curl();
					ring_curl();
					pinky_clench();
					wrist_default();
					return(0);
					break;
				case('E'):
				case('e'):
					switch(character_step)
					{
						case(1):
							thumb_fold_out();
							index_stretch();
							middle_stretch();
							ring_stretch();
							pinky_stretch();
							wrist_default();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
							thumb_fold_in();
							index_curl();
							middle_curl();
							ring_curl();
							pinky_curl();
							flag_interference_thumb = true;
							flag_interference_index = true;
							flag_interference_middle = true;
							flag_interference_ring = true;
							flag_interference_pinky = true;
							character_step = 1;
							return(0);
						default:
							*p_serial_comp << endl << "Error character " << character_to_output << " step " << character_step << endl;
					}
					break;
				case('F'):
				case('f'):
					thumb_flat_up();
					index_clench();
					middle_stretch();
					ring_stretch();
					pinky_stretch();
					wrist_default();
					return(0);
					break;
				case('G'):
				case('g'):
					thumb_flat_up();
					index_stretch();
					middle_clench();
					ring_clench();
					pinky_clench();
					wrist_bent();
					return(0);
					break;
				case('H'):
				case('h'):
					thumb_flat_up();
					index_stretch();
					middle_stretch();
					ring_clench();
					pinky_clench();
					wrist_bent();
					return(0);
					break;
				case('I'):
				case('i'):
					thumb_flat_up();
					index_clench();
					middle_clench();
					ring_clench();
					pinky_stretch();
					wrist_default();
					return(0);
					break;
				case('J'):
				case('j'):
					switch(character_step)
					{
						case(1):
							thumb_flat_up();
							index_clench();
							middle_clench();
							ring_clench();
							pinky_stretch();
							wrist_default();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
							wrist_bent();
							character_step++;
							return(STL_NO_TRANSITION);
						case(3):
							wrist_bent_and_twisted();
							character_step++;
							return(STL_NO_TRANSITION);
						case(4):
							wrist_twisted();
							character_step = 1;
							return(0);
					}
					break;
				case('K'):
				case('k'):
					switch(character_step)
					{
						case(1):
							thumb_flat_up();
							index_stretch();
							middle_stretch();
							ring_clench();
							pinky_clench();
							wrist_default();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
						thumb_fold_in();
							flag_interference_thumb = true;
							character_step = 1;
							return(0);
						default:
							*p_serial_comp << endl << "Error character " << character_to_output << " step " << character_step << endl;
					}
					break;
				case('L'):
				case('l'):
					thumb_stretch();
					index_stretch();
					middle_clench();
					ring_clench();
					pinky_clench();
					wrist_default();
					return(0);
					break;
				case('M'):
				case('m'):
					switch(character_step)
					{
						case(1):
							thumb_fold_in();
							index_stretch();
							middle_stretch();
							ring_stretch();
							pinky_clench();
							wrist_default();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
							index_vert_clench();
							middle_vert_clench();
							ring_curl();
							flag_interference_thumb = true;
							flag_interference_index = true;
							flag_interference_middle = true;
							flag_interference_ring = true;
							character_step = 1;
							return(0);
						default:
							*p_serial_comp << endl << "Error character " << character_to_output << " step " << character_step << endl;
					}
					break;
				case('N'):
				case('n'):
					switch(character_step)
					{
						case(1):
							thumb_fold_in();
							index_stretch();
							middle_stretch();
							ring_clench();
							pinky_clench();
							wrist_default();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
							index_vert_clench();
							middle_vert_clench();
							flag_interference_thumb = true;
							flag_interference_index = true;
							flag_interference_middle = true;
							character_step = 1;
							return(0);
						default:
							*p_serial_comp << endl << "Error character " << character_to_output << " step " << character_step << endl;
					}
					break;
				case('P'):
				case('p'):
					thumb_fold_up();
					index_stretch();
					middle_fold();
					ring_clench();
					pinky_clench();
					wrist_bent();
					flag_interference_thumb = true;
					flag_interference_middle = true;
					return(0);
					break;
				case('Q'):
				case('q'):
					thumb_fold_out();
					index_fold();
					middle_clench();
					ring_clench();
					pinky_clench();
					wrist_bent();
					return(0);
					break;
				case('R'):
				case('r'):
					thumb_flat_up();
					index_cross();
					middle_clench();
					ring_clench();
					pinky_clench();
					wrist_default();
					flag_interference_index = true;
					return(0);
					break;
				case('S'):
				case('s'):
					switch(character_step)
					{
						case(1):
							thumb_fold_out();
							index_clench();
							middle_clench();
							ring_clench();
							pinky_clench();
							wrist_default();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
							thumb_fold_in();
							flag_interference_thumb = true;
							character_step = 1;
							return(0);
						default:
							*p_serial_comp << endl << "Error character " << character_to_output << " step " << character_step << endl;
					}
					break;
				case('T'):
				case('t'):
					switch(character_step)
					{
						case(1):
							thumb_flat_up();
							index_vert_clench();
							middle_clench();
							ring_clench();
							pinky_clench();
							wrist_default();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
							thumb_fold_in();
							flag_interference_thumb = true;
							flag_interference_index = true;
							character_step = 1;
							return(0);
						default:
							*p_serial_comp << endl << "Error character " << character_to_output << " step " << character_step << endl;
					}
					break;
				case('U'):
				case('u'):
					switch(character_step)
					{
						case(1):
							thumb_flat_up();
							index_stretch();
							middle_stretch();
							ring_clench();
							pinky_clench();
							wrist_default();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
							index_u();
							flag_interference_thumb = true;
							flag_interference_index = true;
							character_step = 1;
							return(0);
						default:
							*p_serial_comp << endl << "Error character " << character_to_output << " step " << character_step << endl;
					}
					break;
				case('X'):
				case('x'):
					switch(character_step)
					{
						case(1):
							thumb_fold_out();
							index_stretch();
							middle_clench();
							ring_clench();
							pinky_clench();
							wrist_default();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
							thumb_fold_in();
							index_vert_clench();
							flag_interference_thumb = true;
							flag_interference_index = true;
							character_step = 1;
							return(0);
						default:
							*p_serial_comp << endl << "Error character " << character_to_output << " step " << character_step << endl;
					}
				case('Y'):
				case('y'):
					thumb_stretch();
					index_clench();
					middle_clench();
					ring_clench();
					pinky_stretch();
					wrist_default();
					break;
				case('Z'):
				case('z'):
					switch(character_step)
					{
						case(1):
							thumb_flat_up();
							index_clench();
							middle_clench();
							ring_clench();
							pinky_stretch();
							wrist_z1();
							character_step++;
							return(STL_NO_TRANSITION);
						case(2):
							wrist_z2();
							character_step++;
							return(STL_NO_TRANSITION);
						case(3):
							wrist_z3();
							character_step++;
							return(STL_NO_TRANSITION);
						case(4):
							wrist_bent();
							character_step = 1;
							return(0);
						default:
							*p_serial_comp << endl << "Error character " << character_to_output << " step " << character_step << endl;
					}
			}
				
			return(0);	// Go to state 2 (output) when done
			break;
		// Send Stop Command to a Motor
		case(3):
			flag_stop_motors = false;
			p_slave_chooser->choose(motor_to_stop);
			if(p_serial_slave->ready_to_send())
			{
				*p_serial_slave << "S";
			}
			return(4);
			break;
		// Wait for confirmation on stop command
		case(4):
			if(p_serial_slave->check_for_char())
			{
				input_character = p_serial_slave->getchar();
				if (input_character == 's')
					*p_serial_comp << endl << "Motor " << motor_to_stop << " stopped";
				else
					*p_serial_comp << endl << "Motor stop error " << motor_to_stop << endl;
				
				if (motor_to_stop == 10)
				{
					motor_to_stop = 1;
					return(0);
				}
				else
				{
					motor_to_stop++;
					return(3);
				}
			}
			else
			{
				return(STL_NO_TRANSITION);
			}
		// Send Start Command to a Motor
		case(5):
			flag_start_motors = false;
			p_slave_chooser->choose(motor_to_start);
			if(p_serial_slave->ready_to_send())
			{
				*p_serial_slave << "G";
			}
			return(6);
			break;
		// Wait for confirmation on start command
		case(6):
			if(p_serial_slave->check_for_char())
			{
				input_character = p_serial_slave->getchar();
				if (input_character == 'g')
					*p_serial_comp << endl << "Motor " << motor_to_start << " enabled";
				else
					*p_serial_comp << endl << "Motor start error " << motor_to_start << endl;
				
				if (motor_to_start == 10)
				{
					motor_to_start = 1;
					return(0);
				}
				else
				{
					motor_to_start++;
					return(5);
				}
			}
			else
			{
				return(STL_NO_TRANSITION);
			}
		// Send initialization command to one motor
		case(7):
			p_slave_chooser->choose(motor_to_init);
			if(p_serial_slave->ready_to_send())
			{
				if (motor_to_init > 0 && motor_to_init < 10)
					character_to_output = motor_to_init + 0x30;
				else if (motor_to_init == 10)
					character_to_output = '0';
				else
					*p_serial_comp << endl << "Motor conf error " << motor_to_init << endl;
			
				*p_serial_slave << character_to_output;
				return(8);
			}
			break;
		// Wait for initialization response from motor
		case(8):
			if(p_serial_slave->check_for_char())
			{
				input_character = p_serial_slave->getchar();
				if (input_character == '!')
					*p_serial_comp << endl << "Motor " << motor_to_init << " initialized";
				else
					*p_serial_comp << endl << "Motor init error " << motor_to_init << endl;
				
				if (motor_to_init == 10)
				{
					motor_to_init = 1;
					return(0);
				}
				else
				{
					motor_to_init++;
					return(7);
				}
			}
			else
			{
				return(STL_NO_TRANSITION);
			}
			break;
		default:
			return(0);
			break;
			
	}
	// If we get here, no transition is called for
	return (STL_NO_TRANSITION);
}

void task_output::set_new_character(unsigned char outchar)
{
	character_to_output = outchar;
	*p_serial_comp << endl << "New output character: " << ascii << character_to_output << numeric << endl;
	flag_output_change = true;
}

void task_output::stop_motor(void)
{
	motor_to_stop = 1;
	flag_stop_motors = true;
	flag_motors_enabled = false;
}

void task_output::start_motor(void)
{
	motor_to_start = 1;
	flag_start_motors = true;
	flag_motors_enabled = true;
}

bool task_output::motors_enabled(void)
{
	return(flag_motors_enabled);
}

bool task_output::query_motor(unsigned char motornum)
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
		return(false);
	}
	else
	{
		*p_serial_comp << endl << "Serial port not ready to send to motor " << motornum << endl;
		return(false);
	}
	
}

void task_output::init_motor(void)
{
	motor_to_init = 1;
	flag_init_motors = true;
}

void task_output::output_to_motor (unsigned char motornumber, unsigned char output_value)
{
	//*p_serial_comp << "Select motor " << numeric << motornumber << endl;
	
	if (motornumber <= 10 && motornumber >= 0)
	{
		p_slave_chooser->choose(motornumber);
		*p_serial_slave << output_value;
		*p_serial_comp << ascii << output_value << numeric;
	}
	else if (motornumber == 11)
	{
		if(output_value == 1)
		{
			MOTOR_SWITCH_PORT |= (1 << MOTOR_SWITCH_PIN);
		}
		else if(output_value == 0)
		{
			MOTOR_SWITCH_PORT &= ~(1 << MOTOR_SWITCH_PIN);
		}
	}
	else if (motornumber == 12)
	{
		p_servo_top->output(output_value);
	}
	else if (motornumber == 13)
	{
		p_servo_bottom->output(output_value);
	}
	else
	{
		*p_serial_comp << "Motor number outside bounds" << endl;
	}
}

bool task_output::ready_to_output(void)
{
	return (flag_ready_to_output);
}

void task_output::open_thumb(void)
{
	*p_serial_comp << endl << "thumb" << endl;
	output_to_motor(5,'a');
}

void task_output::open_index(void)
{
	*p_serial_comp << endl << "index" << endl;
	output_to_motor(1,'a');
	output_to_motor(11,0);
}

void task_output::open_middle(void)
{
	*p_serial_comp << endl << "middle" << endl;
	output_to_motor(2,'a');
}

void task_output::open_ring(void)
{
	*p_serial_comp << endl << "ring" << endl;
	output_to_motor(3,'a');
}

void task_output::open_pinky(void)
{
	*p_serial_comp << endl << "pinky" << endl;
	output_to_motor(4,'a');
}

void task_output::thumb_flat_up(void)
{
	*p_serial_comp << endl << "thumb" << endl;
	output_to_motor(5,'a');
	output_to_motor(6,'a');
	output_to_motor(7,'a');
	output_to_motor(8,'a');
}

void task_output::thumb_fold_up(void)
{
	*p_serial_comp << endl << "thumb" << endl;
	output_to_motor(5,'e');
	output_to_motor(6,'a');
	output_to_motor(7,'a');
	output_to_motor(8,'a');
}

void task_output::thumb_fold_in(void)
{
	*p_serial_comp << endl << "thumb" << endl;
	output_to_motor(5,'c');
	output_to_motor(6,'c');
	output_to_motor(7,'e');
	output_to_motor(8,'a');
}

void task_output::thumb_fold_out(void)
{
	*p_serial_comp << endl << "thumb" << endl;
	output_to_motor(5,'e');
	output_to_motor(6,'a');
	output_to_motor(7,'b');
	output_to_motor(8,'b');
}

void task_output::thumb_stretch(void)
{
	*p_serial_comp << endl << "thumb" << endl;
	output_to_motor(5,'a');
	output_to_motor(6,'e');
	output_to_motor(7,'a');
	output_to_motor(8,'a');
}

void task_output::thumb_curl(void)
{
	*p_serial_comp << endl << "thumb" << endl;
	output_to_motor(5,'e');
	output_to_motor(6,'b');
	output_to_motor(7,'b');
	output_to_motor(8,'b');
}

void task_output::index_stretch(void)
{
	*p_serial_comp << endl << "index" << endl;
	output_to_motor(1,'a');
	output_to_motor(9,'a');
}

void task_output::index_curl(void)
{
	*p_serial_comp << endl << "index" << endl;
	output_to_motor(1,'c');
	output_to_motor(9,'c');
}

void task_output::index_clench(void)
{
	*p_serial_comp << endl << "index" << endl;
	output_to_motor(1,'e');
	output_to_motor(9,'e');
}

void task_output::index_vert_clench(void)
{
	*p_serial_comp << endl << "index" << endl;
	output_to_motor(1,'a');
	output_to_motor(9,'e');
}

void task_output::index_cross(void)
{
	*p_serial_comp << endl << "index" << endl;
	output_to_motor(1,'c');
	output_to_motor(9,'a');
	output_to_motor(11,1);
}

void task_output::index_u(void)
{
	*p_serial_comp << endl << "index" << endl;
	output_to_motor(1,'a');
	output_to_motor(9,'a');
	output_to_motor(11,1);
}

void task_output::index_fold(void)
{
	*p_serial_comp << endl << "index" << endl;
	output_to_motor(1,'e');
	output_to_motor(9,'a');
}

void task_output::middle_stretch(void)
{
	*p_serial_comp << endl << "middle" << endl;
	output_to_motor(2,'a');
	output_to_motor(10,'a');
}

void task_output::middle_curl(void)
{
	*p_serial_comp << endl << "middle" << endl;
	output_to_motor(2,'c');
	output_to_motor(10,'c');
}

void task_output::middle_clench(void)
{
	*p_serial_comp << endl << "middle" << endl;
	output_to_motor(2,'e');
	output_to_motor(10,'e');
}

void task_output::middle_vert_clench(void)
{
	*p_serial_comp << endl << "middle" << endl;
	output_to_motor(2,'a');
	output_to_motor(10,'e');
}

void task_output::middle_fold(void)
{
	*p_serial_comp << endl << "middle" << endl;
	output_to_motor(2,'e');
	output_to_motor(10,'a');
}

void task_output::ring_stretch(void)
{
	*p_serial_comp << endl << "ring" << endl;
	output_to_motor(3,'a');
}

void task_output::ring_curl(void)
{
	*p_serial_comp << endl << "ring" << endl;
	output_to_motor(3,'c');
}

void task_output::ring_clench(void)
{
	*p_serial_comp << endl << "ring" << endl;
	output_to_motor(3,'e');
}

void task_output::pinky_stretch(void)
{
	*p_serial_comp << endl << "pinky" << endl;
	output_to_motor(4,'a');
}

void task_output::pinky_curl(void)
{
	*p_serial_comp << endl << "pinky" << endl;
	output_to_motor(4,'c');
}

void task_output::pinky_clench(void)
{
	*p_serial_comp << endl << "pinky" << endl;
	output_to_motor(4,'e');
}

void task_output::wrist_default(void)
{
	*p_serial_comp << endl << "wrist" << endl;
	output_to_motor(12,0);
	output_to_motor(13,0);
}

void task_output::wrist_bent(void)
{
	*p_serial_comp << endl << "wrist" << endl;
	output_to_motor(12,90);
	output_to_motor(13,0);
}

void task_output::wrist_bent_and_twisted(void)
{
	*p_serial_comp << endl << "wrist" << endl;
	output_to_motor(12,90);
	output_to_motor(13,90);
}

void task_output::wrist_twisted(void)
{
	*p_serial_comp << endl << "wrist" << endl;
	output_to_motor(12,0);
	output_to_motor(13,90);
}

void task_output::wrist_z1(void)
{
	*p_serial_comp << endl << "wrist" << endl;
	output_to_motor(12,45);
	output_to_motor(13,45);
}

void task_output::wrist_z2(void)
{
	*p_serial_comp << endl << "wrist" << endl;
	output_to_motor(12,45);
	output_to_motor(13,0);
}

void task_output::wrist_z3(void)
{
	*p_serial_comp << endl << "wrist" << endl;
	output_to_motor(12,90);
	output_to_motor(13,45);
}


