//*************************************************************************************
/** \file stl_task.cpp
 *    This file contains the definition of a task class. This class implements the
 *    tasks in a multitasking system. These tasks are intended to be run in a 
 *    cooperative multitasking framework, but they can also be worked into an 
 *    interrupt framework or run within an RTOS. 
 * 
 *  Revisions
 *    \li 04-21-2007 JRR Original copy of this file, derived from UCB's TranRun4 and
 *                       simplified greatly for efficient use in AVR processors
 *    \li 05-07-2007 JRR Small bug fixes
 *    \li 06-01-2008 JRR Changed debugging/trace to take advantage of base_text_serial
 *    \li 06-03-2008 JRR Cleaned up comments, got rid of Doxygen warnings
 *    \li 12-19-2009 JRR Integrated simple execution time profiling into file, changed
 *                       from *.cc to *.cpp, and set up for global serial debugging
 *
 *  License:
 *    This file released under the Lesser GNU Public License, version 2. This program
 *    is intended for educational use only, but it is not limited thereto. 
 */
//*************************************************************************************

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "base_text_serial.h"				// Base class for various serial devices
#include "global_debug.h"					// Class for serial port debugging
#include "stl_timer.h"						// Timer measures real time
#include "stl_task.h"						// The state transition logic header


//--------------------------------------------------------------------------------------
// This is the static member declaration for class stl_task; see class definition in
// the header file for more information on the item(s)

char stl_task::serial_counter = 0;


/// This is just a time stamp with zeros in it for comparisions
const time_stamp zero_time (0L);


//--------------------------------------------------------------------------------------
/** This constructor creates a task object. It must be called by the constructor of 
 *  each task which is written by the user. This constructor sets the time between runs
 *  of the run() method (a timestamp with zero time can be used if a task needs to run
 *  via interrupt, or every single time its schedule() method is called). This 
 *  constructor also saves a pointer to the serial port or radio object which will be 
 *  used to dump debugging information, if serial debugging is being used. 
 *  @param a_timer A reference to the timer which measures real-time intervals
 *  @param time_interval The time between runs of the task's run() method
 */

stl_task::stl_task (task_timer& a_timer, const time_stamp& time_interval)
	: the_timer (a_timer), interval (time_interval)
{
	// Give this task its serial number, then increment the serial number counter
	serial_number = serial_counter++;

	// This task begins life in its waiting to run operational state
	op_state = TASK_WAITING;
	save_op_state = TASK_WAITING;

	// The task begins running in state 0, with no transitions unless called for
	current_state = 0;

	// The next run time should have been initialized to zero, so the task will run
	// its run() method as soon as possible in most cases

	#ifdef STL_PROFILE
		// Clear the profile data arrays
		clear_profiler ();
	#endif
}


//--------------------------------------------------------------------------------------
/** This method sets or changes the time interval between runs of this task. 
 *  @param time_interval The time between runs of the task's run() method
 */

void stl_task::set_interval (const time_stamp& time_interval)
{
	interval = time_interval;
}


//--------------------------------------------------------------------------------------
/** This method sets the next run time for the task. It should be used if the task
 *  timer's value is adjusted significantly, or if the task timer is started at a time
 *  long after time zero. If this isn't done, the task will run many times in quick
 *  succession, trying to catch up to the correct run time. Note that if this method
 *  is called and sets the next time far in the past, the same problem will occur.
 *  @param next_time A time stamp containing the next time for the task to run
 */

void stl_task::set_next_run_time (const time_stamp& next_time)
{
	next_run_time = next_time;
}


//--------------------------------------------------------------------------------------
/** This method is called by the main task loop to try to run the task. If the task is
 *  in the waiting state, it checks to see if it's time to run yet; if it's in the
 *  suspended state, it doesn't. The task shouldn't be in the running state, because
 *  this method is used by the cooperative scheduler, not the pre-emptive one.
 *  @param til_next_time A pointer to a time stamp which will hold next time this task 
 *                       next needs to run, or NULL (default) if this data isn't needed
 *  @return The operational state of the task after it has been given this chance to
 *          run
 */

task_op_state stl_task::schedule (time_stamp* til_next_time)
{
	char next_state;						// State to which a task will transition
	time_stamp the_time;					// Somewhere to store a time measurement

	switch (op_state)
	{
		// If the task has been suspended, don't bother trying to run it, and don't
		// bother sending back the next run time because this task is suspended
		case (TASK_SUSPENDED):
			return (TASK_SUSPENDED);

		// If the task needs to run, check if it needs to run now; if so, run it
		case (TASK_WAITING):
			// Find out what the current real time is from the system timer
			the_time = the_timer.get_time_now ();

			// If it's not time to run the task yet, exit without running it
			if (next_run_time > the_timer.get_time_now ())
			{
				if (til_next_time != NULL)
				{
					*til_next_time = next_run_time;
				}
				return (TASK_WAITING);
			}
			// If we get here, it is time to run the task; just continue into the
			// task_pending section below, which will cause the task to run right now

		case (TASK_PENDING):
			// Set the state to waiting for the next time interval. If the task needs
			// to run again immediately, run_again_ASAP() will be called within the
			// run() method, causing the state to be set to TASK_PENDING instead
			op_state = TASK_WAITING;
			#ifdef STL_PROFILE						// If execution time profiling is
				start_profiler ();					// activated, start timing
			#endif
			next_state = run (current_state);		// Call the run() method
			#ifdef STL_PROFILE
				end_profiler ();					// End execution time measurement
			#endif
			if (next_state != STL_NO_TRANSITION)	// Detect state transition if any
			{										// has occurred
				#ifdef STL_TRACE
					GLOB_DEBUG ("T" << serial_number << ":" << current_state << "-" 
						<< next_state << endl);
				#endif
				current_state = next_state;			// Go to next state next time
			}

			// If the op-state is TASK_WAITING now, then run_again_ASAP() was not
			// called, and the task will wait until its time to run again has arrived
			if (op_state == TASK_WAITING)
			{
				next_run_time += interval;
				if (til_next_time != NULL)
				{
					*til_next_time = next_run_time;
				}
			}

			// If the op-state isn't TASK_WAITING, someone called run_again_ASAP(), so
			// the op-state is TASK_PENDING and we need to run again right away
			return (op_state);

		// If the operational state is anything else, there has been a serious error
		default:
			error_stop ("Illegal operational state");
			break;
	};

//	 else if (op_state == blocked)					// Blocked tasks are not currently 
//		 return (false);							// implemented

}


//--------------------------------------------------------------------------------------
/** This is a base method which the user should overload in each descendent of this 
 *  task class. The run method is where all the user-defined action in the task takes
 *  place. It is either called by the task's schedule() method, in the case of 
 *  cooperative multitasking, or by an interrupt handler, in the case of pre-emptive
 *  multitasking.
 *  @param a_state The state of the task when this run method begins running
 *  @return The state to which the task will transition, or STL_NO_TRANSITION if no
 *	  transition is called for at this time
 */

char stl_task::run (char a_state)
{
	GLOB_DEBUG (PMS ("Base run() method called for task ") << serial_number << endl);

	return (STL_NO_TRANSITION);
}


//--------------------------------------------------------------------------------------
/** This method suspends the task so that it won't run again until after somebody calls 
 *  the resume() method. The state of the task before suspension is saved so that it 
 *  can be restored after suspension.
 */

void stl_task::suspend (void)
{
	save_op_state = op_state;
	op_state = TASK_SUSPENDED;
}


//--------------------------------------------------------------------------------------
/** This method resumes the task from suspension, so that the task can run again. The
 *  operational state which was saved at the time of suspension is restored. 
 */

void stl_task::resume (void)
{
	op_state = save_op_state;
}


//--------------------------------------------------------------------------------------
/** This method changes the initial state in which the task begins to operate. The 
 *  default initial state is state 0. It should only be used before the task begins to
 *  run, as it sets the current and next state variables and will completely mess up
 *  normal state transition operation if called at any other time. 
 */

void stl_task::set_initial_state (char init_state)
{
	current_state = init_state;
}


//--------------------------------------------------------------------------------------
/** This method displays a message (if the program was compiled with serial debugging
 *  enabled) and then causes the processor to freeze in an infinite loop. It should be
 *  used if something awful happened and the safest thing to do is to just stop. Only
 *  use this function if there isn't a reasonable way to write an error state which 
 *  handles exceptions in a more useful manner, such as by turning motors and other
 *  possibly dangerous devices off and then halting. 
 *  @param message The text to be displayed before the processor stops working
 */

void stl_task::error_stop (char const* message)
{
	GLOB_DEBUG (PMS ("ERROR in task ") << serial_number << PMS (" state ") 
		<< current_state << ": " << message << endl << PMS ("Processing stopped.") 
		<< endl);

	cli ();									// Disable interrupts
	while (1);								// Bang...you're dead (until reset)
}


#ifdef STL_PROFILE
//-------------------------------------------------------------------------------------
/** This method clears the statistical counters so that the profiler is ready to save
 *  some new data. 
 */

void stl_task::clear_profiler (void)
{
	time_stamp zero (0, 0);
	max_time = zero;
	min_time = zero;
	avg_time = zero;
	run_time_sum = zero;
	runs = 0L;
}


//-------------------------------------------------------------------------------------
/** This method begins a measurement run. It simply saves the time at which this 
 *  method is called. A subsequent call to end() will cause the duration between the
 *  calls to start() and end() to be measured and recorded. 
 */

void stl_task::start_profiler (void)
{
	start_time = the_timer.get_time_now ();
}


//-------------------------------------------------------------------------------------
/** This method ends a measurement run, computing the duration of the run and saving
 *  that duration in the statistical counters as necessary. 
 */

void stl_task::end_profiler (void)
{
	// Compute the duration measured for this run
	time_stamp duration = the_timer.get_time_now () - start_time;

	// A duration less than zero is impossible, so it must be wrong
	if (duration < 0)
	{
		return;
	}

	// If we have a maximum record, save it
	if (duration > max_time)
		max_time = duration;

	// If this is the first run, it's the minimum; otherwise, check it
	if (runs == 0L)
		min_time = duration;
	else
		if (duration < min_time)
			min_time = duration;

	// Add to the counters which are used to find the average
	run_time_sum += duration;
	runs++;
}


//-------------------------------------------------------------------------------------
/** This method returns the duration of the longest recorded run of the code
 *  which is being profiled. 
 *  @return The maximum measured run duration
 */

const time_stamp& stl_task::get_max_run_time (void) 
{ 
	return (max_time); 
}


//-------------------------------------------------------------------------------------
/** This method returns the duration of the shortest recorded run of the code
 *  which is being profiled. 
 *  @return The minimum measured run duration
 */

const time_stamp& stl_task::get_min_run_time (void) 
{ 
	return (min_time); 
}


//-------------------------------------------------------------------------------------
/** This method returns the average duration of all the runs of the code which
 *  is being profiled. 
 *  @return The average measured run duration
 */

const time_stamp& stl_task::get_avg_run_time (void) 
{ 
	if (runs > 0L)
	{
		avg_time = run_time_sum;
		avg_time /= runs;
	}
	return (avg_time);
}

#endif  // STL_PROFILE


//-------------------------------------------------------------------------------------
/** This overloaded shift operator writes the vital statistics for this task to a
 *  serial device. Just what is written depends on whether transition tracing and/or
 *  execution time profiling has been enabled. 
 *  @param serial A reference to the serial-type object to which to print
 *  @param task A reference to the task whose information is to be displayed
 */

base_text_serial& operator<< (base_text_serial& serial, stl_task& task)
{
	serial << PMS ("Task: ") << task.get_serial_number ();

	#ifdef STL_PROFILE
		serial << PMS (" runs: ") << task.get_num_runs ();
		time_stamp fred = task.get_avg_run_time ();
		serial << PMS (" avg: ") << fred;
		fred = task.get_max_run_time ();
		serial << PMS (" max: ") << fred;
		fred = task.get_min_run_time ();
		serial << PMS (" min: ") << fred;
	#endif

	return (serial);
}

