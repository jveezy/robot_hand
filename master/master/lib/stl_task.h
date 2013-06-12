//*************************************************************************************
/** \file stl_task.h
 *  This file contains the definition of a task class. This class implements the
 *  tasks in a multitasking system. These tasks are intended to be run in a 
 *  cooperative multitasking framework, but they can also be worked into an 
 *  interrupt framework or run within an RTOS. 
 * 
 *  Revisions
 *    \li 04-21-07 JRR Original of this file, derived from UCB's TranRun4 and
 *                     simplified greatly for efficient use in AVR processors
 *    \li 05-07-07 JRR Small bug fixes
 *    \li 06-01-08 JRR Changed debugging/trace to take advantage of base_text_serial
 *    \li 06-03-08 JRR Cleaned up comments, got rid of Doxygen warnings
 *
 *  License:
 *    This file released under the Lesser GNU Public License, version 2. This program
 *    is intended for educational use only, but it is not limited thereto. 
 */
//*************************************************************************************

/// This define prevents this .h file from being included more than once in a .cc file
#ifndef _STL_TASK_H_
#define _STL_TASK_H_

#include "stl_timer.h"						// Include the header for the task timer


//--------------------------------------------------------------------------------------
/** This define specifies a no-transition code which means that the next state will be
 *  the same as the current state, and with no self-transition. 
 */

const char STL_NO_TRANSITION = 0xFF;


//--------------------------------------------------------------------------------------
/** This enumeration lists the possible operational states of a task. These states are 
 *  not the same as the states which are programmed by the user; these states are only 
 *  used by the task scheduler to figure out which task to run at a given time 
 *  according to priorities (if used) and which tasks are ready to be run. 
 */

enum task_op_state 
{
	TASK_RUNNING,			///< The task's run() function is executing
	TASK_PENDING,			///< The task needs to run again as soon as possible
	TASK_WAITING,			///< The task is waiting for its next run time to occur
	TASK_BLOCKED,			///< The task cannot run because a resource is unavailable
	TASK_SUSPENDED			///< The task is turned off until its resume() is called
};


//--------------------------------------------------------------------------------------
/** This class implements the behavior of a task in the context of a multitasking
 *  system. Each task runs "simultaneously" with other tasks. This means, of course,
 *  that execution switches quickly from one task to another quite quickly under the
 *  control of the main execution scheduler. Since a task can be in any of several 
 *  states at a given time, a task object will have a list of state methods which
 *  can be run and be able to switch between those states. 
 *
 *  \section task_usage Usage
 *    The programmer uses this class by deriving a descendent class from it. Then 
 *    s/he creates a run() method in the new class which implements the states. 
 *    In the run() method, there's a big switch statement which chooses which state
 *    code is to run when the run() method is called. Checking the time and deciding
 *    when to run is taken care of by the schedule() method, which only runs the
 *    task's state code when the task's time interval has been reached; if the time
 *    interval is set to 0, the state code is run every time run() is called.  State
 *    transitions are detected and tracked by this class for testing and debugging
 *    purposes. 
 *
 *  \section task_options Options
 *    \li Serial port debugging can be enabled by defining SERIAL_DEBUG and calling
 *        the constructor which is given a pointer to a uart object. The task will 
 *        then write information about state transitions to the serial port during 
 *        operation. This option is intended to be used during debugging, then turned 
 *        off for production code, as serial device writing takes up time and memory, 
 *        and of course it requires a serial device to be present and connected.
 *    \li Execution time profiling can be enabled by defining STL_PROFILE.  This
 *        option causes the execution times of the state functions to be measured
 *        and a simple set of performance data to be kept. Performance data can be
 *        written to a serial port at a convenient time, generally after the system
 *        has been run in test for a while. 
 * 
 *  \section task_intrn Internal Organization
 *    At any time, a task is in both a <i>user state</i> and an <i>operational 
 *    state</i>. The user state is the state which the user manipulates; it chooses 
 *    the action that the run() method will take each time it is called by the 
 *    scheduler. The user state is the state which appears in the user's state 
 *    transition diagrams. The operational state shows how the scheduler is treating a
 *    task - whether the task is running, waiting for its time to run, ready to run, 
 *    suspended, etc. Operational states for all tasks follow the "state diagram" 
 *    below. 
 *
 *  \dot
 *  digraph op_states {
 *    node [ shape = ellipse, fontname = Arial, fontsize = 10];
 *      waiting   [ label = "Waiting "  ];
 *      suspended [ label = "Suspended "];
 *      pending   [ label = "Pending "  ];
 *      running   [ label = "Running "  ];
 *      start	 [ label = "", shape = "plaintext" ];
 *    edge [ arrowhead = "normal", style = "solid", fontname = Courier, fontsize = 10 ];
 *      start->waiting	 [ label = " start " ];
 *      waiting->running   [ label = " run() " ];
 *      running->waiting   [ label = " run()  finished " ];
 *      waiting->suspended [ label = "suspend() " ];
 *      suspended->waiting [ label = "resume() " ];
 *      running->pending   [ label = "run_again_ASAP() " ];
 *      pending->running   [ label = " run() " ];
 *  }
 *  \enddot
 *    Blocked tasks (due to resource contention) are not currently implemented. 
 *
 *    When a task is run cooperatively (rather than by interrupts), it is run when the
 *    method schedule() is called from within the main while loop in the main() 
 *    routine. The schedule() method belongs to class stl_task and generally does not 
 *    need to be modified by the user. 
 */

class stl_task
{
	private:
		/// This variable, shared by all tasks, counts serial numbers during creation
		static char serial_counter;

		/// This is the automatically assigned serial number of this task
		char serial_number;

		/// This is the operational state (running, suspended, etc.) of this task
		task_op_state op_state;

		/// This saves the previous operational state of a suspended task
		task_op_state save_op_state;

		/// This is the state (as seen by the user) in which this task is right now
		char current_state;

	protected:
		/// This is a reference to the device driver which keeps track of real time
		task_timer& the_timer;

		/// This is the time at which the task should next be run
		time_stamp next_run_time;

		/// This is the time interval between runs of the task
		time_stamp interval;

	public:
		// The constructor sets time interval between runs and debug port (if used)
		stl_task (task_timer&, const time_stamp&);

		// This metod sets or changes the time interval between runs of the task
		void set_interval (const time_stamp&);

		// This method sets the next time the task is to run
		void set_next_run_time (const time_stamp&);

		// This method is called to give a task an opportunity to run its run() method
		virtual task_op_state schedule (time_stamp* = NULL);

		virtual char run (char);			// Base method which the user overloads
		void suspend (void);				// Set operational state to suspended
		void resume (void);					// Un-suspend a task so it can run again
		void set_initial_state (char);		// Set a new state in which to start up

		/** This method returns the task's automatically assigned serial number. 
		 *  @return The task's serial number
		 */
		char get_serial_number (void) { return (serial_number); }

		/** This method returns the state in which the task is currently. The state 
		 *  cannot directly be changed by the user; it can only be changed through
		 *  the returned value from the run() method.
		 *  @return The state in which the task is when this method is called
		 */
		char get_current_state (void) { return (current_state); }

		/** This method returns the task's current operational state. The operational
		 *  state isn't the same as the state transition logic state; it's a separate
		 *  variable which controls if the task is running at a given time. 
		 *  @return The task's current operational state 
		 */
		task_op_state get_op_state (void) { return (op_state); }

		/** This method will cause the task to run again as soon as it can instead of
		 *  waiting for the given time interval. 
		 */
		inline void run_again_ASAP (void) { op_state = TASK_PENDING; }
		
		/** This method tells whether the task needs to run again as soon as possible
		 *  or not. It is convenient to use when determining if the processor should
		 *  be put to sleep for a while.
		 *  @return True if the task needs to run soon, false if it does not
		 */
		inline bool ready (void) 
			{ return (op_state == TASK_PENDING  || op_state == TASK_RUNNING); }

		void error_stop (char const*);	 	// Complain and stop the processor

	// The following block is only compiled if execution time profiling has been 
	// enabled for this project by setting -DSTL_PROFILE in the Makefile
	#ifdef STL_PROFILE
		protected:
			time_stamp max_time;			///< Maximum time function took to run
			time_stamp min_time;			///< Time for the quickest function run
			time_stamp avg_time;			///< Stamp holds recently computed average
			time_stamp run_time_sum;		///< Sum of measured function run times
			uint32_t runs;					///< Number of runs that have been measured
			time_stamp start_time;			///< Time at which measurement was started

		public:
			void clear_profiler (void);		// Clear statistical data items
			void start_profiler (void);		// Begin a measurement run
			void end_profiler (void);		// End a measurement and record times

			// This method returns the duration of the longest recorded run of the
			// code which is being profiled. 
			const time_stamp& get_max_run_time (void);

			// This method returns the duration of the shortest recorded run of the 
			// code which is being profiled. 
			const time_stamp& get_min_run_time (void);

			// This method returns the average duration of all the runs of the code
			// which is being profiled. 
			const time_stamp& get_avg_run_time (void);

			/** This method returns the number of runs of run() whose execution time
			 *  has been measured in the current sample set.
			*  @return The number of runs which have been measured
			*/
			uint32_t get_num_runs (void) { return (runs); }
	#endif  // STL_PROFILE
};

// This overloaded operator prints information about a task to a serial device
base_text_serial& operator<< (base_text_serial&, stl_task&);

#endif // _STL_TASK_H_
