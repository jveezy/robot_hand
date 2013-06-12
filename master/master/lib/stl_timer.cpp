//**************************************************************************************
/** \file stl_timer.cpp
 *    This file contains a class which runs a task timer whose resolution is one
 *    microsecond. The timer is used to help schedule the execution of tasks' run()
 *    functions. The functions can be run from a main loop by checking periodically 
 *    if a task time has expired, or they can be called directly from the timer 
 *    interrupt service routine in this file, or they can be called from some other 
 *    hardware interrupt service routine; in the last case this file isn't involved. 
 *
 *  Revisions:
 *    \li 08-07-2007 JRR Created this file as daytimer.* with 1 second interrupts
 *    \li 08-08-2007 JRR Added event triggers
 *    \li 12-23-2007 JRR Made more general by allowing faster interrupt rates
 *    \li 01-05-2008 JRR Converted from time-of-day version to microsecond version
 *    \li 03-27-2008 JRR Added operators + and - for time stamps
 *    \li 03-31-2008 JRR Merged in stl_us_timer (int, long) and set_time (int, long)
 *    \li 05-15-2008 JRR Changed to use Timer 3 so Timer 1 can run motor PWM's
 *    \li 05-31-2008 JRR Changed time calculations to use CPU_FREQ_MHz from Makefile
 *    \li 01-04-2009 JRR Now uses CPU_FREQ_Hz (rather than MHz) for better precision
 *    \li 11-24-2009 JRR Changed CPU_FREQ_Hz to F_CPU to match AVR-LibC's name
 *
 *  License:
 *    This file copyright 2007 by JR Ridgely. It is released under the Lesser GNU
 *    public license, version 2.
 */
//**************************************************************************************

#include <stdlib.h>							// Used for itoa()
#include <string.h>							// Header for character string functions
#include <avr/interrupt.h>					// For using interrupt service routines

#include "base_text_serial.h"				// Base for text-type serial port objects
#include "stl_timer.h"						// Header for this file


//--------------------------------------------------------------------------------------
// This variables is only used to allow the interrupt service routine to update the
// measured time whenever a timer interrupt occurs, and allow a timer object to read
// the time. The user should not have any reason to read or write it.

/** This variable holds the number of times the hardware timer has overflowed. This
 *  number is equivalent to the upper 16 bits of a 32-bit timer, and is so used. */

uint16_t ust_overflows = 0;


//--------------------------------------------------------------------------------------
/** This constructor creates a time stamp object, with the time set to zero. 
 */

time_stamp::time_stamp (void)
{
	set_time (0L);
}


//--------------------------------------------------------------------------------------
/** This constructor creates a time stamp object and fills the time stamp's variables
 *  with the given values.
 *  @param a_time A 32-bit time number with which the time stamp is filled
 */

time_stamp::time_stamp (uint32_t a_time)
{
	set_time (a_time);
}


//--------------------------------------------------------------------------------------
/** This constructor creates a time stamp object and fills the time stamp's variables
 *  with the number of seconds and microseconds given. 
 *  @param sec A 16-bit number of seconds to preload into the time stamp
 *  @param microsec A 32-bit number of microseconds to preload into the time stamp
 */

time_stamp::time_stamp (uint16_t sec, uint32_t microsec)
{
	set_time (sec, microsec);
}


//--------------------------------------------------------------------------------------
/** This method fills the time stamp with the given value.
 *  @param a_time A 32-bit time number with which the time stamp is filled
 */

void time_stamp::set_time (uint32_t a_time)
{
	data.whole = a_time;
}


//--------------------------------------------------------------------------------------
/** This method fills the time stamp with the given numbers of seconds and microseconds.
 *  @param sec A 16-bit number of seconds to preload into the time stamp
 *  @param microsec A 32-bit number of microseconds to preload into the time stamp
 */

void time_stamp::set_time (uint16_t sec, uint32_t microsec)
{
	data.whole = (microsec * (F_CPU / 1000000UL)) / 8UL;
	data.whole += (uint16_t)sec * (F_CPU / 8UL);
}


//--------------------------------------------------------------------------------------
/** This method allows one to get the time reading from this time stamp as a long 
 *  integer containing the number of time ticks. 
 *  @return The time stamp's raw data
 */

uint32_t time_stamp::get_raw_time (void)
{
	return (data.whole);
}


//--------------------------------------------------------------------------------------
/** This method returns the number of seconds in the time stamp.
 *  @return The number of whole seconds in the time stamp
 */

uint16_t time_stamp::get_seconds (void)
{
	return ((uint16_t)(data.whole / (F_CPU / 8)));
}


//--------------------------------------------------------------------------------------
/** This method returns the number of microseconds in the time stamp, after the seconds
 *  are subtracted out. 
 *  @return The number of microseconds, that is, the fractional part of the time stamp
 */

uint32_t time_stamp::get_microsec (void)
{
	return ((data.whole % (F_CPU / 8UL)) * 8 / (F_CPU / 1000000UL));
}


//--------------------------------------------------------------------------------------
/** This overloaded addition operator adds another time stamp's time to this one.  It 
 *  can be used to find the time in the future at which some event is to be caused to 
 *  happen, such as the next time a task is supposed to run. 
 *  @param addend The other time stamp which is to be added to this one
 *  @return The newly created time stamp
 */

time_stamp time_stamp::operator + (const time_stamp& addend)
{
	time_stamp ret_stamp;
	ret_stamp.data.whole = this->data.whole + addend.data.whole;

	return ret_stamp;
}


//--------------------------------------------------------------------------------------
/** This overloaded subtraction operator finds the duration between this time stamp's 
 *  recorded time and a previous one. 
 *  @param previous An earlier time stamp to be compared to the current one 
 *  @return The newly created time stamp
 */

time_stamp time_stamp::operator - (const time_stamp& previous)
{
	time_stamp ret_stamp;
	ret_stamp.data.whole = this->data.whole - previous.data.whole;

	return ret_stamp;
}


//--------------------------------------------------------------------------------------
/** This overloaded addition operator adds another time stamp's time to this one.  It 
 *  can be used to find the time in the future at which some event is to be caused to 
 *  happen, such as the next time a task is supposed to run. 
 *  @param addend The other time stamp which is to be added to this one
 */

void time_stamp::operator += (const time_stamp& addend)
{
	data.whole += addend.data.whole;
}


//--------------------------------------------------------------------------------------
/** This overloaded subtraction operator finds the duration between this time stamp's 
 *  recorded time and a previous one. Note that the data in this timestamp is 
 *  replaced with that duration. 
 *  @param previous An earlier time stamp to be compared to the current one 
 */

void time_stamp::operator -= (const time_stamp& previous)
{
	data.whole -= previous.data.whole;
}


//--------------------------------------------------------------------------------------
/** This overloaded division operator divides a time stamp by the given integer. Note 
 *  that the data in this timestamp is replaced by the quotient. 
 *  @param divisor The number by which the time stamp is to be divided
 */

void time_stamp::operator /= (const uint32_t& divisor)
{
	data.whole /= divisor;
}


//--------------------------------------------------------------------------------------
/** This overloaded equality test operator checks if all the time measurements in some
 *  other time stamp are equal to those in this one. 
 *  @param other A time stamp to be compared to this one 
 *  @return True if the time stamps contain equal data, false if they don't
 */

bool time_stamp::operator == (const time_stamp& other)
{
	if (other.data.whole == data.whole)
	{
		return (true);
	}
	else
	{
		return (false);
	}
}


//--------------------------------------------------------------------------------------
/** This overloaded inequality operator checks if this time stamp is greater than or
 *  equal to another. The method used to check greater-than-ness needs to work across 
 *  timer overflows, so the following technique is used: subtract the other time stamp 
 *  from this one as unsigned 32-bit numbers, then check if the result is positive (in 
 *  which case this time is greater) or not. 
 *  @param other A time stamp to be compared to this one 
 *  @return True if this time stamp is greater than or equal to the other one
 */

bool time_stamp::operator >= (const time_stamp& other)
{
	int32_t difference = (int32_t)(data.whole - other.data.whole);
	if (difference >= 0L)
	{
		return (true);
	}
	else
	{
		return (false);
	}
}


//--------------------------------------------------------------------------------------
/** This operator tests if a time stamp is greater than this one. It subtracts the 
 *  other time stamp from this time stamp, then checks if the result is positive. 
 *  @param other A time stamp to be compared to this one 
 *  @return True if this time stamp is greater than the other one
 */

bool time_stamp::operator > (const time_stamp& other)
{
	int32_t difference = (int32_t)(data.whole - other.data.whole);
	if (difference > 0L)
	{
		return (true);
	}
	else
	{
		return (false);
	}
}


//--------------------------------------------------------------------------------------
/** This operator tests if a time stamp is less than or equal to this one. It subtracts 
 *  the other time stamp from this time stamp, then checks if the result is not
 *  positive. 
 *  @param other A time stamp to be compared to this one 
 *  @return True if this time stamp is less than or equal to the other one
 */

bool time_stamp::operator <= (const time_stamp& other)
{
	int32_t difference = (int32_t)(data.whole - other.data.whole);
	if (difference <= 0L)
	{
		return (true);
	}
	else
	{
		return (false);
	}
}


//--------------------------------------------------------------------------------------
/** This operator tests if a time stamp is greater than this one. It subtracts the 
 *  other time stamp from this time stamp, then checks if the result is negative. 
 *  @param other A time stamp to be compared to this one 
 *  @return True if this time stamp is less than the other one
 */

bool time_stamp::operator < (const time_stamp& other)
{
	int32_t difference = (int32_t)(data.whole - other.data.whole);
	if (difference < 0L)
	{
		return (true);
	}
	else
	{
		return (false);
	}
}



//--------------------------------------------------------------------------------------
/** This constructor creates a daytime task timer object.  It sets up the hardware timer
 *  to count at ~1 MHz and interrupt on overflow. Note that this method does not enable
 *  interrupts globally, so the user must call sei() at some point to enable the timer
 *  interrupts to run and time to actually be measured. 
 */

task_timer::task_timer (void)
{
	#ifdef TCNT3							// If Timer 3 exists, we'll use it
		TCCR3A = 0x00;						// Set to normal 16-bit counting mode
		TCCR3B = (1 << CS31);				// Set prescaler to main clock / 8
		#ifdef ETIMSK						// For ATmega128
			ETIMSK |= (1 << TOIE3);			// Set Timer 3 overflow interrupt enable
		#endif
		#ifdef TIMSK3						// For ATmega1281
			TIMSK3 |= (1 << TOIE3);			// Set Timer 3 overflow interrupt enable
		#endif
	#else									// No Timer 3, so use Timer 1
		TCCR1A = 0x00;						// Set to normal 16-bit counting mode
		TCCR1B = (1 << CS11);				// Set prescaler to main clock / 8
		#ifdef TIMSK1						// If there's a TIMSK1, set it
			TIMSK1 |= (1 << TOIE1);			// Enable Timer 1 overflow interrupt
		#else								// If no TIMSK1, there's a TIMSK
			TIMSK |= (1 << TOIE1);			// Enable Timer 1 overflow interrupt
		#endif
	#endif
}


//--------------------------------------------------------------------------------------
/** This method grabs the current time stamp from the hardware and overflow counters. 
 *  In order to prevent the data changing during the time when it's being read (which 
 *  would cause invalid data to be saved), interrupts are disabled while the time data 
 *  is being copied. 
 *  @param the_stamp Reference to a time stamp variable which will hold the time
 */

void task_timer::save_time_stamp (time_stamp& the_stamp)
{
	uint8_t temp_sreg = SREG;				// Store interrupt flag status
	the_stamp.data.half[0] = TMR_TCNT_REG;	// Get hardware count
	cli ();									// Prevent interruption
	the_stamp.data.half[1] = ust_overflows;	// Get overflow counter data
	SREG = temp_sreg;						// Re-enable interrupts if they were on
}


//--------------------------------------------------------------------------------------
/** This method saves the current time in the internal time stamp belonging to this 
 *  object, then returns a reference to the time stamp so that the caller can use it as
 *  a measurement of what the time is now. 
 */

time_stamp& task_timer::get_time_now (void)
{
	uint8_t temp_sreg = SREG;				// Store interrupt flag status
	now_time.data.half[0] = TMR_TCNT_REG;	// Get hardware count
	cli ();									// Prevent interruption
	now_time.data.half[1] = ust_overflows;	// Get overflow counter data
	SREG = temp_sreg;						// Re-enable interrupts if they were on

	return (now_time);						// Return a reference to the current time
}


//--------------------------------------------------------------------------------------
/** This method sets the timer to a given value. It's not likely that this method will
 *  be used, but it is provided for compatibility with other task timer implementations
 *  that measure times of day (in hours, minutes, and seconds) and do need to be set by
 *  user programs. 
 *  @param t_stamp A reference to a time stamp containing the time to be set
 */

bool task_timer::set_time (time_stamp& t_stamp)
{
	uint8_t temp_sreg = SREG;				// Store interrupt flag status
	cli ();									// Prevent interruption
	ust_overflows = t_stamp.data.half[1];
	TMR_TCNT_REG = t_stamp.data.half[0];
	SREG = temp_sreg;						// Re-enable interrupts if they were on
}


//--------------------------------------------------------------------------------------
/** This overloaded operator allows a time stamp to be printed on a serial device such
 *  as a regular serial port or radio module in text mode. This allows lines to be set
 *  up in the style of 'cout.' The timestamp is always printed as a decimal number. 
 *  @param serial A reference to the serial-type object to which to print
 *  @param stamp A reference to the time stamp to be displayed
 */

base_text_serial& operator<< (base_text_serial& serial, time_stamp& stamp)
{
	// Get the time in seconds and microseconds
	uint16_t seconds = stamp.get_seconds ();
	uint32_t microseconds = stamp.get_microsec ();

	serial << seconds;
	serial.putchar ('.');

	// For the digits in the fractional part, write 'em in backwards order. We can't
	// use itoa here because we need leading zeros
	for (uint32_t divisor = 100000; divisor > 0; divisor /= 10)
	{
		serial.putchar (microseconds / divisor + '0');
		microseconds %= divisor;
	}

	return (serial);
}


//--------------------------------------------------------------------------------------
/** This overloaded operator allows the task timer to print the current time on a serial 
 *  device such as a regular serial port or radio module in text mode. This allows lines 
 *  to be set up in the style of 'cout.' The printing is done by the time_stamp class;
 *  this method just calls the timer's get_time_now() method to get a time stamp, then 
 *  has the time stamp print itself. 
 *  @param serial A reference to the serial-type object to which to print
 *  @param tmr A reference to the timer whose time is to be displayed
 */

base_text_serial& operator<< (base_text_serial& serial, task_timer& tmr)
{
	serial << tmr.get_time_now ();
	return (serial);
}


//--------------------------------------------------------------------------------------
/** This is the interrupt service routine which is called whenever there is a compare
 *  match on the 16-bit timer's counter. Nearly all AVR processors have a 16-bit timer
 *  called Timer 1, and this is the one we use here. 
 */

ISR (TMR_intr_vect)
{
	ust_overflows++;
}
