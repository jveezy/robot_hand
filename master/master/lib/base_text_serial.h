//*************************************************************************************
/** \file base_text_serial.h
 *    This file contains a base class for devices which send information in text form
 *    over serial devices in an AVR microcontroller. Example devices are serial ports
 *    (both traditional RS-232 ports and USB-serial adapters) and wireless modems. 
 *
 *  Revised:
 *    \li 04-03-2006 JRR For updated version of compiler
 *    \li 06-10-2006 JRR Ported from C++ to C for use with some C-only projects
 *    \li 08-11-2006 JRR Some bug fixes
 *    \li 03-02-2007 JRR Ported back to C++. I've had it with the limitations of C.
 *    \li 04-16-2007 JO  Added write (unsigned long)
 *    \li 07-19-2007 JRR Changed some character return values to bool, added m324p
 *    \li 01-12-2008 JRR Added code for the ATmega128 using USART number 1 only
 *    \li 02-13-2008 JRR Split into base class and device specific classes; changed
 *                       from write() to overloaded << operator in the "cout" style
 *    \li 05-12-2008 ALS Fixed bug in which signed numbers came out unsigned
 *    \li 07-05-2008 JRR Added configuration macro by which to change what "endl" is
 *    \li 07-05-2008 JRR Added 'ascii' and 'numeric' format codes
 *    \li 11-24-2009 JRR Changed operation of 'clrscr' to a function to work with LCD
 *    \li 11-26-2009 JRR Integrated floating point support into this file
 *    \li 12-16-2009 JRR Improved support for constant strings in program memory
 *
 *  Licenses:
 *    This file released under the Lesser GNU Public License, version 2. This program
 *    is intended for educational use only, but it is not limited thereto. This code
 *    incorporates elements from Xmelkov's ftoa_engine.h, part of the avr-libc source,
 *    and users must accept and comply with the license of ftoa_engine.h as well. 
 */
//-------------------------------------------------------------------------------------
/* The following is taken from ftoa_engine.h, part of the avr-libc source. 
   It is subject to the following copyright notice: 
    Copyright (c) 2005, Dmitry Xmelkov
    All rights reserved.
  
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
 
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of the copyright holders nor the names of
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
 
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
	IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
	INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
	NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
	PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
	WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
	POSSIBILITY OF SUCH DAMAGE. */
//*************************************************************************************

/// This define prevents this .h file from being included more than once in a .cc file
#ifndef _BASE_TEXT_SERIAL_H_
#define _BASE_TEXT_SERIAL_H_

#include <avr/pgmspace.h>					// Header for program-space (Flash) data

// Uncomment this line to enable floating point handling by base_text_serial; comment
// it out if you don't need floating point and would like to save lots of memory
#include <math.h>


// The following code block is only compiled if <math.h> has been included
#ifdef M_SQRT2
	extern "C"
	{
		int __ftoa_engine (double val, char *buf,
						unsigned char prec, unsigned char maxdgs);
	}

	#define FTOA_MINUS		1				///< Status code for a negative number
	#define FTOA_ZERO		2				///< Status code for zero
	#define FTOA_INF		4				///< Status code for an infinite result
	#define FTOA_NAN		8				///< Status code for not-a-number
	#define FTOA_CARRY		16				///< Status code for carrying something(?)
#endif // M_SQRT2


/** \brief This define selects what will be sent when a program sends out "endl".
 *  Different recieving programs want different end-of-line markers. Traditionally, 
 *  UNIX uses "\r" while PC's use "\r\n" and Macs use "\n" (I think). */
#define ENDL_STYLE	  "\r\n"

/** \brief This define selects the character which asks a terminal to clear its screen.
 *  This is usually a control-L, which is ASCII character number 12.
 */

#define CLRSCR_STYLE	((unsigned char)12)

/** This define allows strings to be created in program memory and tells the puts()
 *  method to find them in program memory. This saves memory compared to copying all 
 *  the strings to SRAM, which is the rather wasteful default treatment.  See link(s),
 *  noting that you'll have to paste some multi-part links together because we run out
 *  of space in the comment box in the source code...
 *  \li http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=57011
        &start=all&postdays=0&postorder=asc
 */

#define PMS(s)  _p_str << \
	__extension__({static const char __c[] PROGMEM = s; &__c[0]; }) 

// 	__extension__({ static char __c[] PROGMEM = (s); &__c[0]; })


//-------------------------------------------------------------------------------------
/** This enumeration is used to change the display base for the output stream from the
 *  default of 10 (decimal) to and from 2 (binary), 8 (octal), or 16 (hexadecimal).
 *  Also defined are endl for end-of-line and send_now to tell packet based devices to
 *  transmit a packet as soon as possible. The base is changed for the next printing 
 *  of one number only, then the base reverts to the default value of base 10. Codes
 *  are also provided to allow printing of 8-bit numbers as ASCII characters and to 
 *  tell some devices to send packets of data immediately. 
 */

typedef enum {
	bin,					///< Print following numbers in base 2 (binary)
	oct,					///< Print following numbers in base 8 (octal)
	dec,					///< Print following numbers in base 10 (decimal)
	hex,					///< Print following numbers in base 16 (hexadecimal)
	ascii,					///< Print an ASCII character rather than an 8-bit number
	numeric,				///< Print 8-bit numbers as numeric values, not ASCII
	endl,					///< Print a carriage return and linefeed
	clrscr,					///< Send a control-L which clears some terminal screens
	send_now,				///< Tell some devices to send or save data immediately
	manip_set_precision,	///< Set the precision for printing floating point numbers
	_p_str					///< The following string is in program (flash) memory
} ser_manipulator;


//-------------------------------------------------------------------------------------
// This function sets the number of digits to be printed after the decimal point, then
// notifies a serial port that it should set its precision accordingly

ser_manipulator setprecision (unsigned char);


//-------------------------------------------------------------------------------------
/** This is a base class for lots of serial devices which send text over some type of
 *  communication interface. Descendents of this class will be able to send text over
 *  serial lines, radio modems or modules, and whatever other communication devices
 *  might be appropriate. 
 *
 *  The most important methods of this class are the overloaded "<<" operators which 
 *  convert different types of numbers into printable strings, then print them using
 *  methods which are overloaded by descendent classes. Methods are provided which 
 *  transmit most types of integers and floating point numbers; floating point numbers
 *  are only handled if \<math.h\> is defined near the beginning of base_text_serial.h
 *  because one often wants to turn off floating point support in order to save 
 *  memory. Other classes may implement methods to print themselves to any descendent
 *  of this class; see stl_timer.h/cc for examples in which time stamps print the time
 *  conveniently to any serial device. 
 *
 *  The ability to transmit characters is provided by descendent classes. Methods 
 *  which will be inherited or overridden by descendents include the following: 
 *    \li ready_to_send () - Checks if the port is ready to transmit a character
 *    \li putchar() - Sends a single character over the communications line
 *    \li puts() - Sends a character string
 *  Other methods may optionally be overridden; for example clear_screen() is only
 *  needed by devices which have a screen, and some devices may be read-only or 
 *  write-only, and some methods may not be needed. 
 */

class base_text_serial
{
	// Private data and methods are accessible only from within this class and 
	// cannot be accessed from outside -- even from descendents of this class
	private:

	// Protected data and methods are accessible from this class and its descendents
	// only
	protected:
		/// This is the currently used base for converting numbers to text.
		unsigned char base;
		
		/** If this variable is set true, print characters as ASCII characters, not
		 *  as numbers. */
		bool print_ascii;

		/** If this variable is true, the next string to be printed will be found in
		 * program (flash) memory, not data memory. */
		bool pgm_string;

		/** This is the number of digits after a decimal point to be printed when a
		 *  floating point number is being converted to text. */
		char precision;

	// Public methods can be called from anywhere in the program where there is a 
	// pointer or reference to an object of this class
	public:
		base_text_serial (void);			// Simple constructor doesn't do much
		virtual bool ready_to_send (void);  // Virtual and not defined in base class
		virtual bool putchar (char) {}	 	///< Virtual and not defined in base class
		virtual void puts (char const*) {}	///< Virtual and not defined in base class
		virtual bool check_for_char (void); // Check if a character is in the buffer
		virtual char getchar (void);		// Get a character; wait if none is ready
		virtual void transmit_now (void);	// Immediately transmit any buffered data
		virtual void clear_screen (void);	// Clear a display screen if there is one

		// The overloaded left-shift operators convert numbers to strings and send the 
		// strings out the serial device; manipulators change the formatting
		base_text_serial& operator<< (bool);
		base_text_serial& operator<< (const char*);
		base_text_serial& operator<< (unsigned char);
		base_text_serial& operator<< (char num);
		base_text_serial& operator<< (unsigned int);
		base_text_serial& operator<< (int);
		base_text_serial& operator<< (unsigned long);
		base_text_serial& operator<< (long);
		base_text_serial& operator<< (unsigned long long);
		#ifdef M_SQRT2 // Automatically include this code if <math.h> has been included
			base_text_serial& operator<< (float);
			base_text_serial& operator<< (double);
		#endif
		base_text_serial& operator<< (ser_manipulator);
};

#endif  // _BASE_TEXT_SERIAL_H_
