//*************************************************************************************
/** \file base_text_serial.cpp
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
 *    and users must accept and comply with the license of ftoa_engine.h as well. See
 *    base_text_serial.h for a copy of the relevant license terms. 
 */
//*************************************************************************************

#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include "base_text_serial.h"


//-------------------------------------------------------------------------------------
/** This variable, accessible to any function in this file, allows the nonmember 
 *  function setprecision() to communicate the number of digits after the decimal 
 *  point to a specific text serial port object. The setprecision() function will
 *  return a manipulator that causes a specific serial object (the one in whose cout
 *  style output line it is called) to read the value of this variable. 
 */

unsigned char bts_glob_prec = 3;


//-------------------------------------------------------------------------------------
/** This constructor sets up the base serial port object. It sets the default base for
 *  the conversion of numbers to text and the default format for converting chars. 
 */

base_text_serial::base_text_serial (void)
{
	base = 10;								// Numbers are shown as decimal by default
	print_ascii = false;					// Print 8-bit chars as numbers by default
	precision = 3;							// Print 3 digits after a decimal point
	pgm_string = false;						// Print strings from SRAM by default
}


//-------------------------------------------------------------------------------------
/** This function checks if the serial port transmitter is ready to send data.  It's 
 *  an empty base method, overridden by most serial devices. Some serial devices might
 *  always be ready to send data; those can not bother to override this method. 
 *  @return True if the serial port is ready to send, and false if not
 */

bool base_text_serial::ready_to_send (void)
{
	return (true);							// By default the port's always ready
}


//-------------------------------------------------------------------------------------
/** This base method just returns zero, because it shouldn't be called. There might be
 *  classes which only send characters and don't ever receive them, and this method
 *  could be left as a placeholder for those classes. 
 *  @return A zero (null character) which means nothing
 */

char base_text_serial::getchar (void)
{
	return ('\0');							// Nothing to return, really
}


//-------------------------------------------------------------------------------------
/** This method checks if there is a character in the serial port's receiver buffer.
 *  There isn't, as this base class isn't connected to a buffer; descendent classes
 *  will override this method and check a real buffer for real characters. 
 *  @return False, because no character will ever be available
 */

bool base_text_serial::check_for_char (void)
{
	return (false);
}


//-------------------------------------------------------------------------------------
/** This is a base method for causing immediate transmission of a buffer full of data.
 *  The base method doesn't do anything, because it will be implemented in descendent
 *  classes which have no buffers, send everything immediately by default, and don't 
 *  need to respond to calls for immediate transmission. 
 */

void base_text_serial::transmit_now (void)
{
}


//-------------------------------------------------------------------------------------
/** This is a base method to clear a display screen, if there is one. It is called 
 *  when the format modifier 'clrscr' is inserted in an output line. Descendant
 *  classes which send things to displays should respond by clearing themselves.
 */

void base_text_serial::clear_screen (void)
{
}


//-------------------------------------------------------------------------------------
/** This method writes the string whose first character is pointed to by the given
 *  character pointer to the serial device. It acts in about the same way as puts(). 
 *  As with puts(), the string must have a null character (ASCII zero) at the end. 
 *  @param string Pointer to the string to be written
 */

base_text_serial& base_text_serial::operator<< (const char* string)
{
	// If the program-string variable is set, this string is to be found in program
	// memory rather than data memory
	if (pgm_string)
	{
		pgm_string = false;
		while (char ch = pgm_read_byte_near (string++))
			putchar (ch);

	}
	// If the program-string variable is not set, the string is in RAM and printed
	// in the normal way
	else
	{
		while (*string) putchar (*string++);
	}

	return (*this);
}


//-------------------------------------------------------------------------------------
/** This method writes a boolean value to the serial port as a character, either "T"
 *  or "F" depending on the value. 
 *  @param value The boolean value to be written
 */

base_text_serial& base_text_serial::operator<< (bool value)
{
	if (value)
		putchar ('T');
	else
		putchar ('F');

	return (*this);;   
}


//-------------------------------------------------------------------------------------
/** This method writes a character to the serial port as a text string showing the 
 *  8-bit unsigned number in that character. If the format control term 'ascii' was
 *  called prior to this printing, the character will be printed as an ASCII character.
 *  @param num The 8-bit number or character to be sent out
 */

base_text_serial& base_text_serial::operator<< (unsigned char num)
{
	unsigned char temp_char;				// Temporary storage for a nibble

	if (print_ascii)
	{
		putchar (num);
	}
	else if (base == 2)
	{
		for (unsigned char bmask = 0x80; bmask != 0; bmask >>= 1)
		{
			if (num & bmask) putchar ('1');
			else			 putchar ('0');
		}
	}
	else if (base == 16)
	{
		temp_char = (num >> 4) & 0x0F;
		putchar ((temp_char > 9) ? temp_char + ('A' - 10) : temp_char + '0');
		temp_char = num & 0x0F;
		putchar ((temp_char > 9) ? temp_char + ('A' - 10) : temp_char + '0');
	}
	else
	{
		char out_str[9];
		utoa ((unsigned int)num, out_str, base);
		puts (out_str);
	}

	return (*this);
}


//-------------------------------------------------------------------------------------
/** This method writes a character to the serial port as a text string showing the 
 *  8-bit signed number in that character, unless the 'ascii' manipulator has been 
 *  used to put the port in ascii text mode, in which case the character is printed
 *  directly. 
 *  @param num The 8-bit number to be sent out
 */

base_text_serial& base_text_serial::operator<< (char num)
{
	char out_str[5];

	if (print_ascii) 
		putchar (num);
	else
	{
		if (base == 10)
		{
			itoa ((int)num, out_str, 10);
			puts (out_str);
		}
		else
			*this << (unsigned char)num;
	}

	return (*this);
}


//-------------------------------------------------------------------------------------
/** This method writes an integer to the serial port as a text string showing the 
 *  16-bit unsigned number in that integer. 
 *  @param num The 16-bit number to be sent out
 */

base_text_serial& base_text_serial::operator<< (unsigned int num)
{
	if (base == 16 || base == 8 || base == 2)
	{
		union {
			unsigned long whole;
			unsigned char bits[2];
		} parts;
		parts.whole = num;
		*this << parts.bits[1] << parts.bits[0];
	}
	else
	{
		char out_str[17];
		utoa (num, out_str, base);
		puts (out_str);
	}

	return (*this);
}


//-------------------------------------------------------------------------------------
/** This method writes an integer to the serial port as a text string showing the 
 *  16-bit signed number in that integer. 
 *  @param num The 16-bit number to be sent out
 */

base_text_serial& base_text_serial::operator<< (int num)
{
	if (base != 10)
	{
		*this << (unsigned int)num;
	}
	else
	{
		char out_str[17];

		itoa (num, out_str, base);
		puts (out_str);
	}

	return (*this);
}


//-------------------------------------------------------------------------------------
/** This method writes an unsigned long integer to the serial port as a text string 
 *  showing the 32-bit unsigned number in that long integer. 
 *  @param num The 32-bit number to be sent out
 */

base_text_serial& base_text_serial::operator<< (unsigned long num)
{
	if (base == 16 || base == 8 || base == 2)
	{
		union {
			unsigned long whole;
			unsigned char bits[4];
		} parts;
		parts.whole = num;
		*this << parts.bits[3] << parts.bits[2] << parts.bits[1] << parts.bits[0];
	}
	else
	{
		char out_str[33];
		ultoa ((long)num, out_str, base);
		puts (out_str);
	}

	return (*this);
}


//-------------------------------------------------------------------------------------
/** This method writes a long integer to the serial port as a text string showing the 
 *  32-bit signed number in that long integer. 
 *  @param num The 32-bit number to be sent out
 */

base_text_serial& base_text_serial::operator<< (long num)
{
	if (base != 10)
	{
		*this << (unsigned long)num;
	}
	else
	{
		char out_str[34];
		ltoa ((long)num, out_str, base);
		puts (out_str);
	}

	return (*this);
}


//-------------------------------------------------------------------------------------
/** This method writes a long long (64-bit) unsigned integer to the serial port as a 
 *  text string.  It only writes such numbers in unsigned hexadecimal format. The
 *  number is written by breaking it into two unsigned longs, writing them in order.
 *  @param num The 64-bit number to be sent out
 */

base_text_serial& base_text_serial::operator<< (unsigned long long num)
{
	union {
		unsigned long long whole;
		unsigned char bits[8];
	} parts;
	parts.whole = num;
	*this << parts.bits[7] << parts.bits[6] << parts.bits[5] << parts.bits[4]
		  << parts.bits[3] << parts.bits[2] << parts.bits[1] << parts.bits[0];

	return (*this);
}

#ifdef M_SQRT2 // Automatically include this code if <math.h> has been included

//-------------------------------------------------------------------------------------
/** This method writes a single-precision floating point number to the serial port in
 *  exponential format (always with the 'e' notation). It calls the utility function 
 *  __ftoa_engine, which is hiding in the AVR libraries, used by the Xprintf() 
 *  functions when they need to convert a float into text. 
 *  @param num The floating point number to be sent out
 */

base_text_serial& base_text_serial::operator<< (float num)
{
	char digits = precision;
	char buf[20];
	char* p_buf = buf;

	int exponent = __ftoa_engine ((double)num, buf, digits, 16);
	uint8_t vtype = *p_buf++;
	if (vtype & FTOA_NAN)
	{
		*this << "  NaN";
		return (*this);
	}

	// Display the sign if it's negative
	if (vtype & FTOA_MINUS)
		putchar ('-');

	// Show the mantissa
	putchar (*p_buf++);
	if (digits)
		putchar ('.');
	while ((digits-- > 0) && *p_buf)
		putchar (*p_buf++);

	// Now display the exponent
	putchar ('e');
	if (exponent > 0)
		putchar ('+');
	*this << exponent;
}


//-------------------------------------------------------------------------------------
/** This method writes a double-precision floating point number to the serial port in
 *  exponential format (always with the 'e' notation). It calls the utility function 
 *  __ftoa_engine, which is hiding in the AVR libraries, used by the Xprintf() 
 *  functions when they need to convert a double into text. 
 *  @param num The double-precision floating point number to be sent out
 */

base_text_serial& base_text_serial::operator<< (double num)
{
	char digits = precision;
	char buf[20];
	char* p_buf = buf;

	int exponent = __ftoa_engine (num, buf, digits, 16);
	uint8_t vtype = *p_buf++;
	if (vtype & FTOA_NAN)
	{
		*this << "  NaN";
		return (*this);
	}

	// Display the sign if it's negative
	if (vtype & FTOA_MINUS)
		putchar ('-');

	// Show the mantissa
	putchar (*p_buf++);
	if (digits)
		putchar ('.');
	do
		putchar (*p_buf++);
	while (--digits && *p_buf);

	// Now display the exponent
	putchar ('e');
	if (exponent > 0)
		putchar ('+');
	*this << exponent;
}

#endif // M_SQRT2


//-------------------------------------------------------------------------------------
/** This function sets the global precision value, then returns a manipulator which 
 *  causes a serial object to change its floating point precision (the number of digits
 *  after the decimal point which will be printed). The new precision is "sticky" in
 *  that its value persists until this function is called again to change it again. 
 *  @param digits A new value for the number of digits after the decimal point;
 *		a maximum of 7 digits are allowed
 *  @return The serial manipulator called "manip_set_precision"
 */

ser_manipulator setprecision (unsigned char digits)
{
	if (digits > 7) digits = 7;
	bts_glob_prec = digits;

	return (manip_set_precision);
}


//-------------------------------------------------------------------------------------
/** This overload allows manipulators to be used to change the base of displayed 
 *  numbers to binary, octal, decimal, or hexadecimal. Also, and endline is provided
 *  with the name "endl" and the code "send_now" causes immediate transmission by
 *  devices which save stuff to be transmitted in buffers. 
 *  @param new_manip The serial manipulator which was given
 */

base_text_serial& base_text_serial::operator<< (ser_manipulator new_manip)
{
	switch (new_manip)
	{
		case (bin):							// Print integers in binary
			base = 2;
			break;
		case (oct):							// Print integers in octal
			base = 8;
			break;
		case (dec):							// Print integers in decimal
			base = 10;
			break;
		case (hex):							// Print integers in hexadecimal
			base = 16;
			break;
		case (ascii):						// Print chars as ASCII letters, etc.
			print_ascii = true;
			break;
		case (numeric):						// Print chars as numbers
			print_ascii = false;
			break;
		case (endl):						// Send an end-of-line
			puts (ENDL_STYLE);
			break;
		case (clrscr):						// Send a clear-screen code
			clear_screen ();
			break;
		case (send_now):					// Send whatever's in the send buffer
			transmit_now ();
			break;
		case (_p_str):						// The next string is in program memory
			pgm_string = true;
	};

	return (*this);
}
