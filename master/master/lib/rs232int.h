//*************************************************************************************
/** \file rs232int.h
 *    This file contains a class which allows the use of a serial port on an AVR 
 *    microcontroller. This version of the class uses the serial port receiver 
 *    interrupt and a buffer to allow characters to be received in the background.
 *    The port is used in "text mode"; that is, the information which is sent and 
 *    received is expected to be plain ASCII text, and the set of overloaded left-shift 
 *    operators "<<" in base_text_serial.* can be used to easily send all sorts of data 
 *    to the serial port in a manner similar to iostreams (like "cout") in regular C++. 
 *
 *  Revised:
 *    \li 04-03-2006 JRR For updated version of compiler
 *    \li 06-10-2006 JRR Ported from C++ to C for use with some C-only projects; also
 *        serial_avr.h now has defines for compatibility among lots of AVR variants
 *    \li 08-11-2006 JRR Some bug fixes
 *    \li 03-02-2007 JRR Ported back to C++. I've had it with the limitations of C.
 *    \li 04-16-2007 JO  Added write (unsigned long)
 *    \li 07-19-2007 JRR Changed some character return values to bool, added m324p
 *    \li 01-12-2008 JRR Added code for the ATmega128 using USART number 1 only
 *    \li 02-14-2008 JRR Split between base_text_serial and rs232 files
 *    \li 05-31-2008 JRR Changed baud calculations to use CPU_FREQ_MHz from Makefile
 *    \li 06-01-2008 JRR Added getch_tout() because it's needed by 9Xstream modems
 *    \li 07-05-2008 JRR Changed from 1 to 2 stop bits to placate finicky receivers
 *    \li 12-22-2008 JRR Split off stuff in base232.h for efficiency
 *    \li 06-30-2009 JRR Received data interrupt and buffer added
 *
 *  License:
 *		This file is released under the Lesser GNU Public License, version 2. This 
 *		program is intended for educational use only, but it is not limited thereto. 
 */
//*************************************************************************************

/// This define prevents this .h file from being included more than once in a .cc file
#ifndef _RS232_H_
#define _RS232_H_

#include <avr/interrupt.h>					// Header for AVR interrupt programming
#include "base232.h"						// Grab the base RS232-style header file
#include "base_text_serial.h"				// Pull in the base class header file


// If the chip for which we are compiling has dual serial ports, define two character
// received interrupts, one for each port. This is for ATmega128 and 324P and similar
// chips. This section may need to be expanded to work with some other chips
#if defined UCSR0A
	#define RSI_CHAR_RECV_INT_0 USART0_RX_vect 
	#define RSI_CHAR_RECV_INT_1 USART1_RX_vect 
// There's no second serial port, so define one port's interrupt. This is for ATmega8
// and ATmega32 and similar chips. This section will need to be expanded if other
// single-UART/USART chips are used, as the vector may have a different name
#else
	#define RSI_CHAR_RECV_INT_0 USART_RXC_vect
#endif


/// This is the size of the buffer which holds characters received by the serial port.
#define RSINT_BUF_SIZE		128


//-------------------------------------------------------------------------------------
/** This class controls a UART (Universal Asynchronous Receiver Transmitter), a common 
 *  serial interface. It talks to old-style RS232 serial ports (through a voltage
 *  converter chip such as a MAX232) or through a USB to serial converter such as a
 *  FT232RL chip. The UART is also sometimes used to communicate directly with other
 *  microcontrollers, sensors, or wireless modems. 
 */

class rs232 : public base_text_serial, public base232
{
	// This protected data can only be accessed from this class or its descendents
	protected:
		uint8_t port_num;					///< The USART number, 0 or 1

	// Public methods can be called from anywhere in the program where there is a 
	// pointer or reference to an object of this class
	public:
		// The constructor sets up the UART, saving its baud rate and port number
		rs232 (unsigned int = 9600, unsigned char = 0);

		/// This method writes one character to the serial port.
		bool putchar (char);

		void puts (char const*);			// Write a string constant to serial port
		bool check_for_char (void);			// Check if a character is in the buffer
		char getchar (void);				// Get a character; wait if none is ready
		void clear_screen (void);			// Send the 'clear display screen' code
// 		char getch_tout (unsigned int);		// Try a given number of times to get char
};

#endif  // _RS232_H_
