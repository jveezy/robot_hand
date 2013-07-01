//============================================================================================================
/** \file serial.cc
 *	This file contains a program for an ATtiny2313 serial driver.
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
 *    \li 01-30-2009 JRR Added class with port setup in constructor
 *    \li 04-09-2009 JRR Changed to a simpler baud rate calculation formula
 *    \li 04-08-2011 JV	New file based on Dr. Ridgely's base232.h file
 *
 *  License:
 *	This file released under the Lesser GNU Public License, version 2. This program
 *	is intended for educational use only, but it is not limited thereto. 
 */
//============================================================================================================

#include <avr/io.h>
#include "serial.h"

/** This constructor sets up a USART serial port for the ATtiny2313.
 */

serial::serial (void)
{
	p_UDR = &UDR;		// Input/Output data register
	p_USR = &UCSRA;	// Control and Status Register A
	p_UCR = &UCSRB;	// Control and Status Register B
	
	// Setup USART Control and Status Register B (UCSRB)
	UCSRB = (1 << RXEN) | (1 << TXEN);		// Enable RX and TX
	
	// Setup USART Control and Status Register C (UCSRC)
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0);	// Set UCSZ bits to 8 bit character size
	
	// Calculate baud rate divisor
	UBRRH = (unsigned char)(BAUD_DIV >> 8);
	UBRRL = (unsigned char) BAUD_DIV;
	
	// Setup USART Control and Status Register A (UCSRA)
	UCSRA |= U2X;	// Double Speed
	
	// Create Masks
	mask_UDRE = (1 << UDRE);
	mask_RXC = (1 << RXC);
	mask_TXC = (1 << TXC);
}

/** This method will send data out the serial port. 
 *  @param data_out The byte to send out.
 */
void serial::send (unsigned char data_out)
{
	*p_UDR = data_out;
}

/** This method checks if the serial port transmitter is ready to send data.  It 
 *  tests whether transmitter buffer is empty. 
 *  @return True if the serial port is ready to send, and false if not
 */
bool serial::ready_to_send (void)
{
	if (*p_USR & mask_UDRE)
		return (true);

	return (false);
}

/** This method checks if the serial port is currently sending a character. Even if the
 *  buffer is ready to accept a new character, the port might still be busy sending the
 *  last one; it would be a bad idea to put the processor to sleep before the character
 *  has been sent. 
 *  @return True if the port is currently sending a character, false if it's idle
 */
bool serial::is_sending (void)
{
	if (*p_USR & mask_TXC)
		return (false);
	else
		return (true);
}

/** This method checks if the serial port has received a character and stored it in the buffer.
 *  @return True if the port has a character in the buffer, false if it does not
 */
bool serial::check_for_char (void)
{
	if (*p_USR & mask_RXC)
		return (true);
	else
		return (false);
}

/** This method gets one character from the serial port, if one is there.  If not, it
 *  waits until there is a character available.  This can sometimes take a long time
 *  (even forever), so use this function carefully.  One should almost always use
 *  check_for_char() to ensure that there's data available first. 
 *  @return The character which was found in the serial port receive buffer
 */
char serial::getchar (void)
{
	//  Wait until there's something in the receiver buffer
	while ((*p_USR & mask_RXC) == 0);

	//  Return the character retreived from the buffer
	return (*p_UDR);
}

