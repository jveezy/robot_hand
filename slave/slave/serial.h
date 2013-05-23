//============================================================================================================
/** \file motor.h
 *	This file contains a header for an ATtiny2313 motor driver. The driver will output a PWM signal to a
 *	L293D motor driver chip along with two directional signals.
 *
 *  Revised:
 *    \li 04-09-2011 JV	Original file.
 *
 *  License:
 *	This file released under the Lesser GNU Public License, version 2. This program
 *	is intended for educational use only, but it is not limited thereto. 
 */
//============================================================================================================
/// This define prevents this .h file from being included more than once in a .cc file
#ifndef _SERIAL_H_
#define _SERIAL_H_

//============================================================================================================
/* Definitions */

#define CPU_FREQ_Hz	20000000
#define BAUD_RATE	9600
#define BAUD_DIV	(((CPU_FREQ_Hz) / (16UL * (BAUD_RATE))) - 1)

//============================================================================================================

//-------------------------------------------------------------------------------------
/** This class sets up a serial class for the ATtiny 2313
 */

class serial
{
	protected:
		/// This is a pointer to the data register used by the UART
		volatile unsigned char* p_UDR;

		/// This is a pointer to the status register used by the UART
		volatile unsigned char* p_USR;

		/// This is a pointer to the control register used by the UART
		volatile unsigned char* p_UCR;

		/// This bitmask identifies the bit for data register empty, UDRE
		unsigned char mask_UDRE;

		/// This bitmask identifies the bit for receive complete, RXC
		unsigned char mask_RXC;

		/// This bitmask identifies the bit for transmission complete, TXC
		unsigned char mask_TXC;
	public:
		/// The constructor sets up the port with the given baud rate and port number.
		serial (void);

		/// This method sends a byte out
		void send(unsigned char);

		/// This method checks if the serial port is ready to transmit data.
		bool ready_to_send (void);

		/// This method returns true if the port is currently sending a character out.
		bool is_sending (void);
		
		/// This method returns true if a character has been read by the serial port.
		bool check_for_char (void);
		
		/// This method returns the latest character received by the serial port.
		char getchar (void);
};

//============================================================================================================

#endif


