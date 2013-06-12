//*************************************************************************************
/** \file global_debug.h
 *    This file contains a simple system to allow debugging printouts from anywhere in
 *    a simple AVR program. It sets up a global pointer to a base_text_serial object;
 *    this pointer will be set to point to any descendent of base_text_serial, such as
 *    an RS-232 style serial port (USB or classic), a radio modem, or even an SD card. 
 *
 *    The SERIAL_DEBUG macro should be defined in the Makefile so that global serial 
 *    debugging is enabled. If SERIAL_DEBUG is not defined, the serial debugging port
 *    is not activated; this is good for production code as opposed to testing code. 
 *
 *  Revisions
 *    \li 01-31-2009 JRR Original file
 *    \li 02-08-2009 JRR Added code to enable this debugging under Linux
 *    \li 12-20-2009 JRR Added do_reboot() function 
 */
//*************************************************************************************

#ifndef _GLOBAL_DEBUG_H_
#define _GLOBAL_DEBUG_H_

#ifdef SERIAL_DEBUG
	#ifdef __AVR
		#include "base_text_serial.h"		// Header for all the serial devices

		/// This pointer points to a serial device which handles debugging output. It
		/// is declared in global_debug.cc and can be used in any *.cc file which 
		/// includes this header. 
		extern base_text_serial* p_glb_dbg_port;

		/// This function MUST be called prior to using global debugging. It sets the 
		/// global debugging device pointer to the given serial device address. 
		void set_glob_debug_port (base_text_serial* p_port);

		/// This definition allows a bunch of debugging information to be printed if the
		/// SERIAL_DEBUG macro has been defined. If not, this macro expands to nothing. 
		#define GLOB_DEBUG(x) if (p_glb_dbg_port) *p_glb_dbg_port << x
	#else // Not __AVR
		// We don't have to set up a debugging port on a PC, just use 'cout'
		#define set_glob_debug_port(x)

		/// This definition allows the regular 'cout' port to be used for conditionally
		/// activated global debugging.
		#define GLOB_DEBUG(x) cout << x
	#endif

#else // The following defines apply if SERIAL_DEBUG is not defined. They cause the
	  // debugging macros to expand to nothing, thus deactivating serial debugging. 
	/// This defines a method to set the global serial debugging port. Serial debugging
	/// is currently disabled, so it expands to nothing. 
	#define set_glob_debug_port(x)

	/// This defines the global serial debugging macro. Serial debugging is currently
	/// disabled, so it expands to nothing. 
	#define GLOB_DEBUG(x)
#endif // SERIAL_DEBUG


// This function can be called whenever some horrendous error condition is detected
// which is best handled by rebooting the system and starting over. 
void do_reboot (void);

#endif // _GLOBAL_DEBUG_H_
