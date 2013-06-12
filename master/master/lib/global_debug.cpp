//*************************************************************************************
/** \file global_debug.cpp
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

#include <stdlib.h>
#include <avr/wdt.h>
#include "global_debug.h"

#ifdef SERIAL_DEBUG
	/// This is a globally accessible pointer to a debugging port. It is declared 
	/// extern in global_debug.h and can therefore be used within any *.cc file which
	/// includes global_debug.h. It must be set, usually in main(), before it is used. 
	base_text_serial* p_glb_dbg_port = NULL;

	/** This function sets the global pointer to a serial debugging device. The device
	 *  can be any descendent of class base_text_serial.  This method must be called 
	 *  before the pointer is used; otherwise the pointer will be NULL and nothing will 
	 *  be printed. 
	 *  @param p_port A pointer to a serial device to be used for debugging
	 */
	void set_glob_debug_port (base_text_serial* p_port) 
	{
		p_glb_dbg_port = p_port;
	}

#endif


/** This function can be called whenever some horrendous error condition is detected
 *  which is so awful that it is best handled by rebooting the system and starting 
 *  over. 
 */

void do_reboot (void)
{
	wdt_enable (0);							// Enable watchdog timer to timeout soon
	while (true);							// Enter infinite loop until the reset
}
