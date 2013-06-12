//*************************************************************************************
/** \file motor405.h
 *    This file contains a class which runs the motor driver on an ME405 board, 
 *    version 0.60+. This version has two VNH3SP30 motor drivers controlled by the
 *    ATmega128 microcontroller. Please see the class definition for class motor405
 *    for a list of which pins are connected to which signals. 
 *
 *  Revisions:
 *    \li 01-17-2008 JRR Created file
 *    \li 03-01-2008 JRR Fixed bug in brake() which prevented proper braking
 *    \li 05-15-2008 JRR Version for dual-driver ME405 board written
 *    \li 01-18-2009 JRR Reformatted a bit
 *    \li 01-15-2008 JRR Changed to new file/directory layout with ./lib and *.cpp
 *
 *  License:
 *    This file released under the Lesser GNU Public License, version 2. This program
 *    is intended for educational use only, but its use is not limited thereto. 
 */
//*************************************************************************************

#ifndef _MOTOR_405_H_
#define _MOTOR_405_H_						///< Prevents multiple inclusion of file

#define M405_PORT_1  PORTC					///< Name of port connected to motor 1
#define M405_DDR_1   DDRC					///< Specify the data direction register
#define M405_INA_1   0x01					///< Mask for the INA pin of the driver
#define M405_INB_1   0x02					///< Mask for the INB pin of the driver
#define M405_DIAG_1  0x04					///< Mask for the DIAG pin of the driver
#define M405_PWM_P1  PORTB					///< Port used by the PWM pin
#define M405_PWM_D1  DDRB					///< Data direction register for #1 PWM
#define M405_PWM_B1  0x40					///< Bitmask for the PWM pin

#define M405_PORT_2  PORTD					///< Name of port connected to motor 2
#define M405_DDR_2   DDRD					///< Specify the data direction register
#define M405_INA_2   0x20					///< Mask for the INA pin of the driver
#define M405_INB_2   0x40					///< Mask for the INB pin of the driver
#define M405_DIAG_2  0x80					///< Mask for the DIAG pin of the driver
#define M405_PWM_P2  PORTB					///< Port used by the PWM pin
#define M405_PWM_D2  DDRB					///< Data direction register for #2 PWM
#define M405_PWM_B2  0x20					///< Bitmask for the PWM pin


//--------------------------------------------------------------------------------------
/** This class operates two DC motors attached to the ME405 board V0.60+.  The motor
 *  is run through the single VNH3SP30 motor driver on the board. The connections that
 *  run the motors are hooked up as follows: 
 *
 *    \li MOTOR 1
 *    \li INA	 - Port C pin 0  - Mode select bit
 *    \li INB	 - Port C pin 1  - Mode select bit
 *    \li DIAGA/B - Port C pin 2  - Diagnostics from chip
 *    \li PWM	 - Port B pin 6  - PWM control, pin OC1B
 *
 *    \li MOTOR 2
 *    \li INA	 - Port D pin 5  - Mode select bit
 *    \li INB	 - Port D pin 6  - Mode select bit
 *    \li DIAGA/B - Port D pin 7  - Diagnostics from chip
 *    \li PWM	 - Port B pin 5  - PWM control, pin OC1A
 *
 *  The truth table for INA and INB is as follows:
 *    \li INA = 1, INB = 1 - Brake (both sides connect to Vcc)
 *    \li INA = 1, INB = 0 - Torque clockwise
 *    \li INA = 0, INB = 1 - Torque counterclockwise
 *    \li INA = 0, INB = 0 - Brake (both sides connect to ground)
 */

class motor405
{
	protected:
		unsigned char which_motor;			///< Is this object for motor 1 or 2?

	public:
		/// This constructor creates a motor controller object.
		motor405 (unsigned char);

		/// This method sets the motor to be powered in the clockwise direction.
		void clockwise (void);

		/// This method sets the motor to be powered in the counterclockwise direction.
		void counterclockwise (void);

		/// This method puts the motor into braking mode.
		void brake (void);

		/// This method sets the duty cycle of the PWM with a 16-bit value.
		void set_duty_cycle (unsigned char);
};

#endif // _MOTOR_405_H_
