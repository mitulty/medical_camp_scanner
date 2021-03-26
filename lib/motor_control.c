/*! \mainpage Experiment: Motor Control
 *
 * @author     Bitboot Team
 * @date       2021/03/21
 *
 * \subsection Aim
 *  This experiment demonstrates simple motion control.
 *
 * \subsection Connections
 * Motors Connections: 
 * Motors are connected to the Microcontroller through L293D Motor Driver IC. <br>
 *		 Motors Pin	  	Microcontroller Pin      			<br>
 *			  RB  	--> 	PA3								<br>
 *			  RF  	--> 	PA2								<br>
 *			  LF  	--> 	PA1								<br>
 *			  LB 	--> 	PA0								<br>
 *
 * PWM/Enable connections:
 * PWM Pins of the Microcontroller are connected to the L293D Motor Driver IC.
 *		   PWM/Enable Pin	  Microcontroller Pin      		<br>
 *		  Left Motor  	--> 	PL4	(OC5A)					<br>
 *		  Right Motor  	--> 	PL3	(OC5B)					<br>
 *
 * \subsection Macro Definitions
 *		motors_dir_ddr_reg		:  DDRA				<br>
 *		motors_dir_port_reg		:  PORTA			<br>
 *		motors_pwm_ddr_reg		:  DDRL				<br>
 *		motors_pwm_port_reg		:  PORTL			<br>
 *
 *		motors_RB_pin			:  PA3				<br>
 *		motors_RF_pin			:  PA2				<br>
 *		motors_LF_pin			:  PA1				<br>
 *		motors_LB_pin			:  PA0				<br>
 *		motors_pwm_R_pin		:  PL4				<br>
 *		motors_pwm_L_pin		:  PL3				<br>
 */


//---------------------------------- HEADER FILES -----------------------------------------------------

#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>	
#include "motor_control.h"				// Standard AVR Delay Library

//---------------------------------- FUNCTIONS ----------------------------------------------------------

//-----------------------------CONFIGURATION FUNCTIONS --------------------------------------------------

/**
 * @brief      Function to configure motor pins
 */
void motors_pin_config(void)
{
	motors_dir_ddr_reg |= (1 << motors_RB_pin) | (1 << motors_RF_pin) | (1 << motors_LF_pin) | (1 << motors_LB_pin) ;			// motor pin as output
	motors_dir_port_reg &=  ~( (1 << motors_RB_pin) | (1 << motors_RF_pin) | (1 << motors_LF_pin) | (1 << motors_LB_pin) );		// stop motor intially
}

/**
 * @brief      Function to configure left and right channel pins of the L293D Motor Driver IC for PWM
 */
void pwm_pin_config(void)
{
	motors_pwm_ddr_reg |= (1 << motors_pwm_R_pin) | (1 << motors_pwm_L_pin);	// left and right channel pin as output
	motors_pwm_port_reg |= (1 << motors_pwm_R_pin) | (1 << motors_pwm_L_pin);	// turn on left and right channel
}

//----------------------------- MOTION RELATED FUNCTIONS ----------------------------------------------

/**
 * @brief      Function to make Firebird-V move forward.
 */
void forward (void) //both wheels forward
{
	motors_dir_port_reg &=~((1<<motors_RB_pin)|(1<<motors_RF_pin)|(1<<motors_LB_pin) |(1<<motors_LF_pin))  ;	// Make LF, LB, RF, RB LOW
	motors_dir_port_reg |= (1 << motors_RF_pin) | (1 << motors_LF_pin) ;		// Make LF and RF HIGH
}

/**
 * @brief      Function to make Firebird-V move backward.
 */
void back (void) //both wheels backward
{
	motors_dir_port_reg &=~((1<<motors_RB_pin)|(1<<motors_RF_pin)|(1<<motors_LB_pin) |(1<<motors_LF_pin))  ;	// Make LF, LB, RF, RB LOW	
	motors_dir_port_reg |= ((1 << motors_RB_pin) | (1 << motors_LB_pin)) ;		// Make LB and RB HIGH
}

/**
 * @brief      Function to make Firebird-V rotate left.
 */
void left (void) //Left wheel backward, Right wheel forward
{
	motors_dir_port_reg &=~((1<<motors_RB_pin)|(1<<motors_RF_pin)|(1<<motors_LB_pin) |(1<<motors_LF_pin))  ;	// Make LF, LB, RF, RB LOW	
	motors_dir_port_reg |= (1 << motors_RF_pin) | (1 << motors_LB_pin) ;		// Make LB and RF HIGH
}

/**
 * @brief      Function to make Firebird-V rotate right.
 */
void right (void) //Left wheel forward, Right wheel backward
{
	motors_dir_port_reg &=~((1<<motors_RB_pin)|(1<<motors_RF_pin)|(1<<motors_LB_pin) |(1<<motors_LF_pin))  ;	// Make LF, LB, RF, RB LOW	
	motors_dir_port_reg |= (1 << motors_LF_pin) | (1 << motors_RB_pin) ;		// Make LF and RB HIGH
}

/**
 * @brief      Function to make Firebird-V rotate soft left.
 */
void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motors_dir_port_reg &=~((1<<motors_RB_pin)|(1<<motors_RF_pin)|(1<<motors_LB_pin) |(1<<motors_LF_pin))  ;	// Make LF, LB, RF, RB LOW	
	motors_dir_port_reg |= (1 << motors_RF_pin) ;	// Make RF HIGH
}

/**
 * @brief      Function to make Firebird-V rotate soft right.
 */
void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motors_dir_port_reg &=~((1<<motors_RB_pin)|(1<<motors_RF_pin)|(1<<motors_LB_pin) |(1<<motors_LF_pin))  ;	// Make LF, LB, RF, RB LOW	
	motors_dir_port_reg |= (1 << motors_LF_pin) ;	// Make LF HIGH
}

/**
 * @brief      Function to make Firebird-V rotate backward left.
 */
void backward_left (void) //Left wheel backward, right wheel stationary
{
	motors_dir_port_reg &=~((1<<motors_RB_pin)|(1<<motors_RF_pin)|(1<<motors_LB_pin) |(1<<motors_LF_pin))  ;	// Make LF, LB, RF, RB LOW	
	motors_dir_port_reg |= (1 << motors_LB_pin) ;	// Make LB HIGH
}

/**
 * @brief      Function to make Firebird-V rotate backward right.
 */
void backward_right (void) //Left wheel stationary, Right wheel backward
{
	motors_dir_port_reg &=~((1<<motors_RB_pin)|(1<<motors_RF_pin)|(1<<motors_LB_pin) |(1<<motors_LF_pin))  ;	// Make LF, LB, RF, RB LOW	
	motors_dir_port_reg |= (1 << motors_RB_pin) ;	// Make RB HIGH
}

/**
 * @brief      Function to make Firebird-V stop.
 */
void stop (void) // Both wheel stationary
{
  	motors_dir_port_reg &=  ~( (1 << motors_LF_pin) | (1 << motors_RF_pin) | (1 << motors_LB_pin) | (1 << motors_RB_pin));	// Make LF, RF, LB and RB LOW
}


/**
 * @brief      Function to inititalise the timer
 */
void timer_pwm_init(void)
{
	TCCR5B_reg = 0x00;	//Stop
	
	TCNT5H_reg = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L_reg = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	
	OCR5AH_reg = 0x00;	//Output compare register high value for Left Motor
	OCR5AL_reg = 0xFF;	//Output compare register low value for Left Motor
	
	OCR5BH_reg = 0x00;	//Output compare register high value for Right Motor
	OCR5BL_reg = 0xFF;	//Output compare register low value for Right Motor
	
	// Clear on Compare
	TCCR5A_reg |= (1 << COMA1_bit) | (1 << COMB1_bit);
	TCCR5A_reg &= ~( (1 << COMA0_bit) | (1 << COMB0_bit));

	// Configure for FAST PWM
	TCCR5A_reg |= (1 << WGM0_bit);
	TCCR5A_reg &= ~(1 << WGM1_bit);
	TCCR5B_reg |= (1 << WGM2_bit);
	TCCR5B_reg &= ~(1 << WGM3_bit);

	// Set Prescalar to 64
	TCCR5B_reg |= (1 << CS1_bit) | (1 << CS0_bit);
	TCCR5B_reg &= ~(1 << CS2_bit);
}


/**
 * @brief      Function to control the speed of both the motors of Firebird-V
 *
 * @param[in]  left_motor   Left motor speed 0 to 255
 * @param[in]  right_motor  Right motor speed 0 to 255
 */
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL_reg = left_motor;
	OCR5BL_reg = right_motor;
}


/**
 * @brief      Initializes the setup by configuring all the required devices
 */
int motor_init_setup(void)
{
	// Initialize motor pins
	motors_pin_config();

	// Initialize PWM pins as output
	pwm_pin_config();

	// Initialize Timer in Phase Correct PWM mode
	timer_pwm_init();

	return 1;
}

