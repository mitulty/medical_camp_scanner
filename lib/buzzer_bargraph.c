/*! \mainpage Experiment: Buzzer and Bargraph Control
 *
 * @author     Bitboot Team
 * @date       2021/03/21
 *
 * \subsection Aim
 * To interface the Buzzer already present on Firebird V robot and turn it ON and OFF at an interval of a second.
 *
 * \subsection Connections
 * Buzzer 				:  PC3 				<br>
 *
 * \subsection Macro Definitions
 * buzzer_ddr_reg		:  DDRC				<br>
 * buzzer_port_reg		:  PORTC			<br>
 * buzzer_pin			:  PC3				<br>
 * 
 */


//---------------------------------- HEADER FILES -----------------------------------------------------

#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library
#include "buzzer_bargraph.h"

//---------------------------------- FUNCTIONS ----------------------------------------------------------

/**
 * @brief      Function to make **ONLY** 'buzzer_pin' as output and initially set it to low
 */
void buzzer_pin_config (void)
{

	// Make 'buzzer_pin' as output
	buzzer_ddr_reg	|= ( 1 << buzzer_pin );
	
	// Set 'buzzer_pin' to low initially to turn it OFF
	buzzer_port_reg &= ~( 1 << buzzer_pin );
}

/**
 * @brief      Function to configure the pins of ATmega2560 to which the Bar Graph LED is connected
 */
void LED_bargraph_config (void)
{
	bar_graph_led_ddr_reg = 0xFF;  		// Bargraph LED port as output
	bar_graph_led_port_reg = 0x00; 		// LEDs initially off
}


/**
 * @brief      Function to set **ONLY** 'buzzer_pin' to high, hence turn ON the Buzzer
 */
void buzzer_on (void)
{

	// Set 'buzzer_pin' to high to turn it ON
	buzzer_port_reg |= ( 1 << buzzer_pin );
}


/**
 * @brief      Function to set **ONLY** 'buzzer_pin' to low, hence turn OFF the Buzzer
 */
void buzzer_off (void)
{

	// Set 'buzzer_pin' to low to turn it OFF
	buzzer_port_reg &= ~( 1 << buzzer_pin );
}


/**
 * @brief      Function to set LED pins to high, hence turn on all LEDs
 */
void bargraph_led_on(void)
{
	// Turn on all LEDs
	bar_graph_led_port_reg = 0xFF;
}

/**
 * @brief      Function to set LED pins to low, hence turn off all LEDs
 */
void bargraph_led_off(void)
{
	// Turn off all LEDs
	bar_graph_led_port_reg = 0x00;
}

