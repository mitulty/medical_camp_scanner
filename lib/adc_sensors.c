/*! \mainpage Experiment:ADC Sensor Control
 *
 * @author     Bitboot Team
 * @date       2021/03/21
 *
 * \subsection Aim
 * To get the 8-bit ADC result from the sensors in Single Conversion Mode.
 *
 * \subsection Connections:
 *			ACD CH.		PORT	Sensor
 *			0			PF0		Battery Voltage								<br>
 *			1			PF1		White line sensor 3							<br>
 *			2			PF2		White line sensor 2							<br>
 *			3			PF3		White line sensor 1							<br>
 *			4			PF4		IR Proximity analog sensor 1*****			<br>
 *			5			PF5		IR Proximity analog sensor 2*****			<br>
 *			6			PF6		IR Proximity analog sensor 3*****			<br>
 *			7			PF7		IR Proximity analog sensor 4*****			<br>
 *			8			PK0		IR Proximity analog sensor 5				<br>
 *			9			PK1		Sharp IR range sensor 1						<br>
 *			10			PK2		Sharp IR range sensor 2						<br>
 *			11			PK3		Sharp IR range sensor 3						<br>
 *			12			PK4		Sharp IR range sensor 4						<br>
 *			13			PK5		Sharp IR range sensor 5						<br>
 *			14			PK6		Servo Pod 1									<br>
 *			15			PK7		Servo Pod 2									<br>
 *
 *
 * ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2.			<br>
 * To use JTAG via expansion slot of the micro controller socket remove these jumpers.		<br>
 *
 *		LCD Display interpretation:															<br>
 *
 *		***************************************************************************			<br>
 *		BATTERY VOLTAGE	IR PROX.SENSOR 2	IR PROX.SENSOR 3	IR.PROX.SENSOR 4			<br>
 *		LEFT WL SENSOR	CENTER WL SENSOR	RIGHT WL SENSOR		FRONT SHARP DIS				<br>
 *		***************************************************************************			<br>
 *
 */


//---------------------------------- HEADER FILES -----------------------------------------------------
#include "firebird_avr.h"				// Header file included that contains macro definitions essential for Firebird V robot
#include <util/delay.h>					// Standard AVR Delay Library
#include <stdio.h>						// Standard C Library for standard input output
#include "adc_sensors.h"
//---------------------------------- GLOBAL VARIABLES -----------------------------------------------------
// To store 8-bit data of left, center and right white line sensors
unsigned char left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data;

// To store 8-bit data of 5th IR proximity sensors
unsigned char ir_prox_5_sensor_data;

//---------------------------------- FUNCTIONS -----------------------------------------------------
/**
 * @brief      Makes **ONLY** three white line sensor pins as input and dectivates pull-up for **ONLY** these sensor pins
 */
void wl_sensors_port_config(void)
{
	// << NOTE >> : Use Masking and Shift Operators here

	// Make **ONLY** three white line sensor pins as input
	wl_sensors_ddr_reg	&=~((1<<left_wl_sensor_pin)|(1<<center_wl_sensor_pin)|(1<<right_wl_sensor_pin)) ;

	// Deactivate pull-up for **ONLY** for three white line sensor pins
	wl_sensors_port_reg	&=~((1<<left_wl_sensor_pin)|(1<<center_wl_sensor_pin)|(1<<right_wl_sensor_pin)) ;
}

/**
 * @brief      Initializes the Analog-to-Digital converter inside the micro-controller
 */
void adc_init(void)
{
	// << NOTE >> : Use Masking and Shift Operators here

	// In ADCSRA, enable ADC and pre-scalar = 64 (ADEN = 1, ADPS2 = 1, ADPS1 = 1, ADPS0 = 0)
	//				and clear ADC start conversion bit, auto trigger enable bit, interrupt flag bit and interrupt enable bit
	ADCSRA_reg	|=((1<<ADEN_bit)|(1<<ADPS2_bit)|(1<<ADPS1_bit)) ;
	ADCSRA_reg	&=~((1<<ADSC_bit)|(1<<ADATE_bit)|(1<<ADIF_bit)|(1<<ADIE_bit)|(1<<ADPS0_bit)) ;

	// In ADCSRB, disable Analog Comparator Multiplexer, MUX5 bit and ADC Auto Trigger Source bits
	ADCSRB_reg	&= ~( (1 << ACME_bit) | (1 << MUX5_bit) | (1 << ADTS2_bit) | (1 << ADTS1_bit) | (1 << ADTS0_bit) );
	// In ADMUX, set the Reference Selection bits to use the AVCC as reference, and disable the channel selection bits MUX[4:0]
	ADMUX_reg &= ~((1<<MUX4_bit)|(1<<MUX3_bit)|(1<<MUX2_bit)|(1<<MUX1_bit)|(1<<MUX0_bit)|(1<<REFS1_bit)) ;
	ADMUX_reg |=(1<<REFS0_bit) ;

	// In ADMUX, enable the ADLAR bit for 8-bit ADC result
	ADMUX_reg	|=(1<<ADLAR_bit) ;

	// In ACSR, disable the Analog Comparator by writing 1 to ACD_bit
	ACSR_reg |=(1<<ACD_bit) ;
}



/**
 * @brief      Initializes the setup by configuring all the required devices
 */
int adc_init_setup(void)
{
	// Initialize the three white line sensors
	wl_sensors_port_config();

	// Initialize the ADC
	adc_init();
	
	return 1;
}


/**
 * @brief      Sets the MUX[5:0] bits according to the sensor's channel number as input
 *
 * @param[in]  channel_num   ADC channel number of sensor
 */
void select_adc_channel( unsigned char channel_num )
{
	// << NOTE >> : Use Masking and Shift Operators here

	// set the MUX[5:0] bits to select the ADC channel number
	if (channel_num > 7 )
	{
		ADCSRB_reg |=(1<<MUX5_bit) ;					// set the MUX5 bit for selecting channel if its greater than 7
	}

	channel_num	&=0x07 ;						// retain the last 3 bits from the variable for MUX[2:0] bits

	//ADMUX_reg	= ( ( ADMUX_reg & 0xF8 ) | channel_num );
	ADMUX_reg	=(ADMUX_reg | channel_num) ;
}


/**
 * @brief      Starts the ADC by setting the ADSC bit in ADCSRA register
 */
void start_adc(void)
{
	// << NOTE >> : Use Masking and Shift Operators here

	// set the ADSC bit in ADCSRA register
	ADCSRA_reg	|=(1<<ADSC_bit) ;
}


/**
 * @brief      Checks if the ADC conversion for the selected channel is complete or not
 *
 * @return     boolean true if the ADC has completed its conversion for the selected channel, else false.
 */
int check_adc_conversion_complete(void)
{
	// << NOTE >> : Use Masking and Shift Operators here

	/*
	<< TODO >> :
		1. Write an if-else statement with a condition which checks whether the ADC conversion for the selected channel is complete or not.
		2. If the ADC has completed its conversion for the selected channel return true, else return false
	*/
	if( ADCSRA_reg & ( 1 << ADIF_bit ))
		return 1;
	else
		return 0;

}


/**
 * @brief      Get the ADC converted data from ADC data registers
 *
 * @return     adc_8bit_data	ADC converted data of the sensor by reading ADC data registers
 */
unsigned char read_adc_converted_data(void)
{
	// << NOTE >> : Use Masking and Shift Operators here

	unsigned char adc_8bit_data;

	// read the appropriate ADC data register/s

	unsigned char adc_data_high_byte;

	adc_data_high_byte	= ADCH_reg ;

	adc_8bit_data		= adc_data_high_byte;

	return adc_8bit_data;
}


/**
 * @brief      Reset ADC config registers, ADCSRA, ADCSRB and ADMUX
 */
void reset_adc_config_registers(void)
{
	// << NOTE >> : Use Masking and Shift Operators here

	ADCSRA_reg	|=(1<<ADIF_bit) ;					// clear ADIF bit by writing 1 to it

	ADCSRB_reg	&=~(1<<MUX5_bit) ;					// clear the MUX5 bit

	// clear the MUX[4:0] bits
	ADMUX_reg &=~((1<<MUX4_bit) |(1<<MUX3_bit)|(1<<MUX2_bit)|(1<<MUX1_bit)|(1<<MUX0_bit)) ;
}


/**
 * @brief      Convert the analog readings to 8-bit digital format from the sensor's ADC channel number as input
 *
 * @param[in]  sensor_channel_number   ADC channel number of sensor
 *
 * @return     8-bit digital data from the input sensor ADC channel
 */
unsigned char convert_analog_channel_data( unsigned char sensor_channel_number )
{

	// << NOTE >> : You are not allowed to modify or change anything inside this function

	unsigned char adc_8bit_data;

	select_adc_channel( sensor_channel_number );

	start_adc();

	while( !( check_adc_conversion_complete() ) );

	adc_8bit_data = read_adc_converted_data();

	reset_adc_config_registers();

	return adc_8bit_data;
}

