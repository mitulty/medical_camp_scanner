/* 
* adc_sensors.h
* Created: 23/03/21 01:15:58 PM
* Author: Bitboot Team
*/ 

#ifndef ADC_SENSORS_H_
#define ADC_SENSORS_H_

// Makes **ONLY** three white line sensor pins as input and dectivates pull-up for **ONLY** these sensor pins
void wl_sensors_port_config(void);

// Initializes the Analog-to-Digital converter inside the micro-controller
void adc_init(void);

// Initializes the setup by configuring all the required devices
int adc_init_setup(void);

// Sets the MUX[5:0] bits according to the sensor's channel number as input
void select_adc_channel(unsigned char);

// Starts the ADC by setting the ADSC bit in ADCSRA register
void start_adc(void);

// Checks if the ADC conversion for the selected channel is complete or not
int check_adc_conversion_complete(void);

// Get the ADC converted data from ADC data registers
unsigned char read_adc_converted_data(void);

// Reset ADC config registers, ADCSRA, ADCSRB and ADMUX
void reset_adc_config_registers(void);

// Convert the analog readings to 8-bit digital format from the sensor's ADC channel number as input
unsigned char convert_analog_channel_data(unsigned char);
 
#endif