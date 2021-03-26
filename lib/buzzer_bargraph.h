/* 
* buzzer_beep.h
* Created: 23/03/21 01:15:58 PM
* Author: Bitboot Team
*/ 

#ifndef BUZZER_BARGRAPH_H_
#define BUZZER_BARGRAPH_H_

// Function to make **ONLY** 'buzzer_pin' as output and initially set it to low
void buzzer_pin_config (void);

// Function to configure the pins of ATmega2560 to which the Bar Graph LED is connected
void LED_bargraph_config (void);

// Function to set **ONLY** 'buzzer_pin' to high, hence turn ON the Buzzer
void buzzer_on (void);

// Function to set **ONLY** 'buzzer_pin' to low, hence turn OFF the Buzzer
void buzzer_off (void);

// Function to set LED pins to low, hence turn off all LEDs
void bargraph_led_off(void);

// Function to set LED pins to high, hence turn on all LEDs
void bargraph_led_on(void);
#endif
