/*! \mainpage Experiment:Arena Tracing
 *
 * @author     Bitboot Team
 * @date       2021/03/21
 *
 * \subsection Aim
 * Algorithm to trace the Arena.
 */

#include "lib/adc_sensors.h"
#include "lib/buzzer_bargraph.h"
#include "lib/color_sensor.h"
#include "lib/lcd.h"
#include "lib/firebird_avr.h"
#include "lib/motor_control.h"
#include "lib/uart.h"
#include "path_finder.h"
#include "lib/position_control_interrupt.h"
#include <util/delay.h> // Standard AVR Delay Library
#define THRESHOLD_WLS 25

void forward_wls(unsigned char node);
void left_turn_wls(void);
void right_turn_wls(void);
void setup(void);
void wls_readings_print_lcd(void);
int check_path_for_debris(void);
void scan_plot(int);
void L_shape_traversal(tuple);
void check_plot_scan_status(tuple);
// To store 8-bit data of left, center and right white line sensors
unsigned char left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data;

tuple curr_loc = {4, 8}, goal_loc = {8, 4};
int plot_order[16] = {13, 14, 9, 10, 1, 2, 4, 3, 8, 7, 12, 11, 16, 15}, plot_no = 0;
int done = 0;
char dir_flag = 'n';

int main(int argc, char *argv[])
{
	setup();
	forward_mm(100);
	right_degrees(90);
	stop();
	while (1)
		;
	//-----------------------------------------------------------------------------------------------------
	//for (int i = 0; i < 9; i++)
	forward_wls(1);
	left_turn_wls();
	forward_wls(1);
	left_turn_wls();
	forward_wls(2);
	right_turn_wls();
	forward_wls(8); // Reach the node (4,8);
	right_turn_wls();
	forward_wls(4);
	right_turn_wls();
	forward_wls(8);
	left_turn_wls();
	forward_wls(2);
	left_turn_wls();
	forward_wls(4);
	right_turn_wls();
	forward_wls(1);
	stop();
	//------------------------------------------------------------------------------------------------------
	while (1)
		;
	// curr_loc.x = 4;
	// curr_loc.y = 8;
	// while (plot_no < 16)
	// {
	// 	scan_plot(plot_order[plot_no] - 1);
	// 	plot_no++;
	//	}
	return 1;
}
void forward_wls(unsigned char node)
{
	unsigned char node_reached = 0;
	for (int i = 0; i < node; i++)
	{
		forward();
		velocity(200, 180);
		_delay_ms(300);
		while (1)
		{
			// get the ADC converted data of three white line sensors from their appropriate channel numbers
			left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
			center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
			right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
			if (center_wl_sensor_data > THRESHOLD_WLS && (left_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS))
			{
				stop();
				velocity(0, 0);
				lcd_clear();
				lcd_string(1, 2, "Node Reached");
				++node_reached;
				lcd_numeric_value(2, 5, node_reached, 2);
				_delay_ms(500);
				//	wls_readings_print_lcd();
				break;
			}
			else if (center_wl_sensor_data > THRESHOLD_WLS)
			{
				velocity(250, 230);
				// printf("\n forward");
			}
			else if (left_wl_sensor_data > THRESHOLD_WLS)
			{
				velocity(50, 200);
				// printf("\n left");
			}
			else if (right_wl_sensor_data > THRESHOLD_WLS)
			{
				velocity(200, 50);
				// printf("\n right");
			}
			else
			{
				velocity(200, 180);
			}
			_delay_ms(5);
		}
	}
	if (dir_flag == 'n')
		curr_loc.y -= node;
	else if (dir_flag == 'w')
		curr_loc.x -= node;
	else if (dir_flag == 's')
		curr_loc.y += node;
	else
		curr_loc.x += node;
}

/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls();
*
*/

void left_turn_wls(void)
{
	forward();
	velocity(200, 180);
	_delay_ms(500);
	left();
	velocity(150, 150);
	_delay_ms(300);
	while (1)
	{
		// get the ADC converted data of center white line sensors from their appropriate channel number
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		if (center_wl_sensor_data < THRESHOLD_WLS)
		{
			left();
			velocity(150, 150);
		}
		else
			break;
		_delay_ms(5);
	}
	stop();
	if (dir_flag == 'n')
		dir_flag = 'w';
	else if (dir_flag == 'w')
		dir_flag = 's';
	else if (dir_flag == 's')
		dir_flag = 'e';
	else
		dir_flag = 'n';
}

/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls();
*
*/
void left_turn_wls_degress(int degrees)
{
	left_degrees(degrees);
	while (1)
	{
		// get the ADC converted data of center white line sensors from their appropriate channel number

		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		if (center_wl_sensor_data > THRESHOLD_WLS || (left_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS))
		{
			stop();
			break;
		}
		else
		{
			left();
			velocity(150, 150);
		}
		_delay_ms(2);
	}
}

void right_turn_wls_degress(int degrees)
{
	right_degrees(degrees);
	left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
	center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
	right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
	while (1)
	{
		// get the ADC converted data of center white line sensors from their appropriate channel number
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		if (center_wl_sensor_data > THRESHOLD_WLS || (left_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS))
		{
			stop();
			break;
		}
		else
		{
			right();
			velocity(150, 150);
		}
		_delay_ms(2);
	}
}

void right_turn_wls(void)
{
	forward();
	velocity(200, 180);
	_delay_ms(500);
	right();
	velocity(150, 150);
	_delay_ms(300);
	while (1)
	{
		// get the ADC converted data of center white line sensors from their appropriate channel number
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		if (center_wl_sensor_data < THRESHOLD_WLS)
		{

			right();
			velocity(160, 170);
		}
		else
			break;
		_delay_ms(5);
	}
	//	printf("\n\texit value %d \n", convert_analog_channel_data(center_wl_sensor_channel));
	stop();
	velocity(0, 0);
	if (dir_flag == 'n')
		dir_flag = 'e';
	else if (dir_flag == 'e')
		dir_flag = 's';
	else if (dir_flag == 's')
		dir_flag = 'w';
	else
		dir_flag = 'n';
}

void setup(void)
{
	int init_setup_success = 0;
	do
	{
		init_setup_success = motor_init_setup(); // Initialize Motor Controls
	} while (init_setup_success != 1);
	do
	{
		init_setup_success = adc_init_setup(); // Initialize ADC
	} while (init_setup_success != 1);
	position_encoder_pin_config();
	position_encoder_interrupt_config();
	initialize_grid_matrix();
	buzzer_pin_config();
	LED_bargraph_config();
	lcd_port_config();	   // Initialize the LCD port
	lcd_init();			   // Initialize the LCD
	uart_init(UBRR_VALUE); // Initialize the UART
	lcd_clear();
	lcd_string(1, 5, "BitBoot");
	lcd_string(2, 4, "IIT  BOMBAY");
	buzzer_on();
	_delay_ms(500);
	buzzer_off();
}
void wls_readings_print_lcd(void)
{
	left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
	lcd_clear();
	lcd_string(1, 2, "Left Sensor");
	lcd_numeric_value(2, 5, left_wl_sensor_data, 3);
	_delay_ms(1000);
	center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
	lcd_clear();
	lcd_string(1, 2, "Center Sensor");
	lcd_numeric_value(2, 5, center_wl_sensor_data, 3);
	_delay_ms(1000);
	right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
	lcd_clear();
	lcd_string(1, 2, "Right Sensor");
	lcd_numeric_value(2, 5, right_wl_sensor_data, 3);
	_delay_ms(1000);
	lcd_clear();
}