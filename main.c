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
void check_plot_scan_status();
void allign_at_node(void);

// To store 8-bit data of left, center and right white line sensors
unsigned char left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data;

int plot_order[16] = {13, 14, 9, 10, 1, 2, 4, 3, 8, 7, 12, 11, 16, 15}, plot_no = 0;
int done = 0;
int prev = 0;
char dir_flag = 'n';

int main(int argc, char *argv[])
{
	setup();
	forward_wls(1);
	curr_loc.x = 4;
	curr_loc.y = 8;
	//-----------------------------------------------------------------------------------------------------
	left_turn_wls();
	_delay_ms(100);

	forward_wls(2);
	_delay_ms(100);

	right_turn_wls();
	_delay_ms(100);
	for (int i = 0; i < 4; i++)
{
	forward_wls(1);
	_delay_ms(100);
	check_plot_scan_status();
	forward_wls(1);
	_delay_ms(100);
}
	right_turn_wls();
	_delay_ms(100);

	forward_wls(4);
	_delay_ms(100);

	right_turn_wls();
	_delay_ms(100);

	for (int i = 0; i < 4; i++)
{
	forward_wls(1);
	_delay_ms(100);
	check_plot_scan_status();
	forward_wls(1);
	_delay_ms(100);
}

	left_turn_wls();
	_delay_ms(100);

	forward_wls(2);
	_delay_ms(100);

	left_turn_wls();
	_delay_ms(100);

	forward_wls(4);
	_delay_ms(100);

	right_turn_wls();
	_delay_ms(100);

	forward_wls(1);
	_delay_ms(100);

	stop();
	//------------------------------------------------------------------------------------------------------
	while (1)
		;
	curr_loc.x = 4;
	curr_loc.y = 8;
	goal_loc.x = 8;
	goal_loc.y = 4;
	// while (plot_no < 16)
	// {
	// 	scan_plot(plot_order[plot_no] - 1,curr_loc);
	// 	plot_no++;
	//	}
	return 1;
}

void move_robot(tuple n_loc)
{
	int x_dis = n_loc.x - curr_loc.x;
	int y_dis = n_loc.y - curr_loc.y;
	if (x_dis < 0)
	{
		if (dir_flag != 'w')
		{
			if (dir_flag == 'n')
				left_turn_wls();
			else if (dir_flag == 'e') // 180 degrees turn.
			{
				left_turn_wls();
				left_turn_wls();
			}
			else
				right_turn_wls();
		}
	}
	else if (x_dis > 0)
	{
		if (dir_flag != 'e')
		{
			if (dir_flag == 's')
				left_turn_wls();
			else if (dir_flag == 'w') // 180 degrees turn.
			{
				right_turn_wls();
				right_turn_wls();
			}
			else
				right_turn_wls();
		}
	}
	if (y_dis < 0)
	{
		if (dir_flag != 'n')
		{
			if (dir_flag == 'w')
				left_turn_wls();
			else if (dir_flag == 's') // 180 degrees turn.
			{
				left_turn_wls();
				left_turn_wls();
			}
			else
				right_turn_wls();
		}
	}
	else if (y_dis > 0)
	{
		if (dir_flag != 's')
		{
			if (dir_flag == 'e')
				left_turn_wls();
			else if (dir_flag == 'n') // 180 degrees turn.
			{
				right_turn_wls();
				right_turn_wls();
			}
			else
				right_turn_wls();
		}
	}
	forward_wls(1);
	check_plot_scan_status(curr_loc);
	forward_wls(1);
}
int check_path_for_debris(void)
{
	int c = 0;
	forward();
	_delay_ms(100);
	stop();
	_delay_ms(10);
	left();
	velocity(50, 50);
	for (int i = 0; i < 200; i++)
	{
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		if (left_wl_sensor_data < THRESHOLD_WLS && center_wl_sensor_data < THRESHOLD_WLS && right_wl_sensor_data < THRESHOLD_WLS)
			c = 1;
		_delay_ms(1);
	}
	right();
	velocity(50, 50);
	for (int i = 0; i < 400; i++)
	{
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		if (left_wl_sensor_data < THRESHOLD_WLS && center_wl_sensor_data < THRESHOLD_WLS && right_wl_sensor_data < THRESHOLD_WLS)
			c = 1;
		_delay_ms(1);
	}
	left();
	velocity(50, 50);
	for (int i = 0; i < 200; i++)
	{
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		if (left_wl_sensor_data < THRESHOLD_WLS && center_wl_sensor_data < THRESHOLD_WLS && right_wl_sensor_data < THRESHOLD_WLS)
			c = 1;
		_delay_ms(1);
	}
	back();
	_delay_ms(100);
	stop();
	_delay_ms(10);
	return c;
}
void check_plot_scan_status()
{
	lcd_clear();
	lcd_string(1,4,"(");
	lcd_numeric_value(1,7,curr_loc.x,1);
	lcd_string(1,9,",");
	lcd_numeric_value(1,11,curr_loc.y,1);
	lcd_string(1,13,")");
	_delay_ms(100);
	if (curr_loc.x == 0)
	{
		if (grid_matrix[curr_loc.x + 1][curr_loc.y] == -5)
		{
			if (dir_flag == 'n')
			{
				
				left_degrees(90);
				grid_matrix[curr_loc.x + 1][curr_loc.y] = color_type();
				right_degrees(90);
			}
			else
			{
				right_degrees(90);
				grid_matrix[curr_loc.x + 1][curr_loc.y] = color_type();
				left_degrees(90);
			}
		}
		return;
	}
	if (curr_loc.x == 8)
	{
		if (grid_matrix[curr_loc.x - 1][curr_loc.y] == -5)
		{
			if (dir_flag == 'n')
			{
				left_degrees(90);
				grid_matrix[curr_loc.x - 1][curr_loc.y] = color_type();
				right_degrees(90);
			}
			else
			{
				right_degrees(90);
				grid_matrix[curr_loc.x - 1][curr_loc.y] = color_type();
				left_degrees(90);
			}
		}
		return;
	}
	if (curr_loc.y == 0)
	{
		if (grid_matrix[curr_loc.x][curr_loc.y + 1] == -5)
		{
			if (dir_flag == 'w')
			{
				left_degrees(90);
				grid_matrix[curr_loc.x][curr_loc.y + 1] = color_type();
				right_degrees(90);
			}
			else
			{
				right_degrees(90);
				grid_matrix[curr_loc.x][curr_loc.y + 1] = color_type();
				left_degrees(90);
			}
		}
		return;
	}
	if (curr_loc.y == 8)
	{
		if (grid_matrix[curr_loc.x][curr_loc.y - 1] == -5)
		{
			if (dir_flag == 'w')
			{
				left_degrees(90);
				grid_matrix[curr_loc.x][curr_loc.y - 1] = color_type();
				right_degrees(90);
			}
			else
			{
				right_degrees(90);
				grid_matrix[curr_loc.x][curr_loc.y - 1] = color_type();
				left_degrees(90);
			}
		}
		return;
	}
	if (dir_flag == 'e' || dir_flag == 'w')
	{
		if (grid_matrix[curr_loc.x][curr_loc.y - 1] == -5)
		{
			if (dir_flag == 'w')
			{
				left_degrees(90);
				grid_matrix[curr_loc.x][curr_loc.y - 1] = color_type();
				right_degrees(90);
			}
			else
			{
				right_degrees(90);
				grid_matrix[curr_loc.x][curr_loc.y - 1] = color_type();
				left_degrees(90);
			}
		}
		if (grid_matrix[curr_loc.x][curr_loc.y + 1] == -5)
		{
			if (dir_flag == 'w')
			{
				right_degrees(90);
				grid_matrix[curr_loc.x][curr_loc.y + 1] = color_type();
				left_degrees(90);
			}
			else
			{
				left_degrees(90);
				grid_matrix[curr_loc.x][curr_loc.y + 1] = color_type();
				right_degrees(90);
			}
		}
		return;
	}

	if (dir_flag == 'n' || dir_flag == 's')
	{
		if (grid_matrix[curr_loc.x - 1][curr_loc.y] == -5)
		{
			if (dir_flag == 'n')
			{
				left_degrees(90);
				grid_matrix[curr_loc.x - 1][curr_loc.y] = color_type();
				right_degrees(90);
			}
			else
			{
				right_degrees(90);
				grid_matrix[curr_loc.x - 1][curr_loc.y] = color_type();
				left_degrees(90);
			}
		}
		if (grid_matrix[curr_loc.x + 1][curr_loc.y] == -5)
		{
			if (dir_flag == 'n')
			{
				right_degrees(90);
				grid_matrix[curr_loc.x + 1][curr_loc.y] = color_type();
				left_degrees(90);
			}
			else
			{
				left_degrees(90);
				grid_matrix[curr_loc.x + 1][curr_loc.y] = color_type();
				right_degrees(90);
			}
		}
		return;
	}
}
void forward_wls(unsigned char node)
{
	unsigned char node_reached = 0;
	for (int i = 0; i < node; i++)
	{
		allign_at_node();
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
				prev = 0;
				velocity(200, 180);
				// printf("\n forward");
			}
			else if (left_wl_sensor_data > THRESHOLD_WLS)
			{
				prev = 1;
				velocity(50, 200);
				// printf("\n left");
			}
			else if (right_wl_sensor_data > THRESHOLD_WLS)
			{
				prev = 2;
				velocity(200, 50);
				// printf("\n right");
			}
			else
			{
				if (prev == 1)
					velocity(50, 200);
				else if (prev == 2)
					velocity(200, 50);
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
	allign_at_node();
	forward();
	velocity(200, 180);
	_delay_ms(200);
	left();
	velocity(150, 150);
	_delay_ms(700);
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

void right_turn_wls(void)
{
	allign_at_node();
	forward();
	velocity(200, 180);
	_delay_ms(200);
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

void allign_at_node()
{
	left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
	center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
	right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
	if (left_wl_sensor_data > THRESHOLD_WLS || center_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS)			
		return;
	int limit = 50;
	int found = 0;
	while (1)
	{
		left();
		velocity(150, 100);
		for (int i = 0; i < (limit); i++)
		{
			left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
			center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
			right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
			if (left_wl_sensor_data > THRESHOLD_WLS || center_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS)
			{
				stop();
				found = 1;
				break;
			}
			_delay_ms(2);
		}
		stop();
		_delay_ms(5);
		right();
		velocity(150, 100);
		for (int i = 0; i < (limit * 1.7); i++)
		{
			left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
			center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
			right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
			if (left_wl_sensor_data > THRESHOLD_WLS || center_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS)
			{
				stop();
				found = 1;
				break;
			}
			_delay_ms(2);
		}
		stop();
		_delay_ms(5);
		left();
		velocity(150, 100);
		for (int i = 0; i < limit; i++)
		{
			left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
			center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
			right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
			if (left_wl_sensor_data > THRESHOLD_WLS || center_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS)
			{
				stop();
				found = 1;
				break;
			}
			_delay_ms(2);
		}
		stop();
		_delay_ms(1000);
		limit += 50;
		if(found = 1)
			break;

	}
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
	initialize_plot_coord_matrix();
	initialize_adjacency_matrix();
	initialize_node_coord_matrix();
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
	goal_loc.x = 8;
	goal_loc.y = 4;
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

void left_turn_wls_degress(int degrees)
{
	allign_at_node();
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
	allign_at_node();
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
	}	allign_at_node();

}
