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
#include <string.h>
#include <stdio.h>
#define THRESHOLD_WLS 25

void forward_wls(unsigned char);
void left_turn_wls_degress(int);
void right_turn_wls_degress(int);
void setup(void);
void wls_readings_print_lcd(void);
void check_plot_scan_status();
void allign_at_node(void);
void send_data_uart(int, int, int);
void receive_uart_data(void);
void uturn(void);

// To store 8-bit data of left, center and right white line sensors
unsigned char left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data;

int plot_order[16] = {13, 14, 9, 10, 1, 2, 4, 3, 8, 7, 12, 11, 16, 15}, plot_no = 0;
int done = 0;
int prev = 0;
char dir_flag = 'n';
char rx_byte = 0;
uint8_t rx_data;

int main(int argc, char *argv[])
{
	//---------------------------------------------Setup Zone-----------------------------------------------

	setup();
	forward_wls(1);
	curr_loc.x = 4;
	curr_loc.y = 8;
	dir_flag = 'n';
	int status = 0;
	tuple dest_loc, plot_coordinate;
	//---------------------------------------------Test Zone-----------------------------------------------
	// forward_wls(1);
	// left_turn_wls_degress(90);
	// forward_wls(2);
	// right_turn_wls_degress(90);
	// forward_wls(1);
	// while (1)
	// 	;
	// forward_wls(2);
	// right_turn_wls_degress(90);
	// forward_wls(8);
	// right_turn_wls_degress(90);
	// forward_wls(4);
	// right_turn_wls_degress(90);
	// forward_wls(8);
	// left_turn_wls_degress(90);
	// forward_wls(2);
	// left_turn_wls_degress(90);
	// forward_wls(4);
	// right_turn_wls_degress(90);
	// forward_wls(1);
	// while (1)
	// 	;
	//----------------------------------------------Main Code--------------------------------------------------------
	while (plot_no < 16)
	{
		if (status == 0)
			dest_loc = get_nearest_coordinate(curr_loc, plot_order[plot_no] - 1);
		scan_plot(plot_order[plot_no] - 1, dest_loc);
		status = final_mid_point(plot_order[plot_no] - 1);
		if (status == 1)
		{
			plot_no++;
			status = 0;
		}
		else
		{
			status = -1;
			plot_coordinate.x = plot_coord_matrix[plot_order[plot_no] - 1][4].x;
			plot_coordinate.x = plot_coord_matrix[plot_order[plot_no] - 1][4].y;
			if (curr_loc.x - plot_coordinate.x == 1)
				dest_loc.x = curr_loc.x - 2;
			else
				dest_loc.x = curr_loc.x + 2;
			if (curr_loc.y - plot_coordinate.y == 1)
				dest_loc.y = curr_loc.y - 2;
			else
				dest_loc.y = curr_loc.y + 2;
		}
	}

	return 1;
}

void move_robot(tuple n_loc)
{
	_delay_ms(100);
	forward_wls(1);
	_delay_ms(100);
	check_plot_scan_status();
	_delay_ms(100);
	forward_wls(1);
}

void turn_accordingly(tuple n_loc)
{
	int x_dis = n_loc.x - curr_loc.x;
	int y_dis = n_loc.y - curr_loc.y;
	if (x_dis < 0)
	{
		if (dir_flag != 'w')
		{
			if (dir_flag == 'n')
				left_turn_wls_degress(90);
			else if (dir_flag == 'e') // 180 degrees turn.
				uturn();
			else
				right_turn_wls_degress(90);
		}
	}
	else if (x_dis > 0)
	{
		if (dir_flag != 'e')
		{
			if (dir_flag == 's')
				left_turn_wls_degress(90);
			else if (dir_flag == 'w') // 180 degrees turn.
				uturn();
			else
				right_turn_wls_degress(90);
		}
	}
	else if (y_dis < 0)
	{
		if (dir_flag != 'n')
		{
			if (dir_flag == 'w')
				left_turn_wls_degress(90);
			else if (dir_flag == 's') // 180 degrees turn.
				uturn();
			else
				right_turn_wls_degress(90);
		}
	}
	else if (y_dis > 0)
	{
		if (dir_flag != 's')
		{
			if (dir_flag == 'e')
				left_turn_wls_degress(90);
			else if (dir_flag == 'n') // 180 degrees turn.
				uturn();
			else
				right_turn_wls_degress(90);
		}
	}
}

int check_path_for_debris(void)
{
	left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
	center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
	right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
	if (left_wl_sensor_data < THRESHOLD_WLS && center_wl_sensor_data < THRESHOLD_WLS && right_wl_sensor_data < THRESHOLD_WLS)
		return 1;

	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = 40 / distance_resolution; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int)ReqdShaftCount;

	velocity(50, 50);
	left();
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	while ((ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))
	{
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		if (left_wl_sensor_data < THRESHOLD_WLS && center_wl_sensor_data < THRESHOLD_WLS && right_wl_sensor_data < THRESHOLD_WLS)
		{
			stop();
			velocity(0, 0);
			return 1;
		}
	}
	while ((ShaftCountRight >= 2 * ReqdShaftCountInt) || (ShaftCountLeft >= 2 * ReqdShaftCountInt))
	{
		right();
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		if (left_wl_sensor_data < THRESHOLD_WLS && center_wl_sensor_data < THRESHOLD_WLS && right_wl_sensor_data < THRESHOLD_WLS)
		{
			stop();
			velocity(0, 0);
			return 1;
		}
	}
	while ((ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))
	{
		left();
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		if (left_wl_sensor_data < THRESHOLD_WLS && center_wl_sensor_data < THRESHOLD_WLS && right_wl_sensor_data < THRESHOLD_WLS)
		{
			stop();
			velocity(0, 0);
			return 1;
		}
	}
	stop();
	velocity(0, 0);
	return 0;
}

void check_plot_scan_status()
{
	int plot;
	velocity(150, 150);
	forward_mm(60);
	if (curr_loc.x == 0)
	{
		if (grid_matrix[curr_loc.y][curr_loc.x + 1] == -5)
		{
			if (dir_flag == 's')
			{
				left_degrees(90);
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x + 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x + 1, curr_loc.y);
				send_data_uart(1, plot, grid_matrix[curr_loc.y][curr_loc.x + 1]);
				right_degrees(90);
				_delay_ms(100);
			}
			else
			{
				right_degrees(90);
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x + 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x + 1, curr_loc.y);
				send_data_uart(1, plot, grid_matrix[curr_loc.y][curr_loc.x + 1]);
				left_degrees(90);
				_delay_ms(100);
			}
		}
		return;
	}
	else if (curr_loc.x == 8)
	{
		if (grid_matrix[curr_loc.y][curr_loc.x - 1] == -5)
		{
			if (dir_flag == 'n')
			{
				left_degrees(90);
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x - 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x - 1, curr_loc.y);
				send_data_uart(1, plot, grid_matrix[curr_loc.y][curr_loc.x - 1]);
				right_degrees(90);
				_delay_ms(100);
			}
			else
			{
				right_degrees(90);
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x - 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x - 1, curr_loc.y);
				send_data_uart(1, plot, grid_matrix[curr_loc.y][curr_loc.x - 1]);
				left_degrees(90);
				_delay_ms(100);
			}
		}
		return;
	}
	else if (curr_loc.y == 0)
	{
		if (grid_matrix[curr_loc.y + 1][curr_loc.x] == -5)
		{
			if (dir_flag == 'w')
			{
				left_degrees(90);
				_delay_ms(100);
				grid_matrix[curr_loc.y + 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y + 1);
				send_data_uart(1, plot, grid_matrix[curr_loc.y + 1][curr_loc.x]);
				right_degrees(90);
				_delay_ms(100);
			}
			else
			{
				right_degrees(90);
				_delay_ms(100);
				grid_matrix[curr_loc.y + 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y + 1);
				send_data_uart(1, plot, grid_matrix[curr_loc.y + 1][curr_loc.x]);
				left_degrees(90);
				_delay_ms(100);
			}
		}
		return;
	}
	else if (curr_loc.y == 8)
	{
		if (grid_matrix[curr_loc.y - 1][curr_loc.x] == -5)
		{
			if (dir_flag == 'e')
			{
				left_degrees(90);
				_delay_ms(100);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y - 1);
				send_data_uart(1, plot, grid_matrix[curr_loc.y - 1][curr_loc.x]);
				right_degrees(90);
				_delay_ms(100);
			}
			else
			{
				right_degrees(90);
				_delay_ms(100);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y - 1);
				send_data_uart(1, plot, grid_matrix[curr_loc.y - 1][curr_loc.x]);
				left_degrees(90);
				_delay_ms(100);
			}
		}
		return;
	}
	else if (dir_flag == 'e' || dir_flag == 'w')
	{
		if (grid_matrix[curr_loc.y - 1][curr_loc.x] == -5)
		{
			if (dir_flag == 'w')
			{
				left_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y - 1);
				send_data_uart(1, plot, grid_matrix[curr_loc.y - 1][curr_loc.x]);
				right_degrees(90);
				_delay_ms(100);
			}
			else
			{
				right_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y - 1);
				send_data_uart(1, plot, grid_matrix[curr_loc.y - 1][curr_loc.x]);
				left_degrees(90);
				_delay_ms(100);
			}
		}
		if (grid_matrix[curr_loc.y + 1][curr_loc.x] == -5)
		{
			if (dir_flag == 'w')
			{
				right_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y + 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y + 1);
				send_data_uart(1, plot, grid_matrix[curr_loc.y + 1][curr_loc.x]);
				left_degrees(90);
				_delay_ms(100);
			}
			else
			{
				left_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y + 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y + 1);
				send_data_uart(1, plot, grid_matrix[curr_loc.y + 1][curr_loc.x]);
				right_degrees(90);
				_delay_ms(100);
			}
		}
		return;
	}

	else if (dir_flag == 'n' || dir_flag == 's')
	{
		if (grid_matrix[curr_loc.y][curr_loc.x - 1] == -5)
		{
			if (dir_flag == 'n')
			{
				left_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y][curr_loc.x - 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x - 1, curr_loc.y);
				send_data_uart(1, plot, grid_matrix[curr_loc.y][curr_loc.x - 1]);
				right_degrees(90);
				_delay_ms(100);
			}
			else
			{
				right_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y][curr_loc.x - 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x - 1, curr_loc.y);
				send_data_uart(1, plot, grid_matrix[curr_loc.y][curr_loc.x - 1]);
				left_degrees(90);
			}
		}
		if (grid_matrix[curr_loc.y][curr_loc.x + 1] == -5)
		{
			if (dir_flag == 'n')
			{
				right_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y][curr_loc.x + 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x + 1, curr_loc.y);
				send_data_uart(1, plot, grid_matrix[curr_loc.y][curr_loc.x + 1]);
				left_degrees(90);
			}
			else
			{
				left_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y][curr_loc.x + 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x + 1, curr_loc.y);
				send_data_uart(1, plot, grid_matrix[curr_loc.y][curr_loc.x + 1]);
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
		forward();
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
				//	wls_readings_print_lcd();
				break;
			}
			else if (center_wl_sensor_data > THRESHOLD_WLS)
			{
				prev = 0;
				velocity(200, 200);
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
		}
		velocity(200, 200);
		forward_mm(60);
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
	setup_uart(); // UART INIT
	count_time = 0;
	seconds_time = 0;
	ShaftCountLeft = 0;	 //to keep track of left position encoder
	ShaftCountRight = 0; //to keep track of right position encoder
	init_devices();		 // Color Sensor Pins
	color_sensor_scaling();
	position_encoder_pin_config();
	position_encoder_interrupt_config();
	initialize_grid_matrix();
	initialize_plot_coord_matrix();
	initialize_adjacency_matrix();
	initialize_node_coord_matrix();
	buzzer_pin_config();
	LED_bargraph_config();
	lcd_port_config(); // Initialize the LCD port
	lcd_init();		   // Initialize the LCD
	lcd_clear();
	lcd_string(1, 5, "BitBoot");
	lcd_string(2, 4, "IIT  BOMBAY");
	_delay_ms(500);
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

void send_data_uart(int x, int y, int z)
{
	char uart_data_Tx[6];
	sprintf(uart_data_Tx, "%d:%d:%d", x, y, z);
	uart3_puts(uart_data_Tx);
}

void receive_uart_data(void)
{
	rx_byte = uart3_readByte();
	int order = rx_byte - 96;
	lcd_clear();
	lcd_wr_char(2, 2, rx_byte);
	lcd_wr_char(2, 5, order);
	if (order <= 17 && order >= 0)
	{
		send_data_uart(1, 5, 5);
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
		_delay_ms(1000);
	}
	uart3_flush();
}

void left_turn_wls_degress(int degrees)
{
	velocity(200, 200);
	forward_mm(20);
	left_degrees(50);
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	while (1)
	{
		// get the ADC converted data of center white line sensors from their appropriate channel number

		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		ReqdShaftCount = (degrees - 20) / distance_resolution; // division by resolution to get shaft count
		ReqdShaftCountInt = (unsigned long int)ReqdShaftCount;

		ShaftCountRight = 0;
		ShaftCountLeft = 0;

		if ((center_wl_sensor_data > THRESHOLD_WLS) || (right_wl_sensor_data > THRESHOLD_WLS) || (ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))
		{
			stop();
			velocity(0, 0);

			break;
		}
		else
		{
			left();
		}
	}
	if (dir_flag == 'n')
		dir_flag = 'w';
	else if (dir_flag == 'w')
		dir_flag = 's';
	else if (dir_flag == 's')
		dir_flag = 'e';
	else
		dir_flag = 'n';
}

void right_turn_wls_degress(int degrees)
{
	velocity(200, 200);
	forward_mm(20);
	right_degrees(50);
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	while (1)
	{
		// get the ADC converted data of center white line sensors from their appropriate channel number

		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		ReqdShaftCount = (degrees - 20) / distance_resolution; // division by resolution to get shaft count
		ReqdShaftCountInt = (unsigned long int)ReqdShaftCount;

		ShaftCountRight = 0;
		ShaftCountLeft = 0;

		if ((center_wl_sensor_data > THRESHOLD_WLS) || (left_wl_sensor_data > THRESHOLD_WLS) || (ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))

		{
			stop();
			velocity(0, 0);
			break;
		}
		else
		{
			right();
		}
	}
	if (dir_flag == 'n')
		dir_flag = 'e';
	else if (dir_flag == 'e')
		dir_flag = 's';
	else if (dir_flag == 's')
		dir_flag = 'w';
	else
		dir_flag = 'n';
}

void uturn(void)
{
	//allign_at_node();
	velocity(200, 200);
	forward_mm(20);
	left_degrees(160);
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	while (1)
	{
		// get the ADC converted data of center white line sensors from their appropriate channel number

		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		ReqdShaftCount = (50) / distance_resolution; // division by resolution to get shaft count
		ReqdShaftCountInt = (unsigned long int)ReqdShaftCount;

		ShaftCountRight = 0;
		ShaftCountLeft = 0;

		if ((center_wl_sensor_data > THRESHOLD_WLS) || (right_wl_sensor_data > THRESHOLD_WLS) || (ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))

		{
			stop();
			velocity(0, 0);

			break;
		}
		else
		{
			left();
		}
	}
	if (dir_flag == 'n')
		dir_flag = 's';
	else if (dir_flag == 'e')
		dir_flag = 'w';
	else if (dir_flag == 's')
		dir_flag = 'n';
	else
		dir_flag = 'e';
}

int final_mid_point(int plot_no)
{
	tuple mid_point_1, mid_point_2;
	char dir_1, dir_2;
	tuple coord_plot = plot_coord_matrix[plot_no][4];
	mid_point_1.x = coord_plot.x;
	mid_point_1.y = curr_loc.y;
	if ((curr_loc.x == mid_point_1.x) && ((mid_point_1.y - curr_loc.y) == -1))
		dir_1 = 'n';
	else if ((curr_loc.x == mid_point_1.x) && ((mid_point_1.y - curr_loc.y) == 1))
		dir_1 = 's';
	else if ((curr_loc.y == mid_point_1.y) && ((mid_point_1.x - curr_loc.x) == -1))
		dir_1 = 'e';
	else if ((curr_loc.y == mid_point_1.y) && ((mid_point_1.x - curr_loc.x) == 1))
		dir_1 = 'w';

	mid_point_2.x = curr_loc.x;
	mid_point_2.y = coord_plot.y;
	if ((curr_loc.x == mid_point_2.x) && ((mid_point_2.y - curr_loc.y) == -1))
		dir_2 = 'n';
	else if ((curr_loc.x == mid_point_2.x) && ((mid_point_2.y - curr_loc.y) == 1))
		dir_2 = 's';
	else if ((curr_loc.y == mid_point_2.y) && ((mid_point_2.x - curr_loc.x) == -1))
		dir_2 = 'e';
	else if ((curr_loc.y == mid_point_2.y) && ((mid_point_2.x - curr_loc.x) == 1))
		dir_2 = 'w';

	if (dir_flag != dir_1 && dir_flag != dir_2)
	{
		if (dir_flag == 'n')
		{
			if ((dir_2 == 'w') || (dir_1 == 'w'))
			{
				left_turn_wls_degress(90);
			}
			else if ((dir_2 == 'e') || (dir_1 == 'e'))
			{
				right_turn_wls_degress(90);
			}
		}

		else if (dir_flag == 'e')
		{
			if ((dir_2 == 'n') || (dir_1 == 'n'))
			{
				left_turn_wls_degress(90);
			}
			else if ((dir_2 == 's') || (dir_1 == 's'))
			{
				right_turn_wls_degress(90);
			}
		}
		else if (dir_flag == 'w')
		{
			if ((dir_2 == 's') || (dir_1 == 's'))
			{
				left_turn_wls_degress(90);
			}
			else if ((dir_2 == 'n') || (dir_1 == 'n'))
			{
				right_turn_wls_degress(90);
			}
		}
		else if (dir_flag == 's')
		{
			if ((dir_2 == 'w') || (dir_1 == 'w'))
			{
				left_turn_wls_degress(90);
			}
			else if ((dir_2 == 'e') || (dir_1 == 'e'))
			{
				right_turn_wls_degress(90);
			}
		}
	}
	if (dir_flag == dir_1)
	{
		if (grid_matrix[mid_point_1.y][mid_point_1.x] == -1)
			grid_matrix[mid_point_1.y][mid_point_1.x] = check_path_for_debris();
		if (grid_matrix[mid_point_1.y][mid_point_1.x] == 1)
		{
			forward_wls(1);
			_delay_ms(100);
			check_plot_scan_status();
			forward_wls(1);
			_delay_ms(100);
			return 1;
		}
	}
	else
	{
		if (grid_matrix[mid_point_2.y][mid_point_2.x] == -1)
			grid_matrix[mid_point_2.y][mid_point_2.x] = check_path_for_debris();
		if (grid_matrix[mid_point_2.y][mid_point_2.x] == 1)
		{
			forward_wls(1);
			_delay_ms(100);
			check_plot_scan_status();
			forward_wls(1);
			_delay_ms(100);
			return 1;
		}
	}
	return 0;
}