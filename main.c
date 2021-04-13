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
void uturn(void);
void wls_loc_orient_print_lcd(void);
int check_path_for_debris(void);
int final_mid_point(int, int);
void send_data_uart(int, int, int, int);
void traverse(tuple);
void move_robot(int);
void turn_accordingly(tuple);
void turn_towards_mid_point(char);
void service_rpc_request(void);

// To store 8-bit data of left, center and right white line sensors
unsigned char left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data;

int plot_order[16] = {14, 13, 9, 10, 6, 5, 1, 2, 3, 4, 8, 7, 11, 12, 16, 15}, plot_no = 0;
int prev = 0;
char dir_flag = 'n';
char rx_byte = 0;
uint8_t rx_data;
int turn,rpc;
int main(int argc, char *argv[])
{
	//---------------------------------------------Setup Zone-----------------------------------------------

	setup();
	forward_wls(1);
	_delay_ms(10);
	curr_loc.x = 4;
	curr_loc.y = 8;
	dir_flag = 'n';
	top = -1;
	turn = 0;
	int status = 0;
	int plot_scan = -1;
	rpc = 0;
	lcd_clear();
	send_data_uart(2, 4, 8, -1);
	tuple dest_loc, plot_coordinate;
	//wls_loc_orient_print_lcd();

	//---------------------------------------------Test Zone-----------------------------------------------

	//----------------------------------------------Main Code--------------------------------------------------------
	while (plot_no < 16)
	{
		plot_scan = plot_order[plot_no] - 1;
		tuple plot_coord = plot_coord_matrix[plot_scan][4];
		// lcd_clear();
		// lcd_string(1, 2, "Scanning Plot");
		// lcd_numeric_value(2, 1, plot_scan, 2);
		// lcd_wr_char(2, 4, '(');
		// lcd_numeric_value(2, 5, plot_coord.x, 1);
		// lcd_wr_char(2, 6, ',');
		// lcd_numeric_value(2, 7, plot_coord.y, 1);
		// lcd_wr_char(2, 8, ')');
		// _delay_ms(500);
		if (grid_matrix[plot_coord.y][plot_coord.x] == -5)
		{
			while (1)
			{
				if (status == 0)
					dest_loc = get_nearest_coordinate(curr_loc, plot_scan);

				// lcd_clear();
				// lcd_string(1, 2, "Dest Loc");
				// lcd_wr_char(2, 1, '(');
				// lcd_numeric_value(2, 2, dest_loc.x, 1);
				// lcd_wr_char(2, 3, ',');
				// lcd_numeric_value(2, 5, dest_loc.y, 1);
				// lcd_wr_char(2, 6, ')');
				// _delay_ms(500);

				traverse(dest_loc);

				// lcd_clear();
				// lcd_string(1, 1, "Corner Node");
				if (grid_matrix[plot_coord.y][plot_coord.x] != -5)
				{
					status = 0;
					plot_no++;
					break;
				}
				status = final_mid_point(plot_scan, 0);
				// lcd_clear();
				// lcd_string(1, 1, "Status Returned");
				// lcd_numeric_value(2, 2, status, 2);
				// _delay_ms(500);
				if (status == 1)
				{
					plot_no++;
					status = 0;
					break;
				}
				else
				{
					status = -1;
					plot_coordinate.x = plot_coord_matrix[plot_scan][4].x;
					plot_coordinate.y = plot_coord_matrix[plot_scan][4].y;
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
		}
		else
		{
			plot_no++;
			status = 0;
		}
	}
	traverse(goal_loc);
	forward_wls(1);
	return 1;
}
void move_robot(int type)
{
	forward_wls(1);
	send_data_uart(2, curr_loc.x, curr_loc.y, -1);
	_delay_ms(100);
	if (type == 0)
	{
		check_plot_scan_status();
	}
	else
	{
		buzzer_on();
		send_data_uart(3, seconds_time, -1, -1);
		_delay_ms(1000);
		buzzer_off();
	}
	_delay_ms(100);
	forward_wls(1);
	send_data_uart(2, curr_loc.x, curr_loc.y, -1);
	_delay_ms(100);
	//wls_loc_orient_print_lcd();
	//_delay_ms(500);
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
			if (dir_flag == 'e')
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
			if (dir_flag == 'w')
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
	// lcd_clear();
	// lcd_string(1, 2, "Checking Debris");
	// _delay_ms(500);
	left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
	center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
	right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
	//wls_readings_print_lcd();
	if (left_wl_sensor_data > THRESHOLD_WLS || center_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS)
		return 1;

	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = 20 / angle_resolution; // division by resolution to get shaft count
	ReqdShaftCountInt = (float)(unsigned long int)ReqdShaftCount;

	left();
	velocity(120, 120);
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	while ((ShaftCountRight <= ReqdShaftCountInt) && (ShaftCountLeft <= ReqdShaftCountInt))
	{
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		if (left_wl_sensor_data > THRESHOLD_WLS || center_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS)
		{
			stop();
			velocity(0, 0);
			return 1;
		}
	}
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	while ((ShaftCountRight <= 2 * ReqdShaftCountInt) && (ShaftCountLeft <= 2 * ReqdShaftCountInt))
	{
		right();
		velocity(200, 200);
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);

		if (left_wl_sensor_data > THRESHOLD_WLS || center_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS)
		{
			stop();
			velocity(0, 0);
			return 1;
		}
	}
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	while ((ShaftCountRight <= ReqdShaftCountInt) && (ShaftCountLeft <= ReqdShaftCountInt))
	{
		left();
		velocity(200, 200);
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		if (left_wl_sensor_data > THRESHOLD_WLS || center_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS)
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
	if (curr_loc.x == 0)
	{
		if (grid_matrix[curr_loc.y][curr_loc.x + 1] == -5)
		{
			if (dir_flag == 's')
			{
				left_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y][curr_loc.x + 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x + 1, curr_loc.y);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y][curr_loc.x + 1], -1);
				right_degrees(90);
				_delay_ms(10);
			}
			else
			{
				right_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y][curr_loc.x + 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x + 1, curr_loc.y);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y][curr_loc.x + 1], -1);
				left_degrees(90);
				_delay_ms(10);
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
				_delay_ms(10);
				grid_matrix[curr_loc.y][curr_loc.x - 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x - 1, curr_loc.y);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y][curr_loc.x - 1], -1);
				right_degrees(90);
				_delay_ms(10);
			}
			else
			{
				right_degrees(90);
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x - 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x - 1, curr_loc.y);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y][curr_loc.x - 1], -1);
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
				_delay_ms(10);
				grid_matrix[curr_loc.y + 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y + 1);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y + 1][curr_loc.x], -1);
				right_degrees(90);
				_delay_ms(10);
			}
			else
			{
				right_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y + 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y + 1);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y + 1][curr_loc.x], -1);
				left_degrees(90);
				_delay_ms(10);
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
				_delay_ms(10);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y - 1);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y - 1][curr_loc.x], -1);
				right_degrees(90);
				_delay_ms(10);
			}
			else
			{
				right_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y - 1);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y - 1][curr_loc.x], -1);
				left_degrees(90);
				_delay_ms(10);
			}
		}
		return;
	}
	else if (dir_flag == 'e' || dir_flag == 'w')
	{
		if (grid_matrix[curr_loc.y - 1][curr_loc.x] == -5)
		{
			if (dir_flag == 'e')
			{
				left_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y - 1);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y - 1][curr_loc.x], -1);
				right_degrees(90);
				_delay_ms(10);
			}
			else
			{
				right_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y - 1);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y - 1][curr_loc.x], -1);
				left_degrees(90);
				_delay_ms(10);
			}
		}
		if (grid_matrix[curr_loc.y + 1][curr_loc.x] == -5)
		{
			if (dir_flag == 'w')
			{
				left_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y + 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y + 1);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y + 1][curr_loc.x], -1);
				right_degrees(90);
				_delay_ms(10);
			}
			else
			{
				right_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y + 1][curr_loc.x] = color_type();
				plot = get_plot_from_coord(curr_loc.x, curr_loc.y + 1);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y + 1][curr_loc.x], -1);
				left_degrees(90);
				_delay_ms(10);
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
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y][curr_loc.x - 1], -1);
				right_degrees(90);
				_delay_ms(10);
			}
			else
			{
				right_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y][curr_loc.x - 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x - 1, curr_loc.y);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y][curr_loc.x - 1], -1);
				left_degrees(90);
				_delay_ms(10);
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
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y][curr_loc.x + 1], -1);
				left_degrees(90);
				_delay_ms(10);
			}
			else
			{
				left_degrees(90);
				_delay_ms(10);
				grid_matrix[curr_loc.y][curr_loc.x + 1] = color_type();
				plot = get_plot_from_coord(curr_loc.x + 1, curr_loc.y);
				send_data_uart(1, plot + 1, grid_matrix[curr_loc.y][curr_loc.x + 1], -1);
				right_degrees(90);
				_delay_ms(10);
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
				// lcd_clear();
				// lcd_string(1, 2, "Node Reached");
				++node_reached;
				//lcd_numeric_value(2, 5, node_reached, 2);
				//wls_readings_print_lcd();
				_delay_ms(50);
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
				velocity(120, 200);
				// printf("\n left");
			}
			else if (right_wl_sensor_data > THRESHOLD_WLS)
			{
				prev = 2;
				velocity(200, 120);
				// printf("\n right");
			}
			else
			{
				if (prev == 1)
					velocity(120, 200);
				else if (prev == 2)
					velocity(200, 120);
			}
			_delay_ms(10);
		}
		velocity(200, 200);
		forward_mm(35);
		//wls_readings_print_lcd();
		//_delay_ms(2000);
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

	lcd_init(); // Initialize the LCD
	lcd_clear();
	lcd_string(1, 5, "BitBoot");
	lcd_string(2, 4, "IIT  BOMBAY");
	_delay_ms(500);
	goal_loc.x = 8;
	goal_loc.y = 4;
	int c = 0;
	send_data_uart(-1, -1, -1, -1);
	lcd_string(1, 2, "Send 's' to topic");
	lcd_string(2, 2, "data_recv to start");
	while (1)
	{
		c = dequeue();
		if (c == 's')
			break;
	}
}

void wls_readings_print_lcd(void)
{
	left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
	lcd_clear();
	lcd_string(1, 2, "Left Sensor");
	lcd_numeric_value(2, 5, left_wl_sensor_data, 3);
	_delay_ms(500);
	center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
	lcd_clear();
	lcd_string(1, 2, "Center Sensor");
	lcd_numeric_value(2, 5, center_wl_sensor_data, 3);
	_delay_ms(500);
	right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
	lcd_clear();
	lcd_string(1, 2, "Right Sensor");
	lcd_numeric_value(2, 5, right_wl_sensor_data, 3);
	_delay_ms(500);
}

void wls_loc_orient_print_lcd(void)
{
	lcd_clear();
	lcd_string(1, 2, "Loc & Orient");
	lcd_wr_char(2, 1, '(');
	lcd_numeric_value(2, 2, curr_loc.x, 1);
	lcd_wr_char(2, 3, ',');
	lcd_numeric_value(2, 5, curr_loc.y, 1);
	lcd_wr_char(2, 6, ')');
	lcd_wr_char(2, 9, dir_flag);
	_delay_ms(250);
}

void send_data_uart(int a, int x, int y, int z)
{
	char uart_data_Tx[11];
	sprintf(uart_data_Tx, "%d:%d:%d:%d;", a, x, y, z);
	uart3_puts(uart_data_Tx);
}

void left_turn_wls_degress(int degrees)
{
	velocity(200, 200);
	if (turn != 1)
		forward_mm(45);
	left_degrees(70);
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = (float)(degrees - 50) / angle_resolution; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int)ReqdShaftCount;

	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	turn = 1;
	while (1)
	{
		// get the ADC converted data of center white line sensors from their appropriate channel number

		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);

		if ((center_wl_sensor_data > THRESHOLD_WLS) || (right_wl_sensor_data > THRESHOLD_WLS) || (ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))
		{
			stop();
			velocity(0, 0);
			_delay_ms(10);
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
	if (turn != 1)
		forward_mm(45);
	right_degrees(70);
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = (float)(degrees - 50) / angle_resolution; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int)ReqdShaftCount;
	turn = 1;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	while (1)
	{
		// get the ADC converted data of center white line sensors from their appropriate channel number

		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);

		if ((center_wl_sensor_data > THRESHOLD_WLS) || (left_wl_sensor_data > THRESHOLD_WLS) || (ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))

		{
			stop();
			velocity(0, 0);
			_delay_ms(10);
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
	if (turn != 1)
		forward_mm(20);
	left_degrees(160);
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	turn = 1;
	while (1)
	{
		// get the ADC converted data of center white line sensors from their appropriate channel number

		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
		ReqdShaftCount = (40) / angle_resolution; // division by resolution to get shaft count
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

int final_mid_point(int plot_no, int type)
{
	tuple mid_point_1, mid_point_2;
	char dir_1, dir_2;
	tuple coord_plot = plot_coord_matrix[plot_no][4];
	mid_point_1.x = coord_plot.x;
	mid_point_1.y = curr_loc.y;

	//lcd_clear();
	//lcd_string(1, 2, "Mid-Point Check");
	//_delay_ms(1500);

	if ((curr_loc.x == mid_point_1.x) && ((mid_point_1.y - curr_loc.y) == -1))
		dir_1 = 'n';
	else if ((curr_loc.x == mid_point_1.x) && ((mid_point_1.y - curr_loc.y) == 1))
		dir_1 = 's';
	else if ((curr_loc.y == mid_point_1.y) && ((mid_point_1.x - curr_loc.x) == -1))
		dir_1 = 'w';
	else if ((curr_loc.y == mid_point_1.y) && ((mid_point_1.x - curr_loc.x) == 1))
		dir_1 = 'e';

	mid_point_2.x = curr_loc.x;
	mid_point_2.y = coord_plot.y;
	if ((curr_loc.x == mid_point_2.x) && ((mid_point_2.y - curr_loc.y) == -1))
		dir_2 = 'n';
	else if ((curr_loc.x == mid_point_2.x) && ((mid_point_2.y - curr_loc.y) == 1))
		dir_2 = 's';
	else if ((curr_loc.y == mid_point_2.y) && ((mid_point_2.x - curr_loc.x) == -1))
		dir_2 = 'w';
	else if ((curr_loc.y == mid_point_2.y) && ((mid_point_2.x - curr_loc.x) == 1))
		dir_2 = 'e';

	// lcd_clear();
	// lcd_string(1, 2, "Mid-Point 1");
	// lcd_wr_char(2, 1, '(');
	// lcd_numeric_value(2, 2, mid_point_1.x, 1);
	// lcd_wr_char(2, 3, ',');
	// lcd_numeric_value(2, 5, mid_point_1.y, 1);
	// lcd_wr_char(2, 6, ')');
	// lcd_wr_char(2, 9, dir_1);
	// _delay_ms(500);

	// lcd_clear();
	// lcd_string(1, 2, "Mid-Point 2");
	// lcd_wr_char(2, 1, '(');
	// lcd_numeric_value(2, 2, mid_point_2.x, 1);
	// lcd_wr_char(2, 3, ',');
	// lcd_numeric_value(2, 5, mid_point_2.y, 1);
	// lcd_wr_char(2, 6, ')');
	// lcd_wr_char(2, 9, dir_2);
	// _delay_ms(500);
	if (dir_flag == dir_1)
	{
		if (grid_matrix[mid_point_1.y][mid_point_1.x] == -1)
		{
			grid_matrix[mid_point_1.y][mid_point_1.x] = check_path_for_debris();
			send_data_uart(4, mid_point_1.x, mid_point_1.y, grid_matrix[mid_point_1.y][mid_point_1.x]);
			update_adjacency_matrix();
		}
		if (grid_matrix[mid_point_1.y][mid_point_1.x] == 1)
		{
			move_robot(type);
			turn = 0;
			return 1;
		}
	}
	else if (dir_flag == dir_2)
	{
		if (grid_matrix[mid_point_2.y][mid_point_2.x] == -1)
		{
			grid_matrix[mid_point_2.y][mid_point_2.x] = check_path_for_debris();
			send_data_uart(4, mid_point_2.x, mid_point_2.y, grid_matrix[mid_point_2.y][mid_point_2.x]);
			update_adjacency_matrix();
		}
		if (grid_matrix[mid_point_2.y][mid_point_2.x] == 1)
		{
			move_robot(type);
			turn = 0;
			return 1;
		}
	}
	if (dir_flag != dir_1)
	{
		turn_towards_mid_point(dir_1);
		if (grid_matrix[mid_point_1.y][mid_point_1.x] == -1)
		{
			grid_matrix[mid_point_1.y][mid_point_1.x] = check_path_for_debris();
			send_data_uart(4, mid_point_1.x, mid_point_1.y, grid_matrix[mid_point_1.y][mid_point_1.x]);
			update_adjacency_matrix();
		}
		if (grid_matrix[mid_point_1.y][mid_point_1.x] == 1)
		{
			move_robot(type);
			turn = 0;
			return 1;
		}
	}
	if (dir_flag != dir_2)
	{
		turn_towards_mid_point(dir_2);
		if (grid_matrix[mid_point_2.y][mid_point_2.x] == -1)
		{
			grid_matrix[mid_point_2.y][mid_point_2.x] = check_path_for_debris();
			send_data_uart(4, mid_point_2.x, mid_point_2.y, grid_matrix[mid_point_2.y][mid_point_2.x]);
			update_adjacency_matrix();
		}
		if (grid_matrix[mid_point_2.y][mid_point_2.x] == 1)
		{
			move_robot(type);
			turn = 0;
			return 1;
		}
	}
	return 0;
}

void turn_towards_mid_point(char dir)
{
	if (dir_flag != dir)
	{
		if (dir_flag == 'n')
		{
			if (dir == 'w')
				left_turn_wls_degress(90);
			else if (dir == 'e')
				right_turn_wls_degress(90);
			else
				uturn();
		}
		else if (dir_flag == 'e')
		{
			if (dir == 'n')
				left_turn_wls_degress(90);
			else if (dir == 's')
				right_turn_wls_degress(90);
			else
				uturn();
		}
		else if (dir_flag == 'w')
		{
			if (dir == 's')
				left_turn_wls_degress(90);
			else if (dir == 'n')
				right_turn_wls_degress(90);
			else
				uturn();
		}
		else if (dir_flag == 's')
		{
			if (dir == 'e')
				left_turn_wls_degress(90);
			else if (dir == 'w')
				right_turn_wls_degress(90);
			else
				uturn();
		}
	}
}

void traverse(tuple destination_location)
{
	tuple next_loc;
	int d_node, s_node, node;
	d_node = get_node_from_coord(destination_location);
	// lcd_clear();
	// lcd_string(1, 1, "Scanning Algo..");
	// _delay_ms(1000);
	// lcd_clear();
	// lcd_string(1, 1, "Destination Node");
	// lcd_numeric_value(2, 1, d_node, 2);
	// _delay_ms(500);
	while (!((destination_location.x == curr_loc.x) && (destination_location.y == curr_loc.y)))
	{
		s_node = get_node_from_coord(curr_loc);
		// lcd_clear();
		// lcd_string(1, 1, "Source Node");
		// lcd_numeric_value(2, 1, s_node, 2);
		// lcd_wr_char(2, 3, '(');
		// lcd_numeric_value(2, 4, curr_loc.x, 1);
		// lcd_wr_char(2, 5, ',');
		// lcd_numeric_value(2, 6, curr_loc.y, 1);
		// lcd_wr_char(2, 7, ')');
		// _delay_ms(500);
		// lcd_clear();
		// lcd_string(1, 1, "Top Value Initial");
		// lcd_numeric_value(2, 2, top, 2);
		// _delay_ms(2000);
		dijkstra(adjacency_matrix, 25, s_node, d_node);
		while (top > -1)
		{
			// lcd_clear();
			// lcd_string(1, 1, "Top Value Final");
			// lcd_numeric_value(2, 2, top, 2);
			// _delay_ms(2000);
			node = pop();
			next_loc = node_coord_matrix[node];
			// lcd_clear();
			// lcd_string(1, 1, "Next Node");
			// lcd_numeric_value(2, 1, node, 2);
			// lcd_wr_char(2, 3, '(');
			// lcd_numeric_value(2, 4, next_loc.x, 1);
			// lcd_wr_char(2, 5, ',');
			// lcd_numeric_value(2, 6, next_loc.y, 1);
			// lcd_wr_char(2, 7, ')');
			// _delay_ms(500);
			turn_accordingly(next_loc);
			wls_loc_orient_print_lcd();
			if ((next_loc.x == curr_loc.x + 2) && (next_loc.y == curr_loc.y))
			{
				if (grid_matrix[curr_loc.y][curr_loc.x + 1] == -1)
				{
					grid_matrix[curr_loc.y][curr_loc.x + 1] = check_path_for_debris();
					send_data_uart(4, curr_loc.x + 1, curr_loc.y, grid_matrix[curr_loc.y][curr_loc.x + 1]);
					update_adjacency_matrix();
				}
				if (grid_matrix[curr_loc.y][curr_loc.x + 1] == 1)
				{
					move_robot(0);
					turn = 0;
				}
				else
				{
					if(rpc == 0)
						service_rpc_request();
					top = -1;
					break;
				}
			}
			else if ((next_loc.x == curr_loc.x - 2) && (next_loc.y == curr_loc.y))
			{
				if (grid_matrix[curr_loc.y][curr_loc.x - 1] == -1)
				{
					grid_matrix[curr_loc.y][curr_loc.x - 1] = check_path_for_debris();
					send_data_uart(4, curr_loc.x - 1, curr_loc.y, grid_matrix[curr_loc.y][curr_loc.x - 1]);
					update_adjacency_matrix();
				}
				if (grid_matrix[curr_loc.y][curr_loc.x - 1] == 1)
				{
					move_robot(0);
					turn = 0;
				}
				else
				{
					if(rpc == 0)
						service_rpc_request();
					top = -1;
					break;
				}
			}

			else if ((next_loc.x == curr_loc.x) && (next_loc.y == curr_loc.y + 2))
			{
				if (grid_matrix[curr_loc.y + 1][curr_loc.x] == -1)
				{
					grid_matrix[curr_loc.y + 1][curr_loc.x] = check_path_for_debris();
					send_data_uart(4, curr_loc.x, curr_loc.y + 1, grid_matrix[curr_loc.y + 1][curr_loc.x]);
					update_adjacency_matrix();
				}
				if (grid_matrix[curr_loc.y + 1][curr_loc.x] == 1)
				{
					move_robot(0);
					turn = 0;
				}
				else
				{
					if(rpc == 0)
						service_rpc_request();
					top = -1;
					break;
				}
			}
			else if ((next_loc.x == curr_loc.x) && (next_loc.y == curr_loc.y - 2))
			{
				if (grid_matrix[curr_loc.y - 1][curr_loc.x] == -1)
				{
					grid_matrix[curr_loc.y - 1][curr_loc.x] = check_path_for_debris();
					send_data_uart(4, curr_loc.x, curr_loc.y - 1, grid_matrix[curr_loc.y - 1][curr_loc.x]);
					update_adjacency_matrix();
				}
				if (grid_matrix[curr_loc.y - 1][curr_loc.x] == 1)
				{
					move_robot(0);
					turn = 0;
				}
				else
				{
					if(rpc == 0)
						service_rpc_request();
					top = -1;
					break;
				}
			}
			if(rpc == 0)
						service_rpc_request();
			if (rpc == 1)
			{
				rpc = 0;
				break;
			}
		}
	}
}

void service_rpc_request()
{
	int status = 0;
	tuple dest_loc, plot_coordinate;
	char p, t;
	int order, time, Ctime;
	int s_node, d_node;
	lcd_clear();
	lcd_string(1, 2,"RPC Request Served");
	lcd_numeric_value(2,2,counter_queue_val,2);
	_delay_ms(3000);
	while (counter_queue_val > 0 && counter_queue_val % 2 == 0)
	{
		rpc = 0;
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
		_delay_ms(500);
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
		p = dequeue();
		t = dequeue();
		count_time = 0;
		seconds_time = 0;
		order = (int)p - 64;
		time = (int)t;
		if (time >= 48 && time <= 57)
			Ctime = time - 48;
		else if (time >= 65 && time <= 90)
			Ctime = time - 55;
		else
			Ctime = time - 61;
		lcd_clear();
		lcd_string(1, 2,"PT");
		lcd_numeric_value(1, 5, order, 2);
		lcd_string(2, 2,"Time");
		lcd_numeric_value(2, 6, Ctime, 2);
		_delay_ms(3000);
		if (order == 17 || order == 18 || order == 19)
		{
			if (order == 17)
				order = fetch_nearest_plot(5);
			else if (order == 18)
				order = fetch_nearest_plot(4);
			else
				order = fetch_nearest_plot(3);

			if (order == -1)
			{
				lcd_clear();
				lcd_string(1, 2,"No Injury ");
				lcd_string(2, 2,"Type Found");
				_delay_ms(3000);
				send_data_uart(3, -1, -1, -1);
				continue;
			}
		}
		int plot_scan = order - 1;
		tuple plot_coord = plot_coord_matrix[plot_scan][4];
		dest_loc = get_nearest_coordinate(curr_loc, plot_scan);
		d_node = get_node_from_coord(dest_loc);
		s_node = get_node_from_coord(curr_loc);
		dijkstra(adjacency_matrix, 25, s_node, d_node);
		if (top > -1)
		{
			if ((((top + 1) * 1) + 2) >= Ctime)
			{
				lcd_clear();
				lcd_string(1, 2,"Plot Not ");
				lcd_string(2, 2,"Reachable");
				_delay_ms(3000);
				send_data_uart(3, -1, -1, -2);
				continue;
			}
		}
		rpc = 1;
		lcd_clear();
		lcd_string(1, 2,"Serving Plot");
		lcd_numeric_value(2, 1, plot_scan, 2);
		lcd_wr_char(2, 4, '(');
		lcd_numeric_value(2, 5, plot_coord.x, 1);
		lcd_wr_char(2, 6, ',');
		lcd_numeric_value(2, 7, plot_coord.y, 1);
		lcd_wr_char(2, 8, ')');
		_delay_ms(3000);
		while (1)
		{
			if (status == 0)
				dest_loc = get_nearest_coordinate(curr_loc, plot_scan);
			lcd_clear();
			lcd_string(1, 2,"Dest Loc");
			lcd_wr_char(2, 1, '(');
			lcd_numeric_value(2, 2, dest_loc.x, 1);
			lcd_wr_char(2, 3, ',');
			lcd_numeric_value(2, 5, dest_loc.y, 1);
			lcd_wr_char(2, 6, ')');
			_delay_ms(3000);

			traverse(dest_loc);
			lcd_clear();
			lcd_string(1, 2,"Mid Point");
			_delay_ms(3000);

			status = final_mid_point(plot_scan, 1);

			if (status == 1)
			{
				return;
			}
			else
			{
				status = -1;
				plot_coordinate.x = plot_coord_matrix[plot_scan][4].x;
				plot_coordinate.y = plot_coord_matrix[plot_scan][4].y;
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
	}
}