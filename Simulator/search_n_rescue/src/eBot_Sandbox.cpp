/*
 * eBot_Sandbox.cpp
 *  Created on: 21-Mar-2021
 *      Author: TAs of CS 684 Spring 2020
 */

//---------------------------------- INCLUDES -----------------------------------
#include "eBot_Sandbox.h"
#include <math.h>
#include <stdio.h>
#define INFINITE 9999
#define Threshold 200

//------------------------------ GLOBAL VARIABLES -------------------------------

typedef struct tuple
{
    int x, y;
} tuple;

tuple curr_loc, goal_loc;
int grid_matrix[9][9];
tuple plot_coord_matrix[16][5];
tuple node_coord_matrix[25];
int adjacency_matrix[25][25];
int visited[25];
int cost[25][25], distance[25], pred[25],path_stack[25];
int top;
char dir_flag = 'n';
unsigned char left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data;

int plot_order[16] = {14, 13, 9, 10, 6, 5, 1, 2, 3, 4, 8, 7, 11, 12, 16, 15}, plot_no = 0;
int prev = 0;
char rx_byte = 0;
uint8_t rx_data;
int turn, rpc;
tuple plot_coord;

//------------------------------ FUNCTION DECLARATIONS-------------------------------

//push into stack
void push(int);

//pop from the stack
int pop(void);

// initialises the node coordinate matrix
void initialize_node_coord_matrix(void);

// initialises the plot's node's coordinate matrix
void initialize_plot_coord_matrix(void);

// initialises the grid matrix
void initialize_grid_matrix(void);

// initialises the adjacency matrix
void initialize_adjacency_matrix(void);

// gives the nearest coordinate
tuple get_nearest_coordinate(tuple,int);

// gives the node number for a particular coordinate
int get_node_from_coord(tuple);

// gives the plot number for a particular coordinate
int get_plot_from_coord(int,int);

// updates the adjacency matrix from the grid matrix
void update_adjacency_matrix(void);

// implements the BFS Algorithm using the adjacency matrix
void dijkstra(int G[25][25], int , int , int );


// Fetches nearest block of a particular type injury
int fetch_nearest_plot(int);


void print_ind_plot_coord_arr(int);

void print_plot_coord_matrix(void);

void print_node_coord_matrix(void);

void print_adjacency_matrix(void);

void print_grid_matrix(void);

void print_stack_content(void);

void forward_wls();

void left_turn_wls(void);

void right_turn_wls(void);

void check_plot_scan_status();

void u_turn_wls(void);

int check_path_for_debris(void);

int final_mid_point(int, int);

void traverse(tuple);

void move_robot();

void turn_accordingly(tuple);

void turn_towards_mid_point(char);

void get_wls_data(void);
//------------------------------------------------------------------------------MAIN ALGORITHM---------------------------------------------------------------------------------

void scanplot()
{
	initialize_grid_matrix();
	initialize_plot_coord_matrix();
	initialize_adjacency_matrix();
	initialize_node_coord_matrix();
	forward_wls();
	_delay_ms(10);
	curr_loc.x = 4;
	curr_loc.y = 8;
	dir_flag = 'n';
	top = -1;
	turn = 0;
	int status = 0;
	int plot_scan = -1;
	rpc = 0;
	tuple dest_loc;
	update_adjacency_matrix();
	//---------------------------------------------Test Zone-----------------------------------------------
//	forward_wls();
//	forward_wls();
//	forward_wls();
//	forward_wls();
//	while(1);
	//----------------------------------------------Main Code--------------------------------------------------------
	while (plot_no < 16)
	{
		plot_scan = plot_order[plot_no] - 1;
		plot_coord = plot_coord_matrix[plot_scan][4];
		if (grid_matrix[plot_coord.y][plot_coord.x] == -5)
		{
			while (1)
			{
				if (status == 0)
					dest_loc = get_nearest_coordinate(curr_loc, plot_scan);


				traverse(dest_loc);

				if (grid_matrix[plot_coord.y][plot_coord.x] != -5)
				{
					status = 0;
					plot_no++;
					break;
				}
				status = final_mid_point(plot_scan, 0);
				if (status == 1)
				{
					plot_no++;
					status = 0;
					break;
				}
				else
				{
					status = -1;
					if (curr_loc.x - plot_coord.x == 1)
						dest_loc.x = curr_loc.x - 2;
					else
						dest_loc.x = curr_loc.x + 2;
					if (curr_loc.y - plot_coord.y == 1)
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
	forward_wls();
	return;
}
//------------------------------------------------------------------------------MOTION PART---------------------------------------------------------------------------------

void forward_wls()
{
	velocity(20, 20);
	forward();
	prev = 0;
	while (1)
	{

		left_wl_sensor_data = convert_analog_channel_data(
			left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(
			center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(
			right_wl_sensor_channel);
		printf("\n Motion: %d %d %d\n", left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data);
		if ((center_wl_sensor_data > Threshold) && ((left_wl_sensor_data > Threshold) && (right_wl_sensor_data > Threshold)))
		{
			stop();
			_delay_ms(500);
			printf("\n Node Reached\n");
			printf("\n %d %d %d\n", left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data);
			prev = 0;
			break;
		}
		else if (center_wl_sensor_data > Threshold)
			velocity(20, 20);
		else if (left_wl_sensor_data > Threshold)
		{
			prev = 1;
			velocity(10, 40);
		}
		else if (right_wl_sensor_data > Threshold)
		{
			prev = 2;
			velocity(40, 10);
		}
		else
		{
			if (prev == 1)
				velocity(10, 40);
		    else if (prev == 2)
				velocity(40, 10);
		}
	}
	velocity(20,20);
	forward();
	_delay_ms(100);
	stop();
	velocity(0, 0);
	if (dir_flag == 'n')
		curr_loc.y -= 1;
	else if (dir_flag == 'w')
		curr_loc.x -= 1;
	else if (dir_flag == 's')
		curr_loc.y += 1;
	else
		curr_loc.x += 1;
}

void left_turn_wls(void)
{
	velocity(20, 10);
	_delay_ms(100);
	while (1)
	{
		left_wl_sensor_data = convert_analog_channel_data(
			left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(
			center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(
			right_wl_sensor_channel);
		if (center_wl_sensor_data > Threshold )
		{
			stop();
			velocity(0, 0);
			/*printf("\n Left End: %d %d %d\n", left_wl_sensor_data,
				   center_wl_sensor_data, right_wl_sensor_data);*/
			break;
		}
		else
		{

			velocity(10, 20);
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

void right_turn_wls(void)
{
	velocity(20, 10);
	_delay_ms(100);
	while (1)
	{
		left_wl_sensor_data = convert_analog_channel_data(
			left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(
			center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(
			right_wl_sensor_channel);
		if (center_wl_sensor_data > Threshold )
		{
			stop();
			velocity(0, 0);
			//printf("\n Right End: %d %d %d\n", left_wl_sensor_data,
				//   center_wl_sensor_data, right_wl_sensor_data);
			break;
		}
		else
		{
			velocity(20, 10);
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

void u_turn_wls(void)
{
	velocity(10, 10);
	right();
	_delay_ms(180);
	while (1)
	{
		left_wl_sensor_data = convert_analog_channel_data(
			left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(
			center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(
			right_wl_sensor_channel);
		if (center_wl_sensor_data > Threshold )
		{
			stop();
			velocity(0, 0);
			/*printf("\n u_turn_wls End: %d %d %d\n", left_wl_sensor_data,
				   center_wl_sensor_data, right_wl_sensor_data);*/
			break;
		}
		else
			right();
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

void left_turn_90(void)
{
	velocity(20, 20);
	left();
	_delay_ms(150);
	stop();
	velocity(0, 0);
	_delay_ms(500);
	if (dir_flag == 'n')
		dir_flag = 'w';
	else if (dir_flag == 'w')
		dir_flag = 's';
	else if (dir_flag == 's')
		dir_flag = 'e';
	else
		dir_flag = 'n';
}

void right_turn_90(void)
{
	velocity(20, 20);
	right();
	_delay_ms(160);
	stop();
	velocity(0, 0);
	_delay_ms(500);
	if (dir_flag == 'n')
		dir_flag = 'e';
	else if (dir_flag == 'e')
		dir_flag = 's';
	else if (dir_flag == 's')
		dir_flag = 'w';
	else
		dir_flag = 'n';
}

int check_path_for_debris(void)
{
	velocity(10, 10);
	left();
	for (int i = 0; i < 30; i++)
	{
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(
			center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);

		if (left_wl_sensor_data > Threshold || center_wl_sensor_data > Threshold || right_wl_sensor_data > Threshold)
		{
			stop();
			return 1;
		}
	}

	stop();
	right();
	for (int i = 0; i < 60; i++)
	{
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(
			center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);

		if (left_wl_sensor_data > Threshold || center_wl_sensor_data > Threshold || right_wl_sensor_data > Threshold)
		{
			stop();
			return 1;
		}
	}

	stop();
	left();
	for (int i = 0; i < 30; i++)
	{
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(
			center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);

		if (left_wl_sensor_data > Threshold || center_wl_sensor_data > Threshold || right_wl_sensor_data > Threshold)
		{
			stop();
			return 1;
		}
	}
	stop();
	_delay_ms(10);
	return 0;
}

void check_plot_scan_status()
{
	velocity(20, 20);
	forward();
	_delay_ms(50);
	velocity(0, 0);
	stop();
	_delay_ms(100);
	if (curr_loc.x == 0)
	{
		if (grid_matrix[curr_loc.y][curr_loc.x + 1] == -5)
		{
			if (dir_flag == 's')
			{
				left_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x + 1] = print_color_sensor_data();
				right_turn_wls();
				_delay_ms(100);
			}
			else
			{
				right_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x + 1] = print_color_sensor_data();
				left_turn_wls();
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
				left_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x - 1] = print_color_sensor_data();
				right_turn_wls();
				_delay_ms(100);
			}
			else
			{
				right_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x - 1] = print_color_sensor_data();
				left_turn_wls();
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
				left_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y + 1][curr_loc.x] = print_color_sensor_data();
				right_turn_wls();
				_delay_ms(100);
			}
			else
			{
				right_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y + 1][curr_loc.x] = print_color_sensor_data();
				left_turn_wls();
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
				left_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = print_color_sensor_data();
				right_turn_wls();
				_delay_ms(100);
			}
			else
			{
				right_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = print_color_sensor_data();
				left_turn_wls();
				_delay_ms(100);
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
				left_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = print_color_sensor_data();
				right_turn_wls();
				_delay_ms(100);
			}
			else
			{
				right_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = print_color_sensor_data();
				left_turn_wls();
				_delay_ms(100);
			}
		}
		if (grid_matrix[curr_loc.y + 1][curr_loc.x] == -5)
		{
			if (dir_flag == 'e')
			{
				right_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y + 1][curr_loc.x] = print_color_sensor_data();
				left_turn_wls();
				_delay_ms(100);
			}
			else
			{
				left_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y + 1][curr_loc.x] = print_color_sensor_data();
				right_turn_wls();
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
				left_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x - 1] = print_color_sensor_data();
				right_turn_wls();
				_delay_ms(100);
			}
			else
			{
				right_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x - 1] = print_color_sensor_data();
				left_turn_wls();
				_delay_ms(100);
			}
		}
		if (grid_matrix[curr_loc.y][curr_loc.x + 1] == -5)
		{
			if (dir_flag == 'n')
			{
				right_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x + 1] = print_color_sensor_data();
				left_turn_wls();
				_delay_ms(100);
			}
			else
			{
				left_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x + 1] = print_color_sensor_data();
				right_turn_wls();
				_delay_ms(100);
			}
		}
		return;
	}
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
				left_turn_wls();
			else if (dir_flag == 'e')
				u_turn_wls();
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
			else if (dir_flag == 'w')
				u_turn_wls();
			else
				right_turn_wls();
		}
	}
	else if (y_dis < 0)
	{
		if (dir_flag != 'n')
		{
			if (dir_flag == 'e')
				left_turn_wls();
			else if (dir_flag == 's')
				u_turn_wls();
			else
				right_turn_wls();
		}
	}
	else if (y_dis > 0)
	{
		if (dir_flag != 's')
		{
			if (dir_flag == 'w')
				left_turn_wls();
			else if (dir_flag == 'n')
				u_turn_wls();
			else
				right_turn_wls();
		}
	}
}


void move_robot()
{
	forward_wls();
	_delay_ms(100);
	check_plot_scan_status();
	_delay_ms(100);
	forward_wls();
}

void get_wls_data(void)
{
	left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
	center_wl_sensor_data = convert_analog_channel_data(
		center_wl_sensor_channel);
	right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
	printf("\n%d %d %d\n", left_wl_sensor_data, center_wl_sensor_data,
		   right_wl_sensor_data);
}

void turn_towards_mid_point(char dir)
{
	if (dir_flag != dir)
	{
		if (dir_flag == 'n')
		{
			if (dir == 'w')
				left_turn_wls();
			else if (dir == 'e')
				right_turn_wls();
			else
				u_turn_wls();
		}
		else if (dir_flag == 'e')
		{
			if (dir == 'n')
				left_turn_wls();
			else if (dir == 's')
				right_turn_wls();
			else
				u_turn_wls();
		}
		else if (dir_flag == 'w')
		{
			if (dir == 's')
				left_turn_wls();
			else if (dir == 'n')
				right_turn_wls();
			else
				u_turn_wls();
		}
		else if (dir_flag == 's')
		{
			if (dir == 'e')
				left_turn_wls();
			else if (dir == 'w')
				right_turn_wls();
			else
				u_turn_wls();
		}
	}
}

//------------------------------------------------------------------------------ALGORITHM PART---------------------------------------------------------------------------------
int final_mid_point(int plot_no, int type)
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

	if (dir_flag == dir_1)
	{
		if (grid_matrix[mid_point_1.y][mid_point_1.x] == -1)
		{
			grid_matrix[mid_point_1.y][mid_point_1.x] = check_path_for_debris();
			update_adjacency_matrix();
		}
		if (grid_matrix[mid_point_1.y][mid_point_1.x] == 1)
		{
			move_robot();
			turn = 0;
			return 1;
		}
	}
	else if (dir_flag == dir_2)
	{
		if (grid_matrix[mid_point_2.y][mid_point_2.x] == -1)
		{
			grid_matrix[mid_point_2.y][mid_point_2.x] = check_path_for_debris();
			update_adjacency_matrix();
		}
		if (grid_matrix[mid_point_2.y][mid_point_2.x] == 1)
		{
			move_robot();
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
			update_adjacency_matrix();
		}
		if (grid_matrix[mid_point_1.y][mid_point_1.x] == 1)
		{
			move_robot();
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
			update_adjacency_matrix();
		}
		if (grid_matrix[mid_point_2.y][mid_point_2.x] == 1)
		{
			move_robot();
			turn = 0;
			return 1;
		}
	}
	return 0;
}

void traverse(tuple destination_location)
{
	tuple next_loc;
	int d_node, s_node, node;
	d_node = get_node_from_coord(destination_location);
	while (!((destination_location.x == curr_loc.x) && (destination_location.y == curr_loc.y)))
	{
		if (grid_matrix[plot_coord.y][plot_coord.x] != -5)
					break;
		s_node = get_node_from_coord(curr_loc);
		top = -1;
		dijkstra(adjacency_matrix, 25, s_node, d_node);
		while (top > -1)
		{
			node = pop();
			next_loc = node_coord_matrix[node];
			turn_accordingly(next_loc);
			if ((next_loc.x == curr_loc.x + 2) && (next_loc.y == curr_loc.y))
			{
				if (grid_matrix[curr_loc.y][curr_loc.x + 1] == -1)
				{
					grid_matrix[curr_loc.y][curr_loc.x + 1] = check_path_for_debris();
					update_adjacency_matrix();
				}
				if (grid_matrix[curr_loc.y][curr_loc.x + 1] == 1)
				{
					move_robot();
					turn = 0;
				}
				else
				{
					break;
				}
			}
			else if ((next_loc.x == curr_loc.x - 2) && (next_loc.y == curr_loc.y))
			{
				if (grid_matrix[curr_loc.y][curr_loc.x - 1] == -1)
				{
					grid_matrix[curr_loc.y][curr_loc.x - 1] = check_path_for_debris();
					update_adjacency_matrix();
				}
				if (grid_matrix[curr_loc.y][curr_loc.x - 1] == 1)
				{
					move_robot();
					turn = 0;
				}
				else
				{

					break;
				}
			}

			else if ((next_loc.x == curr_loc.x) && (next_loc.y == curr_loc.y + 2))
			{
				if (grid_matrix[curr_loc.y + 1][curr_loc.x] == -1)
				{
					grid_matrix[curr_loc.y + 1][curr_loc.x] = check_path_for_debris();
					update_adjacency_matrix();
				}
				if (grid_matrix[curr_loc.y + 1][curr_loc.x] == 1)
				{
					move_robot();
					turn = 0;
				}
				else
				{
					break;
				}
			}
			else if ((next_loc.x == curr_loc.x) && (next_loc.y == curr_loc.y - 2))
			{
				if (grid_matrix[curr_loc.y - 1][curr_loc.x] == -1)
				{
					grid_matrix[curr_loc.y - 1][curr_loc.x] = check_path_for_debris();
					update_adjacency_matrix();
				}
				if (grid_matrix[curr_loc.y - 1][curr_loc.x] == 1)
				{
					move_robot();
					turn = 0;
				}
				else
				{
					break;
				}
			}
		}
	}
}

void initialize_node_coord_matrix(void)
{
    int j = 0;
    for (int i = 0; i < 25; i++)
    {
        node_coord_matrix[i].x = j * 2;
        if (i < 5)
            node_coord_matrix[i].y = 0;
        else if (i < 10)
            node_coord_matrix[i].y = 2;
        else if (i < 15)
            node_coord_matrix[i].y = 4;
        else if (i < 20)
            node_coord_matrix[i].y = 6;
        else
            node_coord_matrix[i].y = 8;
        if (j == 4)
            j = 0;
        else
            j++;
    }
}
void initialize_plot_coord_matrix(void)
{
    int x = 1;
    int y = -1;
    for (int i = 0; i < 16; i++)
    {
        if (i % 4 == 0)
        {
            x = 1;
            y += 2;
        }
        plot_coord_matrix[i][0].x = x - 1;
        plot_coord_matrix[i][0].y = y - 1;

        plot_coord_matrix[i][1].x = x + 1;
        plot_coord_matrix[i][1].y = y - 1;

        plot_coord_matrix[i][2].x = x + 1;
        plot_coord_matrix[i][2].y = y + 1;

        plot_coord_matrix[i][3].x = x - 1;
        plot_coord_matrix[i][3].y = y + 1;

        plot_coord_matrix[i][4].x = x;
        plot_coord_matrix[i][4].y = y;

        x += 2;
    }
}

void initialize_grid_matrix(void)
{
    for (int i = 0; i < 9; i++)
        for (int j = 0; j < 9; j++)
        {
            if (((i % 2) == 0) && ((j % 2) == 0))
                grid_matrix[i][j] = 9;
            else if (((i % 2) != 0) && ((j % 2) != 0))
                grid_matrix[i][j] = -5;
            else
                grid_matrix[i][j] = -1;
        }
}

void initialize_adjacency_matrix(void)
{
    for (int m = 0; m < 25; m++)
        for (int n = 0; n < 25; n++)
            adjacency_matrix[m][n] = 0;
}

void update_adjacency_matrix(void)
{
    tuple coord_a, coord_b;
    for (int m = 0; m < 25; m++)
    {
        coord_a = node_coord_matrix[m];
        for (int n = 0; n < 25; n++)
        {
            coord_b = node_coord_matrix[n];
            if ((coord_b.x == coord_a.x + 2) && (coord_b.y == coord_a.y))
                adjacency_matrix[m][n] = grid_matrix[coord_a.y][coord_a.x + 1];
            if ((coord_b.x == coord_a.x - 2) && (coord_b.y == coord_a.y))
                adjacency_matrix[m][n] = grid_matrix[coord_a.y][coord_a.x - 1];
            if ((coord_b.x == coord_a.x) && (coord_b.y == coord_a.y + 2))
                adjacency_matrix[m][n] = grid_matrix[coord_a.y + 1][coord_a.x];
            if ((coord_b.x == coord_a.x) && (coord_b.y == coord_a.y - 2))
                adjacency_matrix[m][n] = grid_matrix[coord_a.y - 1][coord_a.x];
        }
    }
}

tuple get_nearest_coordinate(tuple cloc, int plot)
{
    int x_dis, y_dis;
    float d, max = 100.0;
    tuple node;
    for (int i = 0; i < 4; i++)
    {
        x_dis = pow((cloc.x - plot_coord_matrix[plot][i].x), 2);
        y_dis = pow((cloc.y - plot_coord_matrix[plot][i].y), 2);
        d = sqrt(x_dis + y_dis);
        if (d < max)
        {
            node.x = plot_coord_matrix[plot][i].x;
            node.y = plot_coord_matrix[plot][i].y;
            max = d;
        }
    }
    return node;
}

int fetch_nearest_plot(int color_type)
{
	int x_dis, y_dis,plot_no = -1;
	float d, max = 100.0;
	for (int plot = 0; plot < 16; plot++)
	{
		if(grid_matrix[plot_coord_matrix[plot][4].y][plot_coord_matrix[plot][4].x] != color_type)
			continue;
		for (int i = 0; i < 4; i++)
		{
			x_dis = pow((curr_loc.x - plot_coord_matrix[plot][i].x), 2);
			y_dis = pow((curr_loc.y - plot_coord_matrix[plot][i].y), 2);
			d = sqrt(x_dis + y_dis);
			if (d < max)
			{
				max = d;
				plot_no = plot;
			}
		}
	}
	return (plot_no);
}
int get_node_from_coord(tuple coord)
{
    for (int i = 0; i < 25; i++)
        if ((node_coord_matrix[i].x == coord.x) && (node_coord_matrix[i].y == coord.y))
            return i;
    return -1;
}

int get_plot_from_coord(int x, int y)
{
    for (int i = 0; i < 16; i++)
        if ((x == plot_coord_matrix[i][4].x) && (y == plot_coord_matrix[i][4].y))
            return i;
    return -1;
}

void dijkstra(int G[25][25], int n, int startnode, int v)
{
    top = -1;
    int count, mindistance, nextnode, i, j;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            if (G[i][j] == 0) //Will make the 0 to infinte that means no direct edge between i and j
                cost[i][j] = INFINITE;
            else
                cost[i][j] = 1;
    for (i = 0; i < n; i++)
    {
        distance[i] = cost[startnode][i];
        pred[i] = startnode;
        visited[i] = 0;
    }
    distance[startnode] = 0;
    visited[startnode] = 1;
    count = 1;
    while (count < n - 1)
    {
        mindistance = INFINITE;
        for (i = 0; i < n; i++)
            if (distance[i] < mindistance && !visited[i])
            {
                mindistance = distance[i];
                nextnode = i;
            }
        visited[nextnode] = 1;
        for (i = 0; i < n; i++)
            if (!visited[i])
                if (mindistance + cost[nextnode][i] < distance[i])
                {
                    distance[i] = mindistance + cost[nextnode][i];
                    pred[i] = nextnode;
                }
        count++;
    }
    i = v;
    if (i != startnode)
    {
        // printf("\nDistance of node%d = %d", i, distance[i]);
        // printf("\nPath=%d", i);
        //Do while loop for path printing from start node to end node
        j = i;
        do
        {
            push(j);
            j = pred[j];
            // printf("<-%d", j);
        } while (j != startnode);
        //   printf("\n");
    }
}

void push(int x)
{
    if (top >= 24)
    {
        return;
    }
    else
    {
        top++;
        path_stack[top] = x;
    }
}
int pop()
{
    int y;
    if (top <= -1)
    {
        return -1;
    }
    else
    {
        y = path_stack[top];
        top--;
        return y;
    }
}

void print_stack_content(void)
{
    if (top >= 0)
    {
        printf("\n The elements in STACK \n");
        for (int i = top; i >= 0; i--)
            printf("%d\n", path_stack[i]);
    }
    else
    {
        printf("\n The STACK is empty");
    }
}

void print_plot_coord_matrix(void)
{
    for (int i = 0; i < 16; i++)
    {
       printf("\n%d =[",i+1);
        for (int j = 0; j < 5; j++)
            printf("(%d,%d),", plot_coord_matrix[i][j].x, plot_coord_matrix[i][j].y);
       printf("]\n");
    }
    printf("\n");
}

void print_ind_plot_coord_arr(int plot)
{
    printf("\nFor plot %d:", plot);
    for (int j = 0; j < 4; j++)
        printf(" (%d,%d) ", plot_coord_matrix[plot][j].x, plot_coord_matrix[plot][j].y);
    printf("\n");
}

void print_grid_matrix(void)
{
    printf("\n");
	for (int i = 0; i < 9; i++)
    {

		for (int j = 0; j < 9; j++)
        {
            printf(" {(%d,%d):%2d}  ", j,i,grid_matrix[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

void print_adjacency_matrix(void)
{
    printf("\t0     1     2     3     4     5     6     7     8     9    10    11    12     13    14    15    16    17    18    19    20    21    22    23    24\n");
    printf("--------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");

	for (int i = 0; i < 25; i++)
    {
        printf(" %2d| ", i);
        for (int j = 0; j < 25; j++)
        {
            printf("  %2d  ", adjacency_matrix[i][j]);
        }
        printf("\n--------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    }
    printf("\n");
}

void print_node_coord_matrix(void)
{
 //   printf("\n[");
    for (int i = 0; i < 25; i++)
    {

        printf(" i: %d (%d,%d) \n", i + 1, node_coord_matrix[i].x, node_coord_matrix[i].y);
       // printf(" (%d,%d),",node_coord_matrix[i].x, node_coord_matrix[i].y);
    }

    printf("\n");
}
