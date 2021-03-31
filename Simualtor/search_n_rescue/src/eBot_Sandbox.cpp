/*
 * eBot_Sandbox.cpp
 *
 *  Created on: 21-Mar-2021
 *      Author: TAs of CS 684 Spring 2020
 */

//---------------------------------- INCLUDES -----------------------------------

#include "eBot_Sandbox.h"
#include <math.h>
#define INFINITE 9999
#define THRESHOLD_WLS 200
//------------------------------ GLOBAL VARIABLES -------------------------------

// To store 8-bit data of left, center and right white line sensors
unsigned char left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data;

volatile unsigned long int red;       // variable to store the pulse count when read_red function is called
volatile unsigned long int blue;      // variable to store the pulse count when read_blue function is called
volatile unsigned long int green;     // variable to store the pulse count when read_green function is called

int r=-1,f=-1;
int prev;
char dir_flag = 'n';

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
//----------------------------------------Functions-----------------------------------------------------------------
// scans the plot
void scan_plot(int);

//moves robot to the next loc
void move_robot(tuple);

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
int get_plot_from_coord(tuple coord);

// updates the adjacency matrix from the grid matrix
void update_adjacency_matrix(void);

// implements the BFS Algorithm using the adjacency matrix
void dijkstra(int G[25][25], int n, int startnode, int v);

// checks path for debris
int check_path_for_debris(void);

void turn_accordingly(tuple nloc);


void print_ind_plot_coord_arr(int);
void print_plot_coord_matrix(void);
void print_node_coord_matrix(void);
void print_adjacency_matrix(void);
void print_grid_matrix(void);
void print_stack_content(void);
void forward_wls(unsigned char node);
void left_turn_wls(void);
void right_turn_wls(void);
void check_plot_scan_status();
void allign_at_node(void);
void left_turn_90(void);
void right_turn_90(void);
void get_wls_data(void);

int plot_order[16] = {13, 14, 9, 10, 1, 2, 4, 3, 8, 7, 12, 11, 16, 15}, plot_no = 0;
int done = 0;

void get_wls_data(void)
{
	left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
	center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
	right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
	printf("\n%d %d %d\n", left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data);
}
void traverse(void)
{

	//-----------------------------------Test Zone---------------------------------------------------
		forward_wls(1);
		left_turn_wls();
		get_wls_data();
		while(1);
	//-----------------------------Setup Functions----------------------------------------------------	
	initialize_grid_matrix();
	initialize_plot_coord_matrix();
	initialize_adjacency_matrix();
	initialize_node_coord_matrix();
	// print_plot_coord_matrix();
	// print_node_coord_matrix();
	// print_adjacency_matrix();
	// print_grid_matrix();
	// print_stack_content();

	forward_wls(1); // Reach the node (4,8);
	goal_loc.x = 8;
	goal_loc.y = 4;
	curr_loc.x = 4;
	curr_loc.y = 8;
	top = -1;
	//------------------------------------------------------------------------------------------------
	while (plot_no < 16)
	{
		printf("\n Scanning Plot: %d\n",plot_order[plot_no]);
		scan_plot(plot_order[plot_no] - 1);
		printf("\n Scanned Plot: %d\n",plot_order[plot_no]);
		plot_no++;
		
	}
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
	else if (y_dis < 0)
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
	for (int i = 0; i < 30; i++)
	{
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
	//	printf("\nScan 1--> %d 	%d 	%d\n", left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data);
		if (left_wl_sensor_data > THRESHOLD_WLS || center_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS)
			c = 1;
		
	}
	stop();
	_delay_ms(10);
	right();
	velocity(50, 50);
	for (int i = 0; i < 60; i++)
	{
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
	//	printf("\nScan 2--> %d 	%d 	%d\n", left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data);
		if (left_wl_sensor_data > THRESHOLD_WLS || center_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS)
			c = 1;
		
	}
	stop();
	_delay_ms(10);
	left();
	velocity(50, 50);
	for (int i = 0; i < 30; i++)
	{
		left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
	//	printf("\nScan 3--> %d 	%d 	%d\n", left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data);
		if (left_wl_sensor_data > THRESHOLD_WLS || center_wl_sensor_data > THRESHOLD_WLS || right_wl_sensor_data > THRESHOLD_WLS)
			c = 1;
		
	}
	back();
	_delay_ms(100);
	stop();
	velocity(0,0);
	_delay_ms(100);
	return c;
}
void check_plot_scan_status()
{
	forward();
	velocity(150,150);
	_delay_ms(100);
	stop();
	velocity(0,0);
	_delay_ms(100);
	if (curr_loc.x == 0)
	{
		if (grid_matrix[curr_loc.y][curr_loc.x +1] == -5)
		{
			if (dir_flag == 's')
			{

				left_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x +1] = print_color_sensor_data();
				right_turn_wls();
				_delay_ms(100);

			}
			else
			{
				right_turn_90();
				_delay_ms(100);
				grid_matrix[curr_loc.y][curr_loc.x +1] = print_color_sensor_data();
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
			if (dir_flag == 'w')
			{
				left_turn_90();
				_delay_ms(10);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = print_color_sensor_data();
				right_turn_wls();
				_delay_ms(100);

			}
			else
			{
				right_turn_90();
				_delay_ms(10);
				grid_matrix[curr_loc.y - 1][curr_loc.x] = print_color_sensor_data();
				left_turn_wls();
				_delay_ms(100);

			}
		}
		if (grid_matrix[curr_loc.y + 1][curr_loc.x] == -5)
		{
			if (dir_flag == 'w')
			{
				right_turn_90();
				_delay_ms(10);
				grid_matrix[curr_loc.y + 1][curr_loc.x] = print_color_sensor_data();
				left_turn_wls();
				_delay_ms(100);

			}
			else
			{
				left_turn_90();
				_delay_ms(10);
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
				_delay_ms(10);
				grid_matrix[curr_loc.y][curr_loc.x - 1] = print_color_sensor_data();
				right_turn_wls();
				_delay_ms(100);

			}
			else
			{
				right_turn_90();
				_delay_ms(10);
				grid_matrix[curr_loc.y][curr_loc.x - 1] = print_color_sensor_data();
				left_turn_wls();
			}
		}
		if (grid_matrix[curr_loc.y][curr_loc.x +1] == -5)
		{
			if (dir_flag == 'n')
			{
				right_turn_90();
				_delay_ms(10);
				grid_matrix[curr_loc.y][curr_loc.x +1] = print_color_sensor_data();
				left_turn_wls();
			}
			else
			{
				left_turn_90();
				_delay_ms(10);
				grid_matrix[curr_loc.y][curr_loc.x +1] = print_color_sensor_data();
				right_turn_wls();
			}
		}
		return;
	}
	allign_at_node();
}

void forward_wls(unsigned char node)
{
	unsigned char node_reached = 0;
	for (int i = 0; i < node; i++)
	{
		allign_at_node();
		forward();
		velocity(80,80);
		_delay_ms(300);
		while (1)
		{
			// get the ADC converted data of three white line sensors from their appropriate channel numbers
			left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel);
			center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
			right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
			if (center_wl_sensor_data > THRESHOLD_WLS && (left_wl_sensor_data > THRESHOLD_WLS && right_wl_sensor_data > THRESHOLD_WLS))
			{
				stop();
				velocity(0, 0);
				++node_reached;
				//printf("\nStop--> %d 	%d 	%d\n", left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data);
				//printf("node reached: %d\n", node_reached);
				break;
			}
			else if (center_wl_sensor_data > THRESHOLD_WLS)
			{
				velocity(80, 80);
				prev = 0;
				//printf("\n forward\n");
			}
			else if (left_wl_sensor_data > THRESHOLD_WLS)
			{
				velocity(50, 200);
				prev = 1;
				//printf("\n left\n");
			}
			else if (right_wl_sensor_data > THRESHOLD_WLS)
			{
				velocity(200, 50);
				prev = 2;
				//printf("\n right\n");
			}
			else
			{
				if (prev == 1)
					velocity(50, 200);
				else if (prev == 2)
					velocity(200, 50);
				
				// break;
			}
			_delay_ms(10);
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
//	allign_at_node();
	forward();
	velocity(80, 80);
	_delay_ms(350);
	stop();
	left();
	velocity(80, 80);
	_delay_ms(300);
	get_wls_data();
	while (1)
	{
		// get the ADC converted data of center white line sensors from their appropriate channel number
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		if (center_wl_sensor_data > THRESHOLD_WLS)
		{
			stop();
			velocity(0,0);
			get_wls_data();
			break;
		}
		else
		{
			left();
			velocity(80, 80);
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
	velocity(80, 80);
	_delay_ms(350);
	stop();
	right();
	velocity(80, 80);
	_delay_ms(200);
	while (1)
	{
		// get the ADC converted data of center white line sensors from their appropriate channel number
		center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		left_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel);
		if (center_wl_sensor_data > THRESHOLD_WLS || left_wl_sensor_data > THRESHOLD_WLS)
		{
			break;
		}
		else
		{
			right();
			velocity(80, 80);
		}
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

void left_turn_90(void)
{
	left();
	velocity(80, 80);
	_delay_ms(300);
	stop();
	velocity(0,0);
}

void right_turn_90(void)
{
	right();
	velocity(80, 80);
	_delay_ms(300);
	stop();
	velocity(0,0);
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
		}
		stop();
		_delay_ms(1000);
		limit += 50;
		if (found == 1)
			break;
	}
}
//------------------------------------------------------------Algorithm-----------------------------------------------------------
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

int get_node_from_coord(tuple coord)
{
    for (int i = 0; i < 25; i++)
        if ((node_coord_matrix[i].x == coord.x) && (node_coord_matrix[i].y == coord.y))
            return i;
    return -1;
}

int get_plot_from_coord(tuple coord)
{
    for (int i = 0 ; i < 25 ; i++)
            if((coord.x == plot_coord_matrix[i][5].x) && (coord.y == plot_coord_matrix[i][5].y))
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

void scan_plot(int plot)
{
    tuple dest_loc, next_loc;
    dest_loc = get_nearest_coordinate(curr_loc, plot);
	print_stack_content();
	printf("\n Nearest Coordinate: (%d,%d)\n",dest_loc.x,dest_loc.y);
	printf("\n Current Coordinate: (%d,%d)\n",curr_loc.x,curr_loc.y);
    int d_node, s_node, node;
    while (!((dest_loc.x == curr_loc.x) && (dest_loc.y == curr_loc.y)))
    {
        dest_loc = get_nearest_coordinate(curr_loc, plot);
        d_node = get_node_from_coord(dest_loc);
        s_node = get_node_from_coord(curr_loc);
        dijkstra(adjacency_matrix, 25, s_node, d_node);
		print_stack_content();
        while (top > -1)
        {
            node = pop();
            next_loc = node_coord_matrix[node];
			printf("\n Next Coordinate: (%d,%d)\n",next_loc.x,next_loc.y);
			turn_accordingly(next_loc);
			_delay_ms(100);
            if ((next_loc.x == curr_loc.x + 2) && (next_loc.y == curr_loc.y))
            {
                if (grid_matrix[curr_loc.y][curr_loc.x + 1] == -1)
                {
                    printf("\n Checking for debris 1 at (%d,%d)\n",curr_loc.x+1,curr_loc.y);
					grid_matrix[curr_loc.y][curr_loc.x + 1] = check_path_for_debris();
                    update_adjacency_matrix();
                }
                if (grid_matrix[curr_loc.y][curr_loc.x + 1] == 1)
                {
                    printf("\n Moving the robot 1\n");
					move_robot(next_loc);
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
                    printf("\n Checking for debris 2 at (%d,%d)\n",curr_loc.x-1,curr_loc.y);
					grid_matrix[curr_loc.y][curr_loc.x - 1] = check_path_for_debris();
                    update_adjacency_matrix();
                }
                if (grid_matrix[curr_loc.y][curr_loc.x - 1] == 1)
                {
                    printf("\n Moving the robot 2\n");
				    move_robot(next_loc);
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
                    printf("\n Checking for debris 3 at (%d,%d)\n",curr_loc.x,curr_loc.y+1);
					grid_matrix[curr_loc.y + 1][curr_loc.x] = check_path_for_debris();
                    update_adjacency_matrix();
                }
                if (grid_matrix[curr_loc.y + 1][curr_loc.x] == 1)
                {
                    printf("\n Moving the robot 3\n");
					move_robot(next_loc);
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
                    printf("\n Checking for debris 4 at (%d,%d)\n",curr_loc.x,curr_loc.y-1);
					grid_matrix[curr_loc.y - 1][curr_loc.x] = check_path_for_debris();
                    update_adjacency_matrix();
                }
                if (grid_matrix[curr_loc.y - 1][curr_loc.x] == 1)
                {
                    printf("\n Moving the robot 4\n");
					move_robot(next_loc);
                }
                else
                {
                    break;
                }
            }
        }
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
        printf("\n%d= ", (i + 1));
        for (int j = 0; j < 5; j++)
            printf("(%d,%d) ", plot_coord_matrix[i][j].x, plot_coord_matrix[i][j].y);
    }
    printf("\n");
}

void print_ind_plot_coord_arr(int plot)
{
    printf("\nFor plot %d:", plot + 1);
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
    for (int i = 0; i < 25; i++)
    {

        printf(" i: %d (%d,%d) \n", i + 1, node_coord_matrix[i].x, node_coord_matrix[i].y);
    }
    printf("\n");
}
//--------------------------------------------------------------------------------------------------------------------------------
