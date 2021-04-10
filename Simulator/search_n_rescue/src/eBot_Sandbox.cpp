/*
 * eBot_Sandbox.cpp
 *  Created on: 21-Mar-2021
 *      Author: TAs of CS 684 Spring 2020
 */

//---------------------------------- INCLUDES -----------------------------------
#include "eBot_Sandbox.h"
#include <math.h>
#define INFINITE 9999

//------------------------------ GLOBAL VARIABLES -------------------------------

// To store 8-bit data of left, center and right white line sensors
int left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data;

// Some miscellaneous variables
unsigned char Threshold=200;
char dir_flag = 'n';
int prev = 0;
int done = 0;
int top=-1, plot_no = 0;

// Scene definition in matrix
int grid_matrix[9][9];
int plot_order[16] = {14, 13, 9, 10, 6, 5, 1, 2, 3, 4, 8, 7, 11, 12, 16, 15};
int adjacency_matrix[25][25];
int cost[25][25], distance[25], pred[25], path_stack[25];
int visited[25];

// To store the eBot's current and Goal location
typedef struct { int x, y; } tuple;
tuple curr_loc = {4,8}, goal_loc = {8,4}, dest_loc;
tuple plot_coord_matrix[16][5];
tuple node_coord_matrix[25];
tuple plot_coordinate;

//---------------------------------- FUNCTIONS ----------------------------------
// Sensor data capture
void get_wls_data(void);

//Motor or robot control
void forward_wls(unsigned char);
void left_turn_wls(void);
void right_turn_wls(void);
void u_turn_wls(void);
void left_turn_90(void);
void right_turn_90(void);

//Map traversal functions
void initialize_grid_matrix(void);
void initialize_node_coord_matrix(void);
void initialize_plot_coord_matrix(void);
void initialize_adjacency_matrix(void);

void scan_plot(int, tuple);
int check_path_for_debris(void);
void check_plot_scan_status();
void update_adjacency_matrix(void);
void dijkstra(int G[25][25], int n, int startnode, int v);
tuple get_nearest_coordinate(tuple,int);
int final_mid_point(int);
void turn_towards_mid_point(char);

void move_robot(void);

void push(int);
int pop(void);

int get_node_from_coord(tuple);
int get_plot_from_coord(tuple coord);

// implements the BFS Algorithm using the adjacency matrix
void turn_accordingly(tuple nloc);

//---------------------------------- FUNCTIONS ----------------------------------
void traverse(void)
{
	initialize_grid_matrix();
	initialize_plot_coord_matrix();
	initialize_adjacency_matrix();
	initialize_node_coord_matrix();
	forward_wls(1); 						// Reach the node (4,8);

	int status = 0;
	int plot_scan = -1;

	while (plot_no < 16)
		{
			plot_scan = plot_order[plot_no] - 1;
			tuple plot_coord = plot_coord_matrix[plot_scan][4];

			if (grid_matrix[plot_coord.y][plot_coord.x] == -5)
			{
				while (1)
				{
					if (status == 0)
						dest_loc = get_nearest_coordinate(curr_loc, plot_scan);

					scan_plot(plot_scan, dest_loc);
					if (grid_matrix[plot_coord.y][plot_coord.x] != -5)	{plot_no++; status = 0; break;}

					status = final_mid_point(plot_scan);
					if (status == 1){	plot_no++;	status = 0;		break; }
					else
					{
						status = -1;
						plot_coordinate.x = plot_coord_matrix[plot_scan][4].x;
						plot_coordinate.y = plot_coord_matrix[plot_scan][4].y;
						if (curr_loc.x - plot_coordinate.x == 1)
								dest_loc.x = curr_loc.x - 2;
						else	dest_loc.x = curr_loc.x + 2;

						if (curr_loc.y - plot_coordinate.y == 1)
								dest_loc.y = curr_loc.y - 2;
						else	dest_loc.y = curr_loc.y + 2;
					}
				}
			}
			else {plot_no++; status = 0;}
		}

	//---------------------------------- TEST PATH ----------------------------------
	/*forward_wls(5);
	right_turn_90();
	if (check_path_for_debris()==0)
		{ left_turn_wls();
		forward_wls(2);
		right_turn_wls();
		forward_wls(2);
		right_turn_wls();
		forward_wls(2);
		left_turn_wls();
		forward_wls(3);
		}
	else forward_wls(5);*/
	stop();

	while(1);
}

void forward_wls(unsigned char node)
{
		forward();
		while (1)
				{
					_delay_ms(5);
					get_wls_data();

					if 		((center_wl_sensor_data > Threshold) && ((left_wl_sensor_data > Threshold) && (right_wl_sensor_data > Threshold)))
							{		prev=0;		break;		}
					else if (center_wl_sensor_data > Threshold)		velocity(80, 80);
					else if (left_wl_sensor_data > Threshold)		{prev=1; velocity(40, 80);}
					else if (right_wl_sensor_data > Threshold)		{prev=2; velocity(80, 40);}
					else
						if(prev==1) 		velocity(40,80);
						else if(prev==2) 	velocity(80,40);
						else 				forward();
				}forward(); _delay_ms(50);
}

void left_turn_wls(void)
{
	left(); 	_delay_ms(100);
	while (1)
	{
		get_wls_data();
		if(center_wl_sensor_data <= Threshold)
		{
			velocity(20, 60);	_delay_ms(5);
		}
		else break;
	}
}

void right_turn_wls(void)
{
	right(); 	_delay_ms(100);
	while (1)
	{
		get_wls_data();
		if(center_wl_sensor_data < Threshold)
		{
			velocity(60, 20);	_delay_ms(5);
		}
		else	break;
	}
}

void u_turn_wls(void)
{
	right(); 	_delay_ms(220);
	while (1)
	{
		get_wls_data();
		if(center_wl_sensor_data < Threshold)
		{
			velocity(20, 60); _delay_ms(5);
		}
		else	break;
	}
}

void left_turn_90(void)
{
	left(); 	_delay_ms(110);
	stop();
}

void right_turn_90(void)
{
	right();	_delay_ms(110);
	stop();
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
void initialize_adjacency_matrix(void)
{
    for (int m = 0; m < 25; m++)
        for (int n = 0; n < 25; n++)
            adjacency_matrix[m][n] = 0;
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

int get_plot_from_coord(int x, int y)
{
    for (int i = 0; i < 16; i++)
        if ((x == plot_coord_matrix[i][5].x) && (y == plot_coord_matrix[i][5].y))
            return i;
    return -1;
}

void scan_plot(int plot, tuple destination_location)
{
	tuple next_loc;
	int d_node, s_node, node;
	d_node = get_node_from_coord(destination_location);

	while (!((destination_location.x == curr_loc.x) && (destination_location.y == curr_loc.y)))
	{
		s_node = get_node_from_coord(curr_loc);
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
				if (grid_matrix[curr_loc.y][curr_loc.x + 1] == 1)		move_robot(next_loc);
				else	break;
			}
			else if ((next_loc.x == curr_loc.x - 2) && (next_loc.y == curr_loc.y))
			{
				if (grid_matrix[curr_loc.y][curr_loc.x - 1] == -1)
				{
					grid_matrix[curr_loc.y][curr_loc.x - 1] = check_path_for_debris();
					update_adjacency_matrix();
				}
				if (grid_matrix[curr_loc.y][curr_loc.x - 1] == 1)		move_robot(next_loc);
				else	break;
			}

			else if ((next_loc.x == curr_loc.x) && (next_loc.y == curr_loc.y + 2))
			{
				if (grid_matrix[curr_loc.y + 1][curr_loc.x] == -1)
				{
					grid_matrix[curr_loc.y + 1][curr_loc.x] = check_path_for_debris();
					update_adjacency_matrix();
				}
				if (grid_matrix[curr_loc.y + 1][curr_loc.x] == 1)		move_robot(next_loc);
				else	break;
			}
			else if ((next_loc.x == curr_loc.x) && (next_loc.y == curr_loc.y - 2))
			{
				if (grid_matrix[curr_loc.y - 1][curr_loc.x] == -1)
				{
					grid_matrix[curr_loc.y - 1][curr_loc.x] = check_path_for_debris();
					update_adjacency_matrix();
				}
				if (grid_matrix[curr_loc.y - 1][curr_loc.x] == 1)		move_robot(next_loc);
				else	break;
			}
		}
	}
}

int final_mid_point(int plot_no)
{
	tuple mid_point_1, mid_point_2;
	char dir_1, dir_2;
	tuple coord_plot = plot_coord_matrix[plot_no][4];

	mid_point_1.x = coord_plot.x;
	mid_point_1.y = curr_loc.y;
	if ((curr_loc.x == mid_point_1.x) && ((mid_point_1.y - curr_loc.y) == -1))				dir_1 = 'n';
	else if ((curr_loc.x == mid_point_1.x) && ((mid_point_1.y - curr_loc.y) == 1))			dir_1 = 's';
	else if ((curr_loc.y == mid_point_1.y) && ((mid_point_1.x - curr_loc.x) == -1))			dir_1 = 'w';
	else if ((curr_loc.y == mid_point_1.y) && ((mid_point_1.x - curr_loc.x) == 1))			dir_1 = 'e';

	mid_point_2.x = curr_loc.x;
	mid_point_2.y = coord_plot.y;
	if ((curr_loc.x == mid_point_2.x) && ((mid_point_2.y - curr_loc.y) == -1))				dir_2 = 'n';
	else if ((curr_loc.x == mid_point_2.x) && ((mid_point_2.y - curr_loc.y) == 1))			dir_2 = 's';
	else if ((curr_loc.y == mid_point_2.y) && ((mid_point_2.x - curr_loc.x) == -1))			dir_2 = 'w';
	else if ((curr_loc.y == mid_point_2.y) && ((mid_point_2.x - curr_loc.x) == 1))			dir_2 = 'e';

	if(dir_flag == dir_1)
	{
		if (grid_matrix[mid_point_1.y][mid_point_1.x] == -1)
			grid_matrix[mid_point_1.y][mid_point_1.x] = check_path_for_debris();

		if (grid_matrix[mid_point_1.y][mid_point_1.x] == 1)
		{	move_robot();	return 1;		}
	}
	else if(dir_flag == dir_2)
	{
		if (grid_matrix[mid_point_2.y][mid_point_2.x] == -1)
			grid_matrix[mid_point_2.y][mid_point_2.x] = check_path_for_debris();
		if (grid_matrix[mid_point_2.y][mid_point_2.x] == 1)
		{	move_robot();		return 1;		}
	}

	if (dir_flag != dir_1)
	{
		turn_towards_mid_point(dir_1);
		if (grid_matrix[mid_point_1.y][mid_point_1.x] == -1)
			grid_matrix[mid_point_1.y][mid_point_1.x] = check_path_for_debris();

		if (grid_matrix[mid_point_1.y][mid_point_1.x] == 1)
		{	move_robot();		return 1;		}
	}

	if (dir_flag != dir_2)
	{
		turn_towards_mid_point(dir_2);
		if (grid_matrix[mid_point_2.y][mid_point_2.x] == -1)
			grid_matrix[mid_point_2.y][mid_point_2.x] = check_path_for_debris();
		if (grid_matrix[mid_point_2.y][mid_point_2.x] == 1)
		{	move_robot();		return 1;		}
	}
	return 0;
}

void turn_towards_mid_point(char dir)
{
	if (dir_flag != dir)
	{
		if (dir_flag == 'n')
		{
			if 		(dir == 'w')	left_turn_90();
			else if (dir == 'e')	right_turn_90();
			else					u_turn_wls();
		}
		else if (dir_flag == 'e')
		{
			if 		(dir == 'n')	left_turn_90();
			else if (dir == 's')	right_turn_90();
			else					u_turn_wls();
		}
		else if (dir_flag == 'w')
		{
			if 		(dir == 's')	left_turn_90();
			else if (dir == 'n')	right_turn_90();
			else					u_turn_wls();
		}
		else if (dir_flag == 's')
		{
			if (dir == 'e')			left_turn_90();
			else if (dir == 'w')	right_turn_90();
			else					u_turn_wls();
		}
	}
}

int check_path_for_debris(void)
{
	int c = 0;
	for (int i = 0; i < 30; i++)
		{
			get_wls_data();
			if (left_wl_sensor_data > Threshold || center_wl_sensor_data > Threshold || right_wl_sensor_data > Threshold) 			c = 1;
		}

	if (c!=1)
	{
			left();		velocity(10, 50);

			for (int i = 0; i < 30; i++)
			{
				get_wls_data();
				if (left_wl_sensor_data > Threshold || center_wl_sensor_data > Threshold || right_wl_sensor_data > Threshold) 		c = 1;
			}

			stop();		_delay_ms(10);
			right();	velocity(50, 10);

			for (int i = 0; i < 60; i++)
			{
				get_wls_data();
				if (left_wl_sensor_data > Threshold || center_wl_sensor_data > Threshold || right_wl_sensor_data > Threshold)		c = 1;
			}

			stop();		_delay_ms(10);
			left();		velocity(10, 50);

			for (int i = 0; i < 30; i++)
			{
				get_wls_data();
				if (left_wl_sensor_data > Threshold || center_wl_sensor_data > Threshold || right_wl_sensor_data > Threshold)		c = 1;
			}
	}
	stop();		velocity(0,0);		_delay_ms(10);
	return c;
}

void check_plot_scan_status()
{
	forward();	velocity(150,150);	_delay_ms(100);
	stop();		velocity(0,0);		_delay_ms(100);
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
	//allign_at_node();
}

void push(int x)
{
    if (top >= 24)        return;
    else    {	top++;   path_stack[top] = x;    }
}
int pop()
{
    int y;
    if (top <= -1)        return -1;
    else   {   y = path_stack[top];    top--;      return y;   }
}

void turn_accordingly(tuple n_loc)
{
	int x_dis = n_loc.x - curr_loc.x;
	int y_dis = n_loc.y - curr_loc.y;
	if (x_dis < 0)
	{
		if (dir_flag != 'w')
		{
			if (dir_flag == 'n')		left_turn_90();
			else if (dir_flag == 'e') 	u_turn_wls();
			else						right_turn_90();
		}
	}
	else if (x_dis > 0)
	{
		if (dir_flag != 'e')
		{
			if (dir_flag == 's')		left_turn_90();
			else if (dir_flag == 'w') 	u_turn_wls();
			else						right_turn_90();
		}
	}
	else if (y_dis < 0)
	{
		if (dir_flag != 'n')
		{
			if (dir_flag == 'e')		left_turn_90();
			else if (dir_flag == 's') 	u_turn_wls();
			else						right_turn_90();
		}
	}
	else if (y_dis > 0)
	{
		if (dir_flag != 's')
		{
			if (dir_flag == 'w')		left_turn_90();
			else if (dir_flag == 'n') 	u_turn_wls();
			else						right_turn_90();
		}
	}
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

void dijkstra(int G[25][25], int n, int startnode, int v)
{
    top = -1;
    int count, mindistance, nextnode, i, j;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            if (G[i][j] == 0)	cost[i][j] = INFINITE;
            else		    	cost[i][j] = 1;
    //Will make the 0 to infinte that means no direct edge between i and j

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
        j = i;
        do {  push(j);       j = pred[j];
        } while (j != startnode);
    }
}

void move_robot()
{
	forward_wls(1);
	check_plot_scan_status();
	//_delay_ms(100);
	forward_wls(1);
}

void get_wls_data(void)
{
	left_wl_sensor_data = convert_analog_channel_data(left_wl_sensor_channel)  ;
	center_wl_sensor_data = convert_analog_channel_data(center_wl_sensor_channel)  ;
	right_wl_sensor_data = convert_analog_channel_data(right_wl_sensor_channel);
	//printf("\n%d %d %d\n", left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data);
}
