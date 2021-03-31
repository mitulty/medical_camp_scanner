/*
* path_finder.h
* Created: 23/03/21 01:15:58 PM
* Author: Bitboot Team
*/ 

#ifndef PATH_FINDER_H_
#define PATH_FINDER_H_

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

// void print_ind_plot_coord_arr(int);
// void print_plot_coord_matrix(void);
// void print_node_coord_matrix(void);
// void print_adjacency_matrix(void);
// void print_grid_matrix(void);
//void print_stack_content(void);
#endif
