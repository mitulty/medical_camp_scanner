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
int grid_matrix[9][9];
tuple plot_coord_matrix[16][4];

void initialize_grid_matrix(void);
void print_grid_matrix(void);
void update_grid_matrix(int,int,char,int);
int check_plots(int);
int path_to_scan_x(int,int,int,char);
int get_adjacent_node_x(int);
int get_adjacent_node_y(int);
int check_matrix_for_path(int,int,char);
tuple get_nearest_coordinate(tuple,int);
void print_ind_plot_coord_arr(int);
void print_plot_coord_matrix(void);
void initialize_plot_coord_matrix(void);
#endif
