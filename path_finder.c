/*! \mainpage Experiment:Path Finder
 *
 * @author     Bitboot Team
 * @date       2021/03/21
 *
 * \subsection Aim
 * Algorithm to take into account the path and generate a grid matrix
 */
#include "path_finder.h"
#include <stdio.h>
#include <math.h>

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

        x += 2;
    }
}

void print_plot_coord_matrix(void)
{
    for (int i = 0; i < 16; i++)
    {
        printf("\n%d= ", (i + 1));
        for (int j = 0; j < 4; j++)
            printf("(%d,%d) ", plot_coord_matrix[i][j].x, plot_coord_matrix[i][j].y);
    }
    printf("\n");
}

void print_ind_plot_coord_arr(int plot)
{
    printf("\nFor plot %d:",plot+1);
    for (int j = 0; j < 4; j++)
            printf(" (%d,%d) ", plot_coord_matrix[plot][j].x, plot_coord_matrix[plot][j].y);
    printf("\n");
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

void print_grid_matrix(void)
{
    for (int i = 0; i < 9; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            printf("  %2d  ", grid_matrix[i][j]);
        }
        printf("\n");
    }
}

tuple get_nearest_coordinate(tuple curr_loc,int plot)
{
        int x_dis,y_dis;
        float d,max=100.0;
        tuple node;
        print_ind_plot_coord_arr(plot);
        for(int i =0;i<4;i++)
            {
                x_dis=pow((curr_loc.x-plot_coord_matrix[plot][i].x),2);
                y_dis=pow((curr_loc.y-plot_coord_matrix[plot][i].y),2);
                d=sqrt(x_dis+y_dis);
                //printf(" Val Obtained for (%d,%d): %f\n",plot_coord_matrix[plot][i].x,plot_coord_matrix[plot][i].y,d);
                if(d<max)
                    {
                        node.x = plot_coord_matrix[plot][i].x;
                        node.y = plot_coord_matrix[plot][i].y;
                        max = d;
                    }

            }
        return node;    
}


int main(int argc, char *argv[])
{
    initialize_grid_matrix();
    print_grid_matrix();
    initialize_plot_coord_matrix();
    print_plot_coord_matrix();
    tuple curr_loc={4,8}, dest_loc= get_nearest_coordinate(curr_loc,12);
    printf("\nReturned Coodinate: %d %d for 13\n",dest_loc.x,dest_loc.y);    
    dest_loc=get_nearest_coordinate(dest_loc,13);
    printf("\nReturned Coodinate: %d %d for 14\n",dest_loc.x,dest_loc.y);    
    return 1;
}


