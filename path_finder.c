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
#define INFINITE 9999
int r = -1, f = -1;
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

int get_plot_from_coord(int x, int y)
{
    for (int i = 0; i < 16; i++)
        if ((x == plot_coord_matrix[i][5].x) && (y == plot_coord_matrix[i][5].y))
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


/*
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
 //   printf("\n[");
    for (int i = 0; i < 25; i++)
    {

        printf(" i: %d (%d,%d) \n", i + 1, node_coord_matrix[i].x, node_coord_matrix[i].y);
       // printf(" (%d,%d),",node_coord_matrix[i].x, node_coord_matrix[i].y);
    }

    printf("\n");
}

int main(int argc, char *argv[])
{
    initialize_grid_matrix();
    initialize_plot_coord_matrix();
    initialize_adjacency_matrix();
    initialize_node_coord_matrix();
    update_adjacency_matrix();
    print_plot_coord_matrix();
    print_grid_matrix();
    print_adjacency_matrix();
    print_node_coord_matrix();
    print_stack_content();
    printf("\n-------------------------------------------------------------------------------------------------------------------------------\n");
    dijkstra(adjacency_matrix, 25, 0, 23);
    print_stack_content();
    grid_matrix[0][1]= 0;
    update_adjacency_matrix();
    print_grid_matrix();
    print_adjacency_matrix();
    dijkstra(adjacency_matrix, 25, 0, 23);
    print_stack_content();
    printf("\n-------------------------------------------------------------------------------------------------------------------------------\n");
    return 1;
}
*/