import sys
import networkx as nx
import numpy as np
import paho.mqtt.client as mqtt
import math
import time
curr_loc=(4,8)
goal_loc=(8,4)
plot_order= [13, 14, 9, 10, 1, 2, 4, 3, 8, 7, 12, 11, 16, 15]
plot_no = 0
path_array=[]
value_returned = -1
coord_returned=()
plot_coord_matrix = [[(0, 0), (2, 0), (2, 2), (0, 2), (1, 1)],
                     [(2, 0), (4, 0), (4, 2), (2, 2), (3, 1)],
                     [(4, 0), (6, 0), (6, 2), (4, 2), (5, 1)],
                     [(6, 0), (8, 0), (8, 2), (6, 2), (7, 1)],
                     [(0, 2), (2, 2), (2, 4), (0, 4), (1, 3)],
                     [(2, 2), (4, 2), (4, 4), (2, 4), (3, 3)],
                     [(4, 2), (6, 2), (6, 4), (4, 4), (5, 3)],
                     [(6, 2), (8, 2), (8, 4), (6, 4), (7, 3)],
                     [(0, 4), (2, 4), (2, 6), (0, 6), (1, 5)],
                     [(2, 4), (4, 4), (4, 6), (2, 6), (3, 5)],
                     [(4, 4), (6, 4), (6, 6), (4, 6), (5, 5)],
                     [(6, 4), (8, 4), (8, 6), (6, 6), (7, 5)],
                     [(0, 6), (2, 6), (2, 8), (0, 8), (1, 7)],
                     [(2, 6), (4, 6), (4, 8), (2, 8), (3, 7)],
                     [(4, 6), (6, 6), (6, 8), (4, 8), (5, 7)],
                     [(6, 6), (8, 6), (8, 8), (6, 8), (7, 7)]]

node_coord_matrix = [(0, 0), (2, 0), (4, 0), (6, 0), (8, 0), (0, 2), (2, 2), (4, 2), (6, 2), (8, 2), (0, 4), (
    2, 4), (4, 4), (6, 4), (8, 4), (0, 6), (2, 6), (4, 6), (6, 6), (8, 6), (0, 8), (2, 8), (4, 8), (6, 8), (8, 8)]

adjacency_matrix = np.array([
    [0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0]])

grid_matrix = [[9, -1, 9, -1, 9, -1, 9, -1, 9],
               [-1, -5, -1, -5, -1, -5, -1, -5, -1],
               [9, -1, 9, -1, 9, -1, 9, -1, 9],
               [-1, -5, -1, -5, -1, -5, -1, -5, -1],
               [9, -1, 9, -1, 9, -1, 9, -1, 9],
               [-1, -5, -1, -5, -1, -5, -1, 5, -1],
               [9, -1, 9, -1, 9, -1, 9, -1, 9],
               [-1, -5, -1, -5, -1, -5, -1, -5, -1],
               [9, -1, 9, -1, 9, -1, 9, -1, 9]]


def update_adjacency_matrix():
    for m in range(25):
        coord_a= node_coord_matrix[m]
        for n in range(25):
            coord_b= node_coord_matrix[n]
            if ((coord_b[0] == coord_a[0] + 2) and (coord_b[1] == coord_a[1])):
                adjacency_matrix[m][n] = abs(grid_matrix[coord_a[1]][coord_a[0] + 1])
            elif ((coord_b[0] == coord_a[0] - 2) and (coord_b[1] == coord_a[1])):
                adjacency_matrix[m][n] = abs(grid_matrix[coord_a[1]][coord_a[0] - 1])
            elif ((coord_b[0] == coord_a[0]) and (coord_b[1] == coord_a[1] + 2)):
                adjacency_matrix[m][n] = abs(grid_matrix[coord_a[1] + 1][coord_a[0]])
            elif ((coord_b[0] == coord_a[0]) and (coord_b[1] == coord_a[1] - 2)):
                adjacency_matrix[m][n] = abs(grid_matrix[coord_a[1] - 1][coord_a[0]])

def print_grid_matrix():
    for i in range(9):
        for j in range(9):
            print(grid_matrix[i][j],end =" ")
        print("\n")   

def print_adjacency_matrix():
    for i in range(25):
        print(i,":",end="")
        for j in range(25):
            print(adjacency_matrix[i][j],end =" ")
        print("\n")

def get_nearest_coordinate(current_coord,plot):
    d = 0
    node=(-1,-1)
    max =100.0
    for i in range(4): 
        x_dis = ((current_coord[0]-plot_coord_matrix[plot][i][0])**2)
        y_dis = ((current_coord[1]-plot_coord_matrix[plot][i][1])**2)
        d = math.sqrt(x_dis+y_dis)
        if(d<max):
            node=(plot_coord_matrix[plot][i][0],plot_coord_matrix[plot][i][1])
            max = d
    return node       

def get_node_from_coordinate(a):
    for  i in range(25):
        if(node_coord_matrix[i]==a):
            return i
    return -1

def get_plot_from_coordinate(a):
    for  i in range(16):
        if(plot_coord_matrix[i][4]==a):
            return i
    return -1

def on_message(client, userdata, message):
	temp = str(message.payload.decode("utf-8"))
    value_returned = int(temp[0])
	
def on_connect(client,userdata,flags,rc):
	print("connected with status"+str(rc))
	client.subscribe("data_sent")    
    
#G = nx.from_numpy_matrix(adjacency_matrix, create_using=nx.Graph())
# print(nx.dijkstra_path(G, 0, 23))
# print_grid_matrix()
# print_adjacency_matrix()
# grid_matrix[0][1] = 0
# print("-----------------------------------------------------------\n")
# print_grid_matrix()
# update_adjacency_matrix()
# print_adjacency_matrix()
# G = nx.from_numpy_matrix(adjacency_matrix, create_using=nx.Graph())
# print(nx.dijkstra_path(G, 0, 23))
client = mqtt.Client()
client.connect("127.0.0.1",1883,60)
client.loop_start()
client.on_message = on_message
client.on_connect = on_connect

while(plot_no<16):
    plot_scan = plot_order[plot_no] - 1
    dest_loc = get_nearest_coordinate(curr_loc,plot_scan)
    d_node = get_node_from_coordinate(dest_loc)
    while(not((dest_loc[0] == curr_loc[0]) and (dest_loc[1] == curr_loc[1]))): 
        s_node = get_node_from_coordinate(dest_loc)
        G = nx.from_numpy_matrix(adjacency_matrix, create_using=nx.Graph())
        path_array = nx.dijkstra_path(G,s_node,d_node)
        index = 0
        print(path_array,len(path_array))
        while(len(path_array)>0):
            next_node = path_array.pop[0]
            next_loc = node_coord_matrix[next_node]
            print("Publishing Node",next_node)
            message = str(curr_loc[0])+":"+str(curr_loc[1])+":"+str(next_loc[0])+":"+ str(next_loc[1])
            client.publish(curr_loc[0],message)
            grid_matrix
            if(value_returned == 0):
                break
            else:
                curr_loc = ()
            time.sleep(5)
    plot_no+=1
            