import sys
import networkx as nx
import numpy as np
import paho.mqtt.client as mqtt
import math
import time
curr_loc = (4, 8)
dir_flag = 'n'
goal_loc = (8, 4)
plot_order = [14, 13, 9, 10, 6, 5, 1, 2, 3, 4, 8, 7, 11, 12, 16, 15]
plot_no = 0
path_array = []
data = {}
data['a'] = -1
data['b'] = -1
data['c'] = -1
data['d'] = -1
status = 0
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


# G = nx.from_numpy_matrix(adjacency_matrix, create_using=nx.Graph())
# print(nx.dijkstra_path(G, 0, 23)

def update_adjacency_matrix():
    for m in range(25):
        coord_a = node_coord_matrix[m]
        for n in range(25):
            coord_b = node_coord_matrix[n]
            if ((coord_b[0] == coord_a[0] + 2) and (coord_b[1] == coord_a[1])):
                adjacency_matrix[m][n] = abs(
                    grid_matrix[coord_a[1]][coord_a[0] + 1])
            elif ((coord_b[0] == coord_a[0] - 2) and (coord_b[1] == coord_a[1])):
                adjacency_matrix[m][n] = abs(
                    grid_matrix[coord_a[1]][coord_a[0] - 1])
            elif ((coord_b[0] == coord_a[0]) and (coord_b[1] == coord_a[1] + 2)):
                adjacency_matrix[m][n] = abs(
                    grid_matrix[coord_a[1] + 1][coord_a[0]])
            elif ((coord_b[0] == coord_a[0]) and (coord_b[1] == coord_a[1] - 2)):
                adjacency_matrix[m][n] = abs(
                    grid_matrix[coord_a[1] - 1][coord_a[0]])


def print_grid_matrix():
    for i in range(9):
        for j in range(9):
            print(grid_matrix[i][j], end=" ")
        print("\n")


def print_adjacency_matrix():
    for i in range(25):
        print(i, ":", end="")
        for j in range(25):
            print(adjacency_matrix[i][j], end=" ")
        print("\n")


def get_nearest_coordinate(current_coord, plot):
    d = 0
    node = (-1, -1)
    max = 100.0
    for i in range(4):
        x_dis = ((current_coord[0]-plot_coord_matrix[plot][i][0])**2)
        y_dis = ((current_coord[1]-plot_coord_matrix[plot][i][1])**2)
        d = math.sqrt(x_dis+y_dis)
        if(d < max):
            node = (plot_coord_matrix[plot][i][0],
                    plot_coord_matrix[plot][i][1])
            max = d
    return node


def get_node_from_coordinate(a):
    for i in range(25):
        if(node_coord_matrix[i][0] == a[0] and node_coord_matrix[i][1] == a[1]):
            return i
    return -1


def get_plot_from_coordinate(a):
    for i in range(16):
        if(plot_coord_matrix[i][4] == a):
            return i
    return -1


def get_direction(a, b, c):
    d = ''
    if(a[0] == b[0] and a[1] != b[1]):
        if(a[1] > b[1]):
            d = 'n'
        else:
            d = 's'
    elif(a[1] == b[1] and a[0] != b[0]):
        if(a[0] > b[0]):
            d = 'w'
        else:
            d = 'e'
    else:
        d = c
    return d


def on_message(client, userdata, message):
    temp = str(message.payload.decode("utf-8"))
    data['a'] = int(temp[0])
    data['b'] = int(temp[2])
    data['c'] = int(temp[4])
    data['d'] = int(temp[6])
    #print("Data Received", data)


def on_connect(client, userdata, flags, rc):
    print("Connected with status:"+str(rc))
    client.subscribe("DEBRISH_VALUE", qos=0)


def mid_point_reachable(mid_point):
    message = str(2)+":"+str(mid_point[0])+":" + str(mid_point[1])+":9:9"
    client.publish("GET_POSITION", message)
    time.sleep(2)


def update_plot_status(loc, dir):
    if(loc[0] == 0):
        grid_matrix[loc[1]][loc[0]+1] = 7
    elif(loc[0] == 8):
        grid_matrix[loc[1]][loc[0]-1] = 7

    elif(loc[1] == 0):
        grid_matrix[loc[1]+1][loc[0]] = 7
    elif(loc[1] == 8):
        grid_matrix[loc[1]-1][loc[0]] = 7

    elif(dir == 'n' or dir == 's'):
        grid_matrix[loc[1]][loc[0]+1] = 7
        grid_matrix[loc[1]][loc[0]-1] = 7
    else:
        grid_matrix[loc[1]-1][loc[0]] = 7
        grid_matrix[loc[1]+1][loc[0]] = 7


def check_midpoint(a):
    global curr_loc
    global dir_flag
    plotcoord = plot_coord_matrix[a][4]
    midpoint1_x = curr_loc[0]
    midpoint1_y = plotcoord[1]
    dir_mp1 = get_direction(curr_loc, (midpoint1_x, midpoint1_y), dir_flag)
    #print("Midpoint 1: ",(midpoint1_x,midpoint1_y),dir_mp1)
    midpoint2_x = plotcoord[0]
    midpoint2_y = curr_loc[1]
    dir_mp2 = get_direction(curr_loc, (midpoint2_x, midpoint2_y), dir_flag)
    #print("Midpoint 2: ",(midpoint2_x,midpoint2_y),dir_mp2)
    
    if(dir_flag == dir_mp1):
        mid_point_reachable((midpoint1_x, midpoint1_y))
        time.sleep(2)
        if(data['a'] == 8):
            grid_matrix[data['c']][data['b']] = data['d']
            if(data['d'] == 1):
                if(midpoint1_x == curr_loc[0]):
                    if(curr_loc[1] > midpoint1_y):
                        curr_loc = (midpoint1_x, midpoint1_y-1)
                    else:
                        curr_loc = (midpoint1_x, midpoint1_y+1)
                else:
                    if(curr_loc[0] > midpoint1_x):
                        curr_loc = (midpoint1_x - 1, midpoint1_y)
                    else:
                        curr_loc = (midpoint1_x + 1, midpoint1_y)
                print("Moving to Next Location: ", curr_loc,get_node_from_coordinate(curr_loc)+1,dir_flag)
                robot_location = "3:" + \
                    str(curr_loc[0])+":"+str(curr_loc[1])+":" + \
                        str(midpoint1_x)+":"+str(midpoint1_y)
                client.publish("GET_POSITION", robot_location)
                update_plot_status((midpoint1_x, midpoint1_y), dir_flag)
                time.sleep(1)
                return 1

    if(dir_flag == dir_mp2):
        mid_point_reachable((midpoint2_x, midpoint2_y))
        time.sleep(2)
        if(data['a'] != 9):
            grid_matrix[data['c']][data['b']] = data['d']
            if(data['d'] == 1):
                if(midpoint2_x == curr_loc[0]):
                    if(curr_loc[1] > midpoint2_y):
                        curr_loc = (midpoint2_x, midpoint2_y-1)
                    else:
                        curr_loc = (midpoint2_x, midpoint2_y+1)
                else:
                    if(curr_loc[0] > midpoint2_x):
                        curr_loc = (midpoint2_x - 1, midpoint2_y)
                    else:
                        curr_loc = (midpoint2_x + 1, midpoint2_y)
                print("Moving to Next Location: ", curr_loc,get_node_from_coordinate(curr_loc)+1,dir_flag)
                robot_location = "3:" + \
                    str(curr_loc[0])+":"+str(curr_loc[1])+":" + \
                        str(midpoint2_x)+":"+str(midpoint2_y)
                client.publish("GET_POSITION", robot_location)
                update_plot_status((midpoint2_x, midpoint2_y), dir_flag)
                time.sleep(1)
                return 1

    if(dir_flag != dir_mp1):
        dir_flag = dir_mp1
        mid_point_reachable((midpoint1_x, midpoint1_y))
        time.sleep(2)
        if(data['a'] != 9):
            grid_matrix[data['c']][data['b']] = data['d']
            if(data['d'] == 1):
                if(midpoint1_x == curr_loc[0]):
                    if(curr_loc[1] > midpoint1_y):
                        curr_loc = (midpoint1_x, midpoint1_y-1)
                    else:
                        curr_loc = (midpoint1_x, midpoint1_y+1)
                else:
                    if(curr_loc[0] > midpoint1_x):
                        curr_loc = (midpoint1_x - 1, midpoint1_y)
                    else:
                        curr_loc = (midpoint1_x + 1, midpoint1_y)
                print("Moving to Next Location: ", curr_loc,get_node_from_coordinate(curr_loc)+1,dir_flag)
                robot_location = "3:" + \
                    str(curr_loc[0])+":"+str(curr_loc[1])+":" + \
                        str(midpoint1_x)+":"+str(midpoint1_y)
                client.publish("GET_POSITION", robot_location)
                update_plot_status((midpoint1_x, midpoint1_y), dir_flag)
                time.sleep(1)
                return 1
    if(dir_flag != dir_mp2):
        dir_flag = dir_mp2
        mid_point_reachable((midpoint2_x, midpoint2_y))
        time.sleep(2)
        if(data['a'] != 9):
            grid_matrix[data['c']][data['b']] = data['d']
            if(data['d'] == 1):
                if(midpoint2_x == curr_loc[0]):
                    if(curr_loc[1] > midpoint2_y):
                        curr_loc = (midpoint2_x, midpoint2_y-1)
                    else:
                        curr_loc = (midpoint2_x, midpoint2_y+1)
                else:
                    if(curr_loc[0] > midpoint2_x):
                        curr_loc = (midpoint2_x - 1, midpoint2_y)
                    else:
                        curr_loc = (midpoint2_x + 1, midpoint2_y)
                print("Moving to Next Location: ", curr_loc,get_node_from_coordinate(curr_loc)+1,dir_flag)
                robot_location = "3:" + \
                    str(curr_loc[0])+":"+str(curr_loc[1])+":" + \
                        str(midpoint2_x)+":"+str(midpoint2_y)
                client.publish("GET_POSITION", robot_location)
                update_plot_status((midpoint2_x, midpoint2_y), dir_flag)
                time.sleep(1)
                return 1
    return 0


def travel(dest_loc):
    global curr_loc
    global dir_flag
    d_node = get_node_from_coordinate(dest_loc)
    while(not((dest_loc[0] == curr_loc[0]) and (dest_loc[1] == curr_loc[1]))):
        s_node = get_node_from_coordinate(curr_loc)
        #print("Dijkstra from ", s_node, " to ", d_node)
        G = nx.from_numpy_matrix(adjacency_matrix, create_using=nx.Graph())
        path_array = nx.dijkstra_path(G, s_node, d_node)
        next_node = path_array.pop(0)
        index = 0
        #print("Path Returned from source node:", s_node, " to desitnation node:",d_node, " is:", path_array, " of length:", len(path_array))
        while(len(path_array) > 0):
            next_node = path_array.pop(0)
            next_loc = node_coord_matrix[next_node]
            message = str(1)+":"+str(curr_loc[0])+":"+str(curr_loc[1])+":"+str(next_loc[0])+":" + str(next_loc[1])
            #print("Publishing Node", next_node, message)
            client.publish("GET_POSITION", message)
            time.sleep(1)
            if(data['a'] == 9):
                grid_matrix[data['d']][data['c']] = data['b']
            update_adjacency_matrix()
            if(data['b'] == 0):
                # print("Breaking......")
                break
            else:
                dir_flag = get_direction(curr_loc, next_loc, dir_flag)
                curr_loc = next_loc
                update_plot_status((data['c'], data['d']), dir_flag)
                robot_location = "3:" + str(curr_loc[0])+":"+str(curr_loc[1]) + ":"+str(data['c'])+":"+str(data['d'])
                client.publish("GET_POSITION", robot_location)
                time.sleep(1)
                print("Moving to Next Location: ", curr_loc,get_node_from_coordinate(curr_loc)+1,dir_flag)
                time.sleep(1)


client = mqtt.Client()
client.connect("127.0.0.1", 1883, 60)
client.on_message = on_message
client.on_connect = on_connect
client.loop_start()
time.sleep(2)
while(plot_no < 16):
    plot_scan = plot_order[plot_no] - 1
    plot_coord = plot_coord_matrix[plot_scan][4]
    #print("Scanning(No:", plot_no, ")Plot: ", plot_scan, plot_coord)
    if(grid_matrix[plot_coord[1]][plot_coord[0]] == -5):
        while(True):
            if(status == 0):
                dest_loc = get_nearest_coordinate(curr_loc, plot_scan)
            #print("Reaching the corner node",dest_loc)
            # Bot reaches one of the corner nodes
            travel(dest_loc)
            #print("-------------------------------------------REACHED THE CORNER NODE------------------------------------")
            # Mid Point Check
            status = check_midpoint(plot_scan)
            #print("-------------------------------------------CHECKED THE MIDPOINT------------------------------------")            
            if(status == 1):
                plot_no += 1
                status = 0
                break
            else:
                status = -1
                if (curr_loc[0] - plot_coord[0] == 1):
                    x = curr_loc[0] - 2
                else:
                    x = curr_loc[0] + 2
                if (curr_loc[1] - plot_coord[1] == 1):
                    y = curr_loc[1] - 2
                else:
                    y = curr_loc[1] + 2
                dest_loc = (x,y)
    else:
        plot_no += 1
        status = 0
travel(goal_loc)
print("**********************REACHED THE GOAL******************************************")
#travel((2,4))