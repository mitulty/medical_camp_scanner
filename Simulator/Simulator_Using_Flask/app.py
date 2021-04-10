from flask import Flask, request, jsonify, render_template
import paho.mqtt.client as mqtt
import time
import os

PEOPLE_FOLDER = os.path.join('static', 'people_photo')
arr_2d = [[9, 1, 9, 1, 9, 1, 9, 1, 9],
          [0, 5, 1, 3, 0, 5, 0, 3, 1],
          [9, 1, 9, 0, 9, 1, 9, 0, 9],
          [1, 3, 1, 4, 1, 3, 1, 3, 1],
          [9, 0, 9, 1, 9, 0, 9, 1, 9],
          [1, 4, 1, 4, 1, 3, 1, 5, 0],
          [9, 0, 9, 1, 9, 1, 9, 0, 9],
          [1, 5, 0, 3, 1, 3, 1, 4, 1],
          [9, 0, 9, 1, 9, 1, 9, 0, 9]]

arr_2dr = [[9, -1, 9, -1, 9, -1, 9, -1, 9],
           [-1, -5, -1, -5, -1, -5, -1, -5, -1],
           [9, -1, 9, -1, 9, -1, 9, -1, 9],
           [-1, -5, -1, -5, -1, -5, -1, -5, -1],
           [9, -1, 9, -1, 9, -1, 9, -1, 9],
           [-1, -5, -1, -5, -1, -5, -1, -5, -1],
           [9, -1, 9, -1, 9, -1, 9, -1, 9],
           [-1, -5, -1, -5, -1, -5, -1, -5, -1],
           [9, -1, 9, -1, 9, -1, 9, -1, 9]]

robot_position = {
    "mr": -1,
    "nr": -1,
}
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

def update_plot_status(loc, dir):
    if(loc[0] == 0):
        arr_2dr[loc[1]][loc[0]+1] = arr_2d[loc[1]][loc[0]+1]
    elif(loc[0] == 8):
        arr_2dr[loc[1]][loc[0]-1] = arr_2d[loc[1]][loc[0]-1]
    
    elif(loc[1] == 0):
        arr_2dr[loc[1]+1][loc[0]] = arr_2d[loc[1]+1][loc[0]]
    elif(loc[1] == 8):
        arr_2dr[loc[1]-1][loc[0]] = arr_2d[loc[1]-1][loc[0]]
    
    elif(dir == 'n'):
        arr_2dr[loc[1]][loc[0]+1] = arr_2d[loc[1]][loc[0]+1]
        arr_2dr[loc[1]][loc[0]-1] = arr_2d[loc[1]][loc[0]-1]        
    else:
        arr_2dr[loc[1]-1][loc[0]] = arr_2d[loc[1]-1][loc[0]]
        arr_2dr[loc[1]+1][loc[0]] = arr_2d[loc[1]+1][loc[0]]
    
def on_connect(client,userdata,flags,rc):
    print("connected with status"+str(rc))
    client.subscribe("GET_POSITION",qos=0)

def on_message(client, userdata, message):
    temp = (message.payload.decode("utf-8"))
    # print(temp)
    temp = str(temp)
    temp = temp.split(":")
    mtype = int(temp[0])
    start_x = int(temp[1])
    start_y = int(temp[2])
    end_x = int(temp[3])
    end_y = int(temp[4])
    path = " "
    print("Received: ", mtype, start_x, start_y, end_x, end_y)
    if(mtype == 1):
        print("Type 1 Request")
        target_x = -1
        target_y = -1
        if start_x == end_x and start_y != end_y:
            target_x = start_x
            if(end_y < start_y):
                target_y = end_y + 1
            else:
                target_y = end_y - 1
            arr_2dr[target_y][target_x] = arr_2d[target_y][target_x]
            if arr_2d[target_y][target_x] == 1:
                path = str(9)+":"+str(1)+":"+str(target_x)+":"+str(target_y)
            else:
                path = str(9)+":"+str(0)+":"+str(target_x)+":"+str(target_y)
        elif start_y == end_y and start_x != end_x:
            target_y = start_y
            if(end_x < start_x):
                target_x = end_x + 1
            else:
                target_x = end_x - 1
            arr_2dr[target_y][target_x] = arr_2d[target_y][target_x]
            if arr_2d[target_y][target_x] == 1:
                path = str(9)+":"+str(1)+":"+str(target_x)+":"+str(target_y)
            else:
                path = str(9)+":"+str(0)+":"+str(target_x)+":"+str(target_y)
        client.publish("DEBRISH_VALUE", path, qos=0, retain=False)
    
    elif (mtype == 2):
        print("Type 2 Request")
        midpoint_x = start_x
        midpoint_y = start_y
        arr_2dr[midpoint_y][midpoint_x] = arr_2d[midpoint_y][midpoint_x]
        path = str(8)+":"+str(midpoint_x)+":"+str(midpoint_y) + ":"+str(arr_2d[midpoint_y][midpoint_x])
        client.publish("DEBRISH_VALUE", path, qos = 0, retain=False)
    else:
        dir=''
        robot_position["mr"] = start_y
        robot_position["nr"] = start_x
        arr_2dr[end_y][end_x] = arr_2d[end_y][end_x]
        if(end_x == start_x):
            dir = 'n'
        else:
            dir = 'e'
        update_plot_status((end_x,end_y),dir)

app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = PEOPLE_FOLDER
client = mqtt.Client()
client.connect("127.0.0.1", 1883, 60)
client.on_message = on_message
client.on_connect = on_connect
client.loop_start()


@app.route('/', methods=['GET'])
def home():
    aid_img = os.path.join(app.config['UPLOAD_FOLDER'], 'aid.png')
    start_img = os.path.join(app.config['UPLOAD_FOLDER'], 'start.png')
    mr = robot_position["mr"]
    nr = robot_position["nr"]
    return render_template("index.html", matrix=arr_2d, matrixr=arr_2dr, aid_img=aid_img, start_img=start_img, mr=mr, nr=nr)


if __name__ == "__main__":
    app.run(debug=False)