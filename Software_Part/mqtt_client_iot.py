import time
import paho.mqtt.client as mqtt
import json
import requests
import os

fner_data = {'id': -1,'type':'Injury','timeTaken':0,'timeStamp':-1}
scan_data = {'id': -1,'plot':-1,'timeTaken':0,'timeStamp':-1}
dummy_data = {
        "id": -1,
        "plot": -1,
        "timeTaken": -1,
        "timeStamp": -1,
    }
def on_connect(client, userdata, flags, rc):
    print("Connected with Result Code: " + str(rc))
    client.subscribe('v1/devices/me/rpc/request/+', 1)
    with open('data.json', 'w') as fp:
        json.dump([dummy_data],fp)

def on_message(client, userdata, msg):
    req = json.loads(msg.payload)
    taken = 0
    ts = int(time.time())
    if(req['method'] == 'scan'):

        scan_data['id'] = req['params']['id']
        scan_data['plot'] = req['params']['plot']
        scan_data['timeStamp'] = ts
        scan_data['timeTaken'] = 0

        order = int(req['params']['plot']) + 64
        with open('data.json', 'r') as outfile:
            json_data=outfile.read()
        rpc_list = json.loads(json_data)
        rpc_list.append(scan_data)

    else:

        fner_data['id'] = req['params']['id']
        fner_data['type'] = req['params']['type']
        fner_data['timeStamp'] = ts
        fner_data['timeTaken'] = 0

        if(req['params']['type'] == 'majorInjury'):
            order = 17 + 64
        elif(req['params']['type'] == 'minorInjury'):
            order = 18 + 64
        else:
            order = 19 + 64

        with open('data.json', 'r') as outfile:
            json_data=outfile.read()
        rpc_list = json.loads(json_data)
        rpc_list.append(fner_data)

    with open('data.json', 'w') as outfile:
        json.dump(rpc_list,outfile)
    time_taken = req['params']['completeIn']
    if(time_taken < 10):
        taken = time_taken+ 48
    elif(time_taken >= 10 and time_taken < 36):
        taken = time_taken+ 55
    else:
        taken = time_taken+ 61
    os.system("mosquitto_pub -h localhost -t data_recv -m "+ chr(order))
    os.system("mosquitto_pub -h localhost -t data_recv -m " + chr(taken))
    print(req,chr(order),chr(taken))

host_url = "thingsboard.e-yantra.org"
device_access_token ="z6c3tqkoy1AhyM5alCgN"
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(device_access_token)
client.connect(host_url, 1883, 60)
client.loop_start()

try:
    while True:
        pass
except KeyboardInterrupt:
    client.loop_stop()
    client.disconnect()