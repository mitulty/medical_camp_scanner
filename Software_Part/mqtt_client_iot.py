import time
import paho.mqtt.client as mqtt
import json
import requests
import os

fner_data = {}
scan_data = {}
dummy_data = {
        "id": -1,
        "plot": -1,
        "timeTaken": -1
    }
def on_connect(client, userdata, flags, rc):
    print("Connected with Result Code: " + str(rc))
    client.subscribe('v1/devices/me/rpc/request/+', 1)
    with open('data.json', 'w') as fp:
        json.dump([dummy_data],fp)

def on_message(client, userdata, msg):
    req = json.loads(msg.payload)
    
    if(req['method'] == 'scan'):
        scan_data['id'] = req['params']['id']
        scan_data['plot'] = req['params']['plot']
        scan_data['timeTaken'] = 0
        order = int(req['params']['plot']) +96
        with open('data.json', 'r') as outfile:
            json_data=outfile.read()
        rpc_list = json.loads(json_data)
        rpc_list.append(scan_data)
    else:
        fner_data['id'] = req['params']['id']
        fner_data['type'] = req['params']['type']
        if(req['params']['type'] == 'majorInjury'):
            order = 17 + 96
        elif(req['params']['type'] == 'minorInjury'):
            oredr = 18 + 96
        else:
            order = 19 + 96
        fner_data['timeTaken'] = 0
        with open('data.json', 'r') as outfile:
            json_data=outfile.read()
        rpc_list = json.loads(json_data)
        rpc_list.append(fner_data)
    with open('data.json', 'w') as outfile:
        json.dump(rpc_list,outfile)
    os.system("mosquitto_pub -h localhost -t data_recv -m "+chr(order))
    print(req,order)


host_url = "thingsboard.e-yantra.org"
device_access_token ="QoUDGL9Ky2x72LCV6FIe"# "Yju85G2v378HSN7oXhQK"
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