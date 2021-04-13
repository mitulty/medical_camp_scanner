import paho.mqtt.client as mqtt
from coapthon.client.helperclient import HelperClient
import json
import time


def coap_send(payload):
    host = "13.250.13.141"
    port = 5683
    client = HelperClient(server=(host, port))
    token = "z6c3tqkoy1AhyM5alCgN"
    path = f"api/v1/{token}/telemetry"
    print("Sending to thingsboard ", payload)
    response = client.post(path, payload=str(payload))
    print("response.code", response.code)
    client.stop()


def on_message(client, userdata, message):
    temp = str(message.payload.decode("utf-8"))
    # print(temp)
    if(len(temp) > 5):
        temp = temp.replace("\"", "")
        temp_list = temp.split(";")
        print(temp_list)
        for i in temp_list:
            #print(i, len(i))
            if(len(i) < 5):
                continue
            #print(i, i[0], type(i[0]))
            if(i[0] == str(3)):
                j = i.split(":")
                ts = int(time.time())
                # print(i)
                data = {}
                print("COAP SEND")
                with open('data.json', 'r') as outfile:
                    json_data = outfile.read()
                rpc_list = json.loads(json_data)
                print(rpc_list[1], type(rpc_list[1]))
                data = rpc_list[1]
                rpc_list.pop(1)
                with open('data.json', 'w') as outfile:
                    json.dump(rpc_list, outfile)
                if(int(j[1]) != -1):
                    data['timeTaken'] = ts - data['timeStamp']
                else:
                    data['timeTaken'] = 0
                data.pop('timeStamp')
                coap_send(data)
                print("COAP Received")
                gui_data = "\""+str(j[0])+":" + \
                    str(data["id"])+":"+str(j[1])+"\""
                client.publish("LOCATION_CS684", gui_data)
            else:
                temp = "\"" + i + "\""
                client.publish("LOCATION_CS684", temp)


def on_connect(client, userdata, flags, rc):
    print("Connected With Status: "+str(rc))
    client.subscribe("data_sent")


client = mqtt.Client()
client.connect("127.0.0.1", 1883, 60)
client.loop_start()
client.on_message = on_message
client.on_connect = on_connect
try:
    while True:
        pass
except KeyboardInterrupt:
    client.loop_stop()
    client.disconnect()
