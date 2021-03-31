import time
import paho.mqtt.client as mqtt
import json
import requests

def on_connect(client,userdata,flags,rc):
	print("connected with result code:"+ str(rc))
	client.subscribe('v1/devices/me/rpc/request/+',1)

def on_message(client,userdata,msg):
	data =json.loads(msg.payload)
	print(data)

host_url = "thingsboard.e-yantra.org"
device_access_token = "QoUDGL9Ky2x72LCV6FIe"
interval = 10
next_reading = time.time()
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(device_access_token)
client.connect(host_url,1883,60)
client.loop_start()
try:
	while True:
		pass	
except KeyboardInterrupt:
		client.loop_stop()
		client.disconnect()
