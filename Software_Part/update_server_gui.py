import paho.mqtt.client as mqtt
from coapthon.client.helperclient import HelperClient
import json

def coap_send(payload):
    host ="13.250.13.141"
    port =5683
    client = HelperClient(server=(host,port))
    token = "QoUDGL9Ky2x72LCV6FIe"
    path = f"api/v1/{token}/telemetry"
    print("Sending to thingsboard")
    response = client.post(path,payload=str(payload))
    print ("response.code",response.code)
    client.stop ()
def on_message(client, userdata, message):
	temp = str(message.payload.decode("utf-8"))
	if(len(temp) >5 ):
		if(temp[1]== str(3)):
			data = {}
			temp = temp.replace("\"","")
			temp = temp.split(":")
			print("COAP SEND")
			with open('data.json', 'r') as outfile:
				json_data=outfile.read()
			rpc_list = json.loads(json_data)
			print(rpc_list[1],type(rpc_list[1]))
			data = rpc_list[1]
			rpc_list.pop(1)
			with open('data.json', 'w') as outfile:
				json.dump(rpc_list,outfile)
			data['timeTaken'] = int(temp[1])
			coap_send(data)
			gui_data = "\""+str(temp[0])+":"+str(data["id"])+":"+str(temp[1])+"\""
			client.publish("LOCATION_CS684", gui_data)
		else:
			client.publish("LOCATION_CS684",temp)

def on_connect(client,userdata,flags,rc):
	print("connected with status"+str(rc))
	client.subscribe("data_sent")    

client = mqtt.Client()
client.connect("127.0.0.1",1883,60)
client.loop_start()
client.on_message = on_message
client.on_connect = on_connect
try:
	while True:
		pass	
except KeyboardInterrupt:
		client.loop_stop()
		client.disconnect()
