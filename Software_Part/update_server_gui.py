import paho.mqtt.client as mqtt
from coapthon.client.helperclient import HelperClient
def coap_send(payload):
    host ="13.250.13.141"
    port =5683
    client = HelperClient(server=(host,port))
    token = "QoUDGL9Ky2x72LCV6FIe"
    path = f"api/v1/{token}/telemetry"
    response = client.post(path,payload=payload)
    print ("response.code",response.code)
    client.stop ()
def on_message(client, userdata, message):
	temp = str(message.payload.decode("utf-8"))
	print(temp)
	print(int(temp[1]))
	if(int(temp[1]==3)):
		with open('data.json', 'w') as outfile:
			json_data=fp.read()
		#coap_send(json_data)
	client.publish("LOCATION_CS684", temp)
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
