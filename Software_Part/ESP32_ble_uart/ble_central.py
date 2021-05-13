import asyncio
import platform
import os
from bleak import BleakClient
import paho.mqtt.client as mqtt

req_flag = 0
value = 0
REQ_UUID = "b7172833-07fd-4c42-9199-7b2636108949"
POS_UUID = "c3e660d4-c395-4a73-93fc-5392bc42092c"

def on_message(client, userdata, message):
    global req_flag
    global value
    req_flag = 1
    value = str(message.payload.decode("utf-8"))
    print(str(message))
def on_connect(client, userdata, flags, rc):
    print("Connected With Status: "+str(rc))
    client.subscribe("data_recv")


client = mqtt.Client()
client.connect("127.0.0.1", 1883, 60)
client.on_message = on_message
client.on_connect = on_connect
client.loop_start()

address = (
    "AC:67:B2:26:6C:86"  # <--- Change to your device's address here if you are using Windows or Linux
    if platform.system() != "Darwin"
    else "B9EA5233-37EF-4DD6-87A8-2A875E821C46"  # <--- Change to your device's address here if you are using macOS
)


async def get_data(client):
    def keypress_handler(sender, data):
        try:
            data = data.decode()
            print(f"POSITION: {sender} {data}")
            os.system("mosquitto_pub -h localhost -t data_sent -m " + str(data))
        except:
            pass
    await client.start_notify(POS_UUID, keypress_handler)
    while True:
        await asyncio.sleep(1)


async def send_req(client):
    while True:
        global req_flag
        global value
        if(req_flag == 1):
            await client.write_gatt_char(REQ_UUID, bytearray(value,encoding='utf8'))
            print(f'Sent {value}')
            req_flag = 0
            value = 0
        await asyncio.sleep(4.0)


async def main(address):
    async with BleakClient(address) as client:
        await asyncio.gather(get_data(client), send_req(client))


if __name__ == "__main__":
    asyncio.run(main(address))