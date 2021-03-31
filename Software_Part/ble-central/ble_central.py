import asyncio
import platform

from bleak import BleakClient

LED_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BUTTON_UUID = "8801f158-f55e-4550-95f6-d260381b99e7"


address = (
    "84:CC:A8:48:64:C2"  # <--- Change to your device's address here if you are using Windows or Linux
    if platform.system() != "Darwin"
    else "B9EA5233-37EF-4DD6-87A8-2A875E821C46"  # <--- Change to your device's address here if you are using macOS
)


async def notify(client):
    def keypress_handler(sender, data):
        print(f"BUTTON: {sender} {data}")
    await client.start_notify(BUTTON_UUID, keypress_handler)
    while True:
        await asyncio.sleep(0.1)


async def run(client):
    while True:
        switchState = await client.read_gatt_char(LED_UUID)
        switchState = switchState.decode('utf-8')
        if switchState == "0":
            value = bytearray(b'1')
            # value = value.extend(map(ord, "1"))
        else:
            value = bytearray(b'0')
            # value = value.extend(map(ord, "0"))
        await client.write_gatt_char(LED_UUID, value)
        print(f'LED {value}')
        await asyncio.sleep(0.1)


async def main(address):
    async with BleakClient(address) as client:
        await asyncio.gather(notify(client), run(client))


if __name__ == "__main__":
    asyncio.run(main(address))
