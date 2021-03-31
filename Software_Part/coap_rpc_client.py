from coapthon.client.helperclient import HelperClient

if __name__ == '__main__':
    host ="13.250.13.141"
    port =5683
    client = HelperClient(server=(host,port))
    token = "QoUDGL9Ky2x72LCV6FIe"
    path = f"api/v1/{token}/telemetry"
    payload = "{'temperature':'45','humidity':'80','pressure':'70'}"
    response = client.post(path,payload=payload)
    print ("response.code",response.code)
    client.stop ()