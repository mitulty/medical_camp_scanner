import json
import time
import requests


TIME = 1
scan_data = {}
fner_data = {}
DEVICE_ID = '2dd5ffa0-9c78-11eb-950e-efef5c07c810'
JWT_TOKEN = ''
BASE = 'https://thingsboard.e-yantra.org/api'

if __name__ == "__main__":
    headers = {
        'Content-Type': 'application/json',
        'Accept': 'application/json'
    }
    data = '{"username":"bitboot@cs684.edu", "password":"bitboot123#"}'
    response = requests.post('https://thingsboard.e-yantra.org/api/auth/login', headers=headers, data=data)
    JWT_TOKEN = response.json()['token']
    with open('rpc.json','r') as fp:
        json_data=fp.read()
    rpc_list = json.loads(json_data)
    for req in rpc_list:
        time.sleep(TIME)
        print(req)
        try:
            response = requests.post(f'{BASE}/plugins/rpc/oneway/{DEVICE_ID}', 
                json=req,
                headers={
                    'X-Authorization': f'Bearer {JWT_TOKEN}',
                })
            print(response)
            if response.status_code // 100 == 2:
            	print(f'{req} successfully sent.')
            else:
            	print(f'Failed {response.json()}')
        except requests.exceptions.RequestException as e:
            # print(response.json())
            print(f'Problem in sending request: {req}')
            print(e)
#curl -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{"username":"bitboot@cs684.edu", "password":"bitboot123#"}' 'https://thingsboard.e-yantra.org/api/auth/login'

