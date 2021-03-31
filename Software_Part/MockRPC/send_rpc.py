import json
import time
import requests


TIME=10
DEVICE_ID = '29a5e8d0-9167-11eb-950e-efef5c07c810'
JWT_TOKEN = 'eyJhbGciOiJIUzUxMiJ9.eyJzdWIiOiJiaXRib290QGNzNjg0LmVkdSIsInNjb3BlcyI6WyJURU5BTlRfQURNSU4iXSwidXNlcklkIjoiNTdjMzk4MDAtNzAyMS0xMWViLTk1MGUtZWZlZjVjMDdjODEwIiwiZmlyc3ROYW1lIjoiQml0Qm9vdCIsImxhc3ROYW1lIjoiQElJVEIiLCJlbmFibGVkIjp0cnVlLCJpc1B1YmxpYyI6ZmFsc2UsInRlbmFudElkIjoiNTc2MjY1MzAtNzAyMS0xMWViLTk1MGUtZWZlZjVjMDdjODEwIiwiY3VzdG9tZXJJZCI6IjEzODE0MDAwLTFkZDItMTFiMi04MDgwLTgwODA4MDgwODA4MCIsImlzcyI6InRoaW5nc2JvYXJkLmlvIiwiaWF0IjoxNjE3MTI1MjIxLCJleHAiOjE2MTcxMzQyMjF9.CAK6rBIkngJT0Z6LwFfvo0nH4u8ARgtnS1VQI3JftntIN-vbrVbVAIiqs0q6uMK1U7Cpy-UVOjP-gHiDHVqHQg'
BASE = 'https://thingsboard.e-yantra.org/api'

if __name__ == "__main__":
    with open('rpc.json','r') as fp:
        json_data=fp.read()
    rpc_list = json.loads(json_data)
    for req in rpc_list:
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
        time.sleep(TIME)
#curl -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{"username":"bitboot@cs684.edu", "password":"bitboot123#"}' 'https://thingsboard.e-yantra.org/api/auth/login'
