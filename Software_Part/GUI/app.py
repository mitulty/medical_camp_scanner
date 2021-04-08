from flask import Flask, request, jsonify, render_template
import paho.mqtt.client as mqtt
import time
import os

PEOPLE_FOLDER = os.path.join('static', 'people_photo')
arr_2d =    [[9, -1, 9, -1, 9, -1, 9, -1, 9],
			[-1, -5, -1, -5, -1, -5, -1, -5, -1],
			[9, -1, 9, -1, 9, -1, 9, -1, 9],
			[-1, -5, -1, -5, -1, -5, -1, -5, -1],
			[9, -1, 9, -1, 9, -1, 9, -1, 9],
			[-1, -5, -1, -5, -1, -5, -1, -5, -1],
			[9, -1, 9, -1, 9, -1, 9, -1, 9],
			[-1, -5, -1, -5, -1, -5, -1, -5, -1],
			[9, -1, 9, -1, 9, -1, 9, -1, 9]]

plot_value = {"Plot_Val":0}
color_value = {"Color_Val":0}
thisdict = {
  "m": -1,
  "n": -1,
}

def on_message(client, userdata, message):
	temp = (message.payload.decode("utf-8"))
	# print(temp)
	temp = str(temp)
	temp = temp.replace("\"","")
	temp = temp.split(":")
	print(temp[0]+"=="+temp[1]+"=="+temp[2])
	pos = int(temp[0])
	plot = int(temp[1])
	color = int(temp[2])

	if pos == 3 or pos == 4:
		deb_val = int(temp[3])

	if pos == 1:
		if plot == 1:
			arr_2d[1][1] = color
		elif plot ==2:
			arr_2d[1][3] = color
		elif plot ==3:
			arr_2d[1][5] = color
		elif plot ==4:
			arr_2d[3][7] = color
		elif plot ==5:
			arr_2d[3][1] = color
		elif plot ==6:
			arr_2d[3][3] = color
		elif plot ==7:
			arr_2d[3][5] = color
		elif plot ==8:
			arr_2d[3][7] = color
		elif plot ==9:
			arr_2d[5][1] = color
		elif plot ==10:
			arr_2d[5][3] = color
		elif plot ==11:
			arr_2d[5][5] = color
		elif plot ==12:
			arr_2d[5][7] = color
		elif plot ==13:
			arr_2d[7][1] = color
		elif plot ==14:
			arr_2d[7][3] = color
		elif plot ==15:
			arr_2d[7][5] = color
		elif plot == 16:
			arr_2d[7][7] = color
	
	if pos == 3:
		plot_value["Plot_Val"]=plot
		color_value["Color_Val"]=color

	if pos == 2:
		thisdict["m"] = color
		thisdict["n"] = plot

	if pos == 4:
		arr_2d[color][plot] = deb_val


app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = PEOPLE_FOLDER
client = mqtt.Client()
client.connect("127.0.0.1",1883,60)
client.loop_start()
client.on_message = on_message
client.subscribe("LOCATION_CS684")    

@app.route('/',methods = ['GET'])
def home():
	aid_img = os.path.join(app.config['UPLOAD_FOLDER'], 'aid.png')
	start_img = os.path.join(app.config['UPLOAD_FOLDER'], 'start.png')
	value = plot_value["Plot_Val"]
	color_val = color_value["Color_Val"]
	m = thisdict["m"]
	n = thisdict["n"]
	return render_template("index.html", value = value, color_val = color_val, matrix = arr_2d, aid_img = aid_img, start_img = start_img, m = m, n = n)


if __name__ == "__main__":
    app.run(debug=True)
