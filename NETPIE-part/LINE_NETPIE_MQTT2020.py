import paho.mqtt.publish as publish # pip3 install paho-mqtt
import paho.mqtt.client as mqtt
import time
import json
import random
import ssl
import serial
import requests
from time import sleep

#Communication Part
ser = serial.Serial ("/dev/ttyS0", 115200)    #Open port with baud rate
i = 0
def list_to_dict(rlist):
    #print(rlist)
    return dict(map(lambda s : s.split(':'), rlist))

#LINE notify part
url = "https://notify-api.line.me/api/notify"
token = "RNn1daI1r9UDqCL7UjxfepbIbRKRSzlx1pJ5H1VGOyd" # your Line Notify token
headers = {'Authorization':'Bearer '+token}
msg = {
        "message":"It's raining!!!!",
        "stickerPackageId" :"11539",
        "stickerId" :"52114141"
      }

#MQTT Part
port = 1883 # default port
Server_ip = "broker.netpie.io" 

Subscribe_Topic = "@msg/control/#"
Publish_Topic = "@shadow/data/update"

Client_ID = "63a76b3d-4801-4843-a6d1-d2089c1ef538"
Token = "62PVQDyZB2zxcCr8DhbyvAMVbrFHYDTn"
Secret = "nRstNyV(wRy93wthvsS99yIlygwGt(In"

MqttUser_Pass = {"username":Token,"password":Secret}

Mode = ""
Pump_Status = "of"
# sensor_data = {"Water": 0, "Turbid": 0, "Rain": 0, "Flow": 0, "LED" : LED_Status, "PUMP" : Pump_Status}
# sensor_data = {}
RainData = ["off","off"]
Pump_control = ""

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(Subscribe_Topic)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global LED_Status
    global Pump_Status
    global Pump_control
    print(msg.topic+" "+str(msg.payload))
    data_receive = msg.payload.decode("UTF-8")
    if msg.topic == "@msg/control/mode":
        Mode = bytes(data_receive, 'utf-8')
        ser.write(Mode)         #transmit data serially
    elif msg.topic == "@msg/control/PUMP":
        #Pump_Status = data_receive
        Pump_control = bytes(data_receive, 'utf-8')
        ser.write(Pump_control)         #transmit data serially

client = mqtt.Client(protocol=mqtt.MQTTv311,client_id=Client_ID, clean_session=True)
client.on_connect = on_connect
client.on_message = on_message

client.subscribe(Subscribe_Topic)
client.username_pw_set(Token,Secret)
client.connect(Server_ip, port)
client.loop_start()

while True:
        
        #Serial Part
        received_data = ser.read()              #read serial port
        sleep(0.03)
        data_left = ser.inWaiting()             #check for remaining byte
        received_data += ser.read(data_left)
        print(i)
        print (received_data)         #print received data
        RainData.pop(0)
        decoded_data = received_data.decode("UTF-8").strip().split()
        result = list_to_dict(decoded_data)
        RainData.append(result["rainingstatus"])
        #print(decoded_data)
        #print(result)
        #print(Pump_control)
        #print(RainData)
        
        #LINE Notify
        if (RainData[0] == 'off' and RainData[1] == 'on'):
            res = requests.post(url, headers=headers , data = msg)
            print(res.text)

        #Assign Values
        # sensor_data["Water"] = random.randrange(30, 40)
        # sensor_data["Turbid"] = random.randrange(50, 80)
        # sensor_data["Rain"] = random.randrange(0, 100)
        # sensor_data["Flow"] = random.randrange(50, 60)
        # sensor_data["LED"] = LED_Status
        # sensor_data["PUMP"] = Pump_Status
        data_out=json.dumps({"data": result}) # encode object to JSON
        print(data_out)
        client.publish(Publish_Topic, data_out, retain= True)
        print ("Publish.....")
        i += 1
        time.sleep(2)
        
