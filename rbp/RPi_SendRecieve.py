#This whole script is connected to the low level requirements 
# -Send position to backend
# -Connect Arduino to RPi

import serial
import time
from time import sleep
import requests
ser = serial.Serial ("/dev/ttyS0", 115200)   #Connect to Arduino

substringF = b'F'
substringT = b'T'

URL = "https://us-central1-tigk-captain-america.cloudfunctions.net/Nodes"

def sendData(data):
    response = requests.post(URL, data)
    print(response.status_code)

print("Testing")
newDrivePath = {"stringFromRbp": "0,0,0,S"}
sendData(newDrivePath)
ser.write(b'R')                              #send to arduino that rpi is ready
while True:
    received_data = ser.read()               #read serial port
    sleep(0.01)
    data_left = ser.inWaiting()              #check for remaining byte
    received_data += ser.read(data_left)
    print (received_data)                    #print received data
    
    #Handle double data 
    if received_data.count(substringF) >=2 or received_data.count(substringT) >=2:
        if received_data.find(substringF) != -1:
            received_dataFirst = received_data[0: received_data.find(substringF)+1]
            received_dataSecond = received_data[received_data.find(substringF)+1:]
            print(received_dataFirst, "test1", received_dataSecond, "test2")
            sendDataString = {"stringFromRbp": received_dataFirst}
            sendData(sendDataString)
            sendDataString = {"stringFromRbp": received_dataSecond}
            sendData(sendDataString)
            
        if received_data.find(substringT) != -1:
            received_dataFirst = received_data[0: received_data.find(substringT)+1]
            received_dataSecond = received_data[received_data.find(substringT)+1:]
            print(received_dataFirst, "test1", received_dataSecond, "test2")
            sendDataString = {"stringFromRbp": received_dataFirst}
            sendData(sendDataString)
            sendDataString = {"stringFromRbp": received_dataSecond}
            sendData(sendDataString)
            
    if chr(received_data[0]) == 'R':
        ser.write(b'R')
    if chr(received_data[-1]) == 'T' or chr(received_data[-1]) == 'F':
        sendDataString = {"stringFromRbp": received_data}
        sendData(sendDataString)
    else:
        print("error")
        ser.flush()
        