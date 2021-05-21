import serial
import time
from time import sleep
import requests
ser = serial.Serial ("/dev/ttyS0", 115200)

URL = "https://us-central1-tigk-captain-america.cloudfunctions.net/Nodes"

def sendData(data):
    response = requests.post(URL, data)
    print(response.status_code)
print("Testing")
newDrivePath = {"stringFromRbp": "0,0,0,S"}
sendData(newDrivePath)
ser.write(b'R')                              #send to arduino that rpi is ready
while True:
    received_data = ser.read()              #read serial port
    sleep(0.03)
    data_left = ser.inWaiting()             #check for remaining byte
    received_data += ser.read(data_left)
    print (received_data)                   #print received data
    if chr(received_data[0]) == 'R':
        ser.write(b'R')
    if chr(received_data[-1]) == 'T' or chr(received_data[-1]) == 'F':
        sendDataString = {"stringFromRbp": received_data}
        sendData(sendDataString)
    else:
        print("error")
        