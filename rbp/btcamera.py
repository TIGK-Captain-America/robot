import serial
import time
import picamera
from time import sleep
import requests
ser = serial.Serial ("/dev/ttyS0", 9600)

URL = "https://us-central1-tigk-captain-america.cloudfunctions.net/Nodes"

def sendData(data):
    response = requests.post(URL, data)
    print(response.status_code)
print("Testing")
newDrivePath = {"stringFromRbp": "0,0,S"}
sendData(newDrivePath)

while True:
    received_data = ser.read()              #read serial port
    sleep(0.03)
    data_left = ser.inWaiting()             #check for remaining byte
    received_data += ser.read(data_left)
    print (received_data)                   #print received data
    sendDataString = {"stringFromRbp": received_data}
    sendData(sendDataString)
    if chr(received_data[-1]) == 'T':
        with picamera.PiCamera() as camera:
            camera.resolution = (1024, 768)
            camera.start_preview()
            # Camera warm-up time
#             time.sleep()
            camera.capture('test.jpg')
    elif chr(received_data[-1]) == 'F':
        
        print("tjo")
    else:
        print("fel")
        