# Robot
Arduino<br/>
Raspberry Pi (RPi)

## Project 
What does the project do?<br/>
Making the robot able to communicate with a raspberry pi to send data to a database via Wi-Fi. 
Sending data and receiving commands from a mobile application via bluetooth. 
Also creating a software that makes the robot drive autonomous. 

Why the project is useful?<br/>
Making the mower drive autonomous so no person needs to be present. Communicating with bluetooth via a mobile application makes the mower able to receive drive instructions from a person. With the raspberry pi communication the mowers driving path can be stored and when collisions are avoided.

## Project instruction
### Starting the mower
1. Make sure there are batteries
2. Start the mower by pressing the button at the top
3. Wait 40-60 seconds to let the RPi start
4. When the mower is ready it will start driving autonomously 
5. To drive manually, download the app and drive from there
6. To stop the mower, press the button at the top again

### Arduino
1. Install Visual Studio Code (VS Code)
2. Install the extension PlatformIO IDE (PIO) and C/C++ within VS Code
3. Let PIO finish downloading and open PIO Home (house symbol at the bottom)
4. Click on “Open Project” and open the “Mower”-folder downloaded from git

### Raspberry Pi zero W
1. Install Raspbian Lite OS on an SD-card
2. Setup Raspberry Pi to run through a remote desktop
3. Install the latest version of python3 
4. Make sure the following python3 libraries are installed, picamera, requests, serial and time.
5. Enter “sudo nano /etc/rc.local” in the terminal and add sudo bash -c '/usr/bin/python3 /home/pi/.../btcamera.py > /home/pi/.../btcamera.log 2>&1' & to the file above exit 0, to make the script start on powerup and create a log file.
6. Make a voltage divider to make UART communication work between the Arduino and the Raspberry Pi.  

## Mower Diagram
![Mower_Diagram](https://github.com/TIGK-Captain-America/robot/blob/main/WBS_HW.jpg)


