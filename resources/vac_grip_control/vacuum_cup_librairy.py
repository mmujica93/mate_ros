# sudo chmod 666 /dev/ttyACM0 

import serial
arduino = serial.Serial(port = "/dev/ttyACM0", timeout=0)

def gripper_strive():
    arduino.write(str.encode('1'))

def gripper_release():
    arduino.write(str.encode('0'))