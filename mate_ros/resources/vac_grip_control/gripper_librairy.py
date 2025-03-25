# sudo chmod 666 /dev/ttyACM0 

import serial
arduino = serial.Serial(port = "/dev/ttyACM1", timeout=0)

def open_gripper():
    arduino.write(str.encode('1'))

def close_gripper():
    arduino.write(str.encode('0'))