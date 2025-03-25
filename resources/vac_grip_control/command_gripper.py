# sudo chmod 666 /dev/ttyACM0 
from gripper_librairy import open_gripper, close_gripper


#arduino.write(str.encode('1'))


while True:

    print ("Enter '1' open or '0' to close")

    var = str(input())

    if(var == '1'):
        open_gripper()
        print("Open")

    if(var == '0'):
        close_gripper()
        print("Close")

