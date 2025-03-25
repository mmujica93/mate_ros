# sudo chmod 666 /dev/ttyACM0 
from vacuum_cup_librairy import gripper_strive, gripper_release

while True:

    print ("Enter '1' to strive or '0' to release")

    var = str(input())

    if(var == '1'):
        gripper_strive()
        print("Strive")

    if(var == '0'):
        gripper_release()
        print("Release")