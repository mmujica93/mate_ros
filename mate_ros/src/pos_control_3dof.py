########################################################################
#           IMPORTANT              #
#  configure can before start. In the terminal run: 
#  sudo ip link set can0 up type can bitrate 1000000  <- This command is needed at every connection of the motor
########################################################################

import rospy
from geometry.msg import Point
#from resources.utils import RobotController
import time
import matplotlib.pyplot as plt
import numpy as np
import signal

import rospkg
import sys
import os

# Get the path of the ROS package
rospack = rospkg.RosPack()
package_path = rospack.get_path('mate_ros')
# Construct the path to ressources/
resources_path = os.path.join(package_path, 'resources')
# Add it to sys.path
sys.path.append(resources_path)
# Now import the module
from resources.utils import RobotController


ROBOT = "MATE_1" #define type of orobot PENDULUM/3DDL/SCARA
USE_TOOL = False
USER_INPUT = False
if USE_TOOL:
    from resources.Tool import Tool
signal_stop = False

def signal_handler(sig, frame):
    global signal_stop
    signal_stop = True
    rospy.signal_shutdown("User Interruption")
    print('You pressed Ctrl+C!')
    

param = {
    "ts": 0.005,
    "max_torque" : 2.0,
    "max_vel" : 200,
}  # use a dictionnary to store the simulation parameters


def publish_data(robot):
    global param, position_pub, velocity_pub

    rate = rospy.Rate(1 / param["ts"])  # Set publishing rate

    while not rospy.is_shutdown():
        nb_samples = int(4 / param["ts"])  # Time for each movement
        for k in range(nb_samples):
            if rospy.is_shutdown():
                break

            time_prev = time.time()

            # Set joint positions
            robot.set_joint_position(q_d[i])
            q_m = robot.get_joint_position()
            dq_m = robot.get_joint_velocity()

            # Publish position and velocity
            pos_msg = Float32MultiArray(data=q_m)
            vel_msg = Float32MultiArray(data=dq_m)
            position_pub.publish(pos_msg)
            velocity_pub.publish(vel_msg)

            rospy.loginfo(f"Position: {q_m} | Velocity: {dq_m}")

            # Safety check
            if any(abs(x) > param["max_vel"] for x in dq_m) or any(abs(x) > param["max_torque"] for x in robot.get_joint_torque()):
                robot.send_to_init_pose()
                time.sleep(2)
                robot.shutdown()
                rospy.logwarn("LIMIT EXCEEDED - SHUTTING DOWN")
                return

            elapsed_time = time.time() - time_prev
            if elapsed_time < param["ts"]:
                time.sleep(param["ts"] - elapsed_time)

        i = (i + 1) % len(q_d)  # Cycle through positions

def main():
    global param
    
    rospy.init_node("robot_controller_node", anonymous=True)
    position_pub = rospy.Publisher("robot/position", Float32MultiArray, queue_size=10)
    #velocity_pub = rospy.Publisher("robot/velocity", Float32MultiArray, queue_size=10)
    rospy.on_shutdown(lambda: print("Shutting down robot node..."))
    
    robot = RobotController(ROBOT, USE_TOOL)
    while True:
        a = input("mise à 0 ? (y): ")
        if a.lower() == 'y':
            break

    robot.init_offset()
    q_d = []
    q_d.append([45,40,50])
    q_d.append([-90,-10,60])
    q_d.append([0,60,-40])

    i = 0
    while (signal_stop==False):
        tf = 4  # time for each movement
        nb_samples = int((tf / param["ts"]))
        t = np.linspace(0, nb_samples * len(q_d[0]) * param["ts"], nb_samples)



        # create vectors to store data at each sample of time
        tau_m_all = []
        q_m_all = []
        dq_m_all = []
        
        for k in range(nb_samples):
            if(signal_stop==True):
                break

            time_prev = time.time()
            
            # Write control signal for joint 1
            print(q_d[i])
            robot.set_joint_position(q_d[i])
            #print(q_d[i])
            if k < (nb_samples)- 1:
                tau_m_all.append(robot.get_joint_torque())
                #print(robot.get_joint_position())
                q_m_all.append(robot.get_joint_position())
                dq_m_all.append(robot.get_joint_velocity())
                        # Publish position and velocity
            pos_msg = Point(data=q_m_all[-1])
            #vel_msg = Float32MultiArray(data=dq_m)
            position_pub.publish(pos_msg)
            #velocity_pub.publish(vel_msg)
            rospy.loginfo(f"Position: {q_m_all[-1]} | Velocity: {dq_m_all[-1]}")
            #print(robot.get_joint_position())
            # security after reading sensors
            if(any(abs(x)> param["max_vel"] for x in robot.get_joint_velocity()) or any(abs(x)> param["max_torque"] for x in robot.get_joint_torque())) :
                robot.send_to_init_pose()
                time.sleep(2)
                robot.shutdown()
                if(any(abs(x)> param["max_vel"] for x in robot.get_joint_velocity())):
                    print("LIMIT IN MEASURED velocity")
                else:
                    print("LIMIT IN MEASURED TORQUE")
                    print(robot.get_joint_torque())
                break
            
            elapsed_time = time.time()-time_prev
            #print(elapsed_time)
            if (elapsed_time < param['ts']):
                time.sleep(param['ts']-elapsed_time)
            time_prev = time.time()  
        if i <2:
            i=i+1
        else:
            i = 0

    time.sleep(1)
    robot.send_to_init_pose()
    time.sleep(1)
    robot.shutdown()
    
    q_m_all = np.array(q_m_all)
    dq_m_all = np.array(dq_m_all)
    tau_m_all = np.array(tau_m_all)
    print(q_m_all)
    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(t, q_m_all[:,0])
    plt.plot(t, q_m_all[:,1])
    plt.plot(t, q_m_all[:,2])
    plt.ylabel("angle")
    plt.grid()
    plt.subplot(3,1,2)
    plt.plot(t, dq_m_all[:,0])
    plt.plot(t, dq_m_all[:,1])
    plt.plot(t, dq_m_all[:,2])
    plt.ylabel("velocity")
    plt.grid()
    plt.subplot(3,1,3)
    plt.plot(t, tau_m_all[:,0])
    plt.plot(t, tau_m_all[:,1])
    plt.plot(t, tau_m_all[:,2])
    plt.ylabel("torque")
    plt.xlabel("time")
    plt.grid()
    plt.show()
#

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
