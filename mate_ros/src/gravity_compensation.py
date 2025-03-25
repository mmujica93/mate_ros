#!/usr/bin/env python
########################################################################
#           IMPORTANT              #
#  configure can before start. In the terminal run: 
#  sudo ip link set can0 up type can bitrate 1000000  <- This command is needed at every connection of the motor
########################################################################

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped
#from resources.utils import RobotController
import time
import matplotlib.pyplot as plt
import numpy as np
import signal

from resources.utils import RobotController, calculate_MGD

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
    "L1" : 0.11, #6cm
    "m1" : 0.579, # 579g
    "L2b": 0.09, #9cm
    'L2' : 0.216, # 21,6cm
    'L3' : 0.226, #22,6cm
    'm2' : 0.606, #606g
    'm3' : 0.410,#0.115, #115g
    "max_torque" : 3,
    "g" : 9.81,
    "max_vel" : 300,
}  # use a dictionnary to store the simulation parameters

#
def mate2(q,param):
    # this function describes the MDH and geometrics parameters of the mate3 robot
    # INPUT: Joint angles in deg, Segement lengths and geometry in meters
    # OUTPUT: DH table updated for each new joint configuration  
   
    DH=np.array([0,0,q[0],param["L1"]])
    DH=np.vstack((DH,[-90,0,q[1]-90,param["L2b"]]))
    DH=np.vstack((DH,[0,param["L2"],q[2],0]))
    DH=np.vstack((DH,[0,param["L3"],0,0]))
    return DH


def main():
    global param
    
    rospy.init_node("gravity_compensation_node", anonymous=True)
    position_pub = rospy.Publisher("mate/position", PoseStamped, queue_size=10)
    #velocity_pub = rospy.Publisher("robot/velocity", Float32MultiArray, queue_size=10)
    rospy.on_shutdown(lambda: print("Shutting down robot node..."))
    try:
        robot = RobotController(ROBOT, USE_TOOL)
    except:
        print("Problem can connection, try before running: sudo ip link set can0 up type can bitrate 1000000")
        rospy.signal_shutdown("User Interruption")
        quit()
    #
    while True:
        a = input("mise à 0 ? (y): ")
        if a.lower() == 'y':
            break

    robot.init_offset()
    tau_m_all = []
    q_m_all = []
    dq_m_all = []
    tau_grav = np.zeros((3,1))
    q_m_all.append(robot.get_joint_position())
    dq_m_all.append(robot.get_joint_velocity())
    tau_m_all.append(robot.get_joint_torque())
    pose = PoseStamped()
    while (signal_stop==False):
        if(signal_stop==True):
            break

        
        
        #print(q_d[i])
        #tau_m_all.append(robot.get_joint_torque())
        #dq_m_all.append(robot.get_joint_velocity())

        # security after reading sensors
        # if(any(abs(x)> param["max_vel"] for x in robot.get_joint_velocity()) or any(abs(x)> param["max_torque"] for x in robot.get_joint_torque())) :
        #     robot.send_to_init_pose()
        #     time.sleep(2)
        #     robot.shutdown()
        #     if(any(abs(x)> param["max_vel"] for x in robot.get_joint_velocity())):
        #         print("LIMIT IN MEASURED velocity")
        #     else:
        #         print("LIMIT IN MEASURED TORQUE")
        #         print(robot.get_joint_torque())
        #     break

        # Gravity compensation torque
        tau_grav[0] = 0.0 
        tau_grav[1] = -(np.sin((np.deg2rad(q_m_all[-1][1])))*(param["L2"]/2)*param["m2"]+(np.sin(np.deg2rad(q_m_all[-1][1]))*param["L2"]+np.sin(np.deg2rad(q_m_all[-1][1]+q_m_all[-1][2]))*(0.9*param["L3"]))*param["m3"])*param["g"]
        tau_grav[2] = -np.sin((np.deg2rad(q_m_all[-1][1]+q_m_all[-1][2])))*(0.9*param["L3"])*param["m3"]*param["g"]

        tau_c = tau_grav
        print(tau_c)
        # if(any(abs(x)> param["max_torque"] for x in tau_c)):
        #     robot.send_to_init_pose()
        #     time.sleep(2)
        #     robot.shutdown()
        #     print("LIMIT IN COMPUTED TORQUE")
        #     print(tau_c)
        #     break

        # Set joint torque
        time_prev = time.time()
        robot.set_joint_torque([0,0,0.0])
        elapsed_time = time.time()-time_prev
        q_m_all.append(robot.get_joint_position())
        dq_m_all.append(robot.get_joint_velocity())
        tau_m_all.append(robot.get_joint_torque())
        MGD = calculate_MGD(mate2(q_m_all[-1],param))
        T = MGD[:,:,-1]
        X = T[0:3,3]
        # Publish position and velocity
        
        #pose.header.seq=i
        pose.header.stamp = rospy.get_rostime()
        pose.pose.position.x = X[0]
        pose.pose.position.y = X[1]
        pose.pose.position.z = X[2]
        #vel_msg = Float32MultiArray(data=dq_m)
        
        position_pub.publish(pose)
        
        #velocity_pub.publish(vel_msg)
                
       
        print(elapsed_time)
        if (elapsed_time < param['ts']):
            time.sleep(param['ts']-elapsed_time)
        time_prev = time.time()  

    time.sleep(1)
    robot.send_to_init_pose()
    time.sleep(1)
    robot.shutdown()
    t = np.linspace(0,len(q_m_all)*param["ts"],len(q_m_all))
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
