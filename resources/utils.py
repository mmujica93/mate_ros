import numpy as np
import time
import asyncio
import can
import struct
import json
import os

#
def calculate_MGD(DH):

    if len(DH.shape)>1:
        Nframes=DH.shape[0]
    else:
        Nframes=len(DH.shape)
        
    T_mgd=np.empty((4,4,1))

    for i in range(Nframes):
        if  len(DH.shape)>1:
            Ti=calculate_homogeneous_trans_matrix(DH[i,:])
        else:
            Ti=calculate_homogeneous_trans_matrix(DH)

        if i==0:
            T_mgd[:,:,0]=Ti
        else:
            T_mgdi=np.matmul(T_mgd[:,:,i-1],Ti)
            T_mgd=np.dstack((T_mgd,T_mgdi))

    return T_mgd

def calculate_pendulum_jacobian_matrix(T,joint_type):
## This function caculates the Jacobian basis matrix (6x1)
# INPUT: Transformation matrices (4x4xNbJoint) calculed from the FKM for each new joint configuration
# INPUT: joint type 1 is rotational 0 if translational
# OUTPUT: Jacobian matrix

    L=T[0:3,-1,-1]-T[0:3,-1,0]# substract position vectors
    Jv=np.cross(T[0:3,2,0],L)

    #Jw=T[0:3,2,0] 
    
    J=np.vstack(Jv)#,Jw)
    return J


def calculate_homogeneous_trans_matrix(DH):

    d=DH[1]
    r=DH[3]

    alpha=np.deg2rad(DH[0])
    theta=np.deg2rad(DH[2])

    cq=np.cos(theta)
    sq=np.sin(theta)
    ca=np.cos(alpha)
    sa=np.sin(alpha)
    T=np.array([[cq, -sq, 0, d], [ca*sq, cq*ca, -sa, -r*sa], [sa*sq, sa*cq, ca, r*ca], [0, 0, 0, 1]])

    return T

def calculate_RMSE(N,x1,x2):
    somme=0
    for i in range (N): 
        somme +=(x1[i]-x2[i])**2
    RMSE = np.sqrt(somme/N)
    return RMSE

def move_to_zero_position(qi,qf,tf):
    nb_joints=len(qi)
    A=[1,0,0,0]
    A=np.vstack((A,[0,1,0,0]))
    A=np.vstack((A,[1,tf,tf*tf,tf*tf*tf]))
    A=np.vstack((A,[0,1,2*tf,3*tf*tf]))

    for ii in range(nb_joints):
        coeff=np.matmul( np.linalg.inv(A), [qi[ii],0,qf[ii],0] )
    print(coeff)
    return  

def sleep(duration, get_now=time.perf_counter):
    now = get_now()
    end = now + duration
    while now < end:
        now = get_now()

def generate_poly3_traj(pi,pf,tf,param):
    pi = np.atleast_1d(pi)
    pf = np.atleast_1d(pf)
    nb_elements=len(pi)
    ts=param["ts"]
    A=[1,0,0,0]
    A=np.vstack((A,[0,1,0,0]))
    A=np.vstack((A,[1,tf,tf*tf,tf*tf*tf]))
    A=np.vstack((A,[0,1,2*tf,3*tf*tf]))

    p=np.empty((nb_elements,int(tf/ts)))
    t=np.linspace(0,tf,int(tf/ts))
 
    for ii in range(nb_elements):
        coeff=np.matmul( np.linalg.inv(A), [pi[ii],0,pf[ii],0] )
        p[ii,:]=coeff[0]+coeff[1]*t+coeff[2]*t*t+coeff[3]*t*t*t
    
    # If only one trajectory is requested, return a flattened array
    if nb_elements == 1:
        return p.flatten()
    else:
        return p

def calculate_rolling_average(data, n,index):
    #Calculate the rolling average of the last n values for each component in the input data array.
    #Parameters:
    #- data: The input data array .
    #- n: The number of values to consider for the rolling average.
    #- index: current step of the loop
    #Returns:
    #- A new array containing the rolling averages for each component.
    if index > n:
        window = data[-n:]
    else:
        window = data[-index:]
    average = sum(window) / len(window)

    return average

#

read_multi_angle_command = [0x92, 0, 0, 0, 0, 0, 0, 0]
read_torque_command = [0x9C, 0, 0, 0, 0, 0, 0, 0]

class MotorController:
    def __init__(self, param):
        self.bustype = "socketcan"
        self.bus_index = param["bus_index"]
        self.bitrate=1000000
        self.channel = param['channel']
        bus = can.Bus(channel = self.channel, bustype = self.bustype, index = self.bus_index, bitrate = self.bitrate)
        self.bus = bus
        self.ID = param['motor_id']
        self.position = 0.0
        self.torque = 0
        self.velocity = 0
        self.reduction_pos = param['reduction']
        self.compteur = 0
        self.previous_time = time.time()
        #self.sendPosition(0.0,50)
        # Start the notifier to continuously listen for messages
        self.notifier = can.Notifier(self.bus, [self.process_message])
        #self.sendPosition(0.0,50)
        
    def canSend(self, data):
    # This function sends data to the motors via CAN 
    # INPUT: data as 8 byte structure crated by struct.pack("xxxxxxxx",value1,value2,...)
    # OUTPUT: 8 bytes answered by the motor via CAN, to read them struct.unpack("xxxxxxxx",answer)
        self.bus.send(can.Message(is_extended_id=False, arbitration_id=0x140 + self.ID  ,data=data))
        
    def canSendAll(self, data):
        self.bus.send(can.Message(is_extended_id=False, arbitration_id=0x280    ,data=data))

    def update_position_on_response(self, data):
        #current_time = time.time()
        #print(f'{self.ID} : {current_time-self.previous_time}')
        #self.previous_time = current_time
        #data = struct.unpack("Bxxxi", response_data)
        if abs((0.01 * self.reduction_pos * data[1])-self.position)<30: #Check for communication or numerical errors
            self.position = 0.01 * self.reduction_pos * data[1]  # If ok update, otherwise keep previous value
        else:
            self.position = 0.01 * self.reduction_pos * data[1]

    def update_torque_speed_on_response(self, data):
        #data = struct.unpack("Bxxxi", response_data)
        Ki=1
        if abs((0.01*(1/self.reduction_pos)*Ki*data[2])-self.torque)<2: #Check for communication or numerical errors
            self.torque = 0.01*(1/self.reduction_pos)*Ki*data[2]  # If ok update, otherwise keep previous value
        if abs((self.reduction_pos*data[3] )-self.velocity)<1000:
            self.velocity = self.reduction_pos*data[3]  # If ok update, otherwise keep previous value

    def sendTorque(self,tau_pre_reduct):
        Ki=1   # Torque-current gain of the motors
        i=0
        tau_max=9
        tau = tau_pre_reduct
        #tau[-1]=tau[-1]*0.8 # to consider the reduction in the 3DOF
        if abs(tau)>tau_max:
            tau=np.sign(tau)*tau_max     # Saturate at tau_max to avoid limits
        i = (1/Ki)*(self.reduction_pos)*tau # motor gain * reduction axe * value
        #i = 0
        self.canSend(read_multi_angle_command)
        time.sleep(0.001)
        self.canSend(struct.pack("Bxxxhxx",0xA1,int(i*100)))# LSB is 0.01       
        time.sleep(0.002)

    def sendPosition(self,pos,vel_max):
        pos_max = 1000000000000
        if abs(pos)>pos_max:
            pos=np.sign(pos)*pos_max     # Saturate at tau_max to avoid limits
        self.canSend(read_multi_angle_command)
        time.sleep(0.001)
        self.canSend(struct.pack("Bxhi",0xA4,int(vel_max*(1/self.reduction_pos)),int(pos*100*(1/self.reduction_pos)))) # LSB is 0.01
        time.sleep(0.002) 
        
    def sendPosRot(self,rot,pos,vel_max):
        pos_max = 600
        if abs(pos)>pos_max:
            pos=np.sign(pos)*pos_max     # Saturate at tau_max to avoid limits
        self.canSend(read_multi_angle_command)
        time.sleep(0.001)
        self.canSend(struct.pack("BBhi",0xA6, rot, int(vel_max*(1/self.reduction_pos)), int(pos*100*(1/self.reduction_pos)))) # LSB is 0.01
        time.sleep(0.002)
        
    def readVersion(self):
        self.canSend([0xB2, 0, 0, 0, 0, 0, 0, 0])
        time.sleep(0.002)
    
    def update_current_position_on_response(self,data):
        print(data)

    def readMotorStatus(self):
        self.canSend(read_multi_angle_command)
        time.sleep(0.002)
        self.canSend(read_torque_command)
        time.sleep(0.002)

    def sendSpeed(self, speed_dps):
        speedControl = int(speed_dps * 100)  # Convert dps to 0.01dps/LSB
        self.canSend(struct.pack("Bxxxi", 0xA2, speedControl))
        time.sleep(0.002)
        
    def active_reply_function_command(self):
        self.canSend([0xB6, 0x92, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00])
        time.sleep(0.002)
        
    def testExemple(self):
        print('pass')
        self.canSendAll([0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        time.sleep(0.004)

    def process_message(self, message):
        # Process the received message here
        message_type = struct.unpack("Bxxxxxxx", message.data)

        if (message.arbitration_id == 0x240 + self.ID):
            if hex(message_type[0]) == "0x92":
                data = struct.unpack("Bxxxi", message.data)
                self.compteur+=1
                self.update_position_on_response(data)
            if hex(message_type[0]) == "0xa1" or hex(message_type[0]) == "0xa4" or hex(message_type[0]) == "0xa5" or hex(message_type[0]) == "0x9c":
                data = struct.unpack("BBhhh",message.data)
                self.update_torque_speed_on_response(data)
            
            if hex(message_type[0]) == "0xa6":
                data = struct.unpack("BBhhh", message.data)
                self.update_current_position_on_response(data)
                
            if hex(message_type[0]) == "0xb2":
                data = struct.unpack("Bxxxi", message.data)
                print(data[1])
            if hex(message_type[0]) == "0x60":
                data = struct.unpack("Bxxxi", message.data)
                print(data)
                
    def stop_bus(self):
        # Stop the notifier when done
        self.notifier.stop()
        self.bus.shutdown()
        
#

class RobotController():
    def __init__(self, ROBOT, USE_TOOL=False):
        if USE_TOOL:
            from Tool import Tool

        folder_path = os.path.dirname(os.path.abspath(__file__))
        print(folder_path)
        config_path = folder_path+'/robot_config/robot_config.json'
        print(config_path)
        f = open(config_path,)
        data = json.load(f)

        robot_config = {}
        if ROBOT in data:
            robot_config = data[ROBOT]
        else:
            raise ValueError("Unknown robot")

        self.offsets = []
        self.offset_set = False
        self.motors = []
        self.tool = None
        self.param = robot_config['param'] 
        self.q_u_limit = robot_config['q_u_limit']
        self.q_l_limit = robot_config['q_l_limit']

        for p in self.param:
            self.motors.append(MotorController(p))
        self.n = len(self.motors)

        if USE_TOOL and ("tool_init" in robot_config):
            print(robot_config["tool_init"])
            self.tool = Tool(robot_config["tool"], robot_config["tool_init"])

        # Mise à zéro des couples initiaux
        for m in self.motors:
            m.sendTorque(0)

        time.sleep(0.01)

    #
    def init_offset(self):       
        # Lecture des statuts initiaux des moteurs
        for m in self.motors:
            m.readMotorStatus()
            time.sleep(0.01)

        for m in self.motors:
            self.offsets.append(m.position)
        self.offset_set = True

    def set_joint_position(self,q):
        try:
            length = len(q)
        except TypeError:
            length = 1
            q = [q]
        if length != self.n:
            raise ValueError("Wrong command")
        if not self.offset_set:
            raise ValueError("No offset set")
        for i in range(self.n):
            self.motors[i].sendPosition(q[i]+self.offsets[i], self.param[i]['speed'])
        if not self.is_position_ok():
            self.shutdown()
            raise ValueError("Recive command outpass joint limitation")
        
    def set_joint_torque(self,tau):
        try:
            length = len(tau)
        except TypeError:
            length = 1
            tau = [tau]
        if length != self.n:
            raise ValueError("Wrong command")
        if not self.offset_set:
            raise ValueError("No offset set")
        for i in range(self.n):
            self.motors[i].sendTorque(tau[i])
        if not self.is_position_ok():
            self.shutdown()
            raise ValueError("Recive command outpass joint limitation")
        
    def set_joint_velocity(self,vel):
        try:
            length = len(vel)
        except TypeError:
            length = 1
            vel = [vel]
        if length != self.n:
            raise ValueError("Wrong command")
        if not self.offset_set:
            raise ValueError("No offset set")
        for i in range(self.n):
            self.motors[i].sendSpeed(vel[i])
        if not self.is_position_ok():
            self.shutdown()
            raise ValueError("Recive command outpass joint limitation")

    def shutdown(self):
        for m in self.motors:
            m.sendTorque(0)
            m.stop_bus()
        time.sleep(0.01)

    def is_position_ok(self):
        not_ok = []
        for i in range(self.n):
            not_ok.append((self.motors[i].position-self.offsets[i])>self.q_u_limit[i] or (self.motors[i].position-self.offsets[i])<self.q_l_limit[i])
        if True in not_ok:
            print(not_ok)
            return False
        return True
    
    def send_to_init_pose(self):
        for i in range(self.n):
            self.motors[i].sendPosition(self.offsets[i], self.param[i]["speed"])
            time.sleep(0.01)
        time.sleep(2)

    def get_joint_position(self):
        q = []
        for i in range(self.n):
            q.append(self.motors[i].position-self.offsets[i])
        if self.n == 1:
                return q[0]
        else:
            return q
            
    def get_joint_velocity(self):
        dq = []
        for i in range(self.n):
            dq.append(self.motors[i].velocity)
        if self.n == 1:
                return dq[0]
        else:
            return dq
    
    def get_joint_torque(self):
        tau_m = []
        for i in range(self.n):
            tau_m.append(self.motors[i].torque)
        if self.n == 1:
            return tau_m[0]
        else:
            return tau_m