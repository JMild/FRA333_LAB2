#!/usr/bin/python3

import sys
import rclpy
from rclpy.node import Node
from tank_kinematics.dummy_module import dummy_function, dummy_var
from sensor_msgs.msg import JointState
from tank_kinematics_interfaces.srv import GetPosition
from tank_kinematics_interfaces.srv import SolveIK
import numpy as np
import math

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        self.Publisher = self.create_publisher(JointState,'/joint_states',10)   #create publisher (sent message to topic /joint_state)
        self.rate = 10  #Hz
        timer_period = 1/self.rate  #Sec
        self.timer = self.create_timer(timer_period,self.timer_callback)    #create timer and call function timer_callback
        self.q = [0.,0.,0.] #initial position
        self.srv_FK = self.create_service(GetPosition,'/set_joints',self.set_join_callback) #create service server set_join
        self.srv_IK = self.create_service(SolveIK,'/joint_states',self.set_join_state_callback) #create service server set_join
    
    def tx (self,t_x = float):  #function translation matrix about x axis
        return np.array([[1,0,0,t_x], 
                        [0,1,0,0], 
                        [0,0,1,0],
                        [0,0,0,1]])
    def tz (self,t_z = float):  #function translation matrix about z axis
        return np.array([[1,0,0,0], 
                        [0,1,0,0], 
                        [0,0,1,t_z],
                        [0,0,0,1]])
    def rx (self,r_x = float):  #function rotation matrix about x axis
        return np.array([[1,0,0,0], 
                        [0,np.cos(r_x),np.sin(r_x),0], 
                        [0,-np.sin(r_x),np.cos(r_x),0],
                        [0,0,0,1]])
    def rz (self,r_z = float):  #function rotation matrix about z axis
        return np.array([[np.cos(r_z),-np.sin(r_z),0,0], 
                        [np.sin(r_z),np.cos(r_z),0,0], 
                        [0,0,1,0],
                        [0,0,0,1]])

    def set_join_callback(self,request:GetPosition.Request,response:GetPosition.Response):
        self.q = request.joint.position #request joint position
        type_joint = ([1,1,1])  #0: prismatics joint | 1 :revolute joint
        n = 3   #DOF
        H = np.identity(4)      #indentity 4x4 
        DH = np.array([[0,0,0.03,0], [0.025,np.pi/2,0,0],[0.07,0,0,0]]) #DH Table
        H3e = np.array([[0,1,0,0.04], [0,0,1,0],[1,0,0,0],[0,0,0,1]])
        for i in range (n):
            if type_joint[i] == 1:      #check type joint if it is revolute joint
                Hj = self.rz(self.q[i])     # let Hj is rotation matrix about z axis
            else:       #check type joint if it is prismatics joint
                Hj = self.tz(self.q[i])     # let Hj is translation matrix about z axis
            H = H.dot(self.tx(DH[i][0])).dot(self.rx(DH[i][1])).dot(self.tz(DH[i][2])).dot(self.rz(DH[i][3])).dot(Hj) 
        H0e = H.dot(H3e)    #pose of end-effector
        response.position.x = H0e[0][3]     #position x
        response.position.y = H0e[1][3]     #position y
        response.position.z = H0e[2][3]     #position z
        return response
    
    def set_join_state_callback(self,request:SolveIK.Request,response:SolveIK.Response):
        self.position_IK = request.position
        self.arm_config = request.arm_config

        # xx = self.position_IK[0]
        # yy = self.position_IK[1]
        # zz = self.position_IK[2]
        # gram = self.arm_config
        # h1 = 0.03
        # l1 = 0.025
        # l2 = 0.07
        # l3 = 0.04
        # c3 = (xx**2+yy**2-l2**2-l3**2)/(2*l2*l3)
        # r = xx**2+yy**2
        # for i in range (2):
        #     s_3 = math.sqrt(1-c3**2)*gram[i]
        #     q1 = math.atan2(zz-0.03/gram[i],r-0.025/gram[i])
        #     q2 = math.atan2(zz-0.03,r-0.025)-math.atan2(l3*s_3,l2+l3*c3)
        #     q3 = math.atan2(s_3,c3)

        return response
    
    def timer_callback(self):  
        msg = JointState()      # set type for variable
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = self.q
        self.Publisher.publish(msg) #publish message  

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown() 

if __name__=='__main__':
    main()