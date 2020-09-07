#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import pickle
import requests
import numpy as np
import threading
import math
from enum import Enum
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool,SetBoolRequest,SetBoolResponse
from sensor_msgs.msg import JointState
from command_srvs.srv import SetCommand, SetCommandRequest, SetCommandResponse
# from sensor_msgs.msg import JointState

from indy import indyeye_client as indyEYE
from indy import indydcp_client as indyDCP
from transformation import tf

from gripper import control_gripper as dhGRIPPER

indy_command_list = ['moveHome',
                    'moveZero',
                    'moveOrganazation',
                    'getObject',
                    'drawLine',
                    'getMarkerPose',
                    'Move5cmUpInMarkerPose',
                    'getObjectPose',
                    'indy_success',
                    'indy_fail']
indy_command_dict = dict(zip(indy_command_list, range(100,100 + len(indy_command_list))))

# from indy_eye_mark_detection.srv import 
# from 
# from geometry_msgs.msg import PoseWithCovarianceStamped
# import tf2_ros

def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs

def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads

class Indy7_and_Indyeye:
    def __init__(self):
        rospy.init_node('control_indy_node')
        self.joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        self.server_indy7 = rospy.Service("control_indy7", SetCommand, self.IndyServerCB)
        
        self.jointLevel = 1
        self.taskLevel = 1
        self.robotIP = "192.168.1.3"
        self.robotName = "NRMK-Indy7"

        self.indy = indyDCP.IndyDCPClient(self.robotIP, self.robotName)
        self.eye = indyEYE.IndyEyeClient(self.robotIP)

        self.indy.connect()
        print('connect indy7, ip : ' + self.robotIP)
        self.indy.set_joint_vel_level(self.jointLevel)
        self.indy.set_task_vel_level(self.taskLevel)
        
        self.indyJointState = JointState()
        joint_names = []
        for i in range(0,6):
            joint_name = "joint"
            joint_name += str(i)
            joint_names.append(joint_name)
        self.indyJointState.header.frame_id = self.robotName
        self.indyJointState.name = joint_names

        # self.gripper = dhGRIPPER.control_gripper("192.168.1.29", 8888, 1)

    def __del__(self):
        self.indy.disconnect()
        print('disconnect indy7, ip : ' + self.robotIP)

    def switch(self, x):
        print(indy_command_dict.get(x, 'default'))

    def IndyServerCB(self, req):
        # self.switch('moveHome')
        # req = SetCommandRequest()
    
        if req.command == indy_command_dict['moveHome']:
            res = SetCommandResponse()
            rospy.loginfo("I'm moving home.")
            self.indy.go_home()
            self.WaitFinish(self.indy)
            res.message = 'I moved home position.'
            res.success = True
            return res

        elif req.command == indy_command_dict['moveZero']:
            res = SetCommandResponse()
            rospy.loginfo("I'm moving zero.")
            self.indy.go_zero()
            self.WaitFinish(self.indy)
            res.message = 'I moved zero position.'
            res.success = True
            return res
        elif req.command == indy_command_dict['moveOrganazation']:
            res = SetCommandResponse()
            rospy.loginfo("I'm moving organazation.")
            self.indy.joint_move_to([0, -120, 150, 0, -150, 0])
            self.WaitFinish(self.indy)
            res.message = 'I moved zero position.'
            res.success = True
            return res

        elif req.command == indy_command_dict['getObject']:
            res = SetCommandResponse()
            rospy.loginfo("I'm going to get the object")
            self.GetObject()
            self.WaitFinish(self.indy)
            res.message = 'I got the object.'
            res.success = True
            return res

        else:
            print('''what is the command? I don't know received command.''')

    def WaitFinish(self, indy_client):
        # rate = rospy.Rate(5)
        fin = False
        while not fin:
            fin = indy_client.wait_for_move_finish()
            rospy.Rate(5).sleep()

    def GetObject(self):   
        # self.indy.go_zero()
        # self.WaitFinish(self.indy)
        self.indy.go_home()
        self.WaitFinish(self.indy)
        # print(q)
        self.indy.task_move_to([-0.559357421259722, -0.1688319283906048, -0.00022749122549529943, 
                                -0.009210278120808605, 179.93159775690592, 0.08028557317740079])
        self.WaitFinish(self.indy)

        pose_cur = self.indy.get_task_pos()
        base_to_marker_xyzrpy = self.eye.detect(1, pose_cur, self.eye.TBO_OBJECT_POSE_BASE)
        base_to_marker_xyzrpy[2] += 0.1
        print(base_to_marker_xyzrpy)
        base_to_marker = tf.p2T(base_to_marker_xyzrpy)

        marker_to_object_xyzrpy = [0.05, 0, 0, 0, 0, 0]
        marker_to_object = tf.p2T(marker_to_object_xyzrpy)
        print(marker_to_object)
        base_to_object = np.matmul(base_to_marker, marker_to_object)
        base_to_object_xyzrpy = tf.T2p(base_to_object)
        print(base_to_object_xyzrpy)
        self.indy.go_home()
        self.WaitFinish(self.indy)

        for i in range(3,6):
            base_to_object_xyzrpy[i] = pose_cur[i]
        self.indy.task_move_to(base_to_object_xyzrpy)
        self.WaitFinish(self.indy)


    def PublishCurrentJointState(self):
        while not rospy.is_shutdown():
            q = self.indy.get_joint_pos()
            q_rad = degs2rads(q)
            self.indyJointState.position = q_rad

            # q_dot = self.indy.get_joint_vel()
            # q_dot_rad = degs2rads(q_dot)
            # self.indyJointState.velocity = q_dot_rad

            # j_torque = self.indy.get_control_torque()
            # self.indyJointState.effort = j_torque

            self.indyJointState.header.stamp = rospy.Time.now()
            self.joint_state_pub.publish(self.indyJointState)
            

    def thread_run(self):
        t1 = threading.Thread(target = self.PublishCurrentJointState)
        # print(1)
        t1.daemon = True
        t1.start()
        
        

if __name__ == '__main__':
    indy7_and_Indyeye = Indy7_and_Indyeye()
    # rate1 = rospy.Rate(1)
    # indy7_and_Indyeye.doTask()
    # print('do task')
    # t1 = threading.Thread(target=)
    indy7_and_Indyeye.thread_run()
    # indy7_and_Indyeye.doTask()
    rospy.spin()

    # to sustain daemon threading
    while not rospy.is_shutdown():
        rospy.Rate(1).sleep()

    # print(indy_command_dict['moveHome'])
    # indy7_and_Indyeye.switch('moveHome')
