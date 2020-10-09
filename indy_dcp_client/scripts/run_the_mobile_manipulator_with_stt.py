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
from darknet_ros_msgs.msg import ObjectCount

from indy import indyeye_client as indyEYE
from indy import indydcp_client as indyDCP
from transformation import tf

from gripper import control_gripper as dhGRIPPER

from omron_mobile_robot import omron_mobile_driver as Omron

indy_command_list = ['moveHome',
                    'moveZero',
                    'moveOrganazation',
                    'getObject',
                    'putObject',
                    'drawLine',
                    'getMarkerPose',
                    'Move5cmUpInMarkerPose',
                    'getObjectPose',
                    'indy_success',
                    'indy_fail']
indy_command_dict = dict(zip(indy_command_list, range(100,100 + len(indy_command_list))))

speech_command_list = ['안녕',
                        '종료',
                        '멈춰',
                        '출발']
speech_command_dict = dict(zip(speech_command_list, range(1, 1 + len(speech_command_list))))

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
    def __init__(self, robotIP = "192.168.1.3", omronIP = "192.168.1.2"):
        rospy.init_node('control_indy_node')
        self.joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        self.darknet_count_sub = rospy.Subscriber("/darknet_ros/found_object", ObjectCount, self.darknet_CB)
        self.DETECTION_COUNT = 0

        self.stt_server = rospy.Service("stt_to_control", SetCommand, self.SttServerCB)
        
        self.jointLevel = 2
        self.taskLevel = 2
        self.robotIP = robotIP
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

        self.omron = Omron.Omron_mobile_robot(host=omronIP)

        # self.gripper = dhGRIPPER.control_gripper("192.168.1.29", 8888, 1)
        # self.gripper.gripper_initialize()
        # rospy.Rate(0.2).sleep()
        # self.gripper.set_force_of_gripper(20, self.gripper.EXTERNAL_DIRECTION)
        # rospy.Rate(1).sleep()
        # self.gripper.set_force_of_gripper(20, self.gripper.INTERNAL_DIRECTION)
        # rospy.Rate(1).sleep()

    def __del__(self):
        self.indy.disconnect()
        print('disconnect indy7, ip : ' + self.robotIP)
        self.gripper.disconnect()

    def darknet_CB(self, msg = ObjectCount()):
        if (msg.count > 0):
            self.DETECTION_COUNT += 1

    def IsThereObject(self, count_threshold = 1, timeout = 7):
        self.DETECTION_COUNT = 0
        initial_time = rospy.Time.now().to_sec()
        # past_time_for_printing = rospy.Time.now().to_sec() 
        previous_DETECTION_COUNT = 0
        while (rospy.Time.now().to_sec() - initial_time) <= timeout:
        
            elapsed_time = rospy.Time.now().to_sec() - initial_time
            if not (self.DETECTION_COUNT == previous_DETECTION_COUNT):
                print(str(elapsed_time) + "s, self.DETECTION_COUNT : " + str(self.DETECTION_COUNT))
                previous_DETECTION_COUNT = self.DETECTION_COUNT

            if(self.DETECTION_COUNT > count_threshold):
                self.omron.say("물건을 찾았어요.")
                return True

        self.DETECTION_COUNT = 0
        print("I failed to detect white object.")
        self.omron.say("물건을 찾지 못했어요.")
        return True
        

    def SttServerCB(self, req):
        # self.switch('moveHome')
        # req = SetCommandRequest()
    
        if req.command == speech_command_dict['안녕']:
            res = SetCommandResponse()
            rospy.loginfo("I'm moving home.")
            # self.GetObject()
            self.omron.say(what="안녕하세요 반가워요.")
            # self.omron.say(what="오므론입니다")1
            self.indy.set_default_program(10)
            self.indy.start_default_program()
            self.WaitFinish(self.indy)
            # self.WaitFinish(self.indy)
            res.message = 'I moved home position.'
            res.success = True
            return res

        elif req.command == speech_command_dict['종료']:
            res = SetCommandResponse()
            rospy.loginfo("I'm shutting down.")
            
            self.GetObject()
            self.WaitFinish(self.indy)
            res.message = 'I moved zero position.'
            res.success = True
            return res
        elif req.command == speech_command_dict['멈춰']:
            res = SetCommandResponse()
            rospy.loginfo("I'm moving organazation.")
            self.indy.joint_move_to([0, 120, -150, 0, 30, 0])
            self.WaitFinish(self.indy)
            res.message = 'I moved zero position.'
            res.success = True
            return res

        elif req.command == speech_command_dict['출발']:
            res = SetCommandResponse()
            rospy.loginfo("I'm going to get the object")
            self.omron.say("일 번 목표지점으로 출발할게요.")
            # self.omron.say("I a")
            # self.omron.gotogoal(1)
            self.omron.say("일 번 지점에 도착했어요.")
        
            while not self.IsThereObject(count_threshold=5, timeout=6):
                pass

            self.omron.say("이 번 목표지점으로 출발할게요.")
            # self.omron.gotogoal(2)
            self.omron.say("이 번 지점에 도착했어요.")

            # print(222)
            res.message = 'I got the object.'
            res.success = True
            return res

        # elif req.command == speech_command_dict['putObject']:
        #     res = SetCommandResponse()
        #     rospy.loginfo("I'm going to put the object")
        #     self.PutObject()
        #     self.WaitFinish(self.indy)
        #     res.message = 'I put the object.'
        #     res.success = True
        #     return res

        else:
            print('''what is the command? I don't know received command.''')

    def WaitFinish(self, indy_client):
        # rate = rospy.Rate(5)
        fin = False
        while not fin:
            fin = indy_client.wait_for_move_finish()
            rospy.Rate(5).sleep()

    def GetObject(self):   
        self.gripper.set_position(100)
        rospy.Rate(0.5).sleep()
        self.indy.go_home()
        self.WaitFinish(self.indy)
        # self.indy.task_move_by([0, 0, -0.2, 0, 0, 0])
        # self.WaitFinish(self.indy)

        # pose_cur = self.indy.get_task_pos()
        # base_to_marker_xyzrpy = self.eye.detect(1, pose_cur, self.eye.TBO_OBJECT_POSE_BASE)
        # base_to_marker_xyzrpy[2] += 0.35
        # print(base_to_marker_xyzrpy)
        # base_to_marker = tf.p2T(base_to_marker_xyzrpy)

        # marker_to_target_tool_frame_xyzrpy = [-0.112, 0, 0, 180, 0, -90]
        # marker_to_target_tool_frame = tf.p2T(marker_to_target_tool_frame_xyzrpy)
        # print(marker_to_target_tool_frame)
        # base_to_target_tool_frame = np.matmul(base_to_marker, marker_to_target_tool_frame)
        # base_to_target_tool_frame_xyzrpy = tf.T2p(base_to_target_tool_frame)
        # print(base_to_target_tool_frame_xyzrpy)
        # self.indy.go_home()
        # self.WaitFinish(self.indy)

        # for i in range(3,6):
        #     base_to_target_tool_frame_xyzrpy[i] = pose_cur[i]
        # self.indy.task_move_to(base_to_target_tool_frame_xyzrpy)
        # self.WaitFinish(self.indy)
        # self.indy.task_move_by([0, 0, -0.12, 0, 0, 0])
        # self.WaitFinish(self.indy)

        self.gripper.set_position(0)
        rospy.Rate(0.7).sleep()

        # self.indy.task_move_by([0, 0, 0.12, 0, 0, 0])
        # self.WaitFinish(self.indy)
        # self.indy.go_home()
        # self.WaitFinish(self.indy)

    
    def PutObject(self):   
        self.gripper.set_position(0)
        rospy.Rate(0.7).sleep()
        # self.indy.go_home()
        # self.WaitFinish(self.indy)
        # self.indy.task_move_by([0, 0, -0.2, 0, 0, 0])
        # self.WaitFinish(self.indy)

        # pose_cur = self.indy.get_task_pos()
        # base_to_marker_xyzrpy = self.eye.detect(1, pose_cur, self.eye.TBO_OBJECT_POSE_BASE)
        # base_to_marker_xyzrpy[2] += 0.4
        # print(base_to_marker_xyzrpy)
        # base_to_marker = tf.p2T(base_to_marker_xyzrpy)

        # marker_to_target_tool_frame_xyzrpy = [-0.102, 0, 0, 180, 0, -90]
        # marker_to_target_tool_frame = tf.p2T(marker_to_target_tool_frame_xyzrpy)
        # print(marker_to_target_tool_frame)
        # base_to_target_tool_frame = np.matmul(base_to_marker, marker_to_target_tool_frame)
        # base_to_target_tool_frame_xyzrpy = tf.T2p(base_to_target_tool_frame)
        # print(base_to_target_tool_frame_xyzrpy)
        self.indy.go_zero()
        self.WaitFinish(self.indy)

        # for i in range(3,6):
        #     base_to_target_tool_frame_xyzrpy[i] = pose_cur[i]
        # self.indy.task_move_to(base_to_target_tool_frame_xyzrpy)
        # self.WaitFinish(self.indy)
        # self.indy.task_move_by([0, 0, -0.1, 0, 0, 0])
        # self.WaitFinish(self.indy)

        self.gripper.set_position(100)
        rospy.Rate(0.7).sleep()

        # self.indy.task_move_by([0, 0, 0.1, 0, 0, 0])
        # self.WaitFinish(self.indy)
        # self.indy.go_home()
        # self.WaitFinish(self.indy)

    def Test(self):
        self.indy.go_home()
        self.WaitFinish(self.indy)
        # self.indy.task_move_by([0, 0, -0.05, 0, 0, 0])
        # self.WaitFinish(self.indy)

        pose_cur = self.indy.get_task_pos()
        base_to_marker_xyzrpy = self.eye.detect(1, pose_cur, self.eye.TBO_OBJECT_POSE_BASE)
        base_to_marker_xyzrpy[2] += 0.2
        print(base_to_marker_xyzrpy)
        base_to_marker = tf.p2T(base_to_marker_xyzrpy)

        marker_to_target_tool_frame_xyzrpy = [0.30, 0.27, 0, 180, 0, -90]
        marker_to_target_tool_frame = tf.p2T(marker_to_target_tool_frame_xyzrpy)
        print(marker_to_target_tool_frame)
        base_to_target_tool_frame = np.matmul(base_to_marker, marker_to_target_tool_frame)
        base_to_target_tool_frame_xyzrpy = tf.T2p(base_to_target_tool_frame)
        print(base_to_target_tool_frame_xyzrpy)
        print(self.indy.get_task_pos())
        # self.indy.go_home()
        # self.WaitFinish(self.indy)

        # # for i in range(3,6):
        #     # base_to_object_xyzrpy[i] = pose_cur[i]
        # self.indy.task_move_to(base_to_target_tool_frame_xyzrpy)
        # self.WaitFinish(self.indy)
        # self.indy.task_move_by([0, 0, -0.055, 0, 0, 0])
        # self.WaitFinish(self.indy)
        # self.indy.task_move_by([0, 0, 0.055, 0, 0, 0])
        # self.WaitFinish(self.indy)
        # self.indy.go_home()
        # self.WaitFinish(self.indy)



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
            

    def thread_of_PublishCurrentJointState(self):
        t1 = threading.Thread(target = self.PublishCurrentJointState)
        # print(1)
        t1.daemon = True
        t1.start()
        
        

if __name__ == '__main__':
    indy7_and_Indyeye = Indy7_and_Indyeye(robotIP="192.168.1.4", omronIP="192.168.1.2")
    indy7_and_Indyeye.thread_of_PublishCurrentJointState()

    rospy.spin()

    while not rospy.is_shutdown():
        # rospy.loginfo('hello')
        rospy.Rate(10).sleep()
