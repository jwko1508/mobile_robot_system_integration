#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import pickle
import requests
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool,SetBoolRequest,SetBoolResponse
import indyeye_client as indyEYE
from indy_utils import indydcp_client as indyDCP

def WaitFinish(indy_client):
    late = rospy.Rate(100)
    fin = False
    while not fin:
        fin = indy_client.wait_for_move_finish()
        late.sleep()

if __name__ == '__main__':
    rospy.init_node('GetObjectPose')
    loop_rate = rospy.Rate(0.5)
    robotIP = "192.168.1.45"
    eyeIP = robotIP
    name = "NRMK-Indy7"

    indy = indyDCP.IndyDCPClient(robotIP,name)
    eye = indyEYE.IndyEyeClient(robotIP)

    indy.connect()

    indy.set_joint_vel_level(1)
    indy.set_task_vel_level(1)

    # indy.go_zero()
    # WaitFinish(indy)
    indy.go_home()
    WaitFinish(indy)
    # loop_rate.sleep()
    

    task_pos = indy.get_task_pos()
    # print(task_pos)
    print(eye.detect(0, task_pos))
    print(eye.retrieve(0,task_pos))
    print(eye.get_object_dict())
    # print()
    indy.disconnect()
