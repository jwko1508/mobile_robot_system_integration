#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import pickle
import requests
import numpy as np
# from enum import Enum
import enum
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool,SetBoolRequest,SetBoolResponse
from sensor_msgs.msg import JointState
from command_srvs.srv import SetCommand, SetCommandRequest, SetCommandResponse

from indy_utils import indyeye_client as indyEYE
from indy_utils import indydcp_client as indyDCP


if __name__ == '__main__':
    # hi = dict(moveHome = 100)
    command_list = ['moveHome',
                    'moveZero',
                    'moveOrganazation',
                    'drawLine',
                    'getMarkerPose',
                    'Move5cmUpInMarkerPose',
                    'getObjectPose',
                    'indy_success',
                    'indy_fail']
    # print(command_list)
    # print(len(command_list))
    # for i in range(len(dict_test)):
    command_dict = dict(zip(command_list, range(100,100 + len(command_list))))
    print(hi)

    
