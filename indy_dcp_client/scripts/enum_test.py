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

class control_command(enum):
    red = enum.auto()
    green = enum.auto()
    hello = enum.auto()

if __name__ == '__main__':
    # Enum enum()
    
    print(control_command.red.values)
    
    print()
    print(control_command)
