#!/usr/bin/env python3
#-*- coding: utf-8 -*-

url = 'http://192.168.0.129:8088'
import rospy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

def setboolClient():
    rospy.wait_for_service('getMarkerPose')
    try:
        setbool_Cl = rospy.ServiceProxy('getMarkerPose', SetBool)
        req = SetBoolRequest()
        req.data = 3
        resp1 = setbool_Cl(req)
        print(resp1)
        return resp1.message
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    rospy.init_node('test_to_service_client', anonymous=True)
    try:
        setboolClient()
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()