#!/usr/bin/env python3
#-*- coding: utf-8 -*-

url = 'http://192.168.0.129:8088'
import rospy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool,SetBoolResponse



class TestToPublishList:
    def __init__(self):
        self.pub = rospy.Publisher('list_topic', Float64MultiArray, queue_size=10)
        self.server = rospy.Service('service_exam', SetBool, self.serverCB)
        rate = rospy.Rate(10)
        rospy.spin()
        # while not rospy.is_shutdown():
        #     my_list = [[1, 0, 0, 4], [0, 1, 0, 3], [0, 0, 1, 2], [0, 0, 0, 1]]
        #     answer = sum(my_list, [])
        #     print(my_list)
        #     rospy.loginfo(answer)
        #     list_msgs = Float64MultiArray()
        #     list_msgs.data = answer
        #     self.pub.publish(list_msgs)
        #     rate.sleep()

    def serverCB(self, req):
        print("Returning [%s]"%(req))
        hi = SetBoolResponse()
        hi.message = 'hello'
        hi.success = True
        return hi



if __name__ == '__main__':
    rospy.init_node('test_to_publish_list', anonymous=True)
    try:
        TestToPublishList()
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()