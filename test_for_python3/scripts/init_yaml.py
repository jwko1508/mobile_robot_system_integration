#!/usr/bin/env python3

#~/virtualenvironment/test_py3/bin/python3

import rospy

from std_msgs.msg import String

rospy.init_node('issue_yaml')

rate = rospy.Rate(1)
publisher = rospy.Publisher('~counter', String, queue_size=1)

i = 0
while not rospy.is_shutdown():
    publisher.publish(data="hi")
    i += 1
    print("good?")
    rate.sleep()