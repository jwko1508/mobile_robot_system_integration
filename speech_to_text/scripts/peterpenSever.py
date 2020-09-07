#! /usr/bin/env python
 
from omron_msgs.srv import SetCmd,SetCmdResponse
import rospy
 
def handle_add_two_ints(req):
	print("Returning [num = %d]" %(req.num))
	return SetCmdResponse(1,"good")

def add_two_ints_server():
	rospy.init_node('peterpenServer')
	s = rospy.Service('tts_data', SetCmd, handle_add_two_ints)
	print("Ready to peterpen.")
	rospy.spin()

if __name__ == "__main__":
	add_two_ints_server()