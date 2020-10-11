#! /usr/bin/env python
 
from command_srvs.srv import SetCommand, SetCommandResponse
import rospy
 
def handle_add_two_ints(req):
	print("Returning [num = %d]" %(req.command))
	return SetCommandResponse(1,"good")

def add_two_ints_server():
	rospy.init_node('peterpenServer')
	s = rospy.Service('stt_to_control', SetCommand, handle_add_two_ints)
	print("Ready to peterpen.")
	rospy.spin()

if __name__ == "__main__":
	add_two_ints_server()