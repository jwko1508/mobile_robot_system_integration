#!/usr/bin/env python3
#-*- coding: utf-8 -*-

url = 'http://192.168.0.129:8088'
import rospy
import pickle
import requests
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool,SetBoolRequest,SetBoolResponse

class GetMarkerPose:
    def __init__(self):
        self.object_pose_base_1st_list = [0 for i in range(16)]
        self.object_pose_base_2nd_list = [0 for i in range(16)]
        self.isThereMarker = False

        self.marker_pub = rospy.Publisher('MarkerHomogeneousMatrix', Float64MultiArray, queue_size=10)
        self.command_server = rospy.Service('getMarkerPose', SetBool, self.getMarkerPoseServiceCB)
        self.session = requests.session()
        self.COMMAND_CUSTOM = '/detect/module/online/{}/{}'

    def getMarkerPoseServiceCB(self, req):
        self.load()
        res = SetBoolResponse()
        if self.isThereMarker:
            self.FromTwoDimensionalToOneDimensionList()
            self.publishTopic()
            res.message = "Successfully got the marker pose."
            res.success = True
            return res
        else:
            res.message = "Marker pose estimation failed."
            res.success = False
            return res

    def get(self, command, timeout=1, **kwargs):
        resp = self.session.get(url+command, params=kwargs, timeout=timeout)
        return resp

    def put(self, command, timeout=1, **kwargs):
        resp = self.session.put(url+command, params=kwargs, timeout=timeout)
        return resp

    def load(self):
        mids = self.get(self.COMMAND_CUSTOM.format('module','id'), timeout=300).json()
        print(mids)
        if mids == []:
            rospy.INFO("I didn't find the indy-eye ID, Please check the indyeye application.")
            return
        module_id = mids[0]
        print("online module id: {}".format(module_id))

        self.resp = self.get(self.COMMAND_CUSTOM.format(module_id,'pickle'), return_values="all", timeout=300)
        try:
            self.isThereMarker = True
            self.rdict = pickle.load(pickle.io.BytesIO(self.resp.content))
        except Exception as e:
            print('An exception occurred.', e)
            self.isThereMarker = False
            return

        # print(self.rdict.keys())
        self.rrdict = self.rdict["batch"]
        self.object_pose_base_2nd_list = self.rrdict[0]['object_pose_base']
        # print(self.object_pose_base_2nd_list)

    def FromTwoDimensionalToOneDimensionList(self):
        # print(self.object_pose_base_2nd_list.type())
        # self.oneDimentionList = sum(self.object_pose_base_2nd_list, [])
        self.object_pose_base_1st_list = np.ravel(self.object_pose_base_2nd_list, order='C')
        
    def publishTopic(self):
        list_msgs = Float64MultiArray()
        list_msgs.data = self.object_pose_base_1st_list
        self.marker_pub.publish(list_msgs)

if __name__ == '__main__':
    rospy.init_node('GetMarkerPose')
    getO = GetMarkerPose()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        getO.FromTwoDimensionalToOneDimensionList()
        getO.publishTopic()
        rate.sleep()

    rospy.spin()