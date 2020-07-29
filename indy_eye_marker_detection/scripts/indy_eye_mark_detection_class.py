#!/usr/bin/env python3
#-*- coding: utf-8 -*-

url = 'http://192.168.0.129:8088'
import rospy
import pickle
import requests
import numpy as np
from std_msgs.msg import Float64MultiArray


class GetMarkerPose:
    def __init__(self):
        self.marker_pub = rospy.Publisher('MarkerHomogeneousMatrix', Float64MultiArray, queue_size=10)
        self.session = requests.session()
        self.COMMAND_CUSTOM = '/detect/module/online/{}/{}'
        
        

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
        self.rdict = pickle.load(pickle.io.BytesIO(self.resp.content))
        # print(self.rdict.keys())
        self.rrdict = self.rdict["batch"]
        self.object_pose_base_2nd_list = self.rrdict[0]['object_pose_base']
        # std_msgs.msg.Float64MultiArray
    def FromTwoDimensionalToOneDimensionList(self):
        if self.object_pose_base_2nd_list == []:
            return
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
    getO.load()
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        getO.FromTwoDimensionalToOneDimensionList()
        getO.publishTopic()
        rate.sleep()