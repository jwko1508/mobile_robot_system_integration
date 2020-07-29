#!/usr/bin/env python3
#-*- coding: utf-8 -*-

url = 'http://192.168.0.129:8088'
import rospy
import pickle
import requests
# import tf2_msgs.msg
# import tf2_ros
# import geometry_msgs.msg



class GetMarkerPose:
    def __init__(self):
        self.session = requests.session()
        COMMAND_CUSTOM = '/detect/module/online/{}/{}'
        mids = self.get(COMMAND_CUSTOM.format('module','id'), timeout=300).json()
        print(mids)
        if mids == []:
            rospy.INFO("I didn't find the indy-eye ID, Please check the indyeye application.")
            return
        module_id = mids[0]
        print("online module id: {}".format(module_id))
        self.resp = self.get(COMMAND_CUSTOM.format(module_id,'pickle'), return_values="all", timeout=300)
        rdict = pickle.load(pickle.io.BytesIO(self.resp.content))
        print(rdict.keys())

    def get(self, command, timeout=1, **kwargs):
        resp = self.session.get(url+command, params=kwargs, timeout=timeout)
        return resp

    def put(self, command, timeout=1, **kwargs):
        resp = self.session.put(url+command, params=kwargs, timeout=timeout)
        return resp

    def load(self):
        self.rdict = pickle.load(pickle.io.BytesIO(self.resp.content))
        print(self.rdict)

    # def get_homogeneous_matrix(self):
    #     pass

if __name__ == '__main__':
    rospy.init_node('GetMarkerPose')
    getO = GetMarkerPose()
    getO.load()
    rospy.spin()