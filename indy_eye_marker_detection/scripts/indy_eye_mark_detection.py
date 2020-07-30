#!/usr/bin/env python3
#-*- coding: utf-8 -*-

url = 'http://192.168.0.129:8088'
import rospy
import pickle
import requests
session = requests.session()

COMMAND_CUSTOM = '/detect/module/online/{}/{}'

# class GetMarkerPose:
#     def __init__(self):

def get(command, timeout=1, **kwargs):
    resp = session.get(url+command, params=kwargs, timeout=timeout)
    return resp

def put(command, timeout=1, **kwargs):
    resp = session.put(url+command, params=kwargs, timeout=timeout)
    return resp
mids = get(COMMAND_CUSTOM.format('module','id'), timeout=300).json()
print(mids)
module_id = mids[0]
print("online module id: {}".format(module_id))
resp = get(COMMAND_CUSTOM.format(module_id,'pickle'), return_values="all", timeout=300)
rdict = pickle.load(pickle.io.BytesIO(resp.content))
print(rdict.keys())
print(rdict)

if __name__ == '__main__':
    rospy.init_node('GetMarkerPose')
    # tfb = FixedTFBroadcaster()
    rospy.spin()