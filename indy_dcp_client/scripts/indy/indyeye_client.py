import time
import socket
import numpy as np
import sys
import json

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

class IndyEyeClient(object):
    # Run deep learning algorithm / image processing algorithm
    CMD_TASK_DETECT         = 0

    # Pose refinement and post processing
    CMD_TASK_RETRIEVE       = 1

    # Reset detection algorithm
    CMD_TASK_RESET          = 2

    # Request list of detectable object names
    CMD_TASK_GET_LIST       = 3



    # 6D task position to pick object.
    TBE_TASK_POSE_BASE      = 'Tbe'

    # 6D Tool Center Position to pick object.
    TBT_GRIP_POSE_BASE      = 'Tbt'

    # 6D Position of detected object.
    TBO_OBJECT_POSE_BASE    = 'Tbo'

    def __init__(self, eyeIP):
        self.eyeIP = eyeIP
        self.taskserverport = 2002

    def _run_command(self, cmd, cls, pose_cmd = None):
        sock = socket.socket(socket.AF_INET,
                             socket.SOCK_STREAM) # SOCK_STREAM is TCP socket

        try:
            sock.connect((self.eyeIP,self.taskserverport))
            sdict = {'command': int(cmd), 'class_tar': int(cls), }
            if pose_cmd is not None:
                sdict['pose_cmd']= pose_cmd
            sjson = json.dumps(sdict, cls=NumpyEncoder)
            sbuff = sjson.encode()
            sock.send(sbuff)
            # print('sent: ',sjson)

            rbuff = sock.recv(1024)
            rjson = "".join(map(chr, rbuff))
            rdict = json.loads(rjson)
            # print('received: ', rdict)

        finally:
            sock.close()
        return rdict

    def detect(self, cls, task_pos, Tb='Tbe'):
        '''
            cls : object class (use get_object_list to get cls),
            cls = 0 : all,

            task_pos : current task position
        '''
        rdict = self._run_command(cmd=0, cls=cls, pose_cmd=task_pos)
        if rdict['class_detect'] == cls:
            return rdict[Tb]
        else:
            return None

    def detect_by_object_name(self, target_name, task_pos, Tb='Tbe'):
        '''
            target_name : object name (requires name of object from get_object_list),
            task_pos : current task position
        '''
        objs = self.get_object_dict()
        found_target = None
        found_target_idx = -1
        for key in objs.keys():
            if target_name == objs[key]:
                found_target_idx = int(key)
        if found_target_idx != -1:
            result = self._run_command(cmd=0, cls=found_target_idx, pose_cmd=task_pos)
            if result['class_detect'] == found_target_idx:
                found_target = result[Tb]
            else:
                found_target = None
        return found_target
        
    def retrieve(self, cls, task_pos, Tb='Tbe'):
        '''
            cls : object class ( use get_object_list to get cls),
            cls = 0 : all,

            task_pos : current task position
        '''
        rdict = self._run_command(cmd=1, cls=cls, pose_cmd=task_pos)
        if rdict['class_detect'] == cls:
            return rdict[Tb]
        else:
            return None
         
    def get_object_dict(self):
        rdict = self._run_command(cmd=3, cls=0, pose_cmd=None) 
        objlist = rdict['class_list'] 
        objdict = {}
        i = 1
        
        for obj in objlist:
            objdict['{}'.format(i)] = obj
            i = i + 1

        return objdict

        
        
