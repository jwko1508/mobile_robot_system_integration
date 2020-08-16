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
            cls : object class ( use get_object_list to get cls),
            cls = 0 : all,

            task_pos : current task position
        '''
        rdict = self._run_command(cmd=0, cls=cls, pose_cmd=task_pos)
        if rdict['STATE'] == 0:
            return list(rdict[Tb])
        else:
            return None
        
    def retrieve(self, cls, task_pos, Tb='Tbe'):
        '''
            cls : object class ( use get_object_list to get cls),
            cls = 0 : all,

            task_pos : current task position
        '''
        rdict = self._run_command(cmd=1, cls=cls, pose_cmd=task_pos)
        if rdict['STATE'] == 0:
            return list(rdict[Tb])
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

        
        
