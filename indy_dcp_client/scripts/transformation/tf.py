import numpy as np
import math
from . import rotation_utils as utils


def p2T(p):
    xyz = p[:3] 
    uvw = [math.radians(p[i]) for i  in range(3, 6)]
    T = np.identity(4)
    T[:3, :3] = utils.Rot_zyx(uvw[2], uvw[1], uvw[0])
    T[:3, 3] = xyz[:]
    
    return T

def T2p(T):
    xyzuvw = [0, 0, 0, 0, 0, 0]
    xyzuvw[:3] = T[:3, 3]
    wvu_radian = list(utils.Rot2zyx(T[:3, :3]))

    xyzuvw[3:] = [math.degrees(wvu_radian[i]) for i in range(2, -1, -1)]

    return xyzuvw




























# 