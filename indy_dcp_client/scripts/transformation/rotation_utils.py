'''
Created on 2019. 3. 15.

@author: JSK
'''
import numpy as np
from math import *

def rad2deg(rads):
    return rads/np.pi*180
        
def deg2rad(degs):
    return degs/180*np.pi

def Rot_axis( axis, q ):
    '''
    make rotation matrix along axis
    '''
    if axis==1:
        R = np.asarray([[1,0,0],[0,cos(q),-sin(q)],[0,sin(q),cos(q)]])
    if axis==2:
        R = np.asarray([[cos(q),0,sin(q)],[0,1,0],[-sin(q),0,cos(q)]])
    if axis==3:
        R = np.asarray([[cos(q),-sin(q),0],[sin(q),cos(q),0],[0,0,1]])
    return R

def Rot_zyx(zr,yr,xr):
    '''
    zyx rotatio matrix - caution: axis order: z,y,x
    '''
    R = np.matmul(np.matmul(Rot_axis(3,zr),Rot_axis(2,yr)),Rot_axis(1,xr))
    # R = np.matmul((Rot_axis(3,zr),Rot_axis(2,yr)),Rot_axis(1,xr))
    return R

def Rot_zxz(zr1,xr2,zr3):
    '''
    zxz rotatio matrix - caution: axis order: z,x,z
    '''
    R = np.matmul(np.matmul(Rot_axis(3,zr1),Rot_axis(1,xr2)),Rot_axis(3,zr3))
    
    return R

def Rot2zyx(R):
    '''
    rotatio matrix to zyx angles - caution: axis order: z,y,x
    '''
    sy = sqrt(R[0,0]**2 + R[1,0]**2)

    if sy > 0.000001:
        x = atan2(R[2,1] , R[2,2])
        y = atan2(-R[2,0], sy)
        z = atan2(R[1,0], R[0,0])
    else:
        x = atan2(-R[1,2], R[1,1])
        y = atan2(-R[2,0], sy)
        z = 0
    return np.asarray([z,y,x])

def Rot2zxz(R):
    '''
    rotatio matrix to zyx angles - caution: axis order: z,y,x
    '''
    sy = sqrt(R[0,2]**2 + R[1,2]**2)

    if sy > 0.000001:
        z1 = atan2(R[0,2] , -R[1,2])
        x2 = atan2(sy,R[2,2])
        z3 = atan2(R[2,0], R[2,1])
    else:
        z1 = 0
        x2 = atan2(sy,R[2,2])
        z3 = atan2(-R[0,1], R[0,0])
    return np.asarray([z1,x2,z3])

def SE3(R,P):
    T = np.identity(4,dtype='float32')
    T[0:3,0:3]=R
    T[0:3,3]=P
    return T
    
def SE3_inv(T):
    R=T[0:3,0:3].transpose()
    P=-np.matmul(R,T[0:3,3])
    return (SE3(R,P))
    
def SE3_R(T):
    return T[0:3,0:3]
    
def SE3_P(T):
    return T[0:3,3]
   
def SE3_mul_vec3(T,v):
    r=np.matmul(SE3_R(T),v)
    return np.add(r,SE3_P(T))


def align_z(Two):
    Rwo = Two[0:3,0:3]
    Zwo=np.matmul(Rwo,[[0],[0],[1]])
    azim=np.arctan2(Zwo[1],Zwo[0])-np.deg2rad(90)
    altit=np.arctan2(np.linalg.norm(Zwo[:2]),Zwo[2])
    Rwo_=np.matmul(Rot_zxz(azim,altit,-azim),Rwo)
    Two_out = Two.copy()
    Two_out[0:3,0:3] = Rwo_
    return Two_out

def fit_floor(Tcw, Tco, minz):
    Pco = Tco[0:3,3]
    Twc = np.linalg.inv(Tcw)
    Pco_wz = np.dot(Twc[2,0:3],Pco)
    if abs(Pco_wz)<0.00001:
        Pco_wz = 0.00001
    alpha = abs((-minz - Twc[2,3])/Pco_wz)
    Pco_ = Pco*alpha
    Tco_out = Tco.copy()
    Tco_out[0:3,3]=Pco_
    return Tco_out

Tx180 = np.identity(4, 'float32')
Tx180[1,1]=-1
Tx180[2,2]=-1

Ty180 = np.identity(4, 'float32')
Ty180[0,0]=-1
Ty180[2,2]=-1