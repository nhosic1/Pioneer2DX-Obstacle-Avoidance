#!/usr/bin/env python

import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_matrix, euler_matrix, euler_from_matrix

def get_rotation (msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return math.degrees(yaw) 
    
def get_R(msg):
    if msg == None:
        return np.mat([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    R = quaternion_matrix(orientation_list)
    return R[:-1, :-1]
    
def get_camera_R(R):
    (roll, pitch, yaw) = euler_from_matrix(R, 'sxyz')
    R = euler_matrix(pitch, -yaw, -roll, 'sxyz')
    return R[:-1, :-1]
    
def get_camera_T(T):
    t1 = -T[1]
    t2 = -T[2]
    t3 = T[0]
    T = np.array([t1, t2, t3])
    return T
    

def get_relative_T(pos_old, pos_new, R01):
    o = pos_old
    p0 = pos_new
    p1 = - np.dot(R01.T, o) + np.dot(R01.T, p0)
    return p1
  
def get_relative_R(R01, R02):
    R12 = (np.mat(R01).T)*np.mat(R02)
    return R12