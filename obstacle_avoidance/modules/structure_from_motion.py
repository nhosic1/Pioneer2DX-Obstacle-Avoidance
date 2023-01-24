#!/usr/bin/env python

import numpy as np
import cv2
from scipy.optimize import least_squares
from scipy.spatial import KDTree

def correct_M(point1, point2, K1, K2):
    I = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])

    pnts3D = cv2.triangulatePoints(K1*I, K2, point1, point2)
    pnts3D = np.divide(pnts3D, pnts3D[3])
    if pnts3D[2] < 0:
        return False
    else:
        return True

def compute_M(U, s, V, point1, point2, K):
    W = np.matrix([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    R = np.mat(U) * W * np.mat(V)
    T = np.mat(U[:, 2])

    M = np.dot(K, np.append(R, T, axis=1))

    if not correct_M(point1, point2, K, M):

        T = - np.mat(U[:, 2])
        M = np.dot(K, np.append(R, T, axis=1))

        if not correct_M(point1, point2, K, M):

            R = U.dot(W.T).dot(V)
            T = np.mat(U[:, 2])
            M = np.dot(K, np.append(R, T, axis=1))

            if not correct_M(point1, point2, K, M):

                T = - np.mat(U[:, 2])
                M = np.dot(K, np.append(R, T, axis=1))
    return M, R, T

def filter_distant_points_3D(pnts3D, distance_limit):
    pnts3D_new = np.empty((0,3), float)
    mask = np.zeros((np.shape(pnts3D)[0],1))
    i = 0
    for pnt in pnts3D:
        if pnt[1] < 0.195 and pnt[2] > 0 and pnt[2] < distance_limit:
            pnts3D_new = np.vstack((pnts3D_new, pnt))
            mask[i] = 1
        i += 1
    return pnts3D_new, mask

def radius_outlier_removal(pnts3D, radius, n):
    mask = np.zeros((np.shape(pnts3D)[0],1))
    if(pnts3D.shape[0] == 0):
        return pnts3D, mask
    pnts3D =np.unique(pnts3D, axis=0)
    T = KDTree(pnts3D)
    pnts3D_new = np.empty((0,3), float)
    i = 0
    for pnt in pnts3D:
        neighbors1 = T.query_ball_point(pnt, r=radius)
        if np.shape(neighbors1)[0] > n:
            pnts3D_new = np.vstack((pnts3D_new, pnt))
            mask[i] = 1
        i += 1

    return pnts3D_new, mask

def filter_outliers(inliers, pnts):
    pnts_new = np.empty((0,len(pnts[0])), float)
    for i in inliers:
        for j, pnt in enumerate(pnts):
            if i == j:
                pnts_new = np.vstack((pnts_new, pnt))
    return pnts_new

def apply_mask(pnts1, pnts2, mask, inlier_flag):
    p1 = []
    p2 = []
    for i, e in enumerate(mask):
        if e == inlier_flag:
            p1.append(pnts1[i])
            p2.append(pnts2[i])
    return p1, p2

def skew(x):
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])

def SED_filter(pnts1, pnts2, K, T, R):
    E = np.dot(skew(T), R)
    F = np.dot(np.linalg.inv(K).T, np.dot(E, np.linalg.inv(K)))

    mask = np.zeros(len(pnts1))
    for i in range(len(pnts1)):
        p1h = np.append(pnts1[i], np.ones(1))
        p2h = np.append(pnts2[i], np.ones(1))
        l1 = np.matmul(F, p2h)
        l2 = np.matmul(F.T, p1h)
        d1 = abs(np.dot(l1, p1h))/np.linalg.norm(l1[:2])
        d2 = abs(np.dot(l2, p2h))/np.linalg.norm(l2[:2])
        SEU = d1**2 + d2**2
        if SEU < 0.6:
            mask[i] = 1
    
    p1, p2 = apply_mask(pnts1, pnts2, mask, 1)  

    return p1, p2

def triangulate_points(pnts1, pnts2, K, T, R):
    p1 = np.array(pnts1).T
    p2 = np.array(pnts2).T

    M = np.dot(K, np.append(R.T, np.dot(-R.T, T), axis=1))

    I = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])

    pnts3D = cv2.triangulatePoints(K*I, M, p1, p2)

    for i, pnt in enumerate(pnts3D):
        pnts3D[i] = np.divide(pnt, pnts3D[3])

    return pnts3D[:3].T


def extend_structure(pnts3D_old, pnts3D_new, A):
    if np.shape(pnts3D_old)[0] == 0:
        return pnts3D_new[:3].T
    pnts3D_new = np.dot(A, pnts3D_new)
    pnts3D_extended = np.array(np.vstack((pnts3D_old, pnts3D_new[:3].T)))
    pnts3D_extended =np.unique(pnts3D_extended, axis=0)
    return pnts3D_extended

def bundle_adjustment(fun, p1, p2, pnts3D, K, R, T):
    R_vector, _ = cv2.Rodrigues(R)
    variables0 = np.concatenate((R_vector, T, pnts3D), axis=None)
    res = least_squares(fun, variables0, method='lm', ftol=1e-2, max_nfev=1000, verbose=0, args=(p1, p2, K))
    R_new,_ = cv2.Rodrigues(res.x[0:3])
    T_new = (res.x[3:6]).reshape(3,1)
    pos_new = T_new
    pnts3D_new = res.x[6:len(res.x)]
    pnts3D_new = pnts3D_new.reshape(int(len(pnts3D_new)/3),3)
    return R_new, T_new, pos_new, pnts3D_new

def fun(variables, p1, p2, K):
    dist_coeffs = np.zeros((4,1))
    R = variables[0:3]
    T = variables[3:6]
    pnts3D = variables[6:len(variables)]
    pnts3D = np.array(pnts3D.reshape(int(len(pnts3D)/3), 3))
    P1_proj, _ = cv2.projectPoints(pnts3D, np.zeros((3, 1)), np.zeros((3, 1)), K, dist_coeffs)
    P2_proj, _ = cv2.projectPoints(pnts3D, R, T, K, dist_coeffs)
    P1_proj = P1_proj.reshape(len(P1_proj), 2)
    P2_proj = P2_proj.reshape(len(P2_proj), 2)
    P_proj = np.concatenate((P1_proj, P2_proj), axis=None)
    p = np.concatenate((p1, p2), axis=None)
    return p - P_proj


