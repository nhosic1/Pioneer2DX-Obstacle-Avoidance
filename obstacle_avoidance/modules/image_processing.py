#!/usr/bin/env python

import numpy as np
import cv2
import yaml


def read_camera_parameters():
    K = None
    dist = None

    file_path = "/home/nedim/Desktop/Gazebo_files/calibration_matrix_gazebo.yaml"
    with open(file_path) as file:
        camera = yaml.safe_load(file)
        K = np.array(camera['camera_matrix'])
        dist = np.array(camera['dist_coeff'])
        return K, dist

def read_image(image_path):
    image = cv2.imread(image_path)
    return image

def undistort_image(image, K, dist):
    h, w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
        K, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(image, K, dist, None, newcameramtx)
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    return dst

def downsample_image(image, K):
    shp = np.shape(image)
    scale = 1.0
    nPixels = shp[0]*shp[1]
    width = int(shp[1] * scale)
    height = int(shp[0] * scale)
    dim = (width, height)

    if nPixels > 8000000:
        scale = float(8000000) / nPixels
        width = int(shp[1] * scale)
        height = int(shp[0] * scale)
        dim = (width, height)
        # K[0]*=scale
        # K[1]*=scale

    image_new = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    return image_new, K

def FLANN_matcher(kp1, kp2, des1, des2, detector):
    pnts1 = []
    pnts2 = []
    if detector.getDefaultName() == 'Feature2D.ORB':
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm = FLANN_INDEX_LSH,
                    table_number = 6, 
                    key_size = 12,     
                    multi_probe_level = 1) 
    else:
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=120)
    search_params = {}
    flann = cv2.FlannBasedMatcher(index_params,search_params)
    matches = flann.knnMatch(des1, des2, k=2)

    good = []

    if len(matches) == 0:
        return [], []
    
    for values in matches:
        if len(values) == 1:
            m = values[0]
        elif len(values) == 2:
            (m,n) = values
            if m.distance < 0.5*n.distance:
                if kp2[m.trainIdx].pt not in pnts2:
                    good.append(values)
                    pnts2.append(kp2[m.trainIdx].pt)
                    pnts1.append(kp1[m.queryIdx].pt)
                else:
                    index = pnts2.index(kp2[m.trainIdx].pt)
                    if good[index][0].distance/good[index][1].distance > m.distance/n.distance:
                        pnts1[index] = kp1[m.queryIdx].pt

    d = abs(np.array(pnts1)-np.array(pnts2)).reshape(-1, 2).max(-1)
    mask = d >= 8 # U simulaciji 4 je ovde vrijednost 8
    
    pnts1 = np.array(pnts1)[mask].reshape(-1, 2)
    pnts2 = np.array(pnts2)[mask].reshape(-1, 2)

    return pnts1, pnts2

def BF_matcher(kp1, kp2, des1, des2):

    # bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    # matches = bf.match(des1,des2)
    # matches = sorted(matches, key = lambda x:x.distance)
    # pnts1 = [kp1[matches[i].queryIdx].pt for i in range(len(matches))]
    # pnts2 = [kp2[matches[i].trainIdx].pt for i in range(len(matches))]

    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1,des2, k=2)
    pnts1 = []
    pnts2 = []
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.5*n.distance:
            if kp2[m.trainIdx].pt not in pnts2:
                pnts2.append(kp2[m.trainIdx].pt)
                pnts1.append(kp1[m.queryIdx].pt)

    return pnts1, pnts2

def KLT_algorithm(old_image, new_image, mask):
    feature_params = dict( maxCorners = 3000,
                       qualityLevel = 0.6,  
                       minDistance = 10,  
                       blockSize = 3 )   
    lk_params = dict( winSize  = (25,25),
                    maxLevel = 4,   
                    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 12, 0.03)) 
    
    if len(old_image.shape) == 3:
          old_gray = cv2.cvtColor(old_image, cv2.COLOR_BGR2GRAY)
          frame_gray = cv2.cvtColor(new_image, cv2.COLOR_BGR2GRAY)
    else:
          old_gray = old_image
          frame_gray = new_image

    p0 = cv2.goodFeaturesToTrack(old_gray, mask = mask, **feature_params)
    if p0 is None:
        return [], []
    
    # draw_features(old_gray, p0.reshape(len(p0),2))
    p1, st1, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
    p0_reverse, st0, err = cv2.calcOpticalFlowPyrLK(frame_gray, old_gray, p1, None, **lk_params)
    # draw_features(frame_gray, p1.reshape(len(p1),2))

    d = abs(p0-p0_reverse).reshape(-1, 2).max(-1)
    good = d < 1 
    for i in range(len(good)):
        if abs(p1[i] - p0[i]).max() < 6 or st1[i] == 0:
            good[i] = False
    
    good_new = p1[good].reshape(-1, 2)
    good_old = p0[good].reshape(-1, 2)
    
    return good_old, good_new

def find_matches(frame1, frame2, detector):
    frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    if detector == None:
        frame1 = cv2.Canny(frame1,10,30)
        frame2 = cv2.Canny(frame2,10,30)
        pnts1, pnts2 = KLT_algorithm(frame1, frame2, None)
    else:
        # frame1 = cv2.Canny(frame1,10,30)
        # frame2 = cv2.Canny(frame2,10,30)
        kp1, des1 = detector.detectAndCompute(frame1, None)
        kp2, des2 = detector.detectAndCompute(frame2, None)
        if np.shape(kp1)[0] < 3 or np.shape(kp2)[0] < 3:
            return [], []
        pnts1, pnts2 = FLANN_matcher(kp1, kp2, des1, des2, detector)
        # pnts1, pnts2 = BF_matcher(kp1, kp2, des1, des2)

        # img1_kp = cv2.drawKeypoints(frame1, kp1, None, color=(255,0,0))
        # cv2.imshow('image1kp', img1_kp)
        # cv2.waitKey(0)

    return pnts1, pnts2

def count_features(image, detector):
    n = 0
    if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if detector == None:
        feature_params = dict( maxCorners = 3000,
                       qualityLevel = 0.35,
                       minDistance = 15,
                       blockSize = 3 )       
        p0 = cv2.goodFeaturesToTrack(image, mask = None, **feature_params)
        if p0 is not None:
            n = np.shape(p0)[0] 
        else: 
            n = 0
    else:
        kp, des = detector.detectAndCompute(image, None)
        n = np.shape(kp)[0]
    return n

def draw_optical_flow(old_image, good_old, good_new):
    color = np.random.randint(0,255,(5000,3))
    mask = np.zeros_like(old_image)

    for i,(new,old) in enumerate(zip(good_new, good_old)):
        a,b = np.array(new).ravel()
        c,d = np.array(old).ravel()
        
        mask = cv2.line(mask, (int(a),int(b)),(int(c),int(d)), color[i].tolist(), 2)
        frame = cv2.circle(old_image,(int(a),int(b)),5,color[i].tolist(),-1)
    
    # Fokus ekspanzije
    # frame = cv2.circle(old_image,(int(old_image.shape[1]/2),int(old_image.shape[0]/2)),10,color[i].tolist(),-1)
    img = cv2.add(frame,mask)

    cv2.imshow('frame', img)
    cv2.waitKey(0)

def draw_features(image, pnts):
    
    if(len(image.shape)==2):
        image = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)

    for pnt in pnts:
        x = int(pnt[0])
        y = int(pnt[1])
        cv2.circle(image,(x,y), 5, (255,0,0), 2)

    cv2.imshow('Features',image)
    cv2.waitKey(0)
