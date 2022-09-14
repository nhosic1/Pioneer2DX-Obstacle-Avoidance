import sys
sys.path.insert(0,'/usr/local/lib/python3.8/site-packages')
import numpy as np
import cv2
import image_processing as ip
import obstacle_avoidance as oa
import structure_from_motion as sfm
import transformations as tf
from reconstruction_2D_3D import plotReconstruction2D, plotReconstruction3D
from mpl_toolkits.mplot3d import Axes3D
import time
from tf.transformations import euler_from_matrix
import math

def normalize(v):
    if np.linalg.norm(v) == 0:
        return v
    else:
        return v/np.linalg.norm(v)

if __name__ == "__main__":
    K, dist = ip.read_camera_parameters()

    n = 12 # Broj prvog keyframe-a iskoristenog za triangulaciju
    print("Keyframe " + str(n))
    print("Keyframe " + str(n+1))

    image1 = ip.read_image('/home/nedim/Desktop/Gazebo_files/Camera_frames/keyframe' + str(n) + '.png')
    image2 = ip.read_image('/home/nedim/Desktop/Gazebo_files/Camera_frames/keyframe' + str(n+1) + '.png')
    # image1 = read_image('/home/nedim/Desktop/Gazebo_files/Camera_frames_test/camera_test37.jpeg')
    # image2 = read_image('/home/nedim/Desktop/Gazebo_files/Camera_frames_test/camera_test38.jpeg')

    # detector = cv2.ORB_create(nfeatures=30000, nlevels=8, edgeThreshold=31, firstLevel=0, WTA_K=2, patchSize=31, fastThreshold=13)
    detector = cv2.SIFT_create(nOctaveLayers=3, contrastThreshold=0.02, edgeThreshold=10, sigma=1.6)
    # detector = cv2.xfeatures2d.SURF_create(hessianThreshold = 100, nOctaves = 4, nOctaveLayers = 3)
    # detector = None

    R = np.load('/home/nedim/Desktop/Gazebo_files/Camera_position/R_camera' + str(n+1) + '.npy')
    T = np.load('/home/nedim/Desktop/Gazebo_files/Camera_position/T_camera' + str(n+1) + '.npy')

    # draw_img = image1
    # draw_img = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    # canny_img = cv2.Canny(draw_img,10,30)
    
    t0 = time.time()
    pnts1, pnts2 = ip.find_matches(image1, image2, detector)
    t_end = time.time()
    print("Feature matching ({0}) took {1:.3f} seconds".format(np.shape(pnts1)[0], t_end - t0))

    ip.draw_optical_flow(image1, pnts1, pnts2)

    pnts1, pnts2 = sfm.SED_filter(pnts1, pnts2, K, T, R)
    points3D = sfm.triangulate_points(pnts1, pnts2, K, T, R)
    f = open('/home/nedim/Desktop/Gazebo_files/T_delay/t_delay' + str(n+1) + '.txt', 'r')
    t_delay = float(f.readline())
    f.close()
    
    points3D, mask = sfm.filter_distant_points_3D(points3D, 1.5)

    pose_global = np.mat([[0], [0], [0]])
    pose_global_new = np.dot(-R.T, T)

    points3D = (np.dot(R.T, points3D.T) + np.dot(-R.T, T)).T
    points2D = oa.transform_to_2D_XZ_coordinate_system(points3D)

    pose2D = oa.transform_to_2D_XZ_coordinate_system(pose_global.T).T
    
    pos_new = np.array([[-1.27711975], [-0.00373327], [0.05000086]])
    print("Broj 3D tačaka: " + str(np.shape(points2D)[0]))
    angle, x, tangent_point = oa.avoid_obstacle(points2D, t_delay, 1, 0.15, pos_new)

    plotReconstruction3D(points3D, pose_global, pose_global_new)
     
    if angle is None:
        print("Nema prepreka na putanji")
    else:
        points2D_update = []
        for p in points2D:
            points2D_update.append([p[0], p[1]-t_delay*0.2])
        # print("Prepreka na udaljenosti od {0:.3f} m".format(front_limit))
        print("Procesiranje: {0:.3f} sec, Prazan hod: {1:.3f} m".format(t_delay, (t_delay)*0.2))
        print("Rotiraj robota za " + str(int(angle)) + " stepeni")
        plotReconstruction2D(points2D_update, pose2D, tangent_point, x, angle)




