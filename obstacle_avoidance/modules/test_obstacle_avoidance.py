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




if __name__ == "__main__":
    K, dist = ip.read_camera_parameters()

    n = 64 # Broj prvog keyframe-a iskoristenog za triangulaciju
    print("Keyframe " + str(n))
    print("Keyframe " + str(n+1))

    image1 = ip.read_image('/home/nedim/Desktop/Gazebo_files/Camera_frames/keyframe' + str(n) + '.png')
    image2 = ip.read_image('/home/nedim/Desktop/Gazebo_files/Camera_frames/keyframe' + str(n+1) + '.png')
    # image1 = read_image('/home/nedim/Desktop/Gazebo_files/Camera_frames_test/camera_test37.jpeg')
    # image2 = read_image('/home/nedim/Desktop/Gazebo_files/Camera_frames_test/camera_test38.jpeg')

    # detector = cv2.ORB_create(nfeatures=30000, nlevels=8, edgeThreshold=31, firstLevel=0, WTA_K=2, patchSize=31, fastThreshold=20)	
    # detector = cv2.SIFT_create(nOctaveLayers=3 ,contrastThreshold=0.02, edgeThreshold=10, sigma=1.6)
    # detector = cv2.xfeatures2d.SURF_create(hessianThreshold =110, nOctaves = 4, nOctaveLayers = 3)
    detector = None

    pose_global = np.mat([[0], [0], [0]])

    R_camera = np.load('/home/nedim/Desktop/Gazebo_files/Camera_position/R_camera' + str(n+1) + '.npy')
    T_camera = np.load('/home/nedim/Desktop/Gazebo_files/Camera_position/T_camera' + str(n+1) + '.npy')
    (roll, pitch, yaw) = euler_from_matrix(R_camera, 'sxyz')
    print("roll = " + str(math.degrees(roll)))
    print("pitch = " + str(math.degrees(pitch)))
    print("yaw = " + str(math.degrees(yaw)))

    draw_img = image1
    draw_img = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    canny_img = cv2.Canny(draw_img,10,30)
    # t_start = time.time() 
    # print(ip.count_features(image2, detector))
    # t_end = time.time()
    # print("Counting took {0:.3f} seconds".format(t_end - t_start))
    
    t0 = time.time()
    pnts1, pnts2 = ip.find_matches(image1, image2, detector)
    t_end = time.time()
    print("Feature matching ({0}) took {1:.3f} seconds".format(np.shape(pnts1)[0], t_end - t0))

    # print(pnts1)
    # print(pnts2)

    # ip.draw_features(canny_img, pnts1)
    ip.draw_optical_flow(image1, pnts1, pnts2)

    points3D, pnts1_2, pnts2_1, R, T = sfm.triangulate_points(pnts1, pnts2, K, -T_camera, R_camera.T)

    f = open('/home/nedim/Desktop/Gazebo_files/Scale/scale' + str(n+1) + '.txt', 'r')
    scale = float(f.readline())
    f.close()

    print("scale = " + str(scale))
    T = T*scale
    
    # print(np.shape(points3D))
    points3D, mask = sfm.filter_distant_points_3D(points3D*scale, 1.5)
    # pnts1_2, pnts2_1 = sfm.apply_mask(pnts1_2, pnts2_1, mask, 1)
    points3D, mask = sfm.radius_outlier_removal(points3D, 0.15, 1) 
    # pnts1_2, pnts2_1 = sfm.apply_mask(pnts1_2, pnts2_1, mask, 1)

    # R, T, pos, points3D = sfm.bundle_adjustment(sfm.fun, pnts1_2, pnts2_1, points3D, K, R, T)
    
    pose_global_new = T
    
    points3D = (np.dot(R, points3D.T) + T).T
    plotReconstruction3D(points3D, pose_global_new, pose_global)
    
    points2D = oa.transform_to_2D_XZ_coordinate_system(points3D)
    pose2D = oa.transform_to_2D_XZ_coordinate_system(pose_global.T).T
    t1 = time.time()
    # t_delay = t1 - t0
    t_delay = 0


    
    angle, x = oa.avoid_obstacle(points2D, t_delay, 1.1)
    
    if angle is None:
        print("Nema prepreka na putanji")
    else:
        front_limit, left_limit_point, right_limit_point = oa.detect_obstacle(points2D, 1.1)
        print("Prepreka na udaljenosti od {0:.3f} m".format(front_limit))
        print("Procesiranje: {0:.3f} sec, Prazan hod: {1:.3f} m".format(t_delay, (t_delay)*0.3))
        print("Rotiraj robota za " + str(int(angle)) + " stepeni")
        plotReconstruction2D(points2D, pose2D, left_limit_point[0], right_limit_point[0], front_limit, x, angle)




