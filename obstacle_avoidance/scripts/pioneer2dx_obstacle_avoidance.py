#!/usr/bin/env python

import sys
sys.path.insert(0,'/usr/local/lib/python3.8/site-packages')

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_matrix
import message_filters
import numpy as np
import cv2
import time
import math
import modules.image_processing as ip
import modules.obstacle_avoidance as oa
import modules.structure_from_motion as sfm
import modules.transformations as tf

def ndarray_to_pointcloud2(points):
    header = Header()
    header.frame_id = "odom"
    header.stamp = rospy.Time.now()
    pcl = pcl2.create_cloud_xyz32(header, points)
    return pcl

def move_robot(linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = angular_z
        twist_pub.publish(twist)
        

def callback(img, odom):
    global detector
    global flag
    global counter
    global pos_old
    global K
    global angle
    global frame_old
    global z_average
    global pointcloud_publisher
    global rotation_old
    global odom_old
    global t_old
    global linear_vel
    global angular_vel
    global brojac
   
    rotation_new = tf.get_rotation(odom)
    pos_new = np.zeros((3,1))
    pos_new[0] = odom.pose.pose.position.x
    pos_new[1] = odom.pose.pose.position.y
    pos_new[2] = odom.pose.pose.position.z
    T_length = np.linalg.norm(pos_new - pos_old)
    
    if T_length > 0.08 and flag != 2:
        R01 = tf.get_R(odom_old)
        R02 = tf.get_R(odom)
        T12 = tf.get_relative_T(pos_old, pos_new, R01)
        T_camera = tf.get_camera_T(T12)
        R_camera = tf.get_camera_R(R01, R02)
        cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
        (roll, pitch, yaw) = euler_from_matrix(R_camera, 'sxyz')
        if abs(math.degrees(roll)) > 0.2 or abs(math.degrees(pitch)) > 0.2 or abs(math.degrees(yaw)) > 0.2:
            flag = 0
            rospy.loginfo("Camera is shaking!")
        
        if flag == 0:
 
                rospy.loginfo("Keyframe " + str(counter))
                cv2.imwrite('/home/nedim/Desktop/Gazebo_files/Camera_frames/keyframe' + str(counter) + '.png', cv2_img)
                frame_old = cv2_img
                pos_old = pos_new
                odom_old = odom
                flag = 1
                counter += 1
                           
        elif flag == 1:
        
                rospy.loginfo("Keyframe " + str(counter))
                cv2.imwrite('/home/nedim/Desktop/Gazebo_files/Camera_frames/keyframe' + str(counter) + '.png', cv2_img)
                frame_new = cv2_img
                t0 = time.time()
                pnts1, pnts2 = ip.find_matches(frame_old, frame_new, detector)
                t01 = time.time()
                rospy.loginfo("Feature matching ({0}) took {1:.3f} seconds".format(np.shape(pnts1)[0], t01 - t0))
                
                T = np.array(T_camera)
                R = np.array(R_camera)
                
                if np.shape(pnts1)[0] > 0:
                        pnts1, pnts2 = sfm.SED_filter(pnts1, pnts2, K, T, R)
                  
             
                if np.shape(pnts1)[0] > 0:
                
                        np.save('/home/nedim/Desktop/Gazebo_files/Camera_position/T_camera' + str(counter), T_camera)
                        np.save('/home/nedim/Desktop/Gazebo_files/Camera_position/R_camera' + str(counter), R_camera)
                        
                        points3D = sfm.triangulate_points(pnts1, pnts2, K, T, R)
                        points3D, mask = sfm.filter_distant_points_3D(points3D, 1.5)
                        points3D = (np.dot(R.T, points3D.T) + np.dot(-R.T, T)).T
                        
                        points2D = oa.transform_to_2D_XZ_coordinate_system(np.array(points3D))
                        print("Broj 3D tačaka: " + str(np.shape(points2D)[0]))
                        print("pos = ")
                        print(pos_new)
                        
                        t1 = time.time()
                        t_delay = t1 - t0
                        f = open('/home/nedim/Desktop/Gazebo_files/T_delay/t_delay' + str(counter) + '.txt', 'w+')
                        f.write(str(t_delay))
                        f.close()
                        angle, _, _ = oa.avoid_obstacle(points2D, t_delay, 1, linear_vel, pos_new)
                        
                        rospy.loginfo("Processing: {0:.3f} sec, Distance travelled: {1:.3f} m".format(t_delay, (t_delay)*linear_vel))
                        
                        if angle is None:
                            rospy.loginfo("No obstacles in the way")
                            angle = 0
                            flag = 0
                        else:
                            # rospy.loginfo("Prepreka na udaljenosti od {0:.3f} m".format(front_limit))
                            rospy.loginfo("OBSTACLE! Rotate the robot by " + str(int(angle)) + " degrees")
                            move_robot(0, angular_vel*np.sign(angle))
                            flag = 2
                            rotation_old = rotation_new
                            brojac = brojac + 1
                            rospy.loginfo("Prepreka broj " + str(brojac))

                        # pointcloud = ndarray_to_pointcloud2((points3D_new)[:-1, :].T.tolist())
                        # pointcloud_pub.publish(pointcloud)

                else:
                      rospy.loginfo("Not enough matching features: " + str(np.shape(pnts1)[0]))
                      flag = 0 
                      
                counter += 1  
                                          
    if flag == 2:
    
           rotation_rel = abs(rotation_new - rotation_old)
           if rotation_rel > 180:
                rotation_rel = 360 - abs(rotation_new - rotation_old)
           if rotation_rel > abs(angle)-5:
                rospy.loginfo("Keep moving forward")
                move_robot(linear_vel, 0)  
                pos_old = pos_new
                odom_old = odom 
                flag = 0      
        
def main():
    global linear_vel
    rospy.init_node('obstacle_avoidance')
    rospy.loginfo("node started")
    rospy.sleep(2.)
    move_robot(linear_vel, 0)
    image_sub = message_filters.Subscriber('/camera1/image_raw', Image)
    odom_sub = message_filters.Subscriber('/odom', Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, odom_sub], 10, 0.05) # 0.05
    ts.registerCallback(callback)
    rospy.spin()
    

if __name__ == "__main__":
    bridge = CvBridge()
    # detector = cv2.ORB_create(nfeatures=30000, nlevels=8, edgeThreshold=31, firstLevel=0, WTA_K=2, patchSize=31, fastThreshold=13)	
    # detector = cv2.SIFT_create(nOctaveLayers=3, contrastThreshold=0.02, edgeThreshold=10, sigma=1.6)
    # detector = cv2.xfeatures2d.SURF_create(hessianThreshold = 100, nOctaves = 4, nOctaveLayers = 3)
    detector = None
    linear_vel = 0.15
    angular_vel = 0.3
    flag = 0
    counter = 0
    brojac = 0
    pos_old = np.zeros((3,1))
    rotation_old = 0
    odom_old = None
    t_old = time.time()
    K, dist = ip.read_camera_parameters()
    pointcloud_pub = rospy.Publisher("/pioneer2dx_pointcloud_topic", PointCloud2, queue_size = 4000)
    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    main()
