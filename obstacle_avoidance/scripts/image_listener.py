#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, quaternion_matrix, euler_from_matrix
import modules.transformations as tf
import message_filters
import numpy as np
import math
import time
import cv2

bridge = CvBridge()
counter = 0
pos_old = np.zeros((3,1))
rotation_old = 0
odom_old = None
t_old = time.time()
pointcloud_publisher = rospy.Publisher("/pioneer2dx_pointcloud_topic", PointCloud2, queue_size = 4000)


def form_point_cloud(points):
    header = Header()
    header.frame_id = "odom"
    header.stamp = rospy.Time.now()
    pcl = pcl2.create_cloud_xyz32(header, points)
    return pcl


def callback1(img):
    global t_old
    
    t_new = time.time()
    print("callback1 msg: receiving time = {0:.3f} seconds".format(t_new - t_old))
    t_old = t_new
    
def callback2(odom):
    global X
    global Y
    global brojac
    theta = tf.get_rotation(odom)
    pos_new = np.zeros((3,1))
    pos_new[0] = odom.pose.pose.position.x + math.cos(math.radians(theta))*0.1
    pos_new[1] = odom.pose.pose.position.y + math.sin(math.radians(theta))*0.1
    pos_new[2] = odom.pose.pose.position.z
    X.append(pos_new[0])
    Y.append(pos_new[1])
    print(str(pos_new[0]) + " " + str(pos_new[1]))
    np.save('/home/nedim/Desktop/Gazebo_files/Sim_3/Met_4/X_odom', X)
    np.save('/home/nedim/Desktop/Gazebo_files/Sim_3/Met_4/Y_odom', Y)
    

def callback(img, odom):
    global t_old
    
    t_new = time.time()
    print("callback msg: receiving time = {0:.3f} seconds".format(t_new - t_old))
    t_old = t_new
    
                 
                        
def main():
    rospy.init_node('image_listener')
    print("Program je zapocet")
    # Set up your subscriber and define its callback
    # rospy.Subscriber(image_topic, Image, image_callback)
    # sub1 = rospy.Subscriber('/camera1/image_raw', Image, callback1)
    sub2 = rospy.Subscriber('/odom', Odometry, callback2, queue_size=100)
    # image_sub = message_filters.Subscriber('/camera1/image_raw', Image)
    # odom_sub = message_filters.Subscriber('/odom', Odometry)
    # # The required queue size parameter when constructing the TimeSynchronizer tells it how many sets of messages it should store (by timestamp) while waiting for messages to arrive and complete their "set"
    # ts = message_filters.ApproximateTimeSynchronizer([image_sub, odom_sub], 10, 0.05)
    # ts = message_filters.TimeSynchronizer([image_sub, odom_sub], 10)
    # ts.registerCallback(callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == "__main__":
   brojac = 0
   X = []
   Y = []
   main()

