# Pioneer2DX-Obstacle-Avoidance
## Overview

Purpose of this project is application of computer vision methods on data from monocular camera images, in combination with odometry data from robot's differential drive, with intent of recognizing nearby obstacles and avoiding them.

Two methods are used for finding matching features from corresponding pairs of successive camera images. First method is detecting features on both images using detectors such as ORB, SIFT and SURF. 
Feature matching is done with Flann matcher algorithm. Second method is finding features from first image using Schi-Thomasi detector, and then finding matching features from second image using Lucas-Kanade optical flow algorithm. 
Results from these two methods are compared and analyzed to find their respective advantages.

After finding mathcing 2D features, 3D points are calculated using linear triangulation method in combination with RANSAC algorithm for filtering outliers. Distance between two camera images from odometry data is used to determine real scale of 3D points.

Based on obstacle points in 3D space, new robot trajectory is determined using proposed algorithm.

Project is written and organized as a ROS(Robot Operating System) package and tested in Gazebo simulator.

