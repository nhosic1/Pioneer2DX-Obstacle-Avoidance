# Pioneer2DX-Obstacle-Avoidance
## Overview

The purpose of this project is the application of computer vision methods on data from monocular camera and odometry from robot's differential drive, with intent of recognizing nearby obstacles and avoiding them.

Two main methods are used for finding matching features from corresponding pairs of successive camera images. First method starts with detecting features on both images separately using detectors such as ORB, SIFT and SURF. Feature matching is done with an appropriate search algorithm, depending on the selected detector. Second method is finding features from the first image using Shi-Thomasi detector, and then finding matching features from the second image using Lucas-Kanade optical flow algorithm. The feature pairs are filtered in the end based on epipolar constraint. 

From the 2D feature pairs, 3D points are calculated using linear triangulation method. For this step, it is necessary to know the change of position and orientation between two images, which is obtained from odometry data. Based on obstacle points in 3D space, a new robot trajectory is determined using the proposed algorithm.

The project is written and organized as a ROS(Robot Operating System) package and tested in Gazebo simulator. Results from the mentioned methods are compared and analyzed to find their respective advantages.

## Simulation demo:


https://user-images.githubusercontent.com/44199537/186067690-7aed0bbe-86ff-4846-944e-2115105d5e93.mp4

