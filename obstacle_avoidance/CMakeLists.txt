cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES obstacle_avoidance
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/image_listener.py  scripts/pioneer2dx_obstacle_avoidance.py 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

