cmake_minimum_required(VERSION 2.8.3)
project(capstone_manipulator)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  nav_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
)

# ulimit -c unlimited
# ssh firefly@192.168.1.76
add_compile_options(-g -std=c++11)    #-g -> core dumped

catkin_package(
  LIBRARIES capstone_manipulator
#  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs tf
  DEPENDS system_lib
)

# defind robot status massage
# command atkin_make -DCATKIN_WHITELIST_PACKAGES="new_msg"
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(
    include
    include/${PROJECT_NAME}
)


