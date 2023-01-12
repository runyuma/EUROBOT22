cmake_minimum_required(VERSION 2.8.3)
project(main_controller)

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

add_action_files(DIRECTORY action FILES manipulator.action moving.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)
catkin_package(
  LIBRARIES turn_on_wheeltec_robot
  CATKIN_DEPENDS actionlib_msgs
  DEPENDS system_lib
)

# defind robot status massage
# command atkin_make -DCATKIN_WHITELIST_PACKAGES="new_msg"
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(
    include
    include/${PROJECT_NAME}
)

#add_executable(wheeltec_robot_node src/wheeltec_robot.cpp src/Quaternion_Solution.cpp)
#target_link_libraries(wheeltec_robot_node ${catkin_LIBRARIES})
