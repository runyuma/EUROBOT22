cmake_minimum_required(VERSION 3.0.2)
project(aruco_detect)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation tf)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Declare ROS messages and services
add_message_files(DIRECTORY msg FILES aruco_info.msg aruco_list.msg)

# Generate added messages and services
generate_messages(DEPENDENCIES std_msgs tf)

# Declare a catkin package
catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

# Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(aruco_detect src/aruco_detect.cpp)
target_link_libraries(aruco_detect ${catkin_LIBRARIES})
target_link_libraries(aruco_detect ${OpenCV_LIBS})
add_dependencies(aruco_detect aruco_detect_generate_messages_cpp)


