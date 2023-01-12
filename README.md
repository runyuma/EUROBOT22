# BSc thesis: EUROBOT competition 2022
![image](img/Bsc_robot.jpg)
![image](img/Bsc_robot2.jpg)
# Dependencies
- [ROS Navigation Stack](http://wiki.ros.org/navigation)
- [Robot Pose EKF](http://wiki.ros.org/robot_pose_ekf)
# Contribution
- pkg capstone_manipulator/ & aruco_detect/: a grabbing system based on QR code detection
![image](img/Bsc_visual.jpg)
- pkg main_controller/: define working flows of competition, using ROS::ACTION to define and achieve manipulating and moving tasks
- deployed Extended kalman filter localization, AMCL particle localization on this robot.
![image](img/Bsc_costmap.jpg)
- deployed Dijkstra (global planner) and TEB(local planner) algorthim to plan trajectory.
![image](img/Bsc_navigation.jpg)
- Overall control system design
![image](img/Bsc_controlsystem.jpg)