#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import time
import tf
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pandas as pd
import numpy as np


class moving_node(object):
    time_index = 0
    def __init__(self):
        # init node
        rospy.init_node("test_odom")
        
        self.odom_subs = rospy.Subscriber("/odom",Odometry,self.odom_cb)
        self.ekf_subs = rospy.Subscriber("/robot_pose_ekf/odom_combined",PoseWithCovarianceStamped,self.ekf_cb)
        self.amcl_subs = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.amcl_cb)

        # define frequency
        self.rate = rospy.Rate(30)

        self.current_pos_ekf = [0,0,0] #ekf pose
        self.current_pos_odom = [0,0,0]
        self.current_pos_amcl= [0,0,0]
        self.current_vel = [0,0,0] 
        self.dic = {
            "time_index": [],
            "ekf_x":[],
            "ekf_y":[],
            "ekf_z":[],
            "odom_x":[],
            "odom_y":[],
            "odom_z":[],
            "amcl_x":[],
            "amcl_y":[],
            "amcl_z":[],
        }



    def recording(self):
        while not rospy.is_shutdown():
            self.dic["time_index"].append(self.time_index)
            self.dic["ekf_x"].append(self.current_pos_ekf[0])
            self.dic["ekf_y"].append(self.current_pos_ekf[1])
            self.dic["ekf_z"].append(self.current_pos_ekf[2])
            self.dic["odom_x"].append(self.current_pos_odom[0])
            self.dic["odom_y"].append(self.current_pos_odom[1])
            self.dic["odom_z"].append(self.current_pos_odom[2])
            self.dic["amcl_x"].append(self.current_pos_amcl[0])
            self.dic["amcl_y"].append(self.current_pos_amcl[1])
            self.dic["amcl_z"].append(self.current_pos_amcl[2])
            self.time_index += 1
            if self.time_index% 300 ==0:
                df = pd.DataFrame(self.dic)
                df.to_csv("../recording/localize_recording2.csv")        
            self.rate.sleep() 
    

        


    def odom_cb(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w))[2]
        self.current_pos_odom = [x,y,z]

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        self.current_vel = [vx,vy,wz]
        # print("ODOM, current pos: ", self.current_pos_odom,"vel : ",self.current_vel)
        
    def ekf_cb(self,msg):
        # 2D position x,y
        # 1D orientation z
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w))[2]
        self.current_pos_ekf = [x,y,z]
        # print("EKF, current pos: ", self.current_pos)
    
    def amcl_cb(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w))[2]
        self.current_pos_amcl = [x,y,z]



if __name__ == "__main__":
    node = moving_node()
    node.recording()
