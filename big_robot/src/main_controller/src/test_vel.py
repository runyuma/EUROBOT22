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
        self.command_subs = rospy.Subscriber("/cmd_vel",Twist,self.command_cb)

        # define frequency
        self.rate = rospy.Rate(30)

#       self.current_pos = [0,0,0] #ekf pose
#       self.current_pos_odom = [0,0,0]
        self.current_vel = [0,0,0]
        self.command_vwl = [0,0,0]
        self.dic = {
            "time_index": [],
            "Vx":[],
            "Vy":[],
            "Vz":[],
            "CVx": [],
            "CVy": [],
            "CVz": [],
        }



    def recording(self):
        while not rospy.is_shutdown():
            self.dic["time_index"].append(self.time_index)
            self.dic["Vx"].append(self.current_vel[0])
            self.dic["Vy"].append(self.current_vel[1])
            self.dic["Vz"].append(self.current_vel[2])
            self.dic["CVx"].append(self.command_vwl[0])
            self.dic["CVy"].append(self.command_vwl[1])
            self.dic["CVz"].append(self.command_vwl[2])
            self.time_index += 1
            if self.time_index % 100 == 0:
                df = pd.DataFrame(self.dic)
                print df
                print("recording")
                df.to_csv("../recording/recording.csv")        
            self.rate.sleep()
        


    def odom_cb(self,msg):

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        self.current_vel = [vx,vy,wz]
        # print("ODOM, current pos: ", self.current_pos_odom,"vel : ",self.current_vel)
    def command_cb(self,msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        self.command_vwl = [vx, vy, wz]



if __name__ == "__main__":
    node = moving_node()
    node.recording()
