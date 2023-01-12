#!/usr/bin/python
# -*- coding: utf-8 -*-
from sympy import limit
import rospy
import time
import actionlib
from main_controller.msg import movingFeedback, movingResult,movingAction
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf
import numpy as np
FREQ_MOV = 30
TEST = 1
MAXVEL = [0.4,0.2,0.5]
MAXACC = [0.5,0.25,0.5]

class moving_node(object):
    _feedback = movingFeedback()
    _result = movingResult()
    done = 0


    def __init__(self):
        # init node
        rospy.init_node("moving")
        # define server
        self.action_server = actionlib.SimpleActionServer('BIG_ROBOT/moving',movingAction,execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()
        
        self.odom_subs = rospy.Subscriber("/odom",Odometry,self.odom_cb)
        self.ekf_subs = rospy.Subscriber("/robot_pose_ekf/odom_combined",PoseWithCovarianceStamped,self.ekf_cb)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # define frequency
        self.rate = rospy.Rate(FREQ_MOV)

        self.current_pos = [0,0,0]
        self.current_pos_odom = [0,0,0]
        self.current_vel = [0,0,0]



    def execute_cb(self, goal):
        if TEST:
            rospy.loginfo("serve begin")
            test_number = 0
            if goal.moving_mode == 0:
                #position mode
                print("position goal: ", goal.goal_pos.data)
                p = np.array([0.5,0.1,0.25])
                d = np.array([0.25,0.1,0.2])
                limit = np.array(MAXVEL)
                self.pd = pid_controller(p,d,limit)
                while not rospy.is_shutdown():
                    target = np.array(goal.goal_pos.data)
                    vel = self.path_planning(target)
                    self.vel_pub.publish(vel)
                    self.rate.sleep() 
            elif goal.moving_mode == 1:
                #velocity mdoe
                print("velocity goal: ", goal.goal_vel.data)
                
                total_time = goal.goal_vel.data[-1]
                _vel = goal.goal_vel.data[:-1]
                last_time = time.time()

                while not rospy.is_shutdown():
                    vel = Twist()
                    vel.angular.z = _vel[2]
                    vel.linear.x = _vel[0]
                    vel.linear.y = _vel[1]
		            # print(vel)
                    self.vel_pub.publish(vel)
                    if time.time() - last_time >= total_time:
                        break
                    self.rate.sleep() 
                vel = Twist()
                vel.angular.z = 0
                vel.linear.x = 0
                vel.linear.y = 0
                self.vel_pub.publish(vel)
                test_number += 1
                if test_number % 30 ==0:
                    pass
			        # print("runing",test_number,"time: ",time.time()-last_time)

                self._result.done = 1
                self.action_server.set_succeeded(self._result)
                rospy.loginfo("serve done")

        elif not TEST:
            pass
    
    def path_planning(self,target):
        # target: target postion [x,y,z]
        # pd control 
        error = np.array(target) - np.array(self.current_pos)
        square = error.dot(error)
        if square >= 0.1:
            vel = self.pd.calculate(np.array(target),np.array(self.current_pos)).tolist()
        else:
            vel = [0,0,0]
        return vel

        


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
        self.current_pos = [x,y,z]
        # print("EKF, current pos: ", self.current_pos)



class pid_controller(object):
    error = np.zeros(3)
    last_error = np.zeros(3)
    def __init__(self,p,d,limit):
        # pd coefficient np array 3
        self.p = p
        self.d = d
        # limit of result np array 3
        self.limit = limit

    def calculate(self,target_pos, current_pos):
        # calculate in world frame
        self.error = target_pos - current_pos
        derror = self.error - self.last_error
        res = self.p * self.error +  self.d * derror
        self.last_error = self.error
        # transfer to robot frame
        mat = tf.transformations.euler_matrix(0,0,-current_pos[2])[0:3,0:3]
        res = np.dot(mat,res)
        #limit
        for i in range(3):
            if abs(res[i])> self.limit[i]:
                res[i] = self.limit[i]* np.sign(res[i])
        return res





if __name__ == "__main__":
    node = moving_node()
    rospy.spin()
