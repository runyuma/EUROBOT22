#!/usr/bin/python

import actionlib
import roslib
import rospy
roslib.load_manifest('main_controller')
import main_controller.msg
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray,Int32MultiArray,Float64MultiArray
from aruco_detect.msg import aruco_list
import tf
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib.action_client import GoalManager
# from main_controller.msg import movingAction,manipulatorAction,movingActionGoal,manipulatorActionGoal
MOVING = 0
GRABBING = 1

class task():
    def __init__(self,task_mode,args):
        self.task_mode = task_mode
        if task_mode == MOVING:
            self.func = self.robot_moving
            self.args = args
        elif task_mode == GRABBING:
            self.func = self.robot_grabbing
            self.args = args
    
    def robot_moving(self,client,_active_cb,_feedback_cb,_done_cb):
        # args: goal [x,y,theta]
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = self.args[0]
        goal.target_pose.pose.position.y =self.args[1]
        goal.target_pose.pose.orientation.z = np.sin(self.args[2]/2)
        goal.target_pose.pose.orientation.w = np.cos(self.args[2]/2)
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal)
        rospy.loginfo("send navigation goal ")
        #  zijishouxiededaima code
        # args: goal [moving_mode = position(0),x,y,theta]
        # args: goal [moving_mode = velocity(1),vx,vy,dtheta,t]
        # moving_mode = self.args[0]
        # print("moving_mode",moving_mode)
        # goal = main_controller.msg.movingGoal()
        # goal.moving_mode = moving_mode
        # if moving_mode == 0:
        #     goal.goal_pos = Float32MultiArray()
        #     goal.goal_pos.data = self.args[1:]
        #     goal.goal_vel = Float32MultiArray()
        # elif moving_mode == 1:
        #     goal.goal_vel = Float32MultiArray()
        #     goal.goal_vel.data = self.args[1:]
        #     goal.goal_pos = Float32MultiArray()
        # client.send_goal(goal,active_cb=_active_cb,feedback_cb=_feedback_cb,done_cb=_done_cb)

    def robot_grabbing(self,client,_active_cb,_feedback_cb,_done_cb):
        # args: goal [target_color,target_mode,sample_mode] + [position for put] in robot frame
        # self.visual_subs = rospy.Subscriber("aruco_list",aruco_list,self.findsample_callback)
        if self.args[1] == 0:
            data = rospy.wait_for_message("aruco_list",aruco_list,timeout = None)
            aruco_list_len = len(data.aruco_list)

            while aruco_list_len == 0:
                data = rospy.wait_for_message("aruco_list",aruco_list,timeout = None)
                aruco_list_len = len(data.aruco_list)
                print("aruco_nomessage")
            if self.args[0] ==0:
                _tranform = data.aruco_list[0].tf_msg.transforms[0].transform
                m=tf.TransformBroadcaster()
                t = geometry_msgs.msg.TransformStamped()
                t.header.frame_id = 'camera_link'
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = 'sample'
                t.transform = _tranform
                m.sendTransformMessage(t)

            else:
                pass # TODO:color
            cam2sample = matrix_from_tfmsg(_tranform)
            # print("cam2sample",cam2sample)
            base2cam = matrix_from_xyzrpy((0.15,0.00, 0.12, -1.57, 0, -2.09))
            # print("cam2sample",base2cam)
            base2sample = np.matmul(base2cam,cam2sample)
            # print("sample_pose",matrix_to_xyzrpy(base2sample))
            transform_data = matrix_to_xyzrpy(base2sample)
            # print(trans,rot)
            if self.args[2] == 0:
                sample_pos = [transform_data[0],
                            transform_data[1],
                            0.00,
                            0,
                            0,
                            0,
                            1]
            else:
                pass  # TODO: posture
            # print("sample relative pos:",sample_pos)
            goal = main_controller.msg.manipulatorGoal()
            goal.target_value = Float32MultiArray()
            goal.target_value.data =sample_pos
            goal.target_color = self.args[0]
            goal.target_mode = self.args[1]
            goal.sample_mode = self.args[2]
            client.send_goal(goal,active_cb=_active_cb,feedback_cb=_feedback_cb,done_cb=_done_cb)
        elif self.args[1] == 1:
            goal = main_controller.msg.manipulatorGoal()
            goal.target_value = Float32MultiArray()
            goal.target_value.data = self.args[3:]
            goal.target_color = self.args[0]
            goal.target_mode = self.args[1]
            goal.sample_mode = self.args[2]
            client.send_goal(goal,active_cb=_active_cb,feedback_cb=_feedback_cb,done_cb=_done_cb)
        elif self.args[1] == 2:
            goal = main_controller.msg.manipulatorGoal()
            goal.target_value = Float32MultiArray()
            goal.target_value.data = self.args[3:]
            goal.target_color = self.args[0]
            goal.target_mode = self.args[1]
            goal.sample_mode = self.args[2]
            client.send_goal(goal,active_cb=_active_cb,feedback_cb=_feedback_cb,done_cb=_done_cb)
        elif self.args[1] == 3:
            goal = main_controller.msg.manipulatorGoal()
            goal.target_value = Float32MultiArray()
            goal.target_value.data = self.args[3:]
            goal.target_color = self.args[0]
            goal.target_mode = self.args[1]
            goal.sample_mode = self.args[2]
            client.send_goal(goal,active_cb=_active_cb,feedback_cb=_feedback_cb,done_cb=_done_cb)

    
    def is_moving(self):
        if self.task_mode == MOVING:
            return 1
        else:
            return 0 
    def is_grabbing(self):
        if self.task_mode == GRABBING:
            return 1
        else:
            return 0 

    def findsample_callback(self,msg):
        # TODO choose and select data
        _tranform = msg.aruco_list[0].tf_msg.transforms[0].transform 
        self.args = [_tranform.translation.x,
                    _tranform.translation.y,
                    _tranform.translation.z,
                    _tranform.rotation.x,
                    _tranform.rotation.y,
                    _tranform.rotation.z,
                    _tranform.rotation.w]
        print("sample relative pos:",self.args)

def matrix_from_tfmsg(_tranform):
    basetrans = tf.transformations.translation_matrix((_tranform.translation.x,
                                                       _tranform.translation.y,
                                                       _tranform.translation.z))

    baserot = tf.transformations.quaternion_matrix((_tranform.rotation.x,
                                                    _tranform.rotation.y,
                                                    _tranform.rotation.z,
                                                    _tranform.rotation.w))
    combine = np.matmul(basetrans,baserot)
    return combine
def matrix_from_xyzrpy(_tranform):
    x,y,z,rr,rp,ry = _tranform
    basetrans = tf.transformations.translation_matrix((x,
                                                       y,
                                                       z))

    baserot = tf.transformations.euler_matrix(ry,rp,rr)
    combine = np.matmul(basetrans,baserot)
    return combine
def matrix_to_xyzrpy(mat):
    x = mat[0][3]
    y = mat[1][3]
    z = mat[2][3]
    rr,rp,ry = tf.transformations.euler_from_matrix(mat)
    
    return (x,y,z,rr,rp,ry)
    
