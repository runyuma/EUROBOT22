#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import serial
import time
import serial.tools.list_ports
import actionlib
from std_msgs.msg import Float64MultiArray
import geometry_msgs.msg
from aruco_detect.msg import aruco_list
from main_controller.msg import manipulatorFeedback, manipulatorResult,manipulatorAction
import LY_kinematics
import numpy as np
import tf

FREQ_COMMU = 30
SEND_INTERVAL = 4 # 2sec 内只能发送一次数据
NAME_PORT = "/dev/manipulator"
BPS  = 9600 
TEST = 1

MANI_VEL_FAST = 20
MANI_VEL_SLOW = 60
MANI_VEL_MEDD = 40

class manipulator_node(object):
    _feedback = manipulatorFeedback()
    _result = manipulatorResult()

    with_ser = 0

    sample_pos = []
    target_id = 0
    done = 0

    def __init__(self):
        # init node
        rospy.init_node("manipulator")
        # define server
        self.action_server = actionlib.SimpleActionServer('BIG_ROBOT/grabbing',manipulatorAction,execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()
        
        # TODO
        # define visual subscriber
        self.visual_subs = rospy.Subscriber("aruco_list",aruco_list,self.visual_callback)

        # define frequency
        self.rate = rospy.Rate(FREQ_COMMU)
        # init serial port
        port_list = list(serial.tools.list_ports.comports())

        if not TEST:
            if len(port_list) > 0:
                self.ser = serial.Serial(NAME_PORT,BPS,timeout=None)
                print("serial open")
                self.with_ser = 1
            else:
                print("no serial port")
                self.with_ser = 0
        elif TEST:
            if len(port_list) > 1:
                self.ser = serial.Serial(NAME_PORT,BPS,timeout=None)
                print("serial open")
                self.with_ser = 1
            else:
                print("no serial port")
                self.with_ser = 0

        
            
    def write(self, joint_pos,vel_mode):
        # serial write
        message = self.encoding(joint_pos,vel_mode)
        if self.with_ser:
            self.ser.write(message)
            print("serial sent: ", message)
        else:
            print(message)

    def encoding(self, joint_pos,vel_mode):
        # //TODO:protocol 
        # "joint1 joint2 joint3 joint4 joint5 joint6 vel_mode,"
        # vel_mode 50: slow 200:fast
        message = ""
        for i in joint_pos:
            message += str(int(i))
            message += " "
        message+= str(vel_mode) + ','
        message = message.encode()
        return message  

    def execute_cb(self, goal):
        test_number = 0
        message_sent = 0
        rospy.loginfo("serve begin")
        last_time = time.time()
        print("goal: ", goal.target_value.data)
        print("goalmode:",goal.target_mode)
        while not rospy.is_shutdown():
            if not message_sent:
                # pos in robot frame
                if goal.target_mode == 0:
                    pos = goal.target_value.data
                    print("getsample_position: ",pos)
                    self.grab(pos)
                    message_sent = 1
                elif goal.target_mode == 1:
                    pos = goal.target_value.data
                    self.put(pos)
                    message_sent = 1
                    pass
                elif goal.target_mode == 2:
                    joint_value = goal.target_value.data
                    self.write(joint_value,80)
                    time.sleep(3)
                    message_sent = 1
                elif goal.target_mode == 3:
                    time.sleep(10)
                    message_sent = 1
                    # //TODO: push
            self.rate.sleep() 
            test_number += 1
            if test_number % 30 ==0:
                pass
            # print("runing",test_number,"time: ",time.time()-last_time)
            last_time = time.time()
            break
        self._result.done = 1
        self.action_server.set_succeeded(self._result)
        rospy.loginfo("serve done")

    def grab(self,pos):
            print("solution: ",self.get_jointvalue(pos))
            joint_value = [45]+ self.get_jointvalue(pos)[1:]
            # joint_value = [0,0,90,0,90,0]
            print("grabbing")
            self.write(joint_value,MANI_VEL_SLOW)
            time.sleep(15)
            joint_value = [15]+joint_value[1:]
            self.write(joint_value,MANI_VEL_MEDD)
            time.sleep(3)
            joint_value = [15,0,30,30,30,0]
            self.write(joint_value,MANI_VEL_SLOW)
            time.sleep(2)
            

    def put(self,pos):
          # print("solution: ",self.get_jointvalue(pos))
            if len(pos) != 0:
                joint_value = [30]+ self.get_jointvalue(pos)[1:]
                # joint_value = [0,0,90,0,90,0]
                self.write(joint_value,MANI_VEL_SLOW)
                time.sleep(SEND_INTERVAL)
                joint_value = [57]+joint_value[1:]
                self.write(joint_value,MANI_VEL_FAST)
                time.sleep(1.5)
                joint_value = [57,0,30,30,30,0]
                self.write(joint_value,MANI_VEL_MEDD)
                time.sleep(2)
            elif len(pos) == 0:
                joint_value = [57,0,30,30,30,0]
                self.write(joint_value,MANI_VEL_FAST)
                time.sleep(1.5)
            


    def visual_callback(self,msg):
        # TODO
        pass
        # if len(msg.aruco_list) == 0:
        #     self.done = 1
        # else:
        #     _tranform = msg.aruco_list[0].tf_msg.transforms[0].transform
        #     _vec = [_tranform.translation.x,
        #                 _tranform.translation.y,
        #                 _tranform.translation.z,
        #                 _tranform.rotation.x,
        #                 _tranform.rotation.y,
        #                 _tranform.rotation.z,
        #                 _tranform.rotation.w] 
        #     def distance(a, b):
        #         r2 = 0
        #         for i in range(3):
        #             r2 += (a[i] - b[i]) ** 2
        #         return r2 ** 0.5

        #     if self.target_id == 0:
        #         self.done = 0
        #         self.target_id = msg.aruco_list[0].id
        #         self.sample_pos = _vec

        #     #self.color: red = 0, blue = 1, green = 2, brown = 3  
        #     if self.target_id == 47:
        #         self.color = 1
        #     elif self.target_id == 13:
        #         self.color = 2
        #     elif self.target_id == 36:
        #         self.color = 3
        #     elif self.target_id == 17:
        #         self.color = 4

        #     else:
        #         if msg.aruco_list[0].id != self.target_id == 0 or distance(self.sample_pos, _vec) > 10:
        #             # TODO: 万一机械臂遮挡呢
        #             self.done = 1
        #             self.target_id = 0
        #             self.sample_pos = []
    
    def get_jointvalue(self,pos):
        # TODO: pos & camera_manibase_mat
        # pos = [x,y,z,qua] from base_manipulator to sample
        # result [joint_value1,...,joint_value6]
        # inverse kinematic
        
        sample_mat = matrix_from_tf(pos)
        base2arm = matrix_from_xyzrpy((0.05,0.00, 0.20, 0, 0, 0))
        arm_mat = np.matmul(np.linalg.inv(base2arm),sample_mat)
        # print("arm_mat",arm_mat)
        joint_angles = LY_kinematics.new_loop_solution(arm_mat)
        return joint_angles

def matrix_from_tf(tranform):
    x,y,z,qx,qy,qz,qw = tranform
    basetrans = tf.transformations.translation_matrix((x,
                                                       y,
                                                       z))

    baserot = tf.transformations.quaternion_matrix((qx,
                                                    qy,
                                                   qz,
                                                    qw))
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

def boardcast_tf(pos):
    m=tf.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = 'camera_link'
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = 'end_effector'
    t.transform.translation.x = pos[0][3]
    t.transform.translation.y = pos[1][3]
    t.transform.translation.z = pos[2][3]
    q = tf.transformations.quaternion_from_euler(0,-0.24,3.14)
    t.transform.rotation.w=q[3]
    t.transform.rotation.x=q[0]
    t.transform.rotation.y=q[1]
    t.transform.rotation.z=q[2]

    m.sendTransformMessage(t)




if __name__ == "__main__":
    node = manipulator_node()
    rospy.spin()
