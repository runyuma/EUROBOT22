#!/usr/bin/python2
# -*- coding: utf-8 -*-
import rospy
import time
# from move_base_msgs.msg import MoveBaseAction
import roslib
roslib.load_manifest('main_controller')
import actionlib
import main_controller.msg
from std_msgs.msg import Float32MultiArray,Int32MultiArray
# from main_controller.msg import movingAction,manipulatorAction,movingActionGoal,manipulatorActionGoal
import robot_task
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib.action_client import GoalManager
FREQ_MAIN = 30
TEST_GRAB = 0
TEST_MOVE = 0

class BIG_ROBOT():
    def __init__(self):
        rospy.init_node("BIG_ROBOT")
        self.rate = rospy.Rate(FREQ_MAIN)
        self.task_stack = [] 
	    # 任务堆栈
        self.schdular_available = 1 
	    # 是否启用任务调度,
        # self.moving_client = actionlib.SimpleActionClient('BIG_ROBOT/moving', main_controller.msg.movingAction)
        self.moving_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.grabbing_client = actionlib.SimpleActionClient('BIG_ROBOT/grabbing', main_controller.msg.manipulatorAction)
    
    def task_schdular(self):
        # achieve task scheular
        # return the task and put it into the task_stack
        # 伪代码
        # if self.schdular_available = 1：task_schdular()
        # else: pass
        pass
    
    def taskfinish_monitor(self):
        # judge finish or not 
        # remove task in task stack
        # set schdular_available
        if len(self.task_stack) == 0:
            print("no task")
            self.schdular_available = 1
        else:
	    # task_state client.get_state() working:1 done:3
            
            if self.task_stack[0].is_moving():
                task_state = self.moving_client.get_state()
                print("task_state",task_state)
            elif self.task_stack[0].is_grabbing():
                task_state = self.grabbing_client.get_state()
	       

            if task_state == 3:
                self.schdular_available = 1
                time.sleep(0.6)
                self.task_stack = self.task_stack[1:]
            elif task_state == 1:
                self.schdular_available = 0


    def do_task(self):
        if len(self.task_stack ) > 0:
            if self.schdular_available == 1:
                self.schdular_available = 0
                print("set_task")
                if self.task_stack[0].is_moving():
                    self.task_stack[0].func(self.moving_client,self.moving_active_cb,self.moving_feedback_cb,self.moving_done_cb)
                elif self.task_stack[0].is_grabbing():
                    self.task_stack[0].func(self.grabbing_client,self.grabbing_active_cb,self.grabbing_feedback_cb,self.grabbing_done_cb)
    
    def init_task(self):
        # replica
        # _task1 = robot_task.task(robot_task.MOVING,[0.12,-0.9,-1.8])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,40,0,30,30,30,10])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,40,0,30,30,0,10])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.MOVING,[0.12,-0.85,-2.335])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,40,0,-10,70,30,10])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,10,0,-10,70,30,10])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,10,0,-0,30,0,10])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.MOVING,[0.12,0.4,2.35])
        # self.task_stack.append(_task1)
        # blue
        # _task1 = robot_task.task(robot_task.MOVING,[0.16,0.37,2.335])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.MOVING,[0.32,0.38,-0.35])
        # self.task_stack.append(_task1)
    
        # _task3 = robot_task.task(robot_task.GRABBING,[0,0,0])
        # self.task_stack.append(_task3)
        # _task1 = robot_task.task(robot_task.MOVING,[0.32,0.38,1.57])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.MOVING,[0.20,0.44,1.57])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,15,0,20,30,40,10])
        # self.task_stack.append(_task1)
        # bule
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,40,0,20,30,40,10])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.MOVING,[0.35,0.40,1.57])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.MOVING,[0.3,0.3,-0.4])
        # self.task_stack.append(_task1)
        # _task3 = robot_task.task(robot_task.GRABBING,[0,0,0])
        # self.task_stack.append(_task3)
        # _task1 = robot_task.task(robot_task.MOVING,[0.45,0.33,1.57])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.MOVING,[0.5,0.45,1.57])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,15,0,20,30,40,10])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,40,0,20,30,40,10])
        # self.task_stack.append(_task1)
        # green
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,40,0,20,30,40,10])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.MOVING,[0.6,0.38,1.57])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.MOVING,[0.6,0.35,-1.57])
        # self.task_stack.append(_task1)
        # _task3 = robot_task.task(robot_task.GRABBING,[0,0,0])
        # self.task_stack.append(_task3)
        # _task1 = robot_task.task(robot_task.MOVING,[0.8,0.38,1.57])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,15,0,20,30,40,10])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,40,0,20,30,40,10])
        # self.task_stack.append(_task1)
        _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,40,0,20,30,40,10])
        self.task_stack.append(_task1)
        _task1 = robot_task.task(robot_task.MOVING,[0.7,0.38,1.57])
        self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.MOVING,[0.1,-0.4,3.14])
        # self.task_stack.append(_task1)
        # _task3 = robot_task.task(robot_task.GRABBING,[0,3,0])
        # self.task_stack.append(_task3)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,45,0,-30,40,90,10])
        # self.task_stack.append(_task1)
        # _task3 = robot_task.task(robot_task.GRABBING,[0,3,0])
        # self.task_stack.append(_task3)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,20,0,-30,40,90,10])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,20,0,0,40,40,10])
        # self.task_stack.append(_task1)
        _task1 = robot_task.task(robot_task.MOVING,[0.05,0,1.57])
        self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.GRABBING,[0,2,0,40,0,0,40,40,10])
        # self.task_stack.append(_task1)




        # _task1 = robot_task.task(robot_task.MOVING,[0.85,-0.2,1.57])
        # self.task_stack.append(_task1)
        # _task1 = robot_task.task(robot_task.MOVING,[0.85,0.3,1.57])
        # self.task_stack.append(_task1)




        # exhibit gallery
        # _task1 = robot_task.task(robot_task.MOVING,[0.61,-0.55,0])
        # self.task_stack.append(_task1)
        # _task2 = robot_task.task(robot_task.MOVING,[0.61,-0.35,1.57])
        # self.task_stack.append(_task2)
        # _task3 = robot_task.task(robot_task.GRABBING,[0,0,0])
        # self.task_stack.append(_task3)
        # _task4 = robot_task.task(robot_task.MOVING,[0.15,-0.,3.14])
        # self.task_stack.append(_task4)
        # _task5 = robot_task.task(robot_task.GRABBING,[0,1,0])
        # self.task_stack.append(_task5)


    def main(self):
        # 1.self.taskfinish_monitor()监测任务状态，查看是否有新的任务
        # 2.任务规划
        # 3.根据不同的任务执行不同的ros action
        print("main controller on ")
        # self.moving_client.wait_for_server()
        # self.grabbing_client.wait_for_server()
        print("client on")
        if (not TEST_GRAB) and (not TEST_MOVE):
            self.moving_client.wait_for_server()
            self.grabbing_client.wait_for_server()
            self.init_task()
        elif TEST_GRAB:
            _task1 = robot_task.task(robot_task.GRABBING,[0,0,0])
            # _task2 = robot_task.task(robot_task.GRABBING,[10,0,60,90,0,0])
            # _task3 = robot_task.task(robot_task.GRABBING,[0,0,0,0,0,0])
            self.task_stack.append(_task1)
            # self.task_stack.append(_task2)
            # self.task_stack.append(_task3)
            self.grabbing_client.wait_for_server()
        elif TEST_MOVE:
            _task1 = robot_task.task(robot_task.MOVING,[0.8,-0.55,2.356])
            self.task_stack.append(_task1)
            _task2 = robot_task.task(robot_task.MOVING,[0.85,-0.25,2.1])
            self.task_stack.append(_task2)
            self.moving_client.wait_for_server()
        while not rospy.is_shutdown():
            self.taskfinish_monitor() # 监测任务状态
            self.task_schdular() #实现任务调度
            self.do_task()
            self.rate.sleep() 


    def moving_active_cb(self):
        print("moving_active")
    def moving_feedback_cb(self, feedback):
        print("get_feedback")
    def moving_done_cb(self,state,result):
        print("get_done")
    def grabbing_active_cb(self):
        print("grabbing_active")
    def grabbing_feedback_cb(self, feedback):
        print("get_feedback", feedback.relative_pos)
    def grabbing_done_cb(self,state,result):
        print("get_done,done = ", result.done, "state = ",state)

		
if __name__ == "__main__":
    robot = BIG_ROBOT()
    robot.main()
