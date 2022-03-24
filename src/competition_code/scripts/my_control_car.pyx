
#! /usr/bin/env python

from operator import rshift
import re
from unittest import result
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import cv2
import sys
import _thread

target_x = -1
target_y = -1.7
last = 0.0

path_1 = [[-1,-1.7],[-1,-3.1]]
path_2 = [[-1,-1.7],[-1,-3.0],[-0.2,-3.0]]
path_3 = [[-1,-1.7],[-1,-2.8],[-1.6,-2.8]]
path_4 = [[-1,-1.7],[-1,-0.9],[-2.3,-0.9],[-2.3,-0.1]]
path_5 = [[-1,-1.7],[-1,-0.9],[-2.3,-0.9],[-2.3,0.8],[-2.5,0.8]]
path = [path_1,path_2,path_3,path_4,path_5]
# part of initialize

pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
pub_position = rospy.Publisher("cmd_position",Twist,queue_size=1)
arm_gripper_pub = rospy.Publisher("arm_gripper", Point, queue_size = 1)
arm_position_pub = rospy.Publisher("arm_position", Pose, queue_size = 1)



def arm_up():
        # up
    pose = Pose()
    pose.position.x = 0.21
    pose.position.y = 0.1
    arm_position_pub.publish(pose)
    rospy.sleep(1)

def arm_down():
    # down
    pose = Pose()
    pose.position.x = 0.21
    pose.position.y = -0.02
    arm_position_pub.publish(pose)
    rospy.sleep(1)

def arm_catch():
    point = Point()
    point.x = 1
    arm_gripper_pub.publish(point)
    rospy.sleep(1)

def arm_relax():
    point = Point()
    point.x = 0
    arm_gripper_pub.publish(point)
    rospy.sleep(1)



def go_left():
    # part of control
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.angular.z= 0.2
    pub.publish(twist)

def go_right():
    # part of control
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.angular.z= -0.2
    pub.publish(twist)

def stop(z=0):
    twist = Twist()
    twist.linear.x = z
    twist.linear.y = 0 
    twist.angular.z= 0
    pub.publish(twist) 

