#! /usr/bin/env python
from lib2to3.pgen2.token import LESS
from time import sleep
from numpy import rate
import rospy
# from competion_code.srv import Goal,GoalRequest,GoalResponse
import _thread

import numpy as np
import cv2
# from utils import ARUCO_DICT
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import sys

from operator import rshift
import re
from unittest import result
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


from my_redmarkerdetection import *    # image processing by cython
from my_control_car import *    # image processing by cython

status = 0
point_0=[0.2,1.6,0]
point_1=[0.7,3.1,3.14]
point_2=[0.25,2.8,-1.6]
point_3=[2.0,2.7,0]
point_4=[2.2,0.2,3.14]
point_5=[2.6,-0.8,0]
point_6=[0.9,1.7,0]
points = [point_1,point_2,point_3,point_4,point_5]


pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
pub_position = rospy.Publisher("cmd_position",Twist,queue_size=1)
arm_gripper_pub = rospy.Publisher("arm_gripper", Point, queue_size = 1)
arm_position_pub = rospy.Publisher("arm_position", Pose, queue_size = 1)

class visual_status:
    def __init__(self):
        self.flag = 0
        self.sorted_ID = [2,1,0]
        self.count = 0
        #pid 
        self.temp = 0
        self.integral=0
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        load_template()

    def left_move(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0.1
        twist.angular.z= 0
        pub.publish(twist)

    def right_move(self):
    # part of control
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = -0.2
        twist.angular.z= 0
        pub.publish(twist)

    def pid(self,target,now,last):
        err=target-now
        self.integral+=err
        kp=2.5
        ki=0
        kd=1.2
        result=kp*err+kd*(err-last)+ki*(self.integral)
        
        if result>0:
            if result>0.5:
                return 0.5
            else:
                return result*0.5+0.1
        else:
            if result<-0.5:
                return -0.5
            else:
                return result*0.5-0.1


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as err:
            print(err)
        
        (h, w, channel) = cv_image.shape

        seg_papram = np.array([0,15,125,180,46,80],dtype="uint8")
        field_top = -0.15

        if(self.flag==0):
            return

        if(self.flag==1):
            self.sorted_ID = sort_number_label(cv_image,seg_papram)
            print(self.sorted_ID)
            self.flag=0
            index = self.sorted_ID[self.count]
            next_point = points[index]
            print("the next point is ",next_point)
            print("the next cub to get is:",index+1)
            response = client.call(x=next_point[0],y=next_point[1])
            rospy.loginfo("response from move_system is %f ",response.result)
        
        elif(self.flag==2):
            id_list,tvec_list,rvec_list = marker_detection(cv_image,seg_papram)
            # cv2.imshow('frame', cv_image)
            cv2.waitKey(1)
            for i in range(len(id_list)):
                tvec = tvec_list[i]
                #y = tvec[1]
                if id_list[i] == 5 and self.count == 0:
                    #if y > field_top :
                    x = tvec[0]
                    z = tvec[2]
                        
                elif id_list[i] == 6 and self.count == 1:
                    #if y > field_top :
                    x = tvec[0]
                    z = tvec[2]
                        
                elif id_list[i] == 7 and self.count == 2:
                    #if y > field_top :
                    x = tvec[0]
                    z = tvec[2]
            if abs(z)<0.10:
                stop()
                arm_down()
                arm_relax()
                arm_up()
                self.count+=1
                self.flag=0
                cv2.destroyAllWindows()
                #rospy.sleep(5)
                if(self.count<3):
                    index = self.sorted_ID[self.count]
                    next_point = points[index]
                    print(next_point)
                    print("the next cub to get is:",index+1)
                    response = client.call(x=next_point[0],y=next_point[1])
                    rospy.loginfo("response from move_system is %f ",response.result)
                else:
                    print("********* task finished! ***********")
            elif abs(x)<0.02:
                print("go")
                stop(z)
            
            else:
                twist = Twist()
                val_y=self.pid(0.01,x,self.temp)
                twist.linear.y = val_y
                self.temp=0-x  
                pub.publish(twist)

        elif(self.flag==3):
            id_list,tvec_list,rvec_list = marker_detection(cv_image,seg_papram)
            # cv2.imshow('frame', cv_image)
            cv2.waitKey(1)
            target_cub = self.sorted_ID[self.count]
            #print(target_cub)
            for i in range(len(id_list)):
                tvec = tvec_list[i]
                if (id_list[i] == 0 and target_cub == 0):
                    x = tvec[0]
                    z = tvec[2]
                if (id_list[i] == 1 and target_cub == 1):
                    x = tvec[0]
                    z = tvec[2]
                if (id_list[i] == 2 and target_cub == 2):
                    x = tvec[0]
                    z = tvec[2]
                if (id_list[i] == 3 and target_cub == 3):
                    x = tvec[0]
                    z = tvec[2]
                if (id_list[i] == 4 and target_cub == 4):
                    x = tvec[0]
                    z = tvec[2]
                if id_list[i] == 6 :
                    #if y > field_top :
                    x = tvec[0]
                    z = tvec[2]

            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.angular.z= 0
            pub.publish(twist)
            
            
            if abs(z)<0.15:
                twist.linear.x = 0
                twist.linear.y = 0
                twist.angular.z= 0
                pub.publish(twist)
                arm_down()
                arm_catch()
                rospy.sleep(1)
                arm_catch()
                arm_up()
                cv2.destroyAllWindows()
                self.flag=0
                print(point_6)
                print("go to the exchange point")
                response = client.call(x=point_6[0],y=point_6[1])
                rospy.loginfo("response from move_system is %f ",response.result)

            # elif x>0:
            #     twist.linear.x = 0.12
            #     twist.linear.y = -0.2
            #     twist.angular.z= 0
            # elif x<0:
            #     twist.linear.x = 0.12
            #     twist.linear.y = 0.2
            #     twist.angular.z= 0

            if abs(x)<0.01:
                twist.linear.x = 0.2
                twist.linear.y = 0
                twist.angular.z= 0
            else:
                val_y=self.pid(0,x,self.temp)
                twist.linear.y = val_y
                self.temp=0-x 
                print("pid = %f",val_y)          
            print("x = %f",x)
            
            pub.publish(twist)

def doReq(req):
    global status
    sum = req.x + req.y
    rospy.loginfo("now point is: x = %f,y = %f ",req.x, req.y)
    if abs(point_0[0]-req.x)<0.1 and abs(point_0[1]-req.y)<0.1:
        rospy.loginfo("This is begin point")
        # put relative func at here
        status = 1

    elif abs(point_6[0]-req.x)<0.1 and abs(point_6[1]-req.y)<0.1:
        rospy.loginfo("This is exchange point")
        # put relative func at here
        status = 2

    for index, poi in enumerate(points):
        if abs(poi[0]-req.x)<0.1 and abs(poi[1]-req.y)<0.1:
            rospy.loginfo("This is the %d mineral point",index)
            # put relative func at here
            status = 3

    resp = GoalResponse(sum)
    return resp

def main(name):
    global status
    # go to the first point
    response = client.call(x=point_0[0],y=point_0[1])
    rospy.loginfo("response from move_system is %f ",response.result)
    rate = rospy.Rate(10)
    # set a class member
    vs = visual_status()
    
    while 1:
        if status == 1:
            # put relative func at here (at begin point)
            
            rospy.loginfo("scan and range tag")
            vs.flag=1
            status =0

        elif status == 2:
            # put relative func at here (at exchange point)
            rospy.loginfo("exchanging the the mineral at exchange point")
            vs.flag=2
            status =0

        elif status == 3:
            # put relative func at here (at miner point)
            rospy.loginfo("grasping the mineral")
            vs.flag=3
            status =0
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("visual_system")
    pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
    client = rospy.ServiceProxy("target_position",Goal)
    
    try:
        _thread.start_new_thread(main,("main",))
    except Exception as e:
        print ("Error: %s",e)

    while 1:
        server = rospy.Service("grasp_or_loosen",Goal,doReq)
        rospy.loginfo("visual_system is ok")
        rospy.spin()

