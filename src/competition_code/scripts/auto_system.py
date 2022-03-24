from errno import ELIBBAD
from itertools import count
from math import fabs
from os import stat
from matplotlib.pyplot import flag
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import tf2_ros
import tf
import _thread
from my_redmarkerdetection import *    # image processing by cython
sorted_ID = []
#some status flag
status=0
flag=0
letter=0
# path
path_1 = [[-1,-1.7],[-1,-3.2],[-0.5,-3.2]]
path_2 = [[-1,-1.7],[-1,-3.0],[-0.4,-3.0]]
path_3 = [[-1,-1.7],[-1,-2.8],[-1.6,-2.8],[-2.0,-2.7]]
path_4 = [[-1,-1.7],[-1,-0.9],[-2.3,-0.9],[-2.3,-0.05]]
path_5 = [[-1,-1.7],[-1,-0.9],[-2.3,-0.9],[-2.3,0.8],[-2.5,0.9]]
path = [path_1,path_2,path_3,path_4,path_5]
# pid config
integral=0
last=0

def cmd(WAY):
    if status==1:
        face_0()
        for coord in reversed(path[WAY]):
            go(coord[0],coord[1])
    elif status==0:
        for coord in path[WAY]:
            go(coord[0],coord[1])
        turn(WAY+1)

    rospy.sleep(1)
    



def go(target_x,target_y):
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            # part of tf
            tf_car = buffer.lookup_transform("base_link","map",rospy.Time(0))
            x_car = tf_car.transform.translation.x
            y_car = tf_car.transform.translation.y 
            z_car = tf_car.transform.rotation.z
            # part of control
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            if abs(target_x-x_car)>0.1:
                if (target_x-x_car)>0:
                    twist.linear.x = -0.5
                else:
                    twist.linear.x = 0.5
            if abs(target_y-y_car)>0.1:
                if (target_y-y_car)>0:
                    twist.linear.y = -0.5
                else:
                    twist.linear.y = 0.5

            pub.publish(twist)
            
            if(twist.linear.x == 0 and twist.linear.y == 0):
                break
            
        except Exception as e:
            rospy.logwarn("error is: %s",e)
        rate.sleep()

def turn(WAY):

    rate = rospy.Rate(20)
    while True:
        try:
            # part of tf
            tf_car = buffer.lookup_transform("base_link","map",rospy.Time(0))
            x_car = tf_car.transform.translation.x
            y_car = tf_car.transform.translation.y 
            z_car = tf_car.transform.rotation.z
            w_car = tf_car.transform.rotation.w
            _,_,th= tf.transformations.euler_from_quaternion([0,0,z_car,w_car])
            print(th)
            if WAY==1:
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.angular.z = 0.5
                pub.publish(twist)
                if abs(th)>3.1:
                    twist = Twist()
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.angular.z = 0
                    pub.publish(twist)
                    break
            elif WAY==2:
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.angular.z = -0.5
                pub.publish(twist)

                if abs(z_car)>0.7:
                    twist = Twist()
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.angular.z = 0
                    pub.publish(twist)
                    break
            elif WAY==4:
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.angular.z =0.5
                pub.publish(twist)

                if abs(th)>3.1:
                    twist = Twist()
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.angular.z = 0
                    pub.publish(twist)
                    break
                

            elif WAY==3 or WAY==5:
                    return

        except Exception as e:
            rospy.logwarn("error is: %s",e)

        rate.sleep()

def face_0():
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            # part of tf
            tf_car = buffer.lookup_transform("base_link","map",rospy.Time(0))
            x_car = tf_car.transform.translation.x
            y_car = tf_car.transform.translation.y 
            z_car = tf_car.transform.rotation.z
            w_car = tf_car.transform.rotation.w
            _,_,th= tf.transformations.euler_from_quaternion([0,0,z_car,w_car])
            # part of control
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.angular.z = -pid(0,th)
            print(pid(0,th))
            if abs(th)<0.015:
                twist.angular.z = 0
                pub.publish(twist)
                break
            pub.publish(twist)
        except Exception as e:
            rospy.logwarn("error is: %s",e)

        rate.sleep()

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

def pid(target,now):
    global integral,last
    err=target-now
    integral+=err
    kp=2.5
    ki=0
    kd=1.2
    result=kp*err+kd*(err-last)+ki*(integral)
    last=err
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

def img_callback(data):
    global sorted_ID,flag,letter,status
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as err:
        print(err)    
    seg_papram = np.array([0,15,125,180,46,80],dtype="uint8")

    if flag==0:
        return
    if flag==1:
        sorted_ID = sort_number_label(cv_image,seg_papram)
    if flag==2:#exchange market
        id_list,tvec_list,rvec_list = marker_detection(cv_image,seg_papram)
        for i in range(len(id_list)):
            tvec = tvec_list[i]
            if id_list[i] == 5 and letter == 0:
                x = tvec[0]
                z = tvec[2]
            elif id_list[i] == 6 and letter == 1:
                x = tvec[0]
                z = tvec[2]
            elif id_list[i] == 7 and letter == 2:
                x = tvec[0]
                z = tvec[2]
        twist = Twist()
        if abs(z)<0.1:
            twist.linear.x = 0
            twist.linear.y = 0
            twist.angular.z= 0
            pub.publish(twist)
            arm_down()
            arm_relax()
            arm_up()
            letter+=1
            flag=0
            status=1
        elif abs(x-0.02)<0.02:
            twist.linear.x = 0.2
            pub.publish(twist)
        else:
            twist.linear.x = 0
            twist.linear.y = pid(0.02,x)
            twist.angular.z= 0  
            pub.publish(twist)

    if flag==3:
        id_list,tvec_list,rvec_list = marker_detection(cv_image,seg_papram)

        for i in range(len(id_list)):
            tvec = tvec_list[i]
            if (id_list[i] == 0 or id_list[i] == 1 or id_list[i] == 2 or id_list[i] == 3 or id_list[i] == 4 or id_list[i] == 6):
                x = tvec[0]
                z = tvec[2]
        if id_list==[]:
            print("don't detected market")
        else:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.angular.z= 0
            if abs(z)<0.15:
                twist.linear.x = 0
                twist.linear.y = 0
                twist.angular.z= 0
                pub.publish(twist)
                arm_down()
                arm_catch()
                arm_up()
                flag=0
                status=1
            if abs(x)<0.01:
                twist.linear.x = 0.2
            else:
                val_y=pid(0,x)
                twist.linear.y = val_y
                
            pub.publish(twist)

def main(name):
    global status,flag,letter
    rate = rospy.Rate(10)
    
    go(-0.2,-1.7)
    flag=1
    while 1:
        if sorted_ID!=[]:
            print(sorted_ID)
            for way in sorted_ID:
                print("way is %d",way)
                cmd(way)
                flag=3
                while status!=1:
                    rate.sleep()
                    pass
                cmd(way)
                status=0
                flag=2
                while status!=1:
                    rate.sleep()
                    pass
                status=0
            break
        else:
            rate.sleep()
            pass
    print("finish")
if __name__ == "__main__":
    # initialize
    rospy.init_node("auto_system")
    bridge = CvBridge()
    load_template()
    pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
    arm_gripper_pub = rospy.Publisher("arm_gripper", Point, queue_size = 1)
    arm_position_pub = rospy.Publisher("arm_position", Pose, queue_size = 1)
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, img_callback)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    try:
        _thread.start_new_thread(main,("main",))
    except Exception as e:
        print ("Error: %s",e)

    rospy.spin()