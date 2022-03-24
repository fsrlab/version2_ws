#! /usr/bin/env python
from calendar import WEDNESDAY
import imp
from os import stat
from turtle import goto
import rospy
from competion_code.srv import *
from geometry_msgs.msg import Twist
import tf2_ros
import tf
import _thread
status = 0

path_1 = [[-1,-1.75],[-1,-3.2],[-0.5,-3.2]]
path_2 = [[-1,-1.75],[-1,-3.0],[-0.4,-3.0]]
path_3 = [[-1,-1.7],[-1,-2.8],[-1.6,-2.8],[-2.0,-2.7]]
path_4 = [[-1,-1.75],[-1,-0.9],[-2.3,-0.9],[-2.3,-0.05]]
path_5 = [[-1,-1.75],[-1,-0.9],[-2.3,-0.9],[-2.3,0.8],[-2.5,0.9]]
path = [path_1,path_1,path_2,path_3,path_4,path_5]
point_0=[0.2,1.6,0]
point_1=[0.7,3.1,3.14]
point_2=[0.25,2.8,-1.6]
point_3=[2.0,2.7,0]
point_4=[2.2,0.2,3.14]
point_5=[2.6,-0.8,0]
point_6=[0.9,1.7,0]
points=[point_0,point_1,point_2,point_3,point_4,point_5,point_6]
way=-1
def pid(target, now, last):
    kp = 5
    ki = 0
    kd = 0
    result = kp*abs(target-now) - kd*(last-abs(target-now))
    if abs(result) > 0.35:
        result = 0.35
    if (target-now)<0:
        return -1*result
    else:
        return result  

def go(target_x,target_y):
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            # part of tf
            tf_car = buffer.lookup_transform("base_link","map",rospy.Time(0))
            x_car = tf_car.transform.translation.x
            y_car = tf_car.transform.translation.y 
            z_car = tf_car.transform.rotation.z
            #rospy.loginfo("location: x=%f, y=%f, theate=%f",x_car,y_car,z_car)

            # part of control
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            if abs(target_x-x_car)>0.1:
                twist.linear.x = -1*pid(target_x,x_car,0)
            
            if abs(target_y-y_car)>0.1:
                twist.linear.y = -1*pid(target_y,y_car,0)
                # print(twist.linear.y)
            pub.publish(twist)
            
            if(twist.linear.x == 0 and twist.linear.y == 0):
                break
            
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
            # part of control
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.angular.z = 0.4
            if abs(z_car)<0.02:
                twist.angular.z = 0
                pub.publish(twist)
                break
            pub.publish(twist)
            
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

def doReq(req):
    global status,way
    sum = req.x + req.y
    rospy.loginfo("target point is: x = %f,y = %f ",req.x, req.y)
    # set status at here
    
    if req.x==point_0[0] and req.y==point_0[1]:
        way=0
        print(way)
        status = 1
    elif req.x==point_1[0] and req.y==point_1[1]:
        way=1
        print(way)
        status = 1
    elif req.x==point_2[0] and req.y==point_2[1]:
        way=2
        print(way)
        status = 1
    elif req.x==point_3[0] and req.y==point_3[1]:
        way=3
        print(way)
        status = 1
    elif req.x==point_4[0] and req.y==point_4[1]:
        way=4
        print(way)
        status = 1
    elif req.x==point_5[0] and req.y==point_5[1]:
        way=5
        print(way)
        status = 1
    elif req.x==point_6[0] and req.y==point_6[1]:
        status = 2
        print(way)
    
    resp = GoalResponse(sum)
    return resp

def main(name):
    global status
    rate = rospy.Rate(10)
 
    while 1:
        if status == 1:
            print("status= %d",status)
            if way == 0:
                go(-0.2,-1.7)
            else:
                print("going")
                for coord in path[way]:
                    go(coord[0],coord[1])
                    # rospy.sleep(1)
                turn(way)
            client.call(points[way][0],points[way][1])
            status=0
        elif status==2:
            print("status=%d",status)
            print("back to %d",way)
            face_0()
            for coord in reversed(path[way]):
                go(coord[0],coord[1])
            status=0   
            client.call(points[6][0],points[6][1])
        rate.sleep()
        
if __name__ == "__main__":
    rospy.init_node("move_system")
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    client = rospy.ServiceProxy("grasp_or_loosen",Goal)
    pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
    try:
        _thread.start_new_thread(main,("main",))
    except Exception as e:
        print ("Error: %s",e)

    while 1:
        server = rospy.Service("target_position",Goal,doReq)
        rospy.loginfo("move_system is ok")
        rospy.spin()

