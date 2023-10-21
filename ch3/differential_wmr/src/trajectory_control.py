#!/usr/bin/env python  
import roslib
roslib.load_manifest('differential_wmr')
import rospy
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import geometry_msgs.msg
import numpy as np

ESCALAR_FACTOR=70.0
p_ref_ant=np.array([[0.0],[0.0]])
p_ref=np.array([[0.0],[0.0]])
x= 0.0
y= 0.0
theta=0.0

#iNVERSE KINEMATIC PID CONTROL
def Control(angle,x_pos,y_pos,pos_before,pos_ref,K_sin):
    Jr=np.array([[math.cos(angle) ,-0.15 * math.sin(angle)],[math.sin(angle) ,0.15 * math.cos(angle)]])
    Jr_inv=np.linalg.inv(Jr)
    K=np.array([[K_sin[0], 0],[0, K_sin[1]]])
    posp_d=pos_ref - pos_before
    pos=np.array([[x_pos],[y_pos]])
    pos_error=pos_ref - pos
    pos_error[0, 0]=math.tanh(pos_error[0, 0])
    pos_error[1, 0]=math.tanh(pos_error[1, 0])
    c=np.dot(Jr_inv, (10 * posp_d + np.dot(K,pos_error)))
    c1=c[0, 0]
    c2=c[1, 0]
    return c1,c2

#ODOMETRY DATA
def Odom(msg):
    global x
    global y
    global theta

    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    rot_q=msg.pose.pose.orientation
    angles=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta=angles[2]

#CIRCLE REFERENCE
def reference(msg):
    global p_ref

    p_ref[0, 0] = msg.x
    p_ref[1, 0] = msg.y


if __name__ == '__main__':
        #NODE DEFINITION
        rospy.init_node('trajectory_control')
        sub_odom = rospy.Subscriber('/mobile_base_controller/odom',Odometry,Odom)
        sub_reference = rospy.Subscriber('/mobile_base_controller/reference',geometry_msgs.msg.Point,reference)
        diff_vel = rospy.Publisher('/mobile_base_controller/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            linear, angular = Control(theta, x, y, p_ref_ant, p_ref, [0.6, 0.6])
            p_ref_ant = p_ref

            #CONTROL SIGNAL SATURED
            if linear > 1.0:
                linear = 1
            elif linear < -0.5:
                linear = -0.5
            if angular > 1.7 :
                angular = 1.7
            elif angular < -1.7:
                angular = -1.7
            
            print("S. Control [u w]: [%s %s]"%(linear,angular))
            #SEND VELOCITIES
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            diff_vel.publish(cmd)
            rate.sleep()
