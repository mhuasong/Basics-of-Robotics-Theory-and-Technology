#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math
from math import sqrt


if __name__=="__main__":

    rospy.init_node('Simple_Vel_Cmd')
    back_pub = rospy.Publisher('/omni_robot/back_joint_velocity_controller/command', Float64, queue_size=1)
    left_pub = rospy.Publisher('/omni_robot/left_joint_velocity_controller/command', Float64, queue_size=1)
    right_pub = rospy.Publisher('/omni_robot/right_joint_velocity_controller/command', Float64, queue_size=1)

    rate=rospy.Rate(2)

    back_vel = Float64()
    left_vel = Float64()
    right_vel = Float64()

    back_vel.data = 0
    left_vel.data = 0
    right_vel.data = 0

    while not rospy.is_shutdown():
	key = raw_input()
    	## 1 forward line
	if key == 'w':
	   back_vel = 0
	   left_vel = -5*sqrt(3)
	   right_vel = 5*sqrt(3)

    	## 5 back line
	elif key == 's':
	   back_vel = 0
	   left_vel = 5*sqrt(3)
	   right_vel = -5*sqrt(3)

    	## 3 left line    
	elif key == 'a':
	   back_vel = -10
	   left_vel = 5
	   right_vel = 5

    	## 7 right line    
	elif key == 'd':
	   back_vel = 10
	   left_vel = -5
	   right_vel = -5

    	## 2 left forward 
	elif key == 'q':
	   back_vel = -5*sqrt(2)
	   left_vel = 5*(sqrt(2)-sqrt(6))/2
	   right_vel = 5*(sqrt(2)+sqrt(6))/2

    	## 8 right forward
	elif key == 'e':
	   back_vel = 5*sqrt(2)
	   left_vel = -5*(sqrt(2)+sqrt(6))/2
	   right_vel = -5*(sqrt(2)-sqrt(6))/2

    	## 4 left back
	elif key == 'z':
	   back_vel = -5*sqrt(2)
	   left_vel = 5*(sqrt(2)+sqrt(6))/2
	   right_vel = 5*(sqrt(2)-sqrt(6))/2

    	## 6 right back   
	elif key == 'x':
	   back_vel = 5*sqrt(2)
	   left_vel = -5*(sqrt(2)-sqrt(6))/2
	   right_vel = -5*(sqrt(2)+sqrt(6))/2


    	## rotate       
	elif key == 'h':
	   back_vel = 10
	   left_vel = 10
	   right_vel = 10

    	## stop    
	elif key == 'j':
	   back_vel = 0
	   left_vel = 0
	   right_vel = 0
    	## exit   
	else:
	   back_vel = 0
	   left_vel = 0
	   right_vel = 0
	   exit()		
 
	back_pub.publish(back_vel)
	left_pub.publish(left_vel)
	right_pub.publish(right_vel)
	
	rate.sleep()
