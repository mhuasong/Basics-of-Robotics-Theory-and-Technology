#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64


if __name__=="__main__":

    rospy.init_node('Simple_Vel_Cmd')
    pub0 = rospy.Publisher('/omni_robot/back_joint_velocity_controller/command', Float64, queue_size=1)
    pub1 = rospy.Publisher('/omni_robot/left_joint_velocity_controller/command', Float64, queue_size=1)
    pub2 = rospy.Publisher('/omni_robot/right_joint_velocity_controller/command', Float64, queue_size=1)

    rate=rospy.Rate(2)

    vel0 = Float64()
    vel1 = Float64()
    vel2 = Float64()

    vel0.data = 0
    vel1.data = 0
    vel2.data = 0

    while not rospy.is_shutdown():
	key = raw_input()
    	## forward line
	if key == 'f':
	   vel0 = 0
	   vel1 = -10
	   vel2 = 10
    	## left line    
	elif key == 'g':
	   vel0 = 10
	   vel1 = -10
	   vel2 = -10
    	## rotate       
	elif key == 'h':
	   vel0 = 10
	   vel1 = 10
	   vel2 = 10
    	## stop    
	elif key == 'j':
	   vel0 = 0
	   vel1 = 0
	   vel2 = 0
    	## exit   
	else:
	   vel0 = 0
	   vel1 = 0
	   vel2 = 0
	   exit()		
 
	pub0.publish(vel0)
	pub1.publish(vel1)
	pub2.publish(vel2)
	
	rate.sleep()

