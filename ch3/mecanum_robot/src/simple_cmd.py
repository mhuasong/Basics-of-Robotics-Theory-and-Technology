#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64


if __name__=="__main__":

	rospy.init_node('Simple_Vel_Cmd')
	fl_pub = rospy.Publisher('/mecanum_robot/fl_joint_velocity_controller/command', Float64, queue_size=1)
	fr_pub = rospy.Publisher('/mecanum_robot/fr_joint_velocity_controller/command', Float64, queue_size=1)
	bl_pub = rospy.Publisher('/mecanum_robot/bl_joint_velocity_controller/command', Float64, queue_size=1)
	br_pub = rospy.Publisher('/mecanum_robot/br_joint_velocity_controller/command', Float64, queue_size=1)

	rate=rospy.Rate(2)

	fl_vel = Float64()
	fr_vel = Float64()
	bl_vel = Float64()
	br_vel = Float64()


	fl_vel.data = 0
	fr_vel.data = 0
	bl_vel.data = 0
	br_vel.data = 0

	while not rospy.is_shutdown():
		key = raw_input()
		## forward line
		if key == 'w':
			fl_vel = 10
			fr_vel = 10
			bl_vel = -10
			br_vel = -10

			## back line
		elif key == 's':
			fl_vel = -10
			fr_vel = -10
			bl_vel = 10
			br_vel = 10

			## left line
		elif key == 'a':
			fl_vel = -10
			fr_vel = 10
			bl_vel = -20
			br_vel = 20

			## right line
		elif key == 'd':
			fl_vel = 10
			fr_vel = -10
			bl_vel = 10
			br_vel = -10

			## turn left round
		elif key == 'u':
			fl_vel = 10
			fr_vel = -10
			bl_vel = -10
			br_vel = 10

			## turn right round
		elif key == 'i':
			fl_vel = -10
			fr_vel = 10
			bl_vel = 10
			br_vel = -10

			## concerning
		elif key == 'j':
			fl_vel = 0
			fr_vel = 10
			bl_vel = 0
			br_vel = -10

			## turn of fl_fr axis
		elif key == 'k':
			fl_vel = 0
			fr_vel = 0
			bl_vel = 10
			br_vel = -10

			## stop
		elif key == 'l':
			fl_vel = 0
			fr_vel = 0
			bl_vel = 0
			br_vel = 0

			## exit
		else:
			fl_vel = 0
			fr_vel = 0
			bl_vel = 0
			br_vel = 0
			exit()

		fl_pub.publish(fl_vel)
		fr_pub.publish(fr_vel)
		bl_pub.publish(bl_vel)
		br_pub.publish(br_vel)

		rate.sleep()
