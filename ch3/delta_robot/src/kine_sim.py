#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from math import pi
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import geometry_msgs.msg
import tf
from kinematics import forward, calculate_delta_robot_angle, calculate_angle_lower_arms

## move joints in rviz
def moveJ(upper_arm_1, upper_arm_2, upper_arm_3, lower_arm_1,lower_arm_2,lower_arm_3, effector):
    # update joint_state
    joint_state.header.stamp = rospy.Time.now()
    joint_state.position = [upper_arm_1, upper_arm_2, upper_arm_3, lower_arm_1,
    lower_arm_2,lower_arm_3, effector]
    world_trans.header.stamp = rospy.Time.now()
    pub.publish(joint_state)
    broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1), rospy.Time.now(),'base_link','world')


if __name__ == "__main__":
    rospy.init_node('delta_robot')  # node name 'delta_robot'
    broadcaster = tf.TransformBroadcaster()
    world_trans = geometry_msgs.msg.TransformStamped()
    joint_state = JointState()
    world_trans.header.frame_id = 'world'
    world_trans.child_frame_id = 'base_link'
    joint_state.header = Header()
    joint_state.name = ['base_to_arm_1', 'base_to_arm_2', 'base_to_arm_3',
                        'down_arm_1_to_arm_1', 'down_arm_2_to_arm_2',
                        'down_arm_3_to_arm_3', 'tool0_to_down_arm_1']
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    k = 0
    while not rospy.is_shutdown():
        # 循环坐标
        if k == 0:
            B_P_P = [0.0, 0.0, -0.5]
        if k == 1:
            B_P_P = [0.0, 0.0, -0.7]
        if k == 2:
            B_P_P = [0.0, 0.0, -1.0]
        print("B_P_P =", B_P_P, "(m)")

        angle_upper_arm_rad, angle_upper_arm_deg = calculate_delta_robot_angle(B_P_P)
        print("inverse solution =", angle_upper_arm_rad, "(rad)")
        print("inverse solution =", angle_upper_arm_deg, "(degree)")
        upper_arm_1 = angle_upper_arm_rad[0]
        upper_arm_2 = angle_upper_arm_rad[1]
        upper_arm_3 = angle_upper_arm_rad[2]
        lower_arm_1, lower_arm_2, lower_arm_3, effector = calculate_angle_lower_arms(B_P_P, angle_upper_arm_rad)

        """
        θ={0 0 0}(rad),（使用高度相等的z高度的球体相交算法）,计算出的FPK结果为B_P_P={0 0 -1.065}(m)
        θ={10° 20° 30°}(degree),（使用非相等的z高度的球体相交算法）,计算出的FPK结果为B_P_P={0.108 -0.180 -1.244}(m)
        # General Theta
        dtr = pi / 180.0  # degree to rad
        thetas = [10 * dtr, 20 * dtr, 30 * dtr]
        """
        thetas = angle_upper_arm_rad
        B_P_P = forward(thetas[0], thetas[1], thetas[2])
        print("forward solution =", B_P_P, "(m)")

        moveJ(upper_arm_1, upper_arm_2, upper_arm_3, lower_arm_1,lower_arm_2,lower_arm_3, effector)  # Call function send_joint_state_to_urdf
        k = k + 1
        if k == 4:
            k = 0

    	r = rospy.Rate(0.5)
        r.sleep()



