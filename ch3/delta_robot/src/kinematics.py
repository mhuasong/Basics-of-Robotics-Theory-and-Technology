#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import numpy
from math import sin, cos, sqrt, atan, pi
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

joint_pub = None

""" ----------表3-8 Delta机器人几何参数表---------- """
sb = 0.567  #静平台等边三角形的边长mm
sp = 0.076  #动平台等边三角形的边长mm
L = 0.524  #驱动臂等效臂长
l = 1.244  #从动臂等效臂长（平行四边形长度）
h = 0.131
wb = (sqrt(3)/6) *sb  #静坐标系{B}的原点到等腰三角形边的垂直直线距离
ub = (sqrt(3)/3) *sb  #静坐标系{B}的原点到等腰三角形顶点的直线距离
wp = (sqrt(3)/6) *sp  #动坐标系{P}的原点到等腰三角形边的垂直直线距离
up = (sqrt(3)/3) *sp  #动坐标系{P}的原点到等腰三角形顶点的直线距离
dtr = pi / 180.0  #degree to rad
rtd = 180.0 / pi  # rad to degree


""" ----------Delta Robot Kinematics Analysis---------- """
## Forward Kinematics(FK) of Delta robot
def forward(theta1, theta2, theta3):
    ## 三个虚拟的球体中心B_A_iv
    # B_A_1v
    x1 = 0
    y1 = - wb - L * cos(theta1) + up
    z1 = -L * sin(theta1)

    # B_A_2v
    x2 = sqrt(3)* (wb + L * cos(theta2))/2 - sp / 2
    y2 = (wb + L * cos(theta2)) / 2 - wp
    z2 = -L * sin(theta2)

    # B_A_3v
    x3 = -sqrt(3) * (wb + L * cos(theta3)) / 2 + sp / 2
    y3 = (wb + L * cos(theta3)) / 2 - wp
    z3 = -L * sin(theta3)

    l1 = l
    l2 = l
    l3 = l

    a11 = 2 * (x3 - x1)
    a12 = 2 * (y3 - y1)
    a13 = 2 * (z3 - z1)

    a21 = 2 * (x3 - x2)
    a22 = 2 * (y3 - y2)
    a23 = 2 * (z3 - z2)

    b1 = l1**2-l3**2-x1**2-y1**2-z1**2+x3**2+y3**2+z3**2
    b2 = l2**2-l3**2-x2**2-y2**2-z2**2+x3**2+y3**2+z3**2

    if a13 == 0 or a23 == 0 or (a11 / a13 - a21 / a23)==0:
        print("a13=0或a23=0")
        # 使用高度相等的z高度的球体相交算法
        # (3-120)
        x = (a22 * b1 - a12 * b2) / (a11 * a22 - a12 * a21)
        y = (a11 * b2 - a21 * b1) / (a11 * a22 - a12 * a21)
        # (3-121)
        zn = z1
        z1 = (2 * zn + sqrt(4 * l1 * l1 - 4 * (x - x1) * (x - x1) - 4 * (y - y1) * (y - y1))) / 2
        z2 = (2 * zn - sqrt(4 * l1 * l1 - 4 * (x - x1) * (x - x1) - 4 * (y - y1) * (y - y1))) / 2
        # 由于动平台始终在静平台的下方，z<0
        if z1 < 0:
            z = z1
        elif z2 < 0:
            z = z2
        return [x, y, z]
    else:
        # 使用非相等的z高度的球体相交算法
        # x=f(y)
        a1 = a11 / a13 - a21 / a23
        #if a1 == 0:
            #print("a1=0")
            #return None
        a2 = a12 / a13 - a22 / a23
        a3 = b2 / a23 - b1 / a13
        a4 = -a2 / a1
        a5 = -a3 / a1

        # z=f(y)
        a6 = (-a21 * a4 - a22) / a23
        a7 = (b2 - a21 * a5) / a23

        # ay^2+by+c=0
        a = a4 * a4 + 1 + a6 * a6
        b = 2 * a4 * (a5 - x1) - 2 * y1 + 2 * a6 * (a7 - z1)
        c = a5 * (a5 - 2 * x1) + a7 * (a7 - 2 * z1) + x1 * x1 + y1 * y1 + z1 * z1 - l1 * l1
        if a == 0:
            print("a=0")
        if (b * b - 4 * a * c) < 0:
            return None

        y1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a)
        y2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a)
        x1 = a4 * y1 + a5
        x2 = a4 * y2 + a5
        z1 = a6 * y1 + a7
        z2 = a6 * y2 + a7
        # 由于动平台始终在静平台的下方，z<0
        if z1 < 0:
            return [x1, y1, z1]
        elif z2 < 0:
            return [x2, y2, z2]

## Inverse Kinematics(IK) of Delta robot
def inverse(P):
    x, y, z = P

    a = wb - up
    b = sp / 2 - (sqrt(3) / 2) * wb
    c = wp - wb / 2

    e1 = 2 * L * (y + a)
    f1 = 2 * z * L
    g1 = x ** 2 + y ** 2 + z ** 2 + a ** 2 + L ** 2 + 2 * y * a - l ** 2
    e2 = -L * (sqrt(3) * (x + b) + y + c)
    f2 = 2 * z * L
    g2 = x ** 2 + y ** 2 + z ** 2 + b ** 2 + c ** 2 + L ** 2 + 2 * x * b + 2 * y * c - l ** 2
    e3 = L * (sqrt(3) * (x - b) - y - c)
    f3 = 2 * z * L
    g3 = x ** 2 + y ** 2 + z ** 2 + b ** 2 + c ** 2 + L ** 2 - 2 * x * b + 2 * y * c - l ** 2

    sol1max = 2 * atan((-f1 + sqrt((e1 * e1) + (f1 * f1) - (g1 * g1))) / (g1 - e1))
    sol1min = 2 * atan((-f1 - sqrt((e1 * e1) + (f1 * f1) - (g1 * g1))) / (g1 - e1))
    sol2max = 2 * atan((-f2 + sqrt((e2 * e2) + (f2 * f2) - (g2 * g2))) / (g2 - e2))
    sol2min = 2 * atan((-f2 - sqrt((e2 * e2) + (f2 * f2) - (g2 * g2))) / (g2 - e2))
    sol3max = 2 * atan((-f3 + sqrt((e3 * e3) + (f3 * f3) - (g3 * g3))) / (g3 - e3))
    sol3min = 2 * atan((-f3 - sqrt((e3 * e3) + (f3 * f3) - (g3 * g3))) / (g3 - e3))

    thetas = [0, 0, 0]
    if ((sol1min < pi / 2) & (sol1min > -pi / 2)):
        thetas[0] = sol1min
    elif ((sol1max < pi / 2) & (sol1max > -pi / 2)):
        thetas[0] = sol1max

    if ((sol2min < pi / 2) & (sol2min > -pi / 2)):
        thetas[1] = sol2min
    elif ((sol2max < pi / 2) & (sol2max > -pi / 2)):
        thetas[1] = sol2max

    if ((sol3min < pi / 2) & (sol3min > -pi / 2)):
        thetas[2] = sol3min
    elif ((sol3max < pi / 2) & (sol3max > -pi / 2)):
        thetas[2] = sol3max

    return thetas


""" ----------Delta Robot Kinematics Examples---------- """
## Inverse Kinematics Examples
def IK():
    """
    B_P_P={0 0 -0.9}(m),计算出的IPK结果为θ={-20.5° -20.5° -20.5°}
    B_P_P={0.3 0.5 -1.1}(m),计算出的IPK结果为θ={47.5° -11.6° 21.4°}
    """
    # Nominal Position
    B_P_P = [0, 0, -0.9]
    # General Position
    B_P_P = [0.3, 0.5, -1.1]
    thetas = inverse(B_P_P)
    print("θ =", thetas, "(rad)")
    thetas[0] /= dtr
    thetas[1] /= dtr
    thetas[2] /= dtr
    print("θ =", thetas, "(degree)")
#IK()

## Forward Kinematics Examples
def FK():
    """
    θ={0 0 0}(rad),（使用高度相等的z高度的球体相交算法）,计算出的FPK结果为B_P_P={0 0 -1.065}(m)
    θ={10° 20° 30°}(degree),（使用非相等的z高度的球体相交算法）,计算出的FPK结果为B_P_P={0.108 -0.180 -1.244}(m)
    """
    # Nominal Theta
    thetas = [0, 0, 0]
    # General Theta
    thetas = [10 * dtr, 20 * dtr, 30 * dtr]
    B_P_P = forward(thetas[0], thetas[1], thetas[2])
    print("B_P_P =", B_P_P, "(m)")
#FK()

def movej(j1,j2,j3):
    print "move joint"
    joint_pub = rospy.Publisher('joint_states', JointState, queue_size = 10)
    rate = rospy.Rate(1)
    joints_configuration = JointState()
    joints_configuration.header = Header()
    joints_configuration.header.stamp = rospy.Time.now()
    joints_configuration.name = ['link_0_JOINT_1', 'link_0_JOINT_2', 'link_0_JOINT_3']
    joints_configuration.position = [j1 * dtr, j2* dtr, j3* dtr]
    joints_configuration.velocity = []
    joints_configuration.effort = []
    while not rospy.is_shutdown():
       joints_configuration.header.stamp = rospy.Time.now()
       rospy.loginfo(joints_configuration)
       break
    joint_pub.publish(joints_configuration)
#movej()

def kinematics_test():
    """
    循环检查示例
    """
    B_P_P = [0, 0, -0.9]
    B_P_P = [0.3, 0.5, -1.1]
    print "set B_P_P =", B_P_P, "(m)"
    thetas = inverse(B_P_P)
    print "inverse kinematics thetas =", thetas, "(rad)"
    B_P_P = forward(thetas[0], thetas[1], thetas[2])
    print "forward kinematics:B_P_P =", B_P_P, "(m)"
    
    thetas = [0, 0, 0]
    thetas = [10 * dtr, 20 * dtr, 30 * dtr]
    print "set thetas =", thetas, "(rad)" 
    B_P_P = forward(thetas[0], thetas[1], thetas[2])
    print "forward kinematics B_P_P =", B_P_P, "(m)" 
    thetas = inverse(B_P_P)
    print "inverse kinematics thetas =", thetas,  "(rad)"
    
def main():
    
    # Init ROS
    rospy.init_node('delta_joint_publisher', anonymous=True)
    # Subscribers
    #rospy.Subscriber('joint_states', JointState, movej)
    # Publishers
    joint_pub = rospy.Publisher('joint_states', JointState, queue_size = 10)

    movej(10,20,30)
    rospy.sleep(2)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")

