#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
from math import pow, sqrt, pi, atan, sin, cos
import sys

""" ---------- 3.11.2 Delta机器人运动学分析 ---------- """
## 表3-8 Delta机器人几何参数表
L = 0.300 # 驱动臂等效臂长
l = 0.800 # 从动臂等效臂长
S_p = 0.100 # 动平台等边三角形的边长
wb = 0.180 # 静坐标系{B}的原点到等腰三角形边的垂直直线距离
wp = 0.030 # 动坐标系{P}的原点到等腰三角形边的垂直直线距离
up = 0.035 # 动坐标系{P}的原点到等腰三角形顶点的直线距离

## 2)Delta机器人运动学逆解
def calculate_delta_robot_angle(P):
    x, y, z = P

    # 计算Delta机器人的三个运动学约束方程中的a, b, c 公式(3-114,3-114,3-116)
    a = wb - up
    b = (S_p / 2.0) - ((sqrt(3.0) / 2.0) * wb)
    c = wp - (wb / 2.0)

    # reserve storage and define size/shape for the most used variables
    # 为最常用的变量预留存储空间并定义大小/形状
    E = np.zeros(3)  # Variable for calculate angle
    F = np.zeros(3)  # Variable for calculate angle
    G = np.zeros(3)  # Variable for calculate angle
    # 3.11.2 2)Delta机器人运动学逆解
    # Calculate E, F, G
    E[0] = 2.0 * L * (y + a)
    F[0] = 2.0 * z * L
    G[0] = pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(a, 2) + \
           pow(L, 2) + (2.0 * y * a) - pow(l, 2)
    E[1] = -L * ((sqrt(3.0) * (x + b)) + y + c)
    F[1] = 2.0 * z * L
    G[1] = pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(b, 2) + pow(c, 2) + \
           pow(L, 2) + (2.0 * ((x * b) + (y * c))) - pow(l, 2)
    E[2] = L * ((sqrt(3.0) * (x - b)) - y - c)
    F[2] = 2.0 * z * L
    G[2] = pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(b, 2) + pow(c, 2) + \
           pow(L, 2) + (2.0 * (-(x * b) + (y * c))) - pow(l, 2)

    # Calculate angle in rad and degrees
    angle_upper_arm_rad = np.zeros(3)  # solution angle of motors in rad
    angle_upper_arm_deg = np.zeros(3)
    for i in range(0, 3):
        # 对t_i的一元二次方程求根
        t1 = (-F[i] + sqrt(pow(F[i],2) - pow(G[i],2) + pow(E[i],2))) / (G[i] - E[i])
        t2 = (-F[i] - sqrt(pow(F[i],2) - pow(G[i],2) + pow(E[i],2))) / (G[i] - E[i])
        # t_i = tan(θ_i/2) 公式(3-117)
        sol1 = 2 * atan(t1)
        sol2 = 2 * atan(t2)
        # 一般选择支链向外弯曲的姿态
        if ((sol1 < pi / 2) & (sol1 > -pi / 2)):
            angle_upper_arm_rad[i] = sol1
        elif ((sol2 < pi / 2) & (sol2 > -pi / 2)):
            angle_upper_arm_rad[i] = sol2
        angle_upper_arm_deg[i] = math.degrees(angle_upper_arm_rad[i])
    return angle_upper_arm_rad, angle_upper_arm_deg
def calculate_angle_lower_arms(P, angle_upper_arm_rad):
    # 计算角度
    p0 = np.array(P)
    p1 = np.array([0, -up, 0])
    p2 = np.array([S_p/2, wp, 0])
    p3 = np.array([-S_p/2, wp, 0])
    a1 = np.array([0, -wb-L*cos(angle_upper_arm_rad[0]), - L*sin(angle_upper_arm_rad[0])])
    a2 = np.array([((sqrt(3))/2)*(wb+(L*cos(angle_upper_arm_rad[1]))),
                   0.5*(wb+(L*cos(angle_upper_arm_rad[1]))), - L*sin(angle_upper_arm_rad[1])])
    a3 = np.array([-((sqrt(3))/2)*(wb+(L*cos(angle_upper_arm_rad[2]))),
                   0.5*(wb+(L*cos(angle_upper_arm_rad[2]))), - L*sin(angle_upper_arm_rad[2])])
    pf1 = p0 + p2
    pf2 = p0 + p3
    pf3 = p0 + p1

    d1_square = ((pf1 - a1)*(pf1 - a1)).sum()
    d2_square = ((pf2 - a2)*(pf2 - a2)).sum()
    d3_square = ((pf3 - a3)*(pf3 - a3)).sum()
    D1 = math.acos((d1_square - pow(S_p,2)-pow(l,2))/((-2)*S_p*l))
    alpha1 = pi - D1
    beta1 = pi - angle_upper_arm_rad[0]
    final_angle_lower_arm1 = beta1 - alpha1 - (pi/2) + 0.15
    D2 = math.acos((d2_square - pow(S_p,2)-pow(l,2))/((-2)*S_p*l))
    alpha2 = pi - D2
    beta2 = pi - angle_upper_arm_rad[1]
    final_angle_lower_arm2 = beta2 - alpha2 - (pi/2)
    D3 = math.acos((d3_square - pow(S_p,2)-pow(l,2))/((-2)*S_p*l))
    alpha3 = pi - D3
    beta3 = pi - angle_upper_arm_rad[2]
    final_angle_lower_arm3 = beta3 - alpha3 - (pi/2) + 0.1
    end_effector = (angle_upper_arm_rad[0] + final_angle_lower_arm1) * -1
    return final_angle_lower_arm1, final_angle_lower_arm2, final_angle_lower_arm3, end_effector

## 3)Delta机器人运动学正解
def forward(theta1, theta2, theta3):
    ## 三个虚拟的球体中心B_A_iv
    # B_A_1v
    x1 = 0
    y1 = - wb - L * cos(theta1) + up
    z1 = -L * sin(theta1)

    # B_A_2v
    x2 = sqrt(3)* (wb + L * cos(theta2))/2 - S_p / 2
    y2 = (wb + L * cos(theta2)) / 2 - wp
    z2 = -L * sin(theta2)

    # B_A_3v
    x3 = -sqrt(3) * (wb + L * cos(theta3)) / 2 + S_p / 2
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
        # 使用高度相等的z高度的球体相交算法 公式(3-120)
        x = (a22 * b1 - a12 * b2) / (a11 * a22 - a12 * a21)
        y = (a11 * b2 - a21 * b1) / (a11 * a22 - a12 * a21)
        # 公式(3-121)
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
