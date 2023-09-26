#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from math import pi
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from kinematics import forward


def plot_delta(ws):
    x = ws[:, 0]
    y = ws[:, 1]
    z = ws[:, 2]

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter(x, y, z, alpha=0.8, c=z, marker='.', s=0.5)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.title.set_text('Delta Robot Workspace')

    # Front view XY
    plt.figure()
    plt.plot(x, y, '.r', markersize=0.5)
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Superior XY')

    # Front view XZ
    plt.figure()
    plt.plot(x, z, '.r', markersize=0.5)
    plt.xlabel('X [m]')
    plt.ylabel('Z [m]')
    plt.title('Frontal XZ')

    # Front view YZ
    plt.figure()
    plt.plot(y, z, '.r', markersize=0.5)
    plt.xlabel('Y [m]')
    plt.ylabel('Z [m]')
    plt.title('Frontal YZ')

    plt.show()

def Monte_carlo_Workspace():
    N = 25  #15
    res_ang_min = -pi / 2
    P = numpy.empty((N**3, 3))

    cont = 0
    for a in range(N):
        theta1 = res_ang_min + pi * a / N
        for b in range(N):
            theta2 = res_ang_min + pi * b / N
            for c in range(N):
                theta3 = res_ang_min + pi * c / N
                P[cont] = forward(theta1, theta2, theta3)
                cont = cont + 1

    plot_delta(P)


if __name__ == "__main__":
    Monte_carlo_Workspace()
