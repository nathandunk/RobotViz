import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import sys
import pyqtgraph.opengl as gl
from functions import *
import numpy as np
import time
import sympy as s

if __name__ == "__main__":

    theta1, theta2, theta3 = sym.symbols('theta1 theta2 theta3')
    l1 = 83.1
    l2 = 91.75
    l3 = 100
    # link                         1         2       3
    alpha    = np.array([0,        np.pi/2,  0,      0])
    a        = np.array([0,        0,        l1,     l2])
    d        = np.array([0,        0,        0,      0])
    theta    = np.array([theta1,   theta2+np.pi/2,   theta3, 0])
    # link                 1         2         3       4
    # alpha    = np.array([0,        np.pi/2,  0,      0])
    # a        = np.array([0,        l1,       l2,     l3])
    # d        = np.array([0,        0,        0,      0])
    # theta    = np.array([theta1,   theta2,   theta3, 0])

    my_robot = robot('rrr')
    my_robot.dh(alpha,a,d,theta)

    my_robot.set_joint_values([0,0,0])
    my_robot.dhtf()

    my_robot.plot3d()