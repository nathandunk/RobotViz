import pyqtgraph as pg
import sys
import pyqtgraph.opengl as gl
from functions import *
import numpy as np
import time
import sympy as s

if __name__ == "__main__":

    theta1, theta2, theta3 = sym.symbols('theta1 theta2 theta3')
    l1 = 20
    l2 = 30
    # link            1    2         3
    alpha = np.array([0,        np.pi/2,  0,      0])
    a     = np.array([0,        0,        l1,     l2])
    d     = np.array([0,        0,        0,      0])
    theta = np.array([theta1,   theta2,   theta3, 0])
    my_robot = robot('rrr')

    my_robot.dh(alpha,a,d,theta)

    my_robot.joint_values = [0,np.pi/2,np.pi/4, 0]
    my_robot.dhtf(my_robot.alpha,my_robot.a,my_robot.d,my_robot.joint_values)

    # print(my_robot.T_full)
    view = create_3D_plot()

    p = np.array([my_robot.O[0,:],my_robot.O[1,:],my_robot.O[2,:]]).transpose()
    x_base = np.array([[0,3],[0,0],[0,0]]).transpose()
    y_base = np.array([[0,0],[0,3],[0,0]]).transpose()
    z_base = np.array([[0,0],[0,0],[0,3]]).transpose()
    # p = p.transpose()

    C = pg.glColor('w')
    R = pg.glColor('r')
    G = pg.glColor('g')
    
    B = pg.glColor('b')

    plt = gl.GLLinePlotItem(pos=p, width=3, color=C)

    for i in range(0,my_robot.size+1):
        x_unit = np.array([my_robot.O[:,i],my_robot.X[:,i]])
        y_unit = np.array([my_robot.O[:,i],my_robot.Y[:,i]])
        z_unit = np.array([my_robot.O[:,i],my_robot.Z[:,i]])
        pltx = gl.GLLinePlotItem(pos = x_unit, width = 5, color = R)
        plty = gl.GLLinePlotItem(pos = y_unit, width = 5, color = G)
        pltz = gl.GLLinePlotItem(pos = z_unit, width = 5, color = B)
        view.addItem(pltx)
        view.addItem(plty)
        view.addItem(pltz)

    view.addItem(plt)

    if sys.flags.interactive != 1 or not hasattr(QtCore, 'PYQT_VERSION'):
        pg.QtGui.QApplication.exec_()