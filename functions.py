import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import sys
import numpy as np
import sympy as sym
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.dockarea import *
import serial
import time
import thread

# Define the robot class
class robot:

    def __init__(self,rp_vector):
        try:
            self.ser = serial.Serial('COM6', 9600)
        except serial.serialutil.SerialException as e:
            print("No serial connected")
        else:
            pass
        finally:
            pass
        
        self.rp_vector      = rp_vector
        self.size           = len(rp_vector)
        self.joints         = ["0"]*self.size
        self.alpha          = np.zeros(self.size)
        self.a              = np.zeros(self.size)
        self.d              = np.zeros(self.size)
        self.theta          = np.zeros(self.size)
        self.T              = np.zeros((4,4,self.size))
        self.T_full         = np.zeros((4,4))
        self.joint_values   = np.zeros(self.size+1)
        self.joint_values_d = np.zeros(self.size)

        self.O              = np.zeros((3,self.size+1))

        self.X              = np.zeros((3,self.size+1))
        self.Y              = np.zeros((3,self.size+1))
        self.Z              = np.zeros((3,self.size+1))

        self.pltx = [0]*(self.size+1)
        self.plty = [0]*(self.size+1)
        self.pltz = [0]*(self.size+1)

        for i in self.rp_vector:
            self.add_joint(i)

    def add_joint(self,joint_type):
        if joint_type == 'r':
            self.joints.append("revolute")
        else:
            self.joints.append("prismatic")

    def dh(self,alpha_,a_,d_,theta_):
        self.alpha = alpha_
        self.a = a_
        self.d = d_
        self.theta = theta_
        # self.dhtf(self.alpha,self.a,self.d,self.theta)

    def dhtf(self):

        self.T_full = np.identity(4)
        for i in range(0,self.size+1):
            T_hold = np.array([[np.cos(self.joint_values[i]),    -np.sin(self.joint_values[i]),                   0,                 self.a[i]],
                [np.sin(self.joint_values[i])*np.cos(self.alpha[i]), np.cos(self.joint_values[i])*np.cos(self.alpha[i]), -np.sin(self.alpha[i]), -np.sin(self.alpha[i])*self.d[i]],
                [np.sin(self.joint_values[i])*np.sin(self.alpha[i]), np.cos(self.joint_values[i])*np.sin(self.alpha[i]),  np.cos(self.alpha[i]),  np.cos(self.alpha[i])*self.d[i]],
                [0,                                 0,                                  0,                 1]])
            # print(T_hold.shape)
            # print(T_hold)
            self.T_full = np.matmul(self.T_full,T_hold)

            self.O[:,i] = self.T_full[0:3,3]

            self.X[:,i] = self.O[:,i]+self.T_full[0:3,0]*10
            self.Y[:,i] = self.O[:,i]+self.T_full[0:3,1]*10
            self.Z[:,i] = self.O[:,i]+self.T_full[0:3,2]*10
            # print(self.T_full)

    def plot3d(self):

        self.app = QtGui.QApplication(sys.argv)
        self.win = QtGui.QMainWindow()
        self.area = DockArea()
        self.win.setCentralWidget(self.area)
        self.win.resize(800,450)

        self.d1 = Dock("Forward Kinematics", size=(200, 200))
        self.d2 = Dock("3D Plot", size=(800,800))
        self.d3 = Dock("Inverse Kinematics", size=(200,200))
        self.area.addDock(self.d1,'left')
        self.area.addDock(self.d2,'right')
        self.area.addDock(self.d3,'bottom',self.d1)

        self.newfont = QtGui.QFont("Helvetica", 12, QtGui.QFont.Bold) 

        self.w1 = pg.LayoutWidget()
        self.saveBtn = QtGui.QPushButton('Forward')

        self.label_theta1_ = QtGui.QLabel("Theta 1")
        self.label_theta1_.setFont(self.newfont)
        self.label_theta2_ = QtGui.QLabel("Theta 2")
        self.label_theta2_.setFont(self.newfont)
        self.label_theta3_ = QtGui.QLabel("Theta 3")
        self.label_theta3_.setFont(self.newfont)

        self.theta1_ = QLineEdit()
        self.theta2_ = QLineEdit()
        self.theta3_ = QLineEdit()

        self.w1.addWidget(self.label_theta1_, row=0, col=0)
        self.w1.addWidget(self.theta1_, row=0, col=1)
        self.w1.addWidget(self.label_theta2_, row=1, col=0)
        self.w1.addWidget(self.theta2_, row=1, col=1)
        self.w1.addWidget(self.label_theta3_, row=2, col=0)
        self.w1.addWidget(self.theta3_, row=2, col=1)
        self.w1.addWidget(self.saveBtn, row=3, col=1)
        self.saveBtn.clicked.connect(self.capture_angles)

        self.d1.addWidget(self.w1)

        self.w3 = pg.LayoutWidget()
        self.saveBtn = QtGui.QPushButton('Inverse')

        self.label_x = QtGui.QLabel("X Value")
        self.label_x.setFont(self.newfont)
        self.label_y = QtGui.QLabel("Y Value")
        self.label_y.setFont(self.newfont)
        self.label_z = QtGui.QLabel("Z Value")
        self.label_z.setFont(self.newfont)

        self.x_ = QLineEdit()
        self.y_ = QLineEdit()
        self.z_ = QLineEdit()

        self.w3.addWidget(self.label_x, row=0, col=0)
        self.w3.addWidget(self.x_, row=0, col=1)
        self.w3.addWidget(self.label_y, row=1, col=0)
        self.w3.addWidget(self.y_, row=1, col=1)
        self.w3.addWidget(self.label_z, row=2, col=0)
        self.w3.addWidget(self.z_, row=2, col=1)
        self.w3.addWidget(self.saveBtn, row=3, col=1)
        self.saveBtn.clicked.connect(self.capture_angles)

        self.d3.addWidget(self.w3)

        self.w2 = gl.GLViewWidget()
        self.w2.orbit(-135,0)
        self.w2.pan(0, 100, 0)

        # create the background grids
        # gx = gl.GLGridItem()
        # gx.rotate(90, 0, 1, 0)
        # self.w2.addItem(gx)
        # gy = gl.GLGridItem()
        # gy.rotate(90, 1, 0, 0)
        # self.w2.addItem(gy)
        gz = gl.GLGridItem()
        gz.setSize(200,200,200)
        gz.setSpacing(10,10,10)
        self.w2.addItem(gz)

        self.plt = gl.GLLinePlotItem(pos=np.array([self.O[0,:],self.O[1,:],self.O[2,:]]).transpose(), width=3, color=pg.glColor('w'))
        self.w2.opts['distance'] = 600
        for i in range(0,self.size+1):
            self.pltx[i] = gl.GLLinePlotItem(pos = np.array([self.O[:,i],self.X[:,i]]), width = 3, color = pg.glColor('r'))
            self.plty[i] = gl.GLLinePlotItem(pos = np.array([self.O[:,i],self.Y[:,i]]), width = 3, color = pg.glColor('g'))
            self.pltz[i] = gl.GLLinePlotItem(pos = np.array([self.O[:,i],self.Z[:,i]]), width = 3, color = pg.glColor('b'))
            self.w2.addItem(self.pltx[i])
            self.w2.addItem(self.plty[i])
            self.w2.addItem(self.pltz[i])

        self.w2.addItem(self.plt)

        self.d2.addWidget(self.w2)

        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(20)

        self.win.show()

        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()

    def update(self):
        
        p = np.array([self.O[0,:],self.O[1,:],self.O[2,:]]).transpose()
        C = pg.glColor('w')
        R = pg.glColor('r')
        G = pg.glColor('g')
        B = pg.glColor('b')

        # self.plt = gl.GLLinePlotItem(pos=p, width=3, color=C)
        self.plt.setData(pos=p, width=3, color=C)
        # print("1")

        for i in range(0,self.size+1):
            x_unit = np.array([self.O[:,i],self.X[:,i]])
            y_unit = np.array([self.O[:,i],self.Y[:,i]])
            z_unit = np.array([self.O[:,i],self.Z[:,i]])
            self.pltx[i].setData(pos = x_unit, width = 3, color = R)
            self.plty[i].setData(pos = y_unit, width = 3, color = G)
            self.pltz[i].setData(pos = z_unit, width = 3, color = B)

    def capture_angles(self):
        self.joint_values_d[0] = int(self.theta1_.text())/180.0*np.pi
        self.joint_values_d[1] = int(self.theta2_.text())/180.0*np.pi+np.pi/2
        self.joint_values_d[2] = int(self.theta3_.text())/180.0*np.pi
        # print(self.joint_values_d-[0, np.pi/2, 0])
        try:
            self.write_serial(self.joint_values_d-[0, np.pi/2, 0])
        except AttributeError as e:
            print("Still no serial port...lol")
        else:
            pass
        finally:
            pass
        

        thread.start_new_thread(self.sweep, ())

    def sweep(self):
        for i in range(0,self.size):
            if self.joint_values[i] < self.joint_values_d[i]: 
                dir = 1
            else: 
                dir = -1

            while self.joint_values[i]*dir < self.joint_values_d[i]*dir:
                self.joint_values[i] = deg2rad(rad2deg(self.joint_values[i])+dir*1)
                self.dhtf()
                self.update()
                # print("20")
                time.sleep(0.015)

    def write_serial(self,joint_vals):
        for angle in joint_vals:
            angle_deg = rad2deg(angle)
            angle_deg_adj = str(int(angle_deg+90))
            self.ser.write(angle_deg_adj.zfill(3))

    def set_joint_values(self,joint_vals):
        self.joint_values[0] = joint_vals[0]
        self.joint_values[1] = joint_vals[1]+np.pi/2
        self.joint_values[2] = joint_vals[2]
        self.joint_values[3] = 0

def rad2deg(angle_rad):
    return angle_rad*180/np.pi

def deg2rad(angle_deg):
    return angle_deg/180.0*np.pi

