import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import sys
import numpy as np
import sympy as sym
import pyqtgraph as pg
import pyqtgraph.opengl as gl

# Define the robot class
class robot:

    def __init__(self,rp_vector):
        self.rp_vector      = rp_vector
        self.size           = len(rp_vector)
        self.joints         = ["0"]*self.size
        self.alpha          = np.zeros(self.size)
        self.a              = np.zeros(self.size)
        self.d              = np.zeros(self.size)
        self.theta          = np.zeros(self.size)
        self.T              = np.zeros((4,4,self.size))
        self.T_full         = np.zeros((4,4))
        self.joint_values   = np.zeros(self.size)

        self.O              = np.zeros((3,self.size+1))

        self.X              = np.zeros((3,self.size+1))
        self.Y              = np.zeros((3,self.size+1))
        self.Z              = np.zeros((3,self.size+1))

        self.app = QtGui.QApplication(sys.argv)
        self.w = gl.GLViewWidget()

        self.plt = gl.GLLinePlotItem(pos=np.array([self.O[0,:],self.O[1,:],self.O[2,:]]).transpose(), width=3, color=pg.glColor('w'))

        self.pltx = [0]*(self.size+1)
        self.plty = [0]*(self.size+1)
        self.pltz = [0]*(self.size+1)

        for i in range(0,self.size+1):
            self.pltx[i] = gl.GLLinePlotItem(pos = np.array([self.O[:,i],self.X[:,i]]), width = 5, color = pg.glColor('r'))
            self.plty[i] = gl.GLLinePlotItem(pos = np.array([self.O[:,i],self.Y[:,i]]), width = 5, color = pg.glColor('g'))
            self.pltz[i] = gl.GLLinePlotItem(pos = np.array([self.O[:,i],self.Z[:,i]]), width = 5, color = pg.glColor('b'))
            self.w.addItem(self.pltx[i])
            self.w.addItem(self.plty[i])
            self.w.addItem(self.pltz[i])
        self.w.addItem(self.plt)

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

    def dhtf(self, alpha, a, d, theta):

        # theta.subs(theta1,joint_values(1))
        # theta.subs(theta2,joint_values(2))
        # theta.subs(theta3,joint_values(3))

        self.T_full = np.identity(4)
        for i in range(0,self.size+1):
            T_hold = np.array([[np.cos(theta[i]),    -np.sin(theta[i]),                   0,                 a[i]],
                [np.sin(theta[i])*np.cos(alpha[i]), np.cos(theta[i])*np.cos(alpha[i]), -np.sin(alpha[i]), -np.sin(alpha[i])*d[i]],
                [np.sin(theta[i])*np.sin(alpha[i]), np.cos(theta[i])*np.sin(alpha[i]),  np.cos(alpha[i]),  np.cos(alpha[i])*d[i]],
                [0,                                 0,                                  0,                 1]])
            # print(T_hold.shape)
            # print(T_hold)
            self.T_full = np.matmul(self.T_full,T_hold)

            self.O[:,i] = self.T_full[0:3,3]

            self.X[:,i] = self.O[:,i]+self.T_full[0:3,0]*2
            self.Y[:,i] = self.O[:,i]+self.T_full[0:3,1]*2
            self.Z[:,i] = self.O[:,i]+self.T_full[0:3,2]*2
            # print(self.T_full)

    def plot3d(self):
        self.w.opts['distance'] = 40
        self.w.setWindowTitle('pyqtgraph awesomeness')
        self.w.setGeometry(0, 110, 1920, 1080)
        self.w.show()

        # create the background grids
        gx = gl.GLGridItem()
        gx.rotate(90, 0, 1, 0)
        self.w.addItem(gx)
        gy = gl.GLGridItem()
        gy.rotate(90, 1, 0, 0)
        self.w.addItem(gy)
        gz = gl.GLGridItem()
        self.w.addItem(gz)

        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(20)

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

        for i in range(0,self.size+1):
            x_unit = np.array([self.O[:,i],self.X[:,i]])
            y_unit = np.array([self.O[:,i],self.Y[:,i]])
            z_unit = np.array([self.O[:,i],self.Z[:,i]])
            self.pltx[i].setData(pos = x_unit, width = 5, color = R)
            self.plty[i].setData(pos = y_unit, width = 5, color = G)
            self.pltz[i].setData(pos = z_unit, width = 5, color = B)

def create_3D_plot():
    pg.mkQApp()

    ## make a widget for displaying 3D objects
    view = gl.GLViewWidget()
    view.show()

    ## create three grids, add each to the view
    xgrid = gl.GLGridItem()
    ygrid = gl.GLGridItem()
    zgrid = gl.GLGridItem()
    view.addItem(xgrid)
    view.addItem(ygrid)
    view.addItem(zgrid)

    # ## rotate x and y grids to face the correct direction
    xgrid.rotate(90, 0, 1, 0)
    ygrid.rotate(90, 1, 0, 0)

    ## scale each grid differently
    xgrid.scale(1, 1, 1)
    ygrid.scale(1, 1, 1)
    zgrid.scale(1, 1, 1)

    return view

# def draw_axis(location,joint_angles):

# def FK_robot(joint_angles):
