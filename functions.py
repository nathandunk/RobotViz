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

            print(self.O[:,i])
            print(self.T_full[0:3,0])
            self.X[:,i] = self.O[:,i]+self.T_full[0:3,0]*2
            self.Y[:,i] = self.O[:,i]+self.T_full[0:3,1]*2
            self.Z[:,i] = self.O[:,i]+self.T_full[0:3,2]*2
            # print(self.T_full)

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
