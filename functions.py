from numpy import *

class robot(rp_vector):

    robot.size = length(rp_vector)
    for i in rp_vector:
        robot.add_joint(i)

    def add_joint(joint_type):
        if joint_type == r:


def dhtf(alpha, a, d, theta):
    T = np.matrix([[np.cos(theta),        -sin(theta),           0,           a],...
                  [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],...
                  [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha),  cos(alpha)*d],...
                  [0,                     0,                     0,           1]])
    return T

# def draw_axis(location,joint_angles):

# def FK_robot(joint_angles):
