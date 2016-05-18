#! /usr/bin/env python
"""
Follow Mode Test
Written by: Josh Saunders
Date: 4/11/2016

Modified by: David Rojas
Date Modified: 5/16/16

This is to test the PID that controls the 4 Degrees Of Freedom (DOF) of the QC
"""
# We're using ROS
import rospy

# Python libraries
import math

# The classes that we're using
from Controller import Controller
from Navdata import navdata_info

# The messages that we need
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

class Follow(Mode):
    def __init__(self, ctrl.pid_x.setkp, ctrl.pid_x.setKi, ctrl.pid_x.setKd, ctrl.pid_y.setkp, ctrl.pid_y.setKi, ctrl.pid_y.setKd):
        # Initialize the node which is inherited from the Mode super class
        super(self.__class__, self).__init__('follow_mode')

        # Publishers
        self.pub_ctrl = rospy.Publisher('cmd_vel', Twist, queue_size=100)

        # Initialize member variables

        ########################
        # Set the bounding box #
        ########################
        # X is in front and behind QC [0, 360] pixels
        # Y is left and right of QC   [0, 640] pixels
        self.bbx_max = 563
        self.bbx_min = 438
        self.bby_max = 563
        self.bby_min = 438
        self.yaw_max = 350
        self.yaw_min = 10

        ####################################
        # Setup the individual controllers #
        ####################################
        # Note: in order to change the values for integrator max and min, you need
        # to go into Controller.py and adjust the values there
        # Set yaw controller
        # NOTE: this doesn't seem to be working correctly...
        self.ctrl.pid_theta.setKp = (1/260.0)
        # ctrl.pid_theta.setKp(0.0)
        self.ctrl.pid_theta.setKi = (0.0)
        self.ctrl.pid_theta.setKd = (0.0)
        self.ctrl.pid_theta.setPoint = (180.0)
        self.default = 10
        # ctrl.pid_theta.setIntegrator(100)
        # ctrl.pid_theta.setDerivator(100)

        # Set the x (forward/backward) controller
        self.ctrl.pid_x.setKp = ctrl.pid_x.setKp
        self.ctrl.pid_x.setKi = ctrl.pid_x.setKi
        self.ctrl.pid_x.setKd = ctrl.pid_x.setKd
        # ctrl.pid_x.setKd(0.0)
        self.ctrl.pid_x.setPoint = (500.0)
        #ctrl.pid_x.setIntegrator(5000.0)
        #ctrl.pid_x.setDerivator(5000.0)

        # Set the y (left/right) controller
        self.ctrl.pid_y.setKp = ctrl.pid_y.setKp
        # ctrl.pid_y.setKp(0.0)
        self.ctrl.pid_y.setKi = ctrl.pid_y.setKi
        self.ctrl.pid_y.setKd = ctrl.pid_y.setKd
        self.ctrl.pid_y.setPoint = (500.0)
        # ctrl.pid_y.setIntegrator(5000)
        # ctrl.pid_y.setDerivator(5000)

        # Set the z (altitude) controller
        self.ctrl.pid_z.setKp = (0.0)
        self.ctrl.pid_z.setKi = (0.0)
        self.ctrl.pid_z.setKd = (0.0)
        self.ctrl.pid_z.setPoint = (0.0)
        # ctrl.pid_z.setIntegrator(500)
        # ctrl.pid_z.setDerivator(500)

        # Disable hover mode
        self.qc.angular.x = 0.5
        self.qc.angular.y = 0.5

        # controller update values
        self.yaw_update = 0
        self.x_update   = 0
        self.y_update   = 0
        self.z_update   = 0

def is_in_box(minimum, maximum, position):
    """
    Checks if the position is within the given bounds
    """
    if ((minimum < position) and (position < maximum)):
        # print("In box")
        return True
    else:
        # print("Out box")
        return False
#
# def scale_theta(theta, theta_min):
#     """
#     Appropriately scales the given theta using the minimum value of theta. It
#     removes an offset in order to have the QC rotate counterclockwise
#     """
#     if((theta_min < theta) and (theta < 180)):
#         theta -= 360
#
#     # if (-178 < theta) and (theta < 178):
#     #     theta /= (theta - 180)
#     # else:
#     #     theta = default
#     return theta
