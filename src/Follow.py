#! /usr/bin/env python
'''
Follow Mode Test
Written by: Josh Saunders
Date: 4/11/2016

Modified by: David Rojas, Josh Saunders
Date Modified: 5/16/16, 5/20/2016

This is to test the PID that controls the 4 Degrees Of Freedom (DOF) of the QC
'''
# We're using ROS
import rospy

# Python libraries
import math

# The classes that we're using
from Controller import Controller
from Mode       import Mode
from Navdata    import navdata_info

# The messages that we need
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

class Follow(Mode, Controller):
    def __init__(self, bbx, bby, pid_x, pid_y, pid_z, pid_theta, bounding_box=True):
        # Initialize the node which is inherited from the Mode super class
        Mode.__init__(self, 'follow_mode')
        Controller.__init__(self, pid_x, pid_y, pid_z, pid_theta, bounding_box)

        # Publishers
        self.pub_ctrl = rospy.Publisher('cmd_vel', Twist, queue_size=100)

        ########################
        # Set the bounding box #
        ########################

        self.bbx_max = bbx[1]
        self.bbx_min = bbx[0]
        self.bby_max = bby[1]
        self.bby_min = bby[0]

    def is_in_box(self, minimum, maximum, position):
        '''
        Checks if the position is within the given bounds
        '''
        if ((minimum < position) and (position < maximum)):
            # print("In box")
            return True
        else:
            # print("Out box")
            return False
