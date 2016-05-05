#!/usr/bin/env python
"""
This is to hold the navdata transmitted by the QC and calculate the normalized
position of the tags.

Created by: Josh Saunders
Date Created: 4/11/2016

Modified by: Josh Saunders
Date Modified: 5/5/2016
"""
import rospy
from ardrone_autonomy.msg import Navdata

import math
import normalize_position as norm

class navdata_info(object):
    def __init__(self):
        self.navdata = Navdata()
        self.sub_navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.callback)
        self.tag_acquired = False
        self.theta = 0
        # Euler angles
        self.roll = 0
        self.pitch = 0
        # Altitude
        self.altd = 0
        # These are switched for the controller purposes
        # Raw values taken from the QC
        self.tag_x = 0
        self.tag_y = 0
        # These are switched for the controller purposes
        # Calculated, normalized values
        self.tag_norm_x = 0
        self.tag_norm_y = 0
        # For the polar controller
        self.r     = 0

    def callback(self, data):
        self.navdata = data
        self.altd = data.altd
        self.roll = data.rotY
        self.pitch = data.rotX
        if data.tags_count > 0:
            self.tag_acquired = True
            self.theta = self.navdata.tags_orientation[0]
            # These are actually switched for the controller purposes
            self.tag_x = self.navdata.tags_yc[0]
            self.tag_y = self.navdata.tags_xc[0]
            # Calculate the normalized values
            self.tag_norm_x = norm.real_position(self.tag_x, self.altd, self.pitch)
            self.tag_norm_y = norm.real_position(self.tag_y, self.altd, self.roll)
            self.r     = math.hypot(self.tag_x, self.tag_y)
        else:
            self.tag_acquired = False
