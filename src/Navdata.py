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
from ardrone_autonomy.msg import Navdata, navdata_altitude
from std_msgs.msg import Float32
import math

import math
import normalize_position as norm

class navdata_info(object):
    def __init__(self, rate=200):
        self.navdata = Navdata()
        self.sub_navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.navdata_cb)
        self.sub_altitude = rospy.Subscriber('/ardrone/navdata_altitude', navdata_altitude, self.altd_cb)
        # self.pub_alt = rospy.Publisher('/altd', Float32, queue_size=100)
        self.tag_acquired = False
        self.theta = 0
        # Euler angles
        self.roll = 0
        self.pitch = 0
        # Altitude
        self.altitude = 0
        # These are switched for the controller purposes
        # Raw values taken from the QC
        self.tag_x      = 0
        self.prev_tag_x = 0
        self.tag_vx     = 0

        self.tag_y      = 0
        self.prev_tag_y = 0
        self.tag_vy     = 0

        self.rate = rate
        # These are switched for the controller purposes
        # Calculated, normalized values
        self.norm_tag_x = 0
        self.norm_prev_tag_x = 0
        self.norm_tag_vx     = 0

        self.norm_tag_y = 0
        self.norm_prev_tag_y = 0
        self.norm_tag_vy     = 0
        # For the polar controller
        self.r     = 0

    def navdata_cb(self, data):
        self.navdata = data
        self.roll = math.radians(data.rotY)
        self.pitch = math.radians(data.rotX)
        if data.tags_count > 0:
            self.tag_acquired = True
            self.theta = self.navdata.tags_orientation[0]
            # These are actually switched for the controller purposes
            self.prev_tag_x = self.tag_x
            self.tag_x = self.navdata.tags_yc[0]
            self.tag_vx = (self.tag_x - self.prev_tag_x) * self.rate

            self.prev_tag_y = self.tag_y
            self.tag_y = self.navdata.tags_xc[0]
            self.tag_vy = (self.tag_y - self.prev_tag_y) * self.rate
            # Calculate the normalized values
            self.norm_prev_tag_x = self.tag_x
            self.norm_tag_x = norm.real_position(self.tag_x - 500, self.altitude, self.pitch)
            self.norm_tag_vx = (self.norm_tag_x - self.norm_prev_tag_x) * self.rate

            self.norm_prev_tag_y = self.tag_y
            self.tag_norm_y = norm.real_position(self.tag_y - 500, self.altitude, self.roll)
            self.norm_tag_vy = (self.norm_tag_y - self.norm_prev_tag_y) * self.rate

            # Polar controller
            self.r = math.hypot(self.tag_x, self.tag_y)
        else:
            self.tag_acquired = False

    def altd_cb(self, data):
        # Convert the altitude to m (from mm)
        # self.altitude = data.altitude_raw/1000.0
        self.altitude = data.altitude_raw
        # alt = Float32()
        # # alt.data = self.altitude
        # pub_alt.publish(alt)
