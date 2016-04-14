#!/usr/bin/env python
"""
Navdata
Written by: Josh Saunders
Date 4/11/2016
This is to hold the navdata transmitted by the QC
"""
import rospy
from ardrone_autonomy.msg import Navdata

class navdata_info(object):
    def __init__(self):
        self.navdata = Navdata()
        self.sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, self.callback)
        self.tag_acquired = False
        self.theta = 0
        # These are actually switched for the controller purposes
        self.tag_x = 0
        self.tag_y = 0

    def callback(self, data):
        self.navdata = data
        if data.tags_count > 0:
            self.tag_acquired = True
            self.theta = self.navdata.tags_orientation[0]
            # These are actually switched for the controller purposes
            self.tag_x = self.navdata.tags_yc[0]
            self.tag_y = self.navdata.tags_xc[0]
        else:
            self.tag_acquired = False
