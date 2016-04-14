#! /usr/bin/env python
import rospy

from ardrone_autonomy.msg import Navdata

# Simple proportional controller for the orientation
class YawControl(object):
    def __init__(self, max_theta=350, min_theta=10, rotation_rate=0.5):
        self.max_theta = max_theta
        self.min_theta = min_theta

        #  0 < rotation_rate <= 1.0
        self.rotation_rate = rotation_rate
        self.tag_aqcuired = False
        self.yaw = 0.0

        # rosify
        self.sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, self.yaw_callback)

    def check_yaw(self):
        if ((self.yaw < self.max_theta) and (self.yaw > self.min_theta)):
            if ((self.yaw < self.max_theta) and (self.yaw > 180)):
                return self.rotation_rate
            else:
                return -self.rotation_rate
        else:
            self.yaw = 0
            return 0

    def yaw_callback(self, msg):
        if(msg.tags_count > 0):
            self.yaw = msg.tags_orientation[0]
        else:
            # to stop the rotation of the qc when the tag is not acquired
            self.yaw = 0
