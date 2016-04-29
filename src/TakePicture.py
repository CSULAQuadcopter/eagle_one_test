#!/usr/bin/env python
"""
This will save images streamed along the /ardrone/front/image_raw topic for
a specified amount of time

Created by: Josh Saunders and David Rojas
Date Created: 4/1/2016

Modified by: Josh Saunders
Date Modified: 4/28/2016
"""
from __future__ import print_function

import sys
import cv2

from Mode import Mode

# We're using ROS here
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError

# ROS messages
from sensor_msgs.msg import Image
from std_msgs.msg    import String

class TakePicture(Mode):
        #seconds
    def __init__(self, picture_time):
        # Initialize the node which is inherited fromt the Mode super class
        super(self.__class__, self).__init__('take_picture_mode')

        self.sub_navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.navdata_cb)
        self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.img_cb)

        self.rate = rospy.rate(10)

        #OpenCV stuff
        self.bridge = CvBridge()

        self.transition = String()
        self.altitude = 0
        self.state = 0
        self.timer = rospy.Timer(rospy.Duration(timeout), self.goto_reacquisition)

        self.picture_time = picture_time
        self.counter = 0

        # Initialize the the cv_image variable
        self.cv_image = None

        # Initialize timer
        self.pic_cmd_timer = rospy.Timer(rospy.Duration(picture_timer), \
                                 self.pic_cmd)

    def img_cb(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def save_image(self):
        self.counter += 1
        while counter =< 50:
            filename = "recon_%d.png" % self.counter
            cv2.imwrite(filename, self.cv_image)
            self.rate.sleep()

    # Hey 'Ol Timer
    def timer(self):
        return rospy.Time.now().to_sec() - self.start_time

    def goto_land(self):
        self.transition.data = 'PICTURE_TAKEN'
        self.pub_transition.publish(self.transition)

    def pic_cmd(self, event):
        self.transition.data = 'PICTURE_COMMAND'
        self.pub_transition.publish(self.transition)
