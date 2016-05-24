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

import cv2

from Mode import Mode

# We're using ROS here
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError

# ROS messages
from sensor_msgs.msg      import Image
from std_msgs.msg         import String, Empty
from ardrone_autonomy.msg import Navdata

class TakePicture(Mode):
        #seconds
    def __init__(self, picture_time, max_pic_altitude, speed, follow_altitude, height_diff):
        # Initialize the node which is inherited fromt the Mode super class
        super(self.__class__, self).__init__('take_picture_mode')

        #ROS Subscribers
        self.sub_navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.navdata_cb)
        self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.img_cb)

        #ROS Publishers
        self.pub_altitude = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_transition = rospy.Publisher('smach/transition', String, queue_size=1)

        self.rate = rospy.Rate(10)

        #OpenCV stuff
        self.bridge = CvBridge()

        self.altitude = 0
        #self.start_time = None
        self.state = 'nada'

        # We don't go to reacquisition from here
        # self.timer = rospy.Timer(rospy.Duration(timeout), self.goto_reacquisition)

        self.altitude_command = Twist()
        self.altitude_command.linear.z = speed

        self.max_pic_altitude = pic_altitude
        self.speed = speed
        self.follow_altitude = follow_altitude
        self.height_diff = height_diff

        self.picture_time = picture_time
        self.counter = 0
        self.is_finished = False

        # Initialize the the cv_image variable
        self.cv_image = None

        # Initialize timer
        self.pic_cmd_timer = rospy.Timer(rospy.Duration(picture_time), \
                                 self.pic_cmd)

    def change_altitude(self, speed):
        self.altitude_command.linear.z = speed
        self.pub_altitude.publish(self.altitude_command)
        rospy.loginfo("Change altitude")

    def height_diff(self, max_pic_altitude, follow_altitude):
        self.height_diff = self.max_altitudeGoal - self.follow_altitude

    def img_cb(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def save_image(self):
        self.counter += 1
        rospy.loginfo("Taking Pictures")
        while self.counter <= 50:
            filename = "recon_%d.png" % self.counter
            cv2.imwrite(filename, self.cv_image)
            self.counter += 1
            self.rate.sleep()
        rospy.loginfo("Finished Taking Pictures")
        self.goto_land()
        self.finished()

    def finished(self):
        self.is_finished = True

    #def start_timer(self):
    #    self.start_time = rospy.Time.now().to_sec()

    # Hey 'Ol Timer
    # Probably not necessary
    # def handle_timer(self):
    #     if (self.state == 'take_picture'):
    #         self.pic_cmd_timer.run()
    #     else:
    #         self.pic_cmd_timer.shutdown()

    def goto_land(self):
        rospy.loginfo("Going to Land Mode")
        self.transition.data = 'PICTURE_TAKEN'
        self.pub_transition.publish(self.transition)

    def pic_cmd(self, event):
        self.transition.data = 'PICTURE_COMMAND'
        self.pub_transition.publish(self.transition)
        self.save_image()

    def navdata_cb(self, msg):
        self.altitude = msg.altd
        # Mode of the QC, NOT the state of the state machine
        self.mode = msg.state
        if(msg.tags_count > 0):
            self.tag_acquired = True
            # If we do have the tag we need to stop the timer
            # self.turn_off_timer(self.timer, 'Take Picture')
            # rospy.loginfo("Take picture timer turned off.")
        else:
            self.tag_acquired = False
            # If we don't have the tag we need to start the timer
            # self.turn_on_timer(self.timer, 'Take Picture')
            # rospy.loginfo("Take picture timer turned on.")
