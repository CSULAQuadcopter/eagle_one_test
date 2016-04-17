#!/usr/bin/env python
"""
This will save images streamed along the /ardrone/front/image_raw topic for
a specified amount of time

Created by: Josh Saunders and David Rojas
Date Created: 4/1/2016

Modified by: Josh Saunders
Date Modified: 4/17/2016
"""
from __future__ import print_function

import sys
import cv2

# We're using ROS here
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError

# ROS messages
from sensor_msgs.msg import Image

# For integration with the qc_smach_server
import qc_smach_client as sc

class TakePicture(object):
  #seconds
  def __init__(self, max_time):
    self.node = rospy.init_node('take_picture_mode')
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/front/image_raw",Image,self.callback)
    self.start_time = rospy.Time.now().to_sec()

    self.max_time = max_time
    self.counter = 0

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.counter += 1
    filename = "recon_%d.png" % self.counter
    cv2.imwrite(filename, cv_image)

  # Hey 'Ol Timer
  def timer(self):
      return rospy.Time.now().to_sec() - self.start_time

def main(args):
    max_time = 3     # seconds
    takepicture = TakePicture(max_time)
    rate = rospy.Rate(10)
    # TODO add a way to go to the next state, probably after the while loop
    while not rospy.is_shutdown():
        if(takepicture.timer() > takepicture.max_time):
            break
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
