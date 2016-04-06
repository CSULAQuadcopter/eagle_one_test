#!/usr/bin/env python
from __future__ import print_function

import sys

import rospy
import roslib
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

class TakePicture(object):
  #seconds
  def __init__(self, max_time):
    self.node = rospy.init_node('take_picture_mode')
    # self.image_pub = rospy.Publisher("front_camera",Image,queue_size=100)
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
    # print(filename)

   # Hey 'Ol Timer
  def timer(self):
      return rospy.Time.now().to_sec() - self.start_time

def main(args):
    max_time = 3     # seconds
    takepicture = TakePicture(max_time)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if(takepicture.timer() > takepicture.max_time):
            break
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
