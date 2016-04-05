#!/usr/bin/env python
from __future__ import print_function
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from math import cos, sin
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
from cv_bridge import CvBridge, CvBridgeError



class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1000)

    self.bridge = CvBridge()
    # Subscribe to the correct topic
    self.image_sub = rospy.Subscriber("ardrone/bottom/image_raw",Image,self.callback)
    self.navdata_sub = rospy.Subscriber("ardrone/navdata",Navdata,self.navdata_callback)

    self.tag_acquired = False
    self.tag_x = 0
    self.tag_y = 0
    self.tag_length = 0
    self.tag_width = 0
    self.tag_theta = 0
    # These are the coordinates to draw the bounding box
    self.box_tr = (0,0)
    self.box_br = (0,0)
    self.box_tl = (0,0)
    self.box_bl = (0,0)

    # self.center = ((0,0), (0,0),0)
    # self.bounding_box = 0

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    if self.tag_acquired:
      self.set_box_corners()

    #   tl_x = int(self.tag_x - self.tag_length/2)
    #   tl_y = int(self.tag_y - self.tag_width/2)
    #   tl = (tl_x, tl_y)
    #   br_x = int(self.tag_x + self.tag_length/2)
    #   br_y = int(self.tag_y + self.tag_width/2)
    #   br = (br_x, br_y)
    #   cv2.rectangle(cv_image, tl, br,(255,255,0),1)
      # Draw, in order: top, right, bottom, left side of bounding box
      cv2.line(cv_image,self.box_tl,self.box_tr,(255,255,0),2)
      cv2.line(cv_image,self.box_tr,self.box_br,(255,255,0),2)
      cv2.line(cv_image,self.box_bl,self.box_br,(255,255,0),2)
      cv2.line(cv_image,self.box_bl,self.box_tl,(255,255,0),2)
      # Debug information
    #   print("C:(%d, %d) TW&TH:(%d, %d)" % (self.tag_x, self.tag_y, self.tag_width, self.tag_length))
    #   print("TL: (%d, %d) TR: (%d, %d)" % (self.box_tl[0],self.box_tl[1],self.box_tr[0],self.box_tr[1]))
    #   pts = np.array([self.box_tr, self.box_tl, self.box_br, self.box_bl], np.int32)
    #   cv2.polylines(cv_image, [pts], False, (0, 255, 0))
      # Draw a point over the center of the tag
      cv2.circle(cv_image, self.box_tr, 5, (255, 0, 0), 3)
      cv2.circle(cv_image, self.box_tl, 5, (0, 255, 0), 3)
      cv2.circle(cv_image, self.box_br, 5, (0, 0, 255), 3)
      cv2.circle(cv_image, self.box_bl, 5, (0, 255, 255), 3)

      # Drawing some crosshairs
      cv2.line(cv_image,(self.tag_x, self.tag_y + 25),(self.tag_x, self.tag_y - 25),(0,255,0),1)
      cv2.line(cv_image,(self.tag_x - 25, self.tag_y),(self.tag_x + 25, self.tag_y),(0,255,0),1)
      cv2.circle(cv_image, (self.tag_x, self.tag_y), 10, (0, 255, 0), 1)
    cv2.imshow("Image window", cv_image)
    # Just had to add this line!
    cv2.imwrite('image_test.png', cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def navdata_callback(self,data):
    if(data.tags_count > 0):
      self.tag_acquired = True
      # The positions need to be scaled due to the actual resolution
      # Actual resolution = 640 x 360
      # Data given as 1000 x 1000
      self.tag_x = int(data.tags_xc[0] * 640/1000)
      self.tag_y = int(data.tags_yc[0] * 360/1000)
      self.tag_theta = data.tags_orientation[0]
      self.tag_length = data.tags_height[0] * 360/1000
      self.tag_width = data.tags_width[0] * 640/1000

    else:
      self.tag_acquired = False

  def set_box_corners(self):
    self.find_box_tr()
    self.find_box_br()
    self.find_box_tl()
    self.find_box_bl()

  def find_box_tr(self):
    tr_x = self.tag_x + self.tag_width/2
    tr_y = self.tag_y + self.tag_length/2
    tr_x = self.rotate_x(tr_x, tr_y)
    tr_y = self.rotate_y(tr_x, tr_y)
    print("tr: (%d, %d)"%(tr_x, tr_y))
    self.box_tr = (tr_x, tr_y)

  def find_box_tl(self):
    tl_x = self.tag_x - self.tag_width/2
    tl_y = self.tag_y + self.tag_length/2
    tl_x = self.rotate_x(tl_x, tl_y)
    tl_y = self.rotate_y(tl_x, tl_y)
    print("tl: (%d, %d)"%(tl_x, tl_y))
    self.box_tl = (tl_x, tl_y)

  def find_box_br(self):
    br_x = self.tag_x + self.tag_width/2
    br_y = self.tag_y - self.tag_length/2
    br_x = self.rotate_x(br_x, br_y)
    br_y = self.rotate_y(br_x, br_y)
    print("br: (%d, %d)"%(br_x, br_y))
    self.box_br = (br_x, br_y)

  def find_box_bl(self):
    bl_x = self.tag_x - self.tag_width/2
    bl_y = self.tag_y - self.tag_length/2
    bl_x = self.rotate_x(bl_x, bl_y)
    bl_y = self.rotate_y(bl_x, bl_y)
    print("bl: (%d, %d)"%(bl_x, bl_y))
    self.box_bl = (bl_x, bl_y)

  def rotate_x(self, x, y):
    theta = self.tag_theta * 3.14159 / 180

    x -= self.tag_x
    y -= self.tag_y

    xnew = x * cos(theta) - y * sin(theta)
    return int(x + self.tag_x)

  def rotate_y(self, x, y):
    theta = self.tag_theta * 3.14159 / 180
    x -= self.tag_x
    y -= self.tag_y

    ynew = x * sin(theta) + y * cos(theta)
    return int(y + self.tag_y)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
