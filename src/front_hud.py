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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
from cv_bridge import CvBridge, CvBridgeError



class image_converter:

  def __init__(self, camera):
    self.image_pub = rospy.Publisher("heads_up",Image, queue_size=1000)

    self.bridge = CvBridge()
    # Subscribe to the correct topic
    self.image_sub = rospy.Subscriber("ardrone/" + camera + "/image_raw",Image,self.cv_callback)
    self.navdata_sub = rospy.Subscriber("ardrone/navdata",Navdata,self.navdata_callback)
    self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback)

    self.tag_acquired = False
    self.tag_x = 0
    self.tag_y = 0
    self.tag_length = 0
    self.tag_width = 0
    self.tag_theta = 0

    # HUD information
    self.altitude = 0
    self.vx = 0
    self.vy = 0
    self.i = 0
    self.twist = Twist()

  def cv_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    if self.tag_acquired:
      # Drawing some crosshairs
      self.crosshair(cv_image)

    # Write the info
    self.hud_info(cv_image)
    cv2.imshow("Image window", cv_image)
    # Just had to add this line!
    # self.i += 1
    # im_frame = 'image_test_%4d.png' % self.i
    # im_frame = "image_test.png"
    # cv2.imwrite(im_frame, cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def navdata_callback(self,data):
    # HUD information
    self.altitude = data.altd / 1000.0
    self.vx = data.vx
    self.vy = data.vy
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

  def twist_callback(self, msg):
      self.twist.linear.x = msg.linear.x
      self.twist.linear.y = msg.linear.y
      self.twist.linear.z = msg.linear.z
      self.twist.angular.z = msg.angular.z

  def hud_info(self, cv_image):
    font = cv2.FONT_HERSHEY_PLAIN
    font_color = (0, 255, 0)
    # Make the strings with the HUD info
    altd = "Altitude: %.3f m" % self.altitude
    tag_pos = "Tag: (%d, %d) px" % (self.tag_x, self.tag_y)
    tag_theta = "Tag Theta: %.1f" % self.tag_theta
    vx = "Vx: %d mm/s" % self.vx
    vy = "Vy: %d mm/s" % self.vy

    info = "Sent Velocities"
    linear_x = "Vx: %.3f mm/s" % self.twist.linear.x
    linear_y = "Vy: %.3f mm/s" % self.twist.linear.y
    linear_z = "Vz: %.3f mm/s" % self.twist.linear.z
    angular_z = "Rz: %.3f rad/s" % self.twist.angular.z

    # Put the text on the image
    # Top left
    cv2.putText(cv_image, altd, (0, 15), font, 1.25, font_color)
    cv2.putText(cv_image, tag_pos, (0, 32), font, 1.25, font_color)
    cv2.putText(cv_image, tag_theta, (0, 48), font, 1.25, font_color)
    cv2.putText(cv_image, vx, (0, 64), font, 1.25, font_color)
    cv2.putText(cv_image, vy, (0, 80), font, 1.25, font_color)
    # Bottom left
    cv2.putText(cv_image, info, (0, 285), font, 1.25, font_color)
    cv2.putText(cv_image, linear_x, (0, 300), font, 1.25, font_color)
    cv2.putText(cv_image, linear_y, (0, 315), font, 1.25, font_color)
    cv2.putText(cv_image, linear_z, (0, 330), font, 1.25, font_color)
    cv2.putText(cv_image, angular_z, (0, 345), font, 1.25, font_color)

  def crosshair(self, cv_image):
    # Draw the vertical line, then the horizontal, then the circle
    cv2.line(cv_image,(self.tag_x, self.tag_y + 25),(self.tag_x, self.tag_y - 25),(255,255,0),2)
    cv2.line(cv_image,(self.tag_x - 25, self.tag_y),(self.tag_x + 25, self.tag_y),(255,255,0),2)
    cv2.circle(cv_image, (self.tag_x, self.tag_y), 10, (255, 255, 0), 2)


def main(args):
  ic = image_converter("front")
  rospy.init_node('hud', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
