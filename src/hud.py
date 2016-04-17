#!/usr/bin/env python
"""
QC Heads Up Display (HUD)

Displays sensor information from and commands sent to the QC

Created by: Josh Saunders
Date Created: 4/2/2016
Date Modified: 4/16/2016
"""
# Python libraries
from __future__ import print_function
import sys
import cv2
import math
import numpy as np

# We're using ROS here
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError

# ROS messages
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata

class HUD:
  def __init__(self, box_top_left, box_bottom_right):
    self.image_pub = rospy.Publisher("heads_up",Image, queue_size=1000)

    self.bridge = CvBridge()
    # Subscribe to the correct topic
    self.image_sub = rospy.Subscriber("ardrone/image_raw",Image,self.cv_callback)
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
    self.vz = 0
    self.i = 0
    self.twist = Twist()
    self.battery = 0
    self.mode = 0
    self.pwm1 = 0
    self.pwm2 = 0
    self.pwm3 = 0
    self.pwm4 = 0
    self.time = 0

    # Bounding box dimensions
    # These are tuples
    self.box_top_left = box_top_left
    self.box_bottom_right = box_bottom_right

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
    # Draw the bounding box
    red = (0, 0, 255)
    cv2.rectangle(cv_image, self.box_top_left, self.box_bottom_right, red, 1)
    cv2.imshow("QC HUD", cv_image)
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
    self.vz = data.vz
    self.time = data.tm / 1000000.0
    self.battery = data.batteryPercent
    self.mode = data.state
    self.pwm1 = data.motor1
    self.pwm2 = data.motor2
    self.pwm3 = data.motor3
    self.pwm4 = data.motor4
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
    self.twist.angular.x = msg.angular.x
    self.twist.angular.y = msg.angular.y
    self.twist.angular.z = msg.angular.z

  def hud_info(self, cv_image):
    font = cv2.FONT_HERSHEY_PLAIN
    font_color = (0, 255, 0)

    # These are taken from Mike Hamer's ardrone_tutorials package and
    # the ardrone_autonomy package documentation
    mode = [
        'Emergency', 'Inited', 'Landed', 'Flying', 'Hovering', 'Test',
        'Taking Off', 'Flying', 'Landing', 'Looping'
        ]

    # Make the strings with the HUD info
    altd      = "Altitude: %.3f m" % self.altitude
    tag_pos   = "Tag: (%d, %d) px" % (self.tag_x, self.tag_y)
    tag_theta = "Tag Theta: %.1f"  % self.tag_theta
    vx_est    = "Vx: %.2f mm/s"    % self.vx
    vy_est    = "Vy: %.2f mm/s"    % self.vy
    # vz_est = "Vz: %.2f mm/s" % self.vz
    time      = "Time: %f "        % self.time

    info = "Sent Velocities"
    linear_x  = "Vx: %.3f mm/s"  % self.twist.linear.x
    linear_y  = "Vy: %.3f mm/s"  % self.twist.linear.y
    linear_z  = "Vz: %.3f mm/s"  % self.twist.linear.z
    angular_x = "Rx: %.3f rad/s" % self.twist.angular.x
    angular_y = "Ry: %.3f rad/s" % self.twist.angular.y
    angular_z = "Rz: %.3f rad/s" % self.twist.angular.z

    pwm1 = "PWM1: %d" % self.pwm1
    pwm2 = "PWM2: %d" % self.pwm2
    pwm3 = "PWM3: %d" % self.pwm3
    pwm4 = "PWM4: %d" % self.pwm4

    battery = "Battery: %.1f%%" % self.battery
    state   = "Mode: %s"        % mode[self.mode]
    battery_font_color = self.set_battery_font(60, 30)

    # Put the text on the image
    # Top left
    cv2.putText(cv_image, altd,      (0, 15), font, 1.25, font_color)
    cv2.putText(cv_image, tag_pos,   (0, 32), font, 1.25, font_color)
    cv2.putText(cv_image, tag_theta, (0, 48), font, 1.25, font_color)
    cv2.putText(cv_image, vx_est,    (0, 64), font, 1.25, font_color)
    cv2.putText(cv_image, vy_est,    (0, 80), font, 1.25, font_color)
    # cv2.putText(cv_image, vz_est,    (0, 96), font, 1.25, font_color)
    cv2.putText(cv_image, time,      (0, 96), font, 1.25, font_color)
    # Bottom left
    cv2.putText(cv_image, info,      (0, 265), font, 1.25, font_color)
    cv2.putText(cv_image, linear_x,  (0, 280), font, 1.25, font_color)
    cv2.putText(cv_image, linear_y,  (0, 295), font, 1.25, font_color)
    cv2.putText(cv_image, linear_z,  (0, 310), font, 1.25, font_color)
    cv2.putText(cv_image, angular_x, (0, 325), font, 1.25, font_color)
    cv2.putText(cv_image, angular_y, (0, 340), font, 1.25, font_color)
    cv2.putText(cv_image, angular_z, (0, 355), font, 1.25, font_color)
    # Top right
    cv2.putText(cv_image, pwm1, (520, 15), font, 1.25, font_color)
    cv2.putText(cv_image, pwm2, (520, 32), font, 1.25, font_color)
    cv2.putText(cv_image, pwm3, (520, 48), font, 1.25, font_color)
    cv2.putText(cv_image, pwm4, (520, 64), font, 1.25, font_color)
    # Bottom right
    cv2.putText(cv_image, battery, (440, 340), font, 1.25, battery_font_color)
    cv2.putText(cv_image, state,   (440, 355), font, 1.25, font_color)
    # Draw velocity vector
    self.direction_arrow(cv_image)

  def crosshair(self, cv_image):
    # Draw the vertical line, then the horizontal, then the circle
    cv2.line(cv_image,   (self.tag_x, self.tag_y + 25),(self.tag_x, self.tag_y - 25),(255,255,0),2)
    cv2.line(cv_image,   (self.tag_x - 25, self.tag_y),(self.tag_x + 25, self.tag_y),(255,255,0),2)
    cv2.circle(cv_image, (self.tag_x, self.tag_y), 10, (255, 255, 0), 2)

  # work in progress
  def direction_arrow(self, cv_image):
    # Draw the arrow that show the direction in which the QC is moving
    vx = self.twist.linear.x
    vy = self.twist.linear.y
    # find the angle between the velocities
    # TODO check the math here
    if ((vx > 0) and (vy > 0)):
        angle = math.atan2(vx, vy)
        # print("1st")
    elif ((vx < 0) and (vy > 0)):
        angle = math.pi - math.atan2(vx, vy)
        # print("2nd")
    elif ((vx < 0) and (vy < 0)):
        angle = math.pi + math.atan2(vx, vy)
        # print("3rd")
    else:
        angle = 2 * math.pi - math.atan2(vx, vy)
        # print("4th")

    # print("%.3f" % angle)
    color = (255, 2, 255)
    center = (520, 270)
    radius = 50
    thickness = 1
    vel_end = (center[0] + int(radius * math.cos(angle)), \
               center[1] + int(radius * math.sin(angle)))

    # Draw the circle and line
    if not (vx == 0 or vy ==0):
        cv2.line(cv_image,   center, vel_end, color, thickness)
    cv2.circle(cv_image, center, radius,  color, thickness)

  def set_battery_font(self, medium, low):
    # This is to color the battery font based on the battery level
    # green > medium, yellow > low, red < low
    if self.battery > medium:
      battery_font_color = (0, 255, 0)
    elif self.battery > low:
      battery_font_color = (0, 255, 255)
    else:
      battery_font_color = (0, 0, 255)
    return battery_font_color


def main(args):
  rospy.init_node('hud', anonymous=True)
  top_left = (240, 135)
  bottom_right = (400, 225)
  hud = HUD(top_left, bottom_right)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
