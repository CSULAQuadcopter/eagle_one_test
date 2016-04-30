#! /usr/bin/env python
"""
Makes the QC take a picture
Created by: David Rojas
Date Created: March 31, 2016
Modified by: Josh Saunders and David Rojas
Date Modified: 4/25/2016
"""
# We're using ROS here
import rospy

import sys

# ROS message
from std_msgs.msg import String

# The land class
from TakePicture import TakePicture

def main(args):
    picture_time = 30 # seconds
    max_time     = 3 # seconds
    takepicture = TakePicture(picture_time)
    rate = rospy.Rate(10)
    # TODO add a way to go to the next state, probably after the while loop
    while not rospy.is_shutdown():
        if takepicture.state == 'take_picture':
            if(takepicture.finished()):
                break
            rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
