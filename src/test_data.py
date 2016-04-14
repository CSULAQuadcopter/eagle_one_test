#! /usr/bin/env python
"""
Test Data
Written by: Josh Saunders
Date: 4/11/2016

Saves messages to a file
"""
# We're using ROS
import rospy

# custom classes
from Navdata import navdata_info

# Python libraries
from sys import argv

script, test_name = argv

filename1 = 'tag_x_' + test_name + '.txt'
filename2 = 'tag_y_' + test_name + '.txt'
filename3 = 'tag_theta_' + test_name + '.txt'

# The messages that we need
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

target1 = open(filename1, 'w')
target2 = open(filename2, 'w')
target3 = open(filename3, 'w')

rospy.init_node('test_data_saver')
rate = rospy.Rate(50)
nd = navdata_info()
target1.write('Time  Tag X Position\n')
target2.write('Time  Tag Y Position\n')
target3.write('Time  Tag Orientation\n')
while not rospy.is_shutdown():
    line1 = '{}  {}\n'.format(nd.navdata.tm, nd.tag_x)
    line2 = '{}  {}\n'.format(nd.navdata.tm, nd.tag_y)
    line3 = '{}  {}\n'.format(nd.navdata.tm, nd.theta)
    target1.write(line1)
    target2.write(line2)
    target3.write(line3)
    rate.sleep()
