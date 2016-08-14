#! /usr/bin/env python
"""
Test Data
Written by: Josh Saunders
Date: 4/11/2016
Saves messages to a file
Use:      in the terminal type: ./test_data.py [filename_of_bag]
Messages: /ardrone/navdata/tags_xc[0]
          /ardrone/navdata/tags_xc[0]
          /ardrone/navdata/tags_orientation[0]
Files:    'tag_x_'     + argv[1] + '.txt'
          'tag_y_'     + argv[1] + '.txt'
          'tag_theta_' + argv[1] + '.txt'
"""
# We're using ROS
import rospy

# The messages that we need
from std_msgs.msg         import Empty
from ardrone_autonomy.msg import Navdata

# custom classes
from Navdata import navdata_info

# Python libraries
from sys import argv

# Take the argument from the terminal and use it as part of the filename
script, test_name = argv

# Create the file names
tag_x_filename     = 'tag_x_'     + test_name + '.txt'
tag_y_filename     = 'tag_y_'     + test_name + '.txt'
tag_theta_filename = 'tag_theta_' + test_name + '.txt'

# Open/create the files witht the file names
tag_x_target     = open(tag_x_filename,     'w')
tag_y_target     = open(tag_y_filename,     'w')
tag_theta_target = open(tag_theta_filename, 'w')

# ROS initializations
rospy.init_node('test_data_saver')
rate = rospy.Rate(50)

# So we can get the data from the bag
nd = navdata_info()

# Write titles for the data at the top of the file
tag_x_target.write('Time  Tag X Position\n')
tag_y_target.write('Time  Tag Y Position\n')
tag_theta_target.write('Time  Tag Orientation\n')


while not rospy.is_shutdown():
    # Save the data as a string so it cn be written to the files
    line1 = '{}  {}\n'.format(nd.navdata.tm, nd.tag_x)
    line2 = '{}  {}\n'.format(nd.navdata.tm, nd.tag_y)
    line3 = '{}  {}\n'.format(nd.navdata.tm, nd.theta)

    # Write the data to the files
    tag_x_target.write(line1)
    tag_y_target.write(line2)
    tag_theta_target.write(line3)

    # Wait for more data
    rate.sleep()
