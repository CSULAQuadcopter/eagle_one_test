#! /usr/bin/env python
"""
Test Data
Written by: Josh Saunders
Date: 4/15/2016

Saves data from one file and creates an Excel file from it. This currently only
works for a set of data with two columns

NOTE: This can only create new files. It CANNOT read or modify existing files
      It will happily overwrite any existing files though ;)

Use: In the terminal type: ./to_excel.py [source]
"""
# We're using ROS
import rospy

# The messages that we need
from std_msgs.msg         import Empty
from geometry_msgs.msg    import Pose2D
from ardrone_autonomy.msg import Navdata

# custom classes
from Navdata import navdata_info

# Python libraries
from sys import argv

# This let's us write to an Excel file
import xlsxwriter

# Take the argument from the terminal and use it as part of the filename
script, test_name = argv

# Create the file names
tag_x_filename     = 'tag_x_'     + test_name + '.xlsx'
tag_y_filename     = 'tag_y_'     + test_name + '.xlsx'
tag_theta_filename = 'tag_theta_' + test_name + '.xlsx'

# Kalman filtered data
k_tag_x_filename   = 'k_tag_x_'   + test_name + '.xlsx'
k_tag_y_filename   = 'k_tag_y_'   + test_name + '.xlsx'

# tag_x_norm_filename     = 'tag_x_norm_'     + test_name + '.xlsx'
# Create the Excel files with the given file name
tag_x_workbook     = xlsxwriter.Workbook(tag_x_filename)
tag_y_workbook     = xlsxwriter.Workbook(tag_y_filename)
tag_theta_workbook = xlsxwriter.Workbook(tag_theta_filename)

k_tag_x_workbook     = xlsxwriter.Workbook(k_tag_x_filename)
k_tag_y_workbook     = xlsxwriter.Workbook(k_tag_y_filename)
# tag_x_norm_workbook     = xlsxwriter.Workbook(tag_x_norm_filename)
# tag_y_norm_workbook     = xlsxwriter.Workbook(tag_y_norm_filename)

# Add the worksheet
tag_x_workbook_sheet     = tag_x_workbook.add_worksheet()
tag_y_workbook_sheet     = tag_y_workbook.add_worksheet()
tag_theta_workbook_sheet = tag_theta_workbook.add_worksheet()

k_tag_x_workbook_sheet   = k_tag_x_workbook.add_worksheet()
k_tag_y_workbook_sheet   = k_tag_y_workbook.add_worksheet()
# tag_x_norm_workbook_sheet     = tag_x_norm_workbook.add_worksheet()
# tag_y_norm_workbook_sheet     = tag_y_norm_workbook.add_worksheet()

# Widen the first column to make the text clearer.
tag_x_workbook_sheet.set_column('A:A', 20)
tag_y_workbook_sheet.set_column('A:A', 20)
tag_theta_workbook_sheet.set_column('A:A', 20)

k_tag_x_workbook_sheet.set_column('A:A', 20)
k_tag_y_workbook_sheet.set_column('A:A', 20)
# tag_x_norm_workbook_sheet.set_column('A:A', 20)
# tag_y_norm_workbook_sheet.set_column('A:A', 20)

# Write the titles of each column
# Horizontal of graph, then vertical
horizontal = 'Time (s)'
vertical   = 'Tag '
tag_x_workbook_sheet.write('A1', horizontal)
tag_y_workbook_sheet.write('A1', horizontal)
tag_theta_workbook_sheet.write('A1', horizontal)

k_tag_x_workbook_sheet.write('A1', horizontal)
k_tag_y_workbook_sheet.write('A1', horizontal)
# tag_x_norm_workbook_sheet.write('A1', horizontal)
# tag_y_norm_workbook_sheet.write('A1', horizontal)

tag_x_workbook_sheet.write('B1', vertical + 'X')
tag_y_workbook_sheet.write('B1', vertical + 'Y')
tag_theta_workbook_sheet.write('B1', vertical + 'Theta')

k_tag_x_workbook_sheet.write('B1', vertical + 'X')
k_tag_y_workbook_sheet.write('B1', vertical + 'Y')
# tag_x_norm_workbook_sheet.write('B1', vertical + 'X')
# tag_y_norm_workbook_sheet.write('B1', vertical + 'Y')

tag_x_workbook_sheet.write('C1', 'Altitude')
tag_x_workbook_sheet.write('D1', 'Roll')
tag_x_workbook_sheet.write('E1', 'Pitch')

tag_y_workbook_sheet.write('C1', 'Altitude')
tag_y_workbook_sheet.write('D1', 'Roll')
tag_y_workbook_sheet.write('E1', 'Pitch')

# ROS initializations
rospy.init_node('test_data_saver')
rate = rospy.Rate(100)

kalman = Pose2D()

def kalman_cb(msg):
    global kalman
    kalman.x = msg.x
    kalman.y = msg.y

kalman_sub = rospy.Subscriber('/kalman', Pose2D, kalman_cb)

# So we can get the data from the bag
nd = navdata_info()

# To keep tack of what row we're in (start in row 2)
i = 2

while not rospy.is_shutdown():
    # print("(%f, %f)\r"%(kalman.x, kalman.y))
    # Write to each Excel file
    # Division by 1000000 to convert from microseconds to seconds
    tag_x_workbook_sheet.write('A{}'.format(i), nd.navdata.tm/1000000)
    tag_x_workbook_sheet.write('B{}'.format(i), nd.tag_x)
    tag_y_workbook_sheet.write('C{}'.format(i), nd.altitude)
    tag_y_workbook_sheet.write('D{}'.format(i), nd.roll)
    tag_y_workbook_sheet.write('E{}'.format(i), nd.pitch)

    tag_y_workbook_sheet.write('A{}'.format(i), nd.navdata.tm/1000000)
    tag_y_workbook_sheet.write('B{}'.format(i), nd.tag_y)
    tag_y_workbook_sheet.write('C{}'.format(i), nd.altitude)
    tag_y_workbook_sheet.write('D{}'.format(i), nd.roll)
    tag_y_workbook_sheet.write('E{}'.format(i), nd.pitch)

    tag_theta_workbook_sheet.write('A{}'.format(i), nd.navdata.tm/1000000)
    tag_theta_workbook_sheet.write('B{}'.format(i), nd.theta)

    k_tag_x_workbook_sheet.write('A{}'.format(i), nd.navdata.tm/1000000)
    k_tag_x_workbook_sheet.write('B{}'.format(i), kalman.x)

    k_tag_y_workbook_sheet.write('A{}'.format(i), nd.navdata.tm/1000000)
    k_tag_y_workbook_sheet.write('B{}'.format(i), kalman.y)

    # tag_x_norm_workbook_sheet.write('A{}'.format(i), nd.navdata.tm/1000000)
    # tag_x_norm_workbook_sheet.write('B{}'.format(i), nd.tag_norm_x)
    #
    # tag_y_norm_workbook_sheet.write('A{}'.format(i), nd.navdata.tm/1000000)
    # tag_y_norm_workbook_sheet.write('B{}'.format(i), nd.tag_norm_y)

    i += 1

    # Wait for more data
    rate.sleep()

# Close the Excel files
tag_x_workbook.close()
tag_y_workbook.close()
tag_theta_workbook.close()

k_tag_x_workbook.close()
k_tag_y_workbook.close()
# tag_x_norm_workbook.close()
# tag_y_norm_workbook.close()
