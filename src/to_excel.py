#! /usr/bin/env python
"""
Test Data
Written by: Josh Saunders
Date: 4/15/2016

Saves data from one file and creates an Excel file from it. This currently only
works for a set of data with two columns.

NOTE: This can only create new files. It CANNOT read or modify existing files
      It will happily overwrite any existing files though ;)

Use: In the terminal type: ./to_excel.py [source] (The filename will have a
     timestamp appended to it to ensure that the filenames are unique.)
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
import datetime

# This let's us write to an Excel file
import xlsxwriter

# Take the argument from the terminal and use it as part of the filename
script, test_name = argv

# Create the file names
now = datetime.datetime.now()
timestamp = str(now.year) + '-' + str(now.month) + '-' + str(now.hour) + '-'
timestamp += str(now.minute) + '-' + str(now.second)
filename = test_name + '-' + timestamp +  '.xlsx'


# Create the Excel files with the given file name
workbook = xlsxwriter.Workbook(filename)


# Add the worksheet
tag_x_workbook_sheet     = workbook.add_worksheet()
tag_y_workbook_sheet     = workbook.add_worksheet()
tag_theta_workbook_sheet = workbook.add_worksheet()


# Widen the first column to make the text clearer.
tag_x_workbook_sheet.set_column('A:A', 20)
tag_y_workbook_sheet.set_column('A:A', 20)
tag_theta_workbook_sheet.set_column('A:A', 20)


# Write the titles of each column
# Horizontal of graph, then vertical
horizontal = 'Time (s)'
vertical   = 'Tag '
tag_x_workbook_sheet.write('A1', horizontal)
tag_y_workbook_sheet.write('A1', horizontal)
tag_theta_workbook_sheet.write('A1', horizontal)

tag_x_workbook_sheet.write('B1', vertical + 'X')
tag_y_workbook_sheet.write('B1', vertical + 'Y')
tag_theta_workbook_sheet.write('B1', vertical + 'Theta')

tag_x_workbook_sheet.write('C1', 'Altitude')
tag_x_workbook_sheet.write('D1', 'Roll')
tag_x_workbook_sheet.write('E1', 'Pitch')

tag_y_workbook_sheet.write('C1', 'Altitude')
tag_y_workbook_sheet.write('D1', 'Roll')
tag_y_workbook_sheet.write('E1', 'Pitch')

# ROS initializations
rospy.init_node('test_data_saver')
rate = rospy.Rate(100)

# So we can get the data from the bag
nd = navdata_info()

# To keep tack of what row we're in (start in row 2)
i = 2

while not rospy.is_shutdown():
    # Write to each Excel file
    # Division by 1,000,000 to convert from microseconds to seconds
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

    i += 1

    # Wait for more data
    rate.sleep()

# Close the Excel file
workbook.close()
