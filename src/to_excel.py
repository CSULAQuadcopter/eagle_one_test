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

# Create the Excel files with the given file name
tag_x_workbook     = xlsxwriter.Workbook(tag_x_filename)
tag_y_workbook     = xlsxwriter.Workbook(tag_y_filename)
tag_theta_workbook = xlsxwriter.Workbook(tag_theta_filename)

tag_x_workbook_sheet     = tag_x_workbook.add_worksheet()
tag_y_workbook_sheet     = tag_y_workbook.add_worksheet()
tag_theta_workbook_sheet = tag_theta_workbook.add_worksheet()

# Widen the first column to make the text clearer.
tag_x_workbook_sheet.set_column('A:A', 20)
tag_y_workbook_sheet.set_column('A:A', 20)
tag_theta_workbook_sheet.set_column('A:A', 20)

# Write the titles of each column
# Horizontal of graph, then vertical
horizontal = 'Time (us)'
vertical   = 'Tag '
tag_x_workbook_sheet.write('A1', horizontal)
tag_y_workbook_sheet.write('A1', horizontal)
tag_theta_workbook_sheet.write('A1', horizontal)

tag_x_workbook_sheet.write('B1', vertical + 'X')
tag_y_workbook_sheet.write('B1', vertical + 'Y')
tag_theta_workbook_sheet.write('B1', vertical + 'Theta')

# ROS initializations
rospy.init_node('test_data_saver')
rate = rospy.Rate(100)

# So we can get the data from the bag
nd = navdata_info()

# To keep tack of what row we're in (start in row 2)
i = 2

while not rospy.is_shutdown():
    # Write to each Excel file
    tag_x_workbook_sheet.write('A{}'.format(i), nd.navdata.tm)
    tag_x_workbook_sheet.write('B{}'.format(i), nd.tag_x)

    tag_y_workbook_sheet.write('A{}'.format(i), nd.navdata.tm)
    tag_y_workbook_sheet.write('B{}'.format(i), nd.tag_y)

    tag_theta_workbook_sheet.write('A{}'.format(i), nd.navdata.tm)
    tag_theta_workbook_sheet.write('B{}'.format(i), nd.theta)

    i += 1

    # Wait for more data
    rate.sleep()

# Close the Excel files
tag_x_workbook.close()
tag_y_workbook.close()
tag_theta_workbook.close()
