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

# Python libraries
import math
import sys

# ROS message
from std_msgs.msg import String

# The classes that we're using
from Controller import Controller
from Navdata import navdata_info
from geometry_msgs.msg    import Twist
from TakePicture import TakePicture

state = 'nada'

def state_cb(msg):
    global state
    state = msg.data

# pub_transition = rospy.Publisher('/smach/transition', String, queue_size=1)
sub_state = rospy.Subscriber('/smach/state', String, state_cb, queue_size=1000)

#def is_in_box(minimum, maximum, position):
#    """
#    Checks if the position is within the given bounds
#    """
#    if ((minimum < position) and (position < maximum)):
#        # print("In box")
#        return True
#    else:
#        # print("Out box")
#        return False

def main(args):
    picture_time = 1 # seconds
    max_pic_altitude = 4.6 # meters
    speed = 1
    follow_altitude = 2.5 # meters
    takepicture = TakePicture(picture_time, max_pic_altitude, speed, follow_altitude)
    rate = rospy.Rate(10)

    qc = Twist()


    # TODO add a way to go to the next state, probably after the while loop
    while((state != 'take_picture')):
        # print takeoff.state
        rate.sleep()

    while not rospy.is_shutdown():
        # always update the altitude
        ##z_update = ctrl.pid_z.update(z_update)
        # We only want to execute these manuevers if we're in take_picture mode
        if state == 'take_picture':
            if(takepicture.altitude <= takepicture.max_pic_altitude):
                qc.linear.z  = speed
            elif(takepicture.altitude > takepicture.max_pic_altitude):
                speed = 0
                qc.linear.z  = speed
                takepicture.save_image()
                if(takepicture.is_finished == True):
                    speed = -0.05
                    if(takepicture.altitude > takepicture.follow_altitude):
                        qc.linear.z  = speed
                    elif(takepicture.altitude <= takepicture.follow_altitude):
                        takepicture.goto_land()
        ##pub_ctrl.publish(qc)
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
