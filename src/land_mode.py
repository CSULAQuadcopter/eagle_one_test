#! /usr/bin/env python
"""
Makes the QC land and decrease altitude until it reaches a specified
altitude to land.
If the tag is lost, a timer is started and if the tag cannot be recovered
within a specified amount of time, it will enter the reacquisition mode. If the
tag is recovered within the specified amount of time, it will continue on with
its function (getting the QC to decrease to a specified height).

Created by: Amando Aranda
Date Created: March 31, 2016

Modified by: David Rojas
Date Modified: 5/16/2016
"""
# We're using ROS here
import rospy

# Python libraries

# ROS message
from std_msgs.msg import String, Empty

# Classes we need
from Land import Landing
from Controller import Controller
from Navdata import navdata_info

# The messages that we need
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

#
# state = 'nada'
#
# def state_cb(msg):
#     global state
#     state = msg.data
# ##
# #pub_transition = rospy.Publisher('/smach/transition', String, queue_size=1)
# sub_state = rospy.Subscriber('/smach/state', String, state_cb, queue_size=1000)

def main():
    speed = -.5	 # m/s
    min_altitude = 1000  # mm
    max_altitudeGoal = 2500 # mm
    height_diff = 0 #mm
    timeout  = 3 # seconds
    land  = Landing(speed, min_altitude, height_diff, max_altitudeGoal, timeout)
    rate     = rospy.Rate(200) # 200Hz
    pub_ctrl = rospy.Publisher('cmd_vel', Twist, queue_size=100)

    qc       = Twist()
    navdata  = navdata_info()
    ctrl     = Controller()


    while((land.state != 'land')):
        # print takeoff.state
        rate.sleep()

    while not rospy.is_shutdown():
        # print "%d" % land.pwm
        # always update the altitude
        height_diff = land.max_altitudeGoal - land.altitude
        # We only want to execute these manuevers if we're in land mode
        if land.state == 'land':
            # print("%.3f m" % land.altitude)
            if(land.altitude > land.min_altitude):
                print("Go down")
                if(land.height_diff > land.min_altitude):
                    land.change_altitude(speed)
                    print("Descending")
                elif(land.height_diff <= land.min_altitude):
                    pass
            elif((land.altitude < land.min_altitude) and (land.pwm == 0)):
                print("Eagle one has descended!")
                land.land()
                # To change states, we publish the fact that we've
                # reached our land altitude
                rospy.loginfo("Going to secure mode")
                land.goto_secure()

        pub_ctrl.publish(qc)
        rate.sleep()

if __name__=='__main__':
    main()
