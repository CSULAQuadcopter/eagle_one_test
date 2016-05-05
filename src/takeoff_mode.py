#! /usr/bin/env python
"""
Makes the QC takeoff and increase altitude until it reaches a specified
altitude.
If the tag is lost, a timer is started and if the tag cannot be recovered
within a specified amount of time, it will enter the reacquisition mode. If the
tag is recovered within the specified amount of time, it will continue on with
its function (getting the QC to a specified height).

Created by: David Rojas
Date Created: April 11, 2016

Modified by: Josh Saunders
Date Modified: 4/21/2016
"""
# We're using ROS here
import rospy

# ROS message
from std_msgs.msg import String

# The Takeoff class
from Takeoff import Takeoff

# if landed, takeoff
# TODO only works when max_altitudeGoal is 3000 mm or lower, need to fix
def main():
    speed = 1	 # m/s
    max_altitudeGoal = 2000  # mm
    timeout = 5 # seconds
    takeoff = Takeoff(speed, max_altitudeGoal, timeout)
    rate = rospy.Rate(10) # 100Hz

    # To let us know that the mode is working
    rospy.loginfo("Started Takeoff Mode")

    while((takeoff.state != 'takeoff')):
        # print takeoff.state
        rate.sleep()

    # To get this guy to take off! For some reason just calling this function
    # once does not work. This value (50) was determined experimentally
    i = 0
    while i < 50:
        takeoff.launch()
        i += 1
        rate.sleep()

    while not rospy.is_shutdown():
            # We only want to execute these manuevers if we're in takeoff mode
            # print takeoff.state
            if takeoff.state == 'takeoff':
                if(takeoff.altitude < takeoff.max_altitudeGoal):
                    rospy.loginfo("Go up!")
                    takeoff.change_altitude(speed)
                elif(takeoff.altitude >= takeoff.max_altitudeGoal):
                    speed = 0
                    rospy.loginfo("Stop!")
                    takeoff.change_altitude(speed)
                    # To change states, we publish the fact that we've
                    # reached our takeoff altitude
                    rospy.loginfo("Going to follow mode")
                    takeoff.goto_follow()
                rate.sleep()


if __name__=='__main__':
    main()
