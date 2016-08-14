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
Date Modified: 4/21/2016, 5/24/2016
"""
# We're using ROS here
import rospy

# ROS messages
from std_msgs.msg import String

# The Takeoff class
from Takeoff import Takeoff

# if landed, takeoff
# TODO only works when altitude_goal is 3.0 m or lower, need to fix
def main():
    speed = 0.75	 # m/s
    altitude_goal = 2.5 # meters
    tag_timeout = 10 # seconds
    takeoff = Takeoff(speed, altitude_goal, tag_timeout)
    rate = rospy.Rate(200) # 100Hz

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
        if takeoff.state == 'takeoff':
            if(takeoff.altitude < altitude_goal):
                rospy.loginfo("Go up!")
                takeoff.change_altitude()
            elif(takeoff.altitude >= altitude_goal):
                speed = 0
                rospy.loginfo("Stop!")
                # takeoff.change_altitude(speed)
                # To change states, we publish the fact that we've
                # reached our takeoff altitude
                rospy.loginfo("Going to follow mode")
                takeoff.goto_follow()
                # Let's change the launch file and then we can
                # break
                # out from the loop
        # else:
        #     continue
        rate.sleep()


if __name__=='__main__':
    main()
