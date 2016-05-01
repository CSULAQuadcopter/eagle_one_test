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
Date Modified: 4/25/2016
"""
# We're using ROS here
import rospy

# ROS message
from std_msgs.msg import String

# The land class
from Land import Landing

def main():
    speed = -1	 # m/s
    min_altitude = 500  # mm
    max_altitudeGoal = 2000 # mm
    height_diff = 0 #mm
    timeout = 3 # seconds
    landing = Landing(speed, min_altitude, height_diff, max_altitudeGoal, timeout)
    rate = rospy.Rate(100) # 100Hz

    while not rospy.is_shutdown():
        height_diff = landing.max_altitudeGoal - landing.altitude
        #if landing.state == 'land':
        print("%d" % landing.altitude)
            # if(landing.tag_acquired):
        if(landing.altitude > landing.min_altitude):
            print("Go down")
            if(landing.height_diff > landing.min_altitude):
                landing.change_altitude(speed)
                print("Descending")
                rospy.sleep(.5)
            elif(landing.height_diff <= landing.min_altitude):
                pass
        elif(landing.altitude < landing.min_altitude):
            print("Eagle one has descended!")
            landing.land()
        rate.sleep()

if __name__=='__main__':
    main()
