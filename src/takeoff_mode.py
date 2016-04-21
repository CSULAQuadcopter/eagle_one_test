#! /usr/bin/env python
# This is the the code for the takeoff mode
#Date: April 11, 2016

# We're using ROS here
import rospy

# ROS message
from std_msgs.msg import String

# For integration with the qc_smach_server
import qc_smach_client as sc

# The Takeoff class
from Takeoff import Takeoff

# if landed, takeoff
# TODO only works when max_altitudeGoal is 3000 mm or lower, need to fix
def main():
    speed = 1	 # m/s
    max_altitudeGoal = 1000  # mm
    #max_time = 20 	 # seconds
    takeoff = Takeoff(speed, max_altitudeGoal)
    rate = rospy.Rate(100) # 100Hz

    # To get this guy to take off!
    i = 0
    while i < 50:
        takeoff.launch()
        i += 1
        rate.sleep()

    while not rospy.is_shutdown():
            print("%d" % takeoff.max_altitudeGoal)
            if(takeoff.altitude < takeoff.max_altitudeGoal):
                print("Go up!")
                takeoff.change_altitude(speed)
            elif(takeoff.altitude > takeoff.max_altitudeGoal):
                speed = 0
                print("Stop!")
                takeoff.change_altitude(speed)
            rate.sleep()


if __name__=='__main__':
    main()
