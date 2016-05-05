#! /usr/bin/env python
"""
This attempts to reacquire the tag if it is lost. The QC will increase its
altitude to a predetermined height then remain there for a predetermined
time. If it reacquires the tag it goes back to the previous mode. Otherwise,
it lands where ever it is.

Creatd By: Amando A. Aranda and Josh Saunders
Date Created: March 31, 2016

Modified by: Josh Saunders
Date Modified: 4/26/2016
"""

# We're using ROS here
import rospy

from Reacquisition import Reacquisition

# These are the messages that we're going to need
from std_msgs.msg         import String, Empty
from geometry_msgs.msg    import Twist
from ardrone_autonomy.msg import Navdata

def main():
    velocities = (0.3, 0.3, 0.3) 	 # m/s
    max_alt = 3000  # mm
    tag_timeout = 15 	 # seconds
    prev_state_timeout = 5 # seconds
    reacquisition = Reacquisition(velocities, max_alt, tag_timeout, prev_state_timeout)

    rate = rospy.Rate(10) # 10 Hz

    rospy.loginfo("Started Reacquisition Mode")

    while not rospy.is_shutdown():
        # print reacquisition.transition_in
        if(reacquisition.state == 'reacquisition'):
            reacquisition.move()
            # if(reacquisition.timer() > reacquisition.max_time):
            #     reacquisition.land()
        rate.sleep()


if __name__=='__main__':
    main()
