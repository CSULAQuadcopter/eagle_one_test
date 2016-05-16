#! /usr/bin/env python
"""
This mode will command the QC to automatically land.
Created by: Josh Saunders
Date Created: 3/31/2016
Modified by: David Rojas
Date Modified: 4/27/2016
"""
# Let's use ROS
import rospy

# The emergency class
from Emergency import Emergency

state = 'nada'

def state_cb(msg):
    global state
    state = msg.data

# pub_transition = rospy.Publisher('/smach/transition', String, queue_size=1)
sub_state = rospy.Subscriber('/smach/state', String, state_cb, queue_size=1000)

def main():
    emergency = Emergency()

    # Initialize the node and rate
    rospy.init_node('emergency_mode')
    rate = rospy.Rate(100) # 100Hz
    while not rospy.is_shutdown():
        if(state == "emergency"):
            emergency.emergency_land()
        rate.sleep()


if __name__=='__main__':
    main()
