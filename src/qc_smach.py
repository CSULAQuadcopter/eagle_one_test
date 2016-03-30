#! /usr/bin/env python

# So we can use ROS
import rospy

# ROS message to receive and send out messages

# Include our custom state machine
from StateMachine import Smach


if __name__=='__main__':
    rospy.init_node('state_machine')
    smach = Smach()
