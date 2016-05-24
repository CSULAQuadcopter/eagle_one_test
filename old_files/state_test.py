#! /usr/bin/env python
"""
Do we know how to subscribe to topics??? I sure hope so...
"""

# We're using ROS here
import rospy

# ROS message
from std_msgs.msg import String, Empty

def state_cb(msg):
    global state
    state = msg.data
    # print state

rospy.init_node('main')
rate = rospy.Rate(10)

state = 'nada'

sub_state = rospy.Subscriber('/smach/state', String, state_cb, queue_size=1000)

while not rospy.is_shutdown():
    print state
    rate.sleep()
