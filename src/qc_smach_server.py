#! /usr/bin/env python
import rospy

# For the state machine
from StateMachine import Smach

# To graph the state machine diagram
# import pygraphviz

# ROS messages and services
from std_msgs.msg import String, Int32
# from eagle_one_test.msg import State

# Initialize the state machine and variables
smach      = Smach()
state      = String()
transition = String()

# Callback functions to handle the data along the /smach/transition topic
def transition_cb(msg):
    transition.data = msg.data

# ROS initializations
rospy.init_node('qc_smach_server')
rospy.loginfo("Hulk Smach!") # Let's us know that this has loaded
rate = rospy.Rate(100) # 100Hz

# Setup the publishers and subscribers
state_pub = rospy.Publisher('/smach/state', String, queue_size=100000)
transition_sub = rospy.Subscriber('smach/transition', String, transition_cb)
counter_pub = rospy.Publisher('/smach/counter', Int32, queue_size=1000)
counter = 0

if __name__=='__main__':
    while not rospy.is_shutdown():
        smach.change_state(transition.data)
        counter_pub.publish(counter)
        print("New state: %s" % smach.state)
        state.data = smach.state
        state_pub.publish(state)
        counter += 1
        rate.sleep()
