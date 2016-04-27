
#! /usr/bin/env python
import rospy

# For the state machine
from StateMachine import Smach

# To graph the state machine diagram
import pygraphviz

# ROS messages and services
from std_msgs.msg import String
from eagle_one_test.msg import State

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
state_pub = rospy.Publisher('smach/state', String, queue_size=100)
transition_pub = rospy.Subscriber('smach/transition', String, transition_cb)



if __name__=='__main__':
    while not rospy.is_shutdown():
        smach.change_state(transition.data)
        print("New state: %s" % smach.state)
        state.data = smach.state
        state_pub.publish(state)
        rate.sleep()
