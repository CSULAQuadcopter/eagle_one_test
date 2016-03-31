#! /usr/bin/env python
import rospy

# For the state machine
# from transitions import Machine
from StateMachine import Smach

# To graph the state machine diagram
# import pygraphviz

# Import the service
from eagle_one_test.srv import *

# Initialize the state

smach = Smach()

if __name__=='__main__':
    smach.state_change_server()
