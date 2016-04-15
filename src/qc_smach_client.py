#!/usr/bin/env python

import sys
import rospy
from eagle_one_test.srv import *

states = ['secure', 'takeoff', 'follow', 'take_picture', \
          'land', 'emergency', 'reacquisition']

transitions = ['TAKEOFF_COMMAND', 'TAKEOFF_ALT_REACHED', 'PICTURE_COMMAND', \
               'PICTURE_TAKEN', 'LAND_ALT_REACHED', 'EMERGENCY_CONDITION', \
               'TAKEOFF_TAG_LOST', 'FOLLOW_TAG_LOST', 'LAND_TAG_LOST', 'TAKE_PICTURE_TAG_LOST' \
               'TAKEOFF_TAG_FOUND' , 'FOLLOW_TAG_FOUND' , 'LAND_TAG_FOUND' , 'TAKE_PICTURE_TAG_FOUND'\
               'TIMED_OUT' , 'RESET']

def send_transition(current_state, transition):
    rospy.wait_for_service('qc_smach')
    try:
        state_machine = rospy.ServiceProxy('qc_smach', State)
        new_state = state_machine(current_state, transition)
        print "%s %s %s"%(current_state, transition, state_machine(current_state, transition))
        return # new_state.new_state
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    for state in states:
        for transition in transitions:
            send_transition(state, transition)
