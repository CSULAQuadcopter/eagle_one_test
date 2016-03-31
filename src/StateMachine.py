#1 /usr/bin/env python

# Import transitions library
from transitions import Machine

# ROS imports
import rospy
from std_msgs.msg import String
from eagle_one_test.srv import *

class Smach(Machine):
    def __init__(self):
        # Define the different states of the machine
        states = ['secure', 'takeoff', 'follow', 'take_picture', \
                  'land', 'emergency', 'reacquisition']

        # Define the transitions between states
        # FORMAT: ['trigger_event', 'source_state', 'destination_state']
        transitions = [
            ['takeoff_command', 'secure', 'takeoff'],
            ['takeoff_alt_reached', 'takeoff', 'follow'],
            ['picture_command', 'follow', 'take_picture'],
            ['picture_taken', 'take_picture', 'land'],
            ['land_alt_reached', 'land', 'secure'],
            ['emergency_condition', ['takeoff', 'follow', 'land'], 'emergency'],
            ['tag_lost', ['takeoff', 'follow', 'land'], 'reacquisition'],
            ['takeoff_tag_found', 'reacquisition', 'takeoff'],
            ['follow_tag_found', 'reacquisition', 'follow'],
            ['land_tag_found', 'reacquisition', 'land'],
            ['timed_out', 'reacquisition', 'emergency']
        ]

        Machine.__init__(self, states=states, transitions=transitions, \
                         initial='secure')

    def handle_state_change(self, req):
        self.change_state(req.transition)
        return self.state

    def state_change_server(self):
        rospy.init_node('qc_smach_server')
        s = rospy.Service('qc_smach', State, self.handle_state_change)
        rospy.spin()

    def change_state(self, transition):
        if((self.is_secure()) and (transition == 'TAKEOFF_COMMAND')):
            self.takeoff_command()
        elif((self.is_takeoff()) and (transition == 'TAKEOFF_ALT_REACHED')):
            self.takeoff_alt_reached()
        elif((self.is_follow()) and (transition == 'PICTURE_COMMAND')):
            self.picture_command()
        elif((self.is_take_picture()) and (transition == 'PICTURE_TAKEN')):
            self.picture_taken()
        elif((self.is_land()) and (transition == 'LAND_ALT_REACHED')):
            self.land_alt_reached()
        elif(self.is_takeoff() or self.is_follow() or self.is_land()):
            if(transition == 'EMERGENCY_CONDITION'):
                self.emergency_condition()
        elif(self.is_reacquisition() and (transition == 'TIMED_OUT')):
            self.timed_out()
        elif(self.is_reacquisition()):
            if(transition == 'TAKEOFF_TAG_FOUND'):
                self.takeoff_tag_found()
            if(transition == 'FOLLOW_TAG_FOUND'):
                self.follow_tag_found()
            if(transition == 'LAND_TAG_FOUND'):
                self.land_tag_found()
