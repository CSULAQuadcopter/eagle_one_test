#1 /usr/bin/env python
"""
Server for the state machine ROS service. It uses the transitions
library located at https://github.com/tyarkoni/transitions

Important variables:
    self.transitions:
        FORMAT: ['trigger_event', 'source_state', 'destination_state']

    self.states
        Defines the different states of the state machine

Request:
    String current_state (not necessary, because the state machine keeps track
           of this already so this may be removed)
    String transition (this is sent to the server then the server decides what
           the next state should be based on its current state and the trans-
           ition)

Response:
    String next_state (what is returned by the state machine to the ROS service
           client)


Date Created: 3/30/2016
Created by: Josh Saunders

Date Modified: 4/21/2016
Modified by: Josh Saunders
"""

# Import transitions library
from transitions import Machine

# ROS imports
import rospy
from std_msgs.msg import String
from eagle_one_test.srv import State

class Smach(Machine):
    def __init__(self):
        # Define the different states of the state machine
        states = ['secure', 'takeoff', 'follow', 'take_picture', \
                  'land', 'reacquisition', 'emergency']

        # Define the transitions between states
        # FORMAT: ['trigger_event', 'source_state', 'destination_state']
        transitions = [
            ['takeoff_command', 'secure', 'takeoff'],
            ['takeoff_alt_reached', 'takeoff', 'follow'],
            ['picture_command', 'follow', 'take_picture'],
            ['picture_taken', 'take_picture', 'land'],
            ['land_alt_reached', 'land', 'secure'],
            ['emergency_condition', ['takeoff', 'follow', 'take_picture', 'land'], 'emergency'],
            ['takeoff_tag_lost', 'takeoff', 'reacquisition'],
            ['follow_tag_lost', 'follow', 'reacquisition'],
            ['land_tag_lost', 'land', 'reacquisition'],
            ['take_picture_tag_lost', 'take_picture', 'reacquisition'],
            ['takeoff_tag_found', 'reacquisition', 'takeoff'],
            ['follow_tag_found', 'reacquisition', 'follow'],
            ['land_tag_found', 'reacquisition', 'land'],
            ['take_picture_tag_found', 'reacquisition', 'take_picture'],
            ['timed_out', 'reacquisition', 'emergency'],
            ['reset', 'emergency', 'secure']
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
        elif(self.is_takeoff()):
            if(transition == 'TAKEOFF_ALT_REACHED'):
                self.takeoff_alt_reached()
                rospy.loginfo("Altitude Goal Reached")
            if(transition == 'TAKEOFF_TAG_LOST'):
                self.takeoff_tag_lost()
                rospy.loginfo("Tag lost during takeoff")
            if(transition == 'EMERGENCY_CONDITION'):
                self.emergency_condition()
                rospy.loginfo("Emergency During Takeoff")
        elif(self.is_follow()):
            if(transition == 'PICTURE_COMMAND'):
                self.picture_command()
                rospy.loginfo("Set to Take Picture")
            if(transition == 'FOLLOW_TAG_LOST'):
                self.follow_tag_lost()
                rospy.loginfo("Tag lost during follow")
            if(transition == 'EMERGENCY_CONDITION'):
                self.emergency_condition()
                rospy.loginfo("Emergency During Follow")
        elif(self.is_take_picture()):
            if(transition == 'PICTURE_TAKEN'):
                self.picture_taken()
                rospy.loginfo("Picture Saved")
            if(transition == "EMERGENCY_CONDITION"):
                self.emergency_condition()
                rospy.loginfo("Emergency During Picture Attempt")
        # this can be taken out, transition from take picture to reacquisition
        # if tag is lost during picture taking
            if(transition == "TAKE_PICTURE_TAG_LOST"):
                self.take_picture_tag_lost()
                rospy.loginfo("Tag Lost After Picture Taking")
        elif(self.is_land()):
            if(transition == 'LAND_ALT_REACHED'):
                self.land_alt_reached()
                rospy.loginfo("Prepare to Land")
            if(transition == 'LAND_TAG_LOST'):
                rospy.loginfo("Tag lost during landing")
                self.land_tag_lost()
            if(transition == 'EMERGENCY_CONDITION'):
                self.emergency_condition()
                rospy.loginfo("Emergency During Landing")
        elif((self.is_emergency()) and (transition == 'RESET')):
            rospy.loginfo("Return to secure mode")
            self.reset()
        elif(self.is_reacquisition()):
            if(transition == 'TAKEOFF_TAG_FOUND'):
                self.takeoff_tag_found()
                rospy.loginfo("Tag Found, Returning to Takeoff mode")
            if(transition == 'FOLLOW_TAG_FOUND'):
                self.follow_tag_found()
                rospy.loginfo("Tag Found, Returning to Follow Mode")
            if(transition == 'LAND_TAG_FOUND'):
                self.land_tag_found()
                rospy.loginfo("Tag Found, Returning to Land Mode")
            if(transition == 'TAKE_PICTURE_TAG_FOUND'):
                self.take_picture_tag_found()
                rospy.loginfo("Tag Found, Returning to Take Picture Mode")
            if(transition == 'TIMED_OUT'):
                self.timed_out()
                rospy.loginfo("Reacquisition Timer Expired, Entering Emergency Mode")
