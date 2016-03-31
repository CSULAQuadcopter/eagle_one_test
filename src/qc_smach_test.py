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

    # Define the different states of the machine
    # states = ['secure', 'takeoff', 'follow', 'take_picture', \
    #           'land', 'emergency', 'reacquisition']

    # Define the transitions between states
    # FORMAT: ['trigger_event', 'source_state', 'destination_state']
    # transitions = [
    #     ['takeoff_command', 'secure', 'takeoff'],
    #     ['takeoff_alt_reached', 'takeoff', 'follow'],
    #     ['picture_command', 'follow', 'take_picture'],
    #     ['picture_taken', 'take_picture', 'land'],
    #     ['land_alt_reached', 'land', 'secure'],
    #     ['emergency_condition', ['takeoff', 'follow', 'land'], 'emergency'],
    #     ['tag_lost', ['takeoff', 'follow', 'land'], 'reacquisition'],
    #     ['takeoff_tag_found', 'reacquisition', 'takeoff'],
    #     ['follow_tag_found', 'reacquisition', 'follow'],
    #     ['land_tag_found', 'reacquisition', 'land']
    # ]
    # foo = Foo()
    # smach = Machine(states=states, transitions=transitions, initial='secure')


    # Check the first state
    # print(smach.state)
    # smach.takeoff_command()
    # smach.to_takeoff()
    # print(smach.state)

    # To draw the state diagram
    # 'auto_transitions=False' Removes the auto_transitions from the state diagram
    # smach = Machine(states=states, transitions=transitions, auto_transitions=False, initial='secure')
    # graph = smach.get_graph()
    # graph.draw('my_state_diagram.png', prog='dot')
