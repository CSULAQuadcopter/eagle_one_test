#! /usr/bin/env python
import os, sys, inspect

cmd_folder = os.path.realpath(
    os.path.dirname(
        os.path.abspath(os.path.split(inspect.getfile( inspect.currentframe() ))[0])))

if cmd_folder not in sys.path:
    sys.path.insert(0, cmd_folder)

from transitions import *
from transitions.extensions import MachineGraphSupport as MGraph
from IPython.display import Image, display, display_png

class Matter(object):
    pass

# Mixin the graph
class MyGraphMachine(MGraph, Machine):
    def __init__(self, *args, **kwargs):
        super(MyGraphMachine, self).__init__(*args, **kwargs)

    def show_graph(self):
        self.graph.draw('state.png', prog='dot')
        display(Image('state.png'))


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
    ['tag_lost', ['takeoff', 'follow', 'land'], 'reacquisition']
]
# foo = Foo()
smach = Machine(states=states, transitions=transitions, initial='secure')

machine = MyGraphMachine(model=Matter(),
                         states=states,
                         transitions=transitions,
                         auto_transitions=False,
                         initial='solid',
                         title="QC States")
machine.show_graph()
