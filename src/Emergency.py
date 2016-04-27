#! /usr/bin/env python
# Let's use ROS
import rospy

from Mode import Mode

# We're going to need to use String and Empty messages
from std_msgs.msg import String, Empty

# For integration with the qc_smach_server
import qc_smach_client as sc

class Emergency(object):
    def __init__(self):
        # Subscribe to the state machine transition topic
        # TODO remove this
        self.sub_transition = rospy.Subscriber('qc_smach/transitions', String, self.transCallback)

        # Allow the mode to publish a land command
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=100)

        # Initialize the transition
        self.transition = ""

    def emergency_land(self):
        # Land!
        self.pub_land.publish(Empty())
        print("Emergency detected, Landing immediately")

    def transCallback(self, msg):
        self.transition = msg.data

    def goto_secure(self):
        self.transition.data = 'RESET'
        self.pub_transition.publish(self.transition)
