#! /usr/bin/env python
# Let's use ROS
import rospy

from Mode import Mode

# We're going to need to use String and Empty messages
from std_msgs.msg import String, Empty

class Emergency(Mode):
    def __init__(self):
        # Subscribe to the state machine state topic
        self.sub_state = rospy.Subscriber('/smach/state', \
                                       String, self.state_cb)

        # Allow the mode to publish a land command
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=100)

        self.state = 'nada'

    def emergency_land(self):
        # Land!
        self.pub_land.publish(Empty())
        print("Emergency detected, Landing immediately")

    def transition_cb(self, msg):
        self.transition = msg.data

    def state_cb(self, msg):
    	self.state = msg.data

    def goto_secure(self):
        self.transition.data = 'RESET'
        self.pub_transition.publish(self.transition)
