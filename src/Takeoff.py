# We're using ROS here
"""

Created by: Josh Saunders
Date Created: 4/21/2016

Modified by:
Date Modified:
"""
import rospy

from Mode import Mode

# ROS messages
from ardrone_autonomy.msg import Navdata
from std_msgs.msg         import String, Empty
from geometry_msgs.msg    import Twist


class Takeoff(Mode):
                       # m/s  mm		        seconds
    def __init__(self, speed, max_altitudeGoal, timeout):
        # Initialize the node and rate
        self.node = rospy.init_node('takeoff_mode')

        # Subscribers
        # None yet

        # Publishers
        self.pub_altitude = rospy.Publisher('/cmd_vel', \
                                             Twist, queue_size=1)
        #self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=100)
        self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', \
                                            Empty, queue_size=1)

        # Initialize member variables
        self.timer = rospy.Timer(rospy.Duration(timeout), \
                                 self.goto_reacquisition)

        self.tag_acquired = False

        self.altitude_command = Twist()
        self.altitude_command.linear.z = speed

        # to disable hover mode
        self.altitude_command.angular.x = 0.5
        self.altitude_command.angular.y = 0.5

        self.max_altitudeGoal = max_altitudeGoal
        self.speed = speed

    def navdata_cb(self, msg):
        self.altitude = msg.altd
        # Mode of the QC, NOT the state of the state machine
        self.mode = msg.state
        if(msg.tags_count > 0):
            self.tag_acquired = True
            # If we do have the tag we need to stop the timer
            self.timer.shutdown()
        else:
            self.tag_acquired = False
            # If we don't have the tag we need to start the timer
            self.timer.run()

    def state_cb(self, msg):
    	self.state = msg.data

    def launch(self):
        self.pub_takeoff.publish(Empty())
        rospy.loginfo("Moving on up!")

    # If we're above the max altitude don't increase the altitude,
    # otherwise go up!
    def change_altitude(self, speed):
        self.altitude_command.linear.z = speed
        self.pub_altitude.publish(self.altitude_command)
        rospy.loginfo("Change altitude")

    def goto_follow(self):
        self.transition.data = 'TAKEOFF_ALT_REACHED'
        self.pub_transition.publish(self.transition)

    def goto_reacquisition(self, event):
        self.transition.data = 'TAKEOFF_TAG_LOST'
        self.pub_transition.publish(self.transition)
        rospy.loginfo("Transitioning to reacquisition mode")
        # Stop the timer so that it doesn't keep going
        self.timer.shutdown()
