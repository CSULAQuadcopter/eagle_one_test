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
    def __init__(self, speed, max_altitude, tag_timeout):
        # Initialize the node which is inherited from the Mode super class
        super(self.__class__, self).__init__('takeoff_mode')

        # Subscribers
        self.sub_state = rospy.Subscriber('/smach/state', \
                                             String, self.handle_timer_cb)

        # Publishers
        self.pub_altitude = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1000)

        # Initialize member variables
        self.timer = rospy.Timer(rospy.Duration(tag_timeout), self.goto_reacquisition)

        self.altitude_command = Twist()
        self.altitude_command.linear.z = speed

        self.max_altitude = max_altitude
        self.speed = speed
        self.state = 'nada'

    def navdata_cb(self, msg):
        # Mode of the QC, NOT the state of the state machine
        self.mode = msg.state
        if(msg.tags_count > 0):
            self.tag_acquired = True
            # If we do have the tag we need to stop the timer
            self.turn_off_timer(self.timer, 'Takeoff')
        else:
            self.tag_acquired = False
            # If we don't have the tag we need to start the timer
            self.turn_on_timer(self.timer, 'Takeoff')


    def launch(self):
        self.pub_takeoff.publish(Empty())
        rospy.loginfo("Moving on up!")

    # If we're above the max altitude don't increase the altitude,
    # otherwise go up!
    def change_altitude(self):
        self.altitude_command.linear.z = speed
        self.pub_altitude.publish(self.altitude_command)
        rospy.loginfo("Change altitude")

    def goto_follow(self):
        self.transition.data = 'TAKEOFF_ALT_REACHED'
        self.pub_transition.publish(self.transition)

    def goto_reacquisition(self, event):
        self.transition.data = 'TAKEOFF_TAG_LOST'
        self.pub_transition.publish(self.transition)
        rospy.loginfo("Transitioning to Reacquisition from Takeoff")
        # Stop the timer so that it doesn't keep going
        self.turn_off_timer(self.timer, 'Takeoff')

    def handle_timer_cb(self, msg):
        if(self.state == 'takeoff'):
            if (self.tag_acquired):
                self.turn_off_timer(self.timer, 'Takeoff')
            elif (not self.tag_acquired):
                self.turn_on_timer(self.timer, 'Takeoff')
        else:
            self.turn_off_timer(self.timer, 'Takeoff')
