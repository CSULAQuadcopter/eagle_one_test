"""
If the tag is lost, this will increase the altitude until a specified height
(or ceiling). Transitions to the previous state if the tag is acquired for a
certain amount of time, otherwise it lands the QC.

This inherits from the Mode class.

Subscribers:
    Look to the Mode class

Publishers:
    /cmd_vel to control the altitude of the QC
    /ardrone/land to land the QC
    /smach/transition to let the system know of a state change

Note: prev_state_timeout should be less than tag_timeout otherwise the QC will
      land before it has a lock on the tag for the specified amount of time.


Created by: Josh Saunders
Date Created: 4/21/2016

Modified by: Josh Saunders
Date Modified: 4/28/2016
"""
# We're using ROS here
import rospy

from Mode import Mode

# ROS messages
from ardrone_autonomy.msg import Navdata
from std_msgs.msg         import String, Empty
from geometry_msgs.msg    import Twist

class Reacquisition(Mode):
    #                  m/s   mm		  seconds      seconds
    def __init__(self, vels, ceiling, tag_timeout, prev_state_timeout):
        # Initialize the node which is inherited fromt the Mode super class
        super(self.__class__, self).__init__('reacquisition_mode')

        # Subscribers
        # None yet

        # Publishers
        self.pub_twist = rospy.Publisher('/cmd_vel', \
                                             Twist, queue_size=1)
        self.pub_land = rospy.Publisher('/ardrone/land', \
                                            Empty, queue_size=1)

        # Initialize member variables
        self.altitude = 0
        self.prev_state_timeout = rospy.Timer(rospy.Duration(prev_state_timeout), \
                                 self.goto_previous_state)

        self.tag_timer = rospy.Timer(rospy.Duration(tag_timeout), \
                                 self.land)

        self.transition_in = ''
        self.tag_acquired = False

        self.vels = vels

        self.twist = Twist()
        self.twist.linear.x = self.vels[0]
        self.twist.linear.y = self.vels[1]
        self.twist.linear.z = self.vels[2]

        # to disable hover mode
        self.twist.angular.x = 0.5
        self.twist.angular.y = 0.5

        self.ceiling = ceiling

        # To land, we send an empty message
        self.land_msg = Empty()

    def navdata_cb(self, msg):
        self.altitude = msg.altd
        # Mode of the QC, NOT the state of the state machine
        self.mode = msg.state
        if(msg.tags_count > 0):
            self.tag_acquired = True
            # If we do have the tag we need to stop the tag timer
            self.tag_timer.shutdown()
        else:
            self.tag_acquired = False
            # If we don't have the tag we need to start the tag timer
            self.tag_timer.run()

    def land(self, event):
        self.pub_land.publish(Empty())
        rospy.loginfo("LAND HO!")

    # If we're above the max altitude don't increase the altitude,
    # otherwise go up!
    def check_altitude(self):
        if self.altitude < self.ceiling:
            self.twist.linear.z = 0
        else:
            self.twist.linear.z = self.vels[2]

    def move(self, speed):
        self.check_altitude()
        self.pub_twist.publish(self.twist)
        rospy.loginfo("Gitty up!")

    def goto_previous_state(self, event):
        if(self.transition_in == 'TAKEOFF_TAG_LOST'):
            self.transition.data = 'TAKEOFF_TAG_FOUND'
        elif(self.transition_in == 'FOLLOW_TAG_LOST'):
            self.transition.data = 'FOLLOW_TAG_FOUND'
        elif(self.transition_in == 'LAND_TAG_LOST'):
            self.transition.data = 'LAND_TAG_FOUND'
        self.pub_transition.publish(self.transition)
        rospy.loginfo("Transitioning to reacquisition mode")
        # Stop the previous state timer so that it doesn't keep going
        self.timer.shutdown()

    def land(self):
        rospy.loginfo("Tag could not be reacquired. Landing.")
        self.pub_land.publish(land_msg)

    def transition_cb(self, msg):
        self.transition_in = msg.data
