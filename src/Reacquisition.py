"""
If the tag is lost, this will increase the altitude until a specified height
(or ceiling). Transitions to the previous state if the tag is acquired for a
certain amount of time, otherwise it lands the QC.

This inherits from the Mode class.

Subscribers:
    /smach/state to check state and turn on timers if we're in reacquisition
    and turn off if we'er not
    /smach/transition stores the transition to the state so that if the tag is
    reacquired within the specified amount of time, we know which state to go
    back to

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
        # self.sub_state = rospy.Subscriber('/smach/state', \
        #                                      String, self.handle_timer_cb)

        self.sub_transition = rospy.Subscriber('/smach/transition', \
                                             String, self.transition_cb)

        # Publishers
        self.pub_twist = rospy.Publisher('/cmd_vel', \
                                             Twist, queue_size=1)
        self.pub_land = rospy.Publisher('/ardrone/land', \
                                            Empty, queue_size=1)

        # Initialize the timers
        self.altitude = 0
        self.prev_state_timer = rospy.Timer(rospy.Duration(prev_state_timeout), \
                                 self.goto_previous_state)

        self.land_timer = rospy.Timer(rospy.Duration(tag_timeout), \
                                 self.land)

        # Initialize member variables
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
        """
        Saves the QC's altitude, state (NOT the system state), and whether or
        not the tag has been acquired. If the tag is acquired (and the system
        is in reacquisition mode) the land_timer is shutoff. If the tag is not
        acquired (and the system is in reacquisition mode) the land_timer is
        turned on.
        """
        self.altitude = msg.altd
        # Mode of the QC, NOT the state of the state machine
        self.mode = msg.state
        if((msg.tags_count > 0) and (self.state == 'reacquisition')):
            self.tag_acquired = True
            # If we do have the tag we need to stop the tag timer
            self.turn_off_timer(self.land_timer, 'Reac land')
        elif((msg.tags_count < 1) and (self.state == 'reacquisition')):
            self.tag_acquired = False
            # If we don't have the tag we need to start the tag timer
            self.turn_on_timer(self.land_timer, 'Reac land')

    def land(self, event):
        """
        Executed when the land_timer runs out. If the tag has not been
        reacquired within the stated time, this lands the QC
        """
        self.transition.data = 'TIMED_OUT'
        self.pub_land.publish(Empty())
        self.pub_transition.publish(self.transition)
        rospy.loginfo("LAND HO!")

    # If we're above the max altitude don't increase the altitude,
    # otherwise go up!
    def check_altitude(self):
        """
        Makes sure that the QC is at the stated ceiling (max altitude)
        """
        if self.altitude < self.ceiling:
            self.twist.linear.z = 0
        else:
            self.twist.linear.z = self.vels[2]

    def move(self):
        self.check_altitude()
        self.pub_twist.publish(self.twist)
        rospy.loginfo("Gitty up!")

    def goto_previous_state(self, event):
        """
        Goes to the previous state if the tag has been reacquired for the
        stated time
        """
        if(self.transition_in == 'TAKEOFF_TAG_LOST'):
            self.transition.data = 'TAKEOFF_TAG_FOUND'
        elif(self.transition_in == 'FOLLOW_TAG_LOST'):
            self.transition.data = 'FOLLOW_TAG_FOUND'
        elif(self.transition_in == 'LAND_TAG_LOST'):
            self.transition.data = 'LAND_TAG_FOUND'
        self.pub_transition.publish(self.transition)
        rospy.loginfo("Transitioning to back to the previous mode")
        # Stop the previous state timer so that it doesn't keep going
        # NOTE may not be necessary
        # self.turn_off_timer(self.prev_state_timer, 'Reac')

    def transition_cb(self, msg):
        """
        Listens in on the /smach/transition topic and saves the transition so
        that we know which state to go back to if the tag has been reacquired
        """
        self.transition_in = msg.data

    # def handle_timer_cb(self, msg):
    #     """
    #     Turns off the timers if the system is not in reacquisition mode. Turns
    #     on the timers if the system is in the reacquisition mode
    #     """
    #     if(self.state == 'reacquisition'):
    #         self.turn_on_timer(self.prev_state_timer, 'Reac prev state')
    #         self.turn_on_timer(self.land_timer, 'Reac land')
    #         # rospy.loginfo("Reacquisition timers turned on.")
    #     else:
    #         self.turn_off_timer(self.prev_state_timer, 'Reac prev state')
    #         self.turn_off_timer(self.land_timer, 'Reac land')
    #         # rospy.loginfo("Reacquisition timers turned off.")
