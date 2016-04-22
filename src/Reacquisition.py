"""

Created by: Josh Saunders
Date Created: 4/21/2016

Modified by:
Date Modified:
"""
# We're using ROS here
import rospy

# ROS messages
from ardrone_autonomy.msg import Navdata
from std_msgs.msg         import String, Empty
from geometry_msgs.msg    import Twist


class Reacquisition(object):
    #                  m/s   mm		  seconds
    def __init__(self, vels, ceiling, timeout):
        # Initialize the node and rate
        self.node = rospy.init_node('reacquisition_mode')

        # Subscribers
        self.sub_navdata = rospy.Subscriber('ardrone/navdata', \
                                             Navdata, self.navdata_cb)
        self.sub_state = rospy.Subscriber('smach/state', \
                                       String, self.state_cb)
        self.sub_transition = rospy.Subscriber('smach/transition', \
                                       String, self.transition_cb)

        # Publishers
        self.pub_twist = rospy.Publisher('/cmd_vel', \
                                             Twist, queue_size=1)
        self.pub_land = rospy.Publisher('/ardrone/land', \
                                            Empty, queue_size=1)

        # NOTE changed to passing transition and state info along
        # /smach/transition and /smach/state topics, respectively
        # self.pub_return_to_state = rospy.Publisher('qc_smach/transitions', String, queue_size=1)
        self.pub_transition = rospy.Publisher('smach/transition', \
                                               String, queue_size=1)

        # Initialize member variables
        self.transition = String()
        self.altitude = 0
        self.state = 0
        self.timer = rospy.Timer(rospy.Duration(timeout), \
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
        self.transition.data = 'TAKEOFF_TAG_LOST'
        self.pub_transition.publish(self.transition)
        rospy.loginfo("Transitioning to reacquisition mode")
        # Stop the timer so that it doesn't keep going
        self.timer.shutdown()

    def transition_cb(self, msg):
        self.transition_in = msg.data
