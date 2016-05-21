# We're using ROS here
"""
This is the base class for all of the modes. It provides the basic
functionality needed for a mode to operate:
    Creates a node
    Know the current state of the system (subscribers)
    Let the system know when key events have occured (publishers)

Subscribes to:
    /ardrone/navdata
    /smach/state

Publishes to:
    /smach/transition

Created by: Josh Saunders
Date Created: 4/26/2016

Modified by:
Date Modified:
"""
import rospy

# ROS messages
from ardrone_autonomy.msg import Navdata, navdata_altitude
from std_msgs.msg         import String

from Navdata import navdata_info

class Mode(object):
    def __init__(self, node_name):
        # Initialize the node and rate
        self.node = rospy.init_node(node_name)

        # Subscribers
        # self.sub_navdata = rospy.Subscriber('/ardrone/navdata', \
        #                                      Navdata, self.navdata_cb)
        self.sub_state = rospy.Subscriber('/smach/state', \
                                       String, self.state_cb)

        self.sub_altitude = rospy.Subscriber('/ardrone/navdata_altitude', \
                                       navdata_altitude, self.altitude_cb)

        self.sub_pwm = rospy.Subscriber('/ardrone/navdata', \
                                       Navdata, self.pwm_cb)

        self.sub_tag = rospy.Subscriber('/ardrone/navdata', \
                                       Navdata, self.has_tag_cb)
        # Publishers
        # NOTE changed to passing transition and state info along
        # /smach/transition and /smach/state topics, respectively
        self.pub_transition = rospy.Publisher('/smach/transition', \
                                               String, queue_size=1)

        # Initialize member variables
        self.transition = String()
        self.state = 'nada'
        self.mode = 0
        self.roll = 0
        self.pitch = 0
        self.altitude = 0
        self.pwm = 0

        self.tag_acquired = False

    def navdata_cb(self, msg):
        """
        Retrieves necessary information about the QC from the /ardrone/navdata
        topic.
        """
        # Mode of the QC, NOT the state of the state machine
        self.mode = msg.state
        self.roll = math.radians(data.rotY)
        self.pitch = math.radians(data.rotX)
        if data.tags_count > 0:
            self.theta = self.navdata.tags_orientation[0]
            # These are actually switched for the controller purposes
            self.tag_x = self.navdata.tags_yc[0]
            self.tag_y = self.navdata.tags_xc[0]

    def pwm_cb(self, msg):
        self.pwm = msg.motor1
        # # faking data, change this to get real data
        # self.pwm = 1

    def altitude_cb(self, msg):
        self.altitude = msg.altitude_raw/1000.0

    def has_tag_cb(self, msg):
        if msg.tags_count > 0:
            self.tag_acquired = True
        else:
            self.tag_acquired = False

    def state_cb(self, msg):
        """
        Sets the state of the system
        """
    	self.state = msg.data

    def turn_off_timer(self, timer, mode):
        """
        Checks if a timer is running. If it is, it turns it off. If it isn't
        running, then it leaves it alone.
        """
        if not (timer._shutdown == True):
            timer.shutdown()
            timer._shutdown = True
        rospy.loginfo("%s timer turned off" % mode)

    def turn_on_timer(self, timer, mode):
        """
        Checks if a timer is running. If it isn't running, it turns it on. If
        it is, then it leaves it alone.
        """
        if timer._shutdown:
            timer._shutdown = False
            timer.run()
        rospy.loginfo("%s timer turned on" % mode)
