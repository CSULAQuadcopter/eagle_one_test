# We're using ROS here
import rospy

# These are the messages that we're going to need for ROS
from std_msgs.msg           import String, Empty
from geometry_msgs.msg      import Twist
from ardrone_autonomy.msg   import Navdata

class Landing(object):
    # m/s	mm		seconds
    def __init__(self, speed, min_altitude):
        # Initialize the node and rate
        self.node = rospy.init_node('land_mode')

        # Subscribers
        self.sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, self.navdata_cb)
        self.sub_state = rospy.Subscriber('smach/state', String, self.state_cb)

        # Publishers
        self.pub_altitude = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=100) #this turns off the motors

        # NOTE changed to passing transition and state info along
        # /smach/transition and /smach/state topics, respectively
        # self.pub_return_to_state = rospy.Publisher('qc_smach/transitions', String, queue_size=1)
        self.pub_transition = rospy.Publisher('smach/transition', String, queue_size=1)

        # Initialize member variables
        self.transition = String()
        self.altitude = 0
        self.state = 0
        self.timer = rospy.Timer(rospy.Duration(timeout), self.goto_reacquisition)

        self.tag_acquired = False

        self.altitude_command = Twist()
        self.altitude_command.linear.z = speed

        # to disable hover mode
        self.altitude_command.angular.x = 0.5
        self.altitude_command.angular.y = 0.5

        self.min_altitude = min_altitude
        self.speed = speed


    def navdata_cb(self, msg):
        self.altitude = msg.altd
        # Mode of the QC NOT the state of the state machine
        self.mode = msg.state
        if(msg.tags_count > 0):
            self.tag_acquired = True
            self.timer.shutdown()
        else:
            self.tag_acquired = False
            self.timer.run()

    def state_cb(self, msg):
        self.state = msg.data

#This is what makes the drone land
    def land(self):
        self.pub_land.publish(Empty())
        print("LAND HO!")

    # THis is the algo for landing
    def change_altitude(self, min_altitude):
        self.altitude_command.linear.z = self.speed
        self.pub_altitude.publish(self.altitude_command)

    def goto_secure(self):
        self.transition.data = 'LAND_ALT_REACHED'
        self.pub_transition.publish(self.transition)

    def goto_reacquisition(self, event):
        self.transition.data = 'LAND_TAG_LOST'
        self.pub_transition.publish(self.transition)
        rospy.loginfo("Transitioning to reacquisition mode")
        # Stop the timer so that it doesn't keep going
        self.timer.shutdown()
