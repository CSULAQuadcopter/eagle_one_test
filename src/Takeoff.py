# We're using ROS here
import rospy

# ROS messages
from ardrone_autonomy.msg import Navdata
from std_msgs.msg         import String, Empty
from geometry_msgs.msg    import Twist


class Takeoff(object):
    # m/s	mm		seconds
    def __init__(self, speed, max_altitudeGoal, timeout):
        # Initialize the node and rate
        self.node = rospy.init_node('takeoff_mode')

        # Subscribers
        self.sub_navdata = rospy.Subscriber('ardrone/navdata', \
                                             Navdata, self.navdata_cb)
        self.sub_state = rospy.Subscriber('smach/state', \
                                       String, self.state_cb)

        # Publishers
        self.pub_altitude = rospy.Publisher('/cmd_vel', \
                                             Twist, queue_size=1)
        #self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=100)
        self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', \
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

        self.timeout = timeout
        self.tag_acquired = False

        self.altitude_command = Twist()
        self.altitude_command.linear.z = speed

        # to disable hover mode
        self.altitude_command.angular.x = 0.5
        self.altitude_command.angular.y = 0.5

        self.max_altitudeGoal = max_altitudeGoal
        self.speed = speed

        self.timer_start = rospy.Time.now().to_sec()

    def navdata_cb(self, msg):
        self.altitude = msg.altd
        # Mode of the QC NOT the state of the state machine
        self.mode = msg.state
        if(msg.tags_count > 0):
            self.tag_acquired = True
            self.timer_start = 0
        else:
            self.tag_acquired = False
            self.timer_start = rospy.Time.now().to_sec()

    def state_cb(self, msg):
    	self.state = msg.data

    def launch(self):
        # Take off QC command
        self.pub_takeoff.publish(Empty())
        rospy.loginfo("Moving on up!")

    # If we're above the max altitude don't increase the altitude,
    # otherwise go up!
    def change_altitude(self, speed):
        self.altitude_command.linear.z = speed
        self.pub_altitude.publish(self.altitude_command)
        rospy.loginfo("Change altitude")

    def timer(self):
        return rospy.Time.now().to_sec() - self.timer_start

    def state_transition(self, transition):
        self.transition.data = transition
        self.pub_transition.publish(self.transition)
