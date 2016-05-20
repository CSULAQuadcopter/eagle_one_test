# We're using ROS here
import rospy

# These are the messages that we're going to need for ROS
from std_msgs.msg           import String, Empty
from geometry_msgs.msg      import Twist
from ardrone_autonomy.msg   import Navdata

from Mode import Mode

class Landing(Mode):
    # m/s	mm		seconds
    def __init__(self, speed, min_altitude, max_altitudeGoal, height_diff, timeout):
        # Initialize the node and rate
        super(self.__class__, self).__init__('land_mode')

        # Subscribers
        # self.sub_navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.navdata_cb)
        ##self.sub_state = rospy.Subscriber('/smach/state', String, self.state_cb)
        self.sub_state = rospy.Subscriber('/smach/state',String, self.handle_timer_cb)

        # Publishers
        self.pub_altitude = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=100) #this turns off the motors

        # NOTE changed to passing transition and state info along
        # /smach/transition and /smach/state topics, respectively
        # self.pub_return_to_state = rospy.Publisher('qc_smach/transitions', String, queue_size=1)
        self.pub_transition = rospy.Publisher('smach/transition', String, queue_size=1)

        # Initialize member variables
        self.transition = String()
        self.state = 'nada'
        self.timer = rospy.Timer(rospy.Duration(timeout), self.goto_reacquisition)

        self.altitude_command = Twist()
        self.altitude_command.linear.z = speed

        self.min_altitude = min_altitude
        self.speed = speed
        self.max_altitudeGoal = max_altitudeGoal
        self.height_diff = height_diff

    ###
    def state_cb(self, msg):
        self.state = msg.data

    def height_diff(self, max_altitudeGoal, altitude):
        self.height_diff = self.max_altitudeGoal - self.altitude

#This is what makes the drone land
    def land(self):
        self.pub_land.publish(Empty())
        print("LAND!")

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
        # self.turn_off_timer(self.timer, 'Land')

    def handle_timer_cb(self, msg):
        print self.state
        if(self.state == 'land'):
            if (self.tag_acquired ==  True):
                print 'Tag acquired'
                self.turn_off_timer(self.timer, 'Land')
            elif (self.tag_acquired == False):
                print 'Tag not acquired'
                self.turn_on_timer(self.timer, 'Land')
            # rospy.loginfo("Land timers turned on.")
        else:
            self.turn_off_timer(self.timer, 'Land')
            # rospy.loginfo("Land timers turned off.")
