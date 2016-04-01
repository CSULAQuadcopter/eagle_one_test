#! /usr/bin/env python
# This is the the code for the reacqusition mode
#By: Amando A. Aranda and Josh Saunders
#Date: March 31, 2016

# We're using ROS here
import rospy

# These are the messages that we're going to need
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

class Reacquisition(object):
    # m/s	meters		seconds
    def __init__(self, speed, max_altitude, max_time):
        # Subscribers
        self.sub_transition = rospy.Subscriber('qc_smach/transition', String, self.transCallback)
        self.sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, self.navdataCallback)
        self.sub_previous_state = rospy.Subscriber('qc_smach/previous_state', String, self.previousStateCallback)

        # Publishers
        self.pub_altitude = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=100)
        # TODO need to set this up as a client to the smach server
        self.pub_return_to_state = rospy.Publisher('qc_smach/transitions', String, queue_size=100)

        # Initialize the node and rate
        self.node = rospy.init_node('reacquisition_mode')

        # Initialize member variables
        self.transition = ""
        self.altitude = 0

        self.tag_acquired = False

        self.altitude_command = Twist()
        self.altitude_command.linear.z = speed

        # to disable hover mode
        self.altitude_command.angular.x = 0.5
        self.altitude_command.angular.y = 0.5

        self.max_altitude = max_altitude

        self.start_time = rospy.Time.now()
        self.max_time = max_time
        self.previous_state = 0

    def transCallback(self, msg):
        self.transition = msg.data

    def navdataCallback(self, msg):
        self.altitude = msg.altd
        if(msg.tags_count > 1):
        	self.tag_acquired = True
        else:
        	self.tag_acquired = False

    def previousStateCallback(self, msg):
    	self.previous_state = msg.data

    def change_altitude(self):
      	self.set_altitude_speed()
    	self.pub_altitude(altitude_command)

    def land(self):
        # Land!
        self.pub_land.publish(Empty())
        print("LAND HO!")

    # If we're above the max altitude don't increase the altitude, other go up!
    def set_altitude_speed(self):
    	if(self.atlitude < self.max_altitude):
        	self.altitude_command.linear.z = speed
        else:
        	self.altitude_command.linear.z = 0

    # Hey 'Ol Timer
    def timer(self):
        return self.start_time - rospy.Time.now()

def main():
    speed = 0.1 	 # m/s
    max_altitude = 3 # meters
    max_time = 10 	 # seconds
    reacquisition = Reacquisition(speed, max_altitude, max_time)

    rate = rospy.Rate(100) # 100Hz
    while not rospy.is_shutdown():
        if(reacquisition.transition == "TAG_LOST"):
            reacquisition.change_altitude()
            if(reacquisition.timer() > reacquisition.max_time):
                reacquisition.land()
        rate.sleep()


if __name__=='__main__':
    main()
