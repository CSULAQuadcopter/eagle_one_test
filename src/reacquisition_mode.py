#! /usr/bin/env python
"""
This attempts to reacquire the tag if it is lost. The QC will increase its
altitude to a predetermined height then remain there for a predetermined
time. If it reacquires the tag it goes back to the previous mode. Otherwise,
it lands where ever it is.

By: Amando A. Aranda and Josh Saunders
Date Created: March 31, 2016
Date Modified: 4/17/2016
"""

# We're using ROS here
import rospy

# These are the messages that we're going to need
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

# For integration with the qc_smach_server
import qc_smach_client as sc

class Reacquisition(object):
    # m/s	mm		seconds
    def __init__(self, speed, max_altitude, max_time):
        # Subscribers
        self.sub_transition = rospy.Subscriber('qc_smach/transitions', String, self.transCallback)
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
        self.speed = speed

        self.start_time = rospy.Time.now().to_sec()
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

    def land(self):
        # Land!
        self.pub_land.publish(Empty())
        print("LAND HO!")

    # If we're above the max altitude don't increase the altitude, other go up!
    def change_altitude(self):
    	if(self.altitude < self.max_altitude):
            self.altitude_command.linear.z = self.speed
            print "Increase altitude"
        else:
            self.altitude_command.linear.z = 0
            print "Stay put"
	    self.pub_altitude.publish(self.altitude_command)

    # Hey 'Ol Timer
    def timer(self):
        print rospy.Time.now().to_sec() - self.start_time
        return rospy.Time.now().to_sec() - self.start_time

def main():
    speed = 0.3 	 # m/s
    max_altitude = 3000  # mm
    max_time = 20 	 # seconds
    reacquisition = Reacquisition(speed, max_altitude, max_time)

    rate = rospy.Rate(100) # 100Hz
    # TODO add integration with the smach server
    # TODO add a way to go back to the previous state
    while not rospy.is_shutdown():
        if(reacquisition.transition == "TAG_LOST"):
            reacquisition.change_altitude()
            if(reacquisition.timer() > reacquisition.max_time):
                reacquisition.land()
        rate.sleep()


if __name__=='__main__':
    main()
