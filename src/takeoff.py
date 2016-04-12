#! /usr/bin/env python
# This is the the code for the takeoff mode
#Date: April 11, 2016

# We're using ROS here

import rospy

from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

class Takeoff(object):
    # m/s	mm		seconds
    def __init__(self, speed, max_altitudeGoal):
        # Subscribers
        self.sub_transition = rospy.Subscriber('qc_smach/transitions', String, self.transCallback)
        self.sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, self.navdataCallback)
        self.sub_previous_state = rospy.Subscriber('qc_smach/previous_state', String, self.previousStateCallback)

        # Publishers
        self.pub_altitude = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        #self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=100)
        self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=100)
        # TODO need to set this up as a client to the smach server
        self.pub_return_to_state = rospy.Publisher('qc_smach/transitions', String, queue_size=100)

        # Initialize the node and rate
        self.node = rospy.init_node('takeoff_mode')

        # Initialize member variables
        self.transition = ""
        self.altitude = 0

        self.tag_acquired = True

        self.altitude_command = Twist()
        self.altitude_command.linear.z = speed

        # to disable hover mode
        self.altitude_command.angular.x = 0.5
        self.altitude_command.angular.y = 0.5

        self.max_altitudeGoal = max_altitudeGoal
        self.speed = speed

        self.start_time = rospy.Time.now().to_sec()
        #self.max_time = max_time
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
    	
    def launch(self):
        # Take off QC command
        self.pub_takeoff.publish(Empty())
        print("Moving on up!")


    # If we're above the max altitude don't increase the altitude, other go up!
    def change_altitude(self):
    	if(self.altitude < self.max_altitudeGoal):
            self.altitude_command.linear.z = self.speed
            print "Increase altitude"
        else:
            self.altitude_command.linear.z = 0
            print "Stay put"
	    self.pub_altitude.publish(self.altitude_command)

def main():
    speed = 1 	 # m/s
    max_altitudeGoal = 1000  # mm
    #max_time = 20 	 # seconds
    takeoff = Takeoff(speed, max_altitudeGoal)

    rate = rospy.Rate(100) # 100Hz
    while not rospy.is_shutdown():
        takeoff.launch()
        takeoff.change_altitude()
        rate.sleep()


if __name__=='__main__':
    main()
