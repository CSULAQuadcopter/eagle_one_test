#! /usr/bin/env python
# This is the the code for the landing
#By: Amando A. Aranda
#Date: March 31, 2016

# We're using ROS here
import rospy
import sys
from eagle_one_test.srv import *
from StateMachine import Smach
from transitions import Machine
from eagle_one_test.srv import State

# These are the messages that we're going to need
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

class Landing(object):
    # m/s	mm		seconds
    def __init__(self, speed, min_altitude):
        # Subscribers
        self.sub_transition = rospy.Subscriber('qc_smach/transitions', String, self.transCallback)
        self.sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, self.navdataCallback)
        self.sub_previous_state = rospy.Subscriber('qc_smach/previous_state', String, self.previousStateCallback)

        # Publishers
        self.pub_altitude = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=100) #this turns off the motors
        # TODO need to set this up as a client to the smach server
        self.pub_return_to_state = rospy.Publisher('qc_smach/transitions', String, queue_size=100)

        # Initialize the node and rate
        self.node = rospy.init_node('land_mode_')

        # Initialize member variables
        self.transition = ""
        self.altitude = 0

        self.tag_acquired = False

        self.altitude_command = Twist()
        self.altitude_command.linear.z = speed


        #self.start_time = rospy.Time.now().to_sec()
        #self.max_time = max_time

        # to disable hover mode
        self.altitude_command.angular.x = 0.5
        self.altitude_command.angular.y = 0.5

        self.min_altitude = min_altitude
        self.speed = speed


    def transCallback(self, msg):
        self.transition = msg.data

    def navdataCallback(self, msg):
        self.altitude = msg.altd
        if((msg.tags_count >= 1) or (msg.tags_count > 0)):
        	self.tag_acquired = True
        else:
        	self.tag_acquired = False

    def previousStateCallback(self, msg):
    	self.previous_state = msg.data

    def returntoStateCallback(self, msg):
        self.pub_return_to_state = msg.data

#This is what makes the drone land
    def land(self):
        self.pub_land.publish(Empty())
        print("LAND HO!")

    # THis is the algo for landing
    def change_altitude(self, min_altitude):
        self.altitude_command.linear.z = self.speed
        self.pub_altitude.publish(self.altitude_command)


def main():
    speed = -1	 # m/s
    min_altitude = 500  # mm, this  is 8 inches
    #max_time = 5 	 # seconds
    landing = Landing(speed, min_altitude)
    rate = rospy.Rate(100) # 100Hz

    while not rospy.is_shutdown():
            print("%d" % landing.altitude)
#if(landing.tag_acquired == True):
            if(landing.altitude > landing.min_altitude):
                print("Go down, mofo!")
                landing.change_altitude(speed)
            elif(landing.altitude < landing.min_altitude):
                print("Eagle one has descended!")
                landing.land()
            rate.sleep()

if __name__=='__main__':
    main()
