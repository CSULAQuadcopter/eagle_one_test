#! /usr/bin/env python

# Josh Saunders
# Created 3/31/2016

# Let's use ROS
import rospy

# We're going to need to use String and Empty messages
from std_msgs.msg import String, Empty

class Emergency(object):
    def __init__(self):
        # Subscribe to the state machine transition topic
        self.sub_transition = rospy.Subscriber('qc_smach/transitions', String, self.transCallback)

        # Allow the mode to publish a land command
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=100)

        # Initialize the transition
        self.transition = ""

    def emergency_land(self):
        # Land!
        self.pub_land.publish(Empty())
        print("DANGER WILL ROBINSON! DANGER!")

    def transCallback(self, msg):
        self.transition = msg.data

def main():
    emergency = Emergency()

    # Initialize the node and rate
    rospy.init_node('emergency_mode')
    rate = rospy.Rate(100) # 100Hz
    while not rospy.is_shutdown():
        if(emergency.transition == "EMERGENCY_CONDITION"):
            emergency.emergency_land()
        rate.sleep()
        # rospy.spin()



if __name__=='__main__':
    main()
