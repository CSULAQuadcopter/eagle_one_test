#! /usr/bin/env python
"""
This mode will command the QC to automatically land.
Created by: Josh Saunders
Date Created: 3/31/2016
Modified by: David Rojas
Date Modified: 4/27/2016
"""
# Let's use ROS
import rospy

# We're going to need to use String and Empty messages
from std_msgs.msg import String, Empty

# For integration with the qc_smach_server
import qc_smach_client as sc

# The emergency class
from Emergency import Emergency

def main():
    emergency = Emergency()

    # Initialize the node and rate
    rospy.init_node('emergency_mode')
    rate = rospy.Rate(100) # 100Hz
    while not rospy.is_shutdown():
        # TODO add integration with the smach server
        if(emergency.transition == "EMERGENCY_CONDITION"):
            emergency.emergency_land()
        rate.sleep()


if __name__=='__main__':
    main()
