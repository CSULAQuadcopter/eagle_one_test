#! /usr/bin/env python
from Kalman import Kalman

import rospy
from geometry_msgs.msg import Pose2D

from Navdata import navdata_info

import numpy as np

def main():
    measurement = np.zeros((2,1)) # Measurement vector
    state = np.zeros((2,1))       # Initial state vector [x,y,vx,vy]
    pose  = Pose2D()
    kalman = Kalman()
    navdata = navdata_info()

    rospy.init_node('filtered_tag_data')
    pub_kalman = rospy.Publisher('/kalman', Pose2D, queue_size=100)

    rate = rospy.Rate(200) # 200Hz

    while not rospy.is_shutdown():
        # measurement[0, 0] = navdata.norm_tag_x
        # measurement[1, 0] = navdata.norm_tag_y
        # measurement[2, 0] = navdata.tag_vx
        # measurement[3, 0] = navdata.tag_vy

        measurement[0, 0] = navdata.tag_x
        measurement[1, 0] = navdata.tag_y

        kalman.state_callback()
        kalman.measurement_callback(measurement)

        state = kalman.x

        pose.x = state[0, 0]
        pose.y = state[1, 0]

        pub_kalman.publish(pose)

        rate.sleep()


if __name__ == '__main__':
    main()
