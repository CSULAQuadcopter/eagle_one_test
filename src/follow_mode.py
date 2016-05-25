#! /usr/bin/env python
"""
Makes the QC follow and decrease altitude until it reaches a specified
altitude to follow.
If the tag is lost, a timer is started and if the tag cannot be recovered
within a specified amount of time, it will enter the reacquisition mode. If the
tag is recovered within the specified amount of time, it will continue on with
its function (getting the QC to decrease to a specified height).

Created by: Josh Saunders
Date Created: 5/24/2016

Modified by:
Date Modified:
"""
# We're using ROS here
import rospy

# Python libraries

# ROS message
from std_msgs.msg import String, Empty

# Classes we need
from Follow import Follow
from Navdata import navdata_info as nd

# The messages that we need
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

#
# state = 'nada'
#
# def state_cb(msg):
#     global state
#     state = msg.data
# ##
# #pub_transition = rospy.Publisher('/smach/transition', String, queue_size=1)
# sub_state = rospy.Subscriber('/smach/state', String, state_cb, queue_size=1000)

def main():
    speed = -.05	 # m/s
    min_altitude = 0.8  # mm
    altitude_goal = 2.5# mm
    height_diff = 0 #mm
    timeout  = 300# seconds
    # pid_i = (kp, ki, kd, integrator, derivator, set_point)
    pid_x = (6, 0.625, 2.5, 0, 0, 500)
    pid_y = (6, 0.875, 3.75, 0, 0, 500)
    pid_z = (0.25,0,0,0,0,2.5)
    pid_theta = (1/260,0,0,0,0,0)

    # set the bounding box
    bbx = (375, 625)
    bby = (375, 625)
    bounding_box = True

    bbx_min, bbx_max = bbx
    bby_min, bby_max = bby

    yaw_min = 10
    yaw_max = 350

    # controller update values
    yaw_update = 0
    x_update   = 0
    y_update   = 0
    z_update   = 0

    navdata = nd()

    # Twist commands
    qc = Twist()

    follow  = Follow(bbx, bby, pid_x, pid_y, pid_z, pid_theta, bounding_box)
    rate = rospy.Rate(200)

    while((follow.state != 'follow')):
        # print takeoff.state
        rate.sleep()

    while not rospy.is_shutdown():
        # always update the altitude
        z_update = follow.pid_z.update(z_update)
        # print("Theta %.2f"  % navdata.theta)
        # print("(%d, %d)"  % (navdata.tag_x, navdata.tag_y))

        if (navdata.tag_acquired):
            # If 10 < theta < 350 then let's rotate
            # if ((yaw_min < navdata.theta) and (navdata.theta < yaw_max)):
            if ((yaw_min < navdata.theta < yaw_max)):
                print "Yaw!"
                yaw_update  = follow.pid_theta.update(navdata.theta)
            else:
                print "No yaw!"
                yaw_update = 0

            # is_in_box(minimum, maximum, position)
            if (follow.is_in_box(bbx_min, bbx_max, navdata.tag_y) and follow.is_in_box(bby_min, bby_max, navdata.tag_x)):
                # If the QC is in the bounding box then we should enter 'Hover'
                # mode and just hang there
                x_update = 0
                y_update = 0
                # # qc.angular.x = 0.0
                # qc.angular.y = 0.0
                # print("In the Box")

            else:
                # It's not in the bounding box therefore we should update the PIDs
                x_update  = follow.pid_x.update(navdata.tag_x)
                y_update  = follow.pid_y.update(navdata.tag_y)
                # print("%.3f" % follow.pid_x.getError())

        qc.angular.z = yaw_update
        qc.linear.x  = x_update
        qc.linear.y  = y_update
        # qc.linear.z  = z_update

        follow.pub_ctrl.publish(qc)
        rate.sleep()

if __name__=='__main__':
    main()
