#! /usr/bin/env python
"""
Makes the QC land and decrease altitude until it reaches a specified
altitude to land.
If the tag is lost, a timer is started and if the tag cannot be recovered
within a specified amount of time, it will enter the reacquisition mode. If the
tag is recovered within the specified amount of time, it will continue on with
its function (getting the QC to decrease to a specified height).

Created by: Amando Aranda
Date Created: March 31, 2016

Modified by: David Rojas
Date Modified: 5/16/2016
"""
# We're using ROS here
import rospy

# Python libraries
import math

# ROS message
from std_msgs.msg import String, Empty

# Classes we need
from Land import Landing
from Follow import Follow
from Controller import Controller
from Navdata import navdata_info

# The messages that we need
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata


state = 'nada'

def state_cb(msg):
    global state
    state = msg.data
##
#pub_transition = rospy.Publisher('/smach/transition', String, queue_size=1)
sub_state = rospy.Subscriber('/smach/state', String, state_cb, queue_size=1000)


def main():
    speed = -.5	 # m/s
    min_altitude = 700  # mm
    max_altitudeGoal = 2500 # mm
    height_diff = 0 #mm
    timeout  = 3 # seconds
    landing  = Landing(speed, min_altitude, height_diff, max_altitudeGoal, timeout)
    rate     = rospy.Rate(200) # 200Hz
    pub_ctrl = rospy.Publisher('cmd_vel', Twist, queue_size=100)

    qc       = Twist()
    navdata  = navdata_info()
    ctrl     = Controller()

    while((state != 'land')):
        # print takeoff.state
        rate.sleep()

    while not rospy.is_shutdown():
        # always update the altitude
        z_update = ctrl.pid_z.update(z_update)
        height_diff = landing.max_altitudeGoal - landing.altitude
        # We only want to execute these manuevers if we're in land mode
        if landing.state == 'land':
            print("%d" % landing.altitude)
            if(landing.tag_acquired):
                # If 10 < theta < 350 then let's rotate
                if ((yaw_min < navdata.theta) and (navdata.theta < yaw_max)):
                    # We need to make sure that we have an offset so that the QC
                    # rotates correctly
                    # if((180 < navdata.theta) and (navdata.theta < yaw_max)):
                    #     navdata.theta -= 360
                    yaw_update  = ctrl.pid_theta.update(navdata.theta)
                else:
                    yaw_update = 0

                # If the QC is in the bounding box then we should enter 'Hover'
                # mode and just hang there
                # is_in_box(minimum, maximum, position)
                if (follow.is_in_box(bbx_min, bbx_max, navdata.tag_y) and follow.is_in_box(bby_min, bby_max, navdata.tag_x)):
                    x_update = 0
                    y_update = 0
                    qc.angular.x = 0.0
                    qc.angular.y = 0.0
                    # print("In the Box")
                # It's not in the bounding box therefore we should update the PIDs
                # and disable Hover mode
                else:
                    x_update  = ctrl.pid_x.update(navdata.tag_x)
                    y_update  = ctrl.pid_y.update(navdata.tag_y)
                    qc.angular.x = 0.5
                    qc.angular.y = 0.5
                    # print("%.3f" % ctrl.pid_x.getError())

                if(landing.altitude > landing.min_altitude):
                    print("Go down")
                    if(landing.height_diff > landing.min_altitude):
                        landing.change_altitude(speed)
                        print("Descending")
                    elif(landing.height_diff <= landing.min_altitude):
                        pass
                elif(landing.altitude < landing.min_altitude):
                    print("Eagle one has descended!")
                    landing.land()
                    # To change states, we publish the fact that we've
                    # reached our takeoff altitude
                    rospy.loginfo("Going to secure mode")
                    land.goto_f()

        # Make sure that we're not making any drastic updates
        # qc.angular.z = ctrl.pid_theta.avoid_drastic_corrections(yaw_update)
        # qc.linear.x  = ctrl.pid_x.avoid_drastic_corrections(x_update)
        # qc.linear.y  = ctrl.pid_y.avoid_drastic_corrections(y_update)
        # qc.linear.z  = ctrl.pid_z.avoid_drastic_corrections(z_update)

        qc.angular.z = yaw_update
        qc.linear.x  = x_update
        qc.linear.y  = y_update
        # qc.linear.z  = z_update

        pub_ctrl.publish(qc)
        rate.sleep()

if __name__=='__main__':
    main()
