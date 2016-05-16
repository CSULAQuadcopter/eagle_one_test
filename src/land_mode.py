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


def is_in_box(minimum, maximum, position):
    """
    Checks if the position is within the given bounds
    """
    if ((minimum < position) and (position < maximum)):
        # print("In box")
        return True
    else:
        # print("Out box")
        return False


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

    ########################
    # Set the bounding box #
    ########################
    # X is in front and behind QC [0, 360] pixels
    # Y is left and right of QC   [0, 640] pixels
    bbx_max = 563
    bbx_min = 438
    bby_max = 563
    bby_min = 438
    yaw_max = 350
    yaw_min = 10

    ####################################
    # Setup the individual controllers #
    ####################################
    # Note: in order to change the values for integrator max and min, you need
    # to go into Controller.py and adjust the values there
    # Set yaw controller
    # NOTE: this doesn't seem to be working correctly...
    ctrl.pid_theta.setKp(1/260.0)
    # ctrl.pid_theta.setKp(0.0)
    ctrl.pid_theta.setKi(0.0)
    ctrl.pid_theta.setKd(0.0)
    ctrl.pid_theta.setPoint(180.0)
    default = 10
    # ctrl.pid_theta.setIntegrator(100)
    # ctrl.pid_theta.setDerivator(100)

    # Set the x (forward/backward) controller
    ctrl.pid_x.setKp(10.0)
    ctrl.pid_x.setKi(5000)
    ctrl.pid_x.setKd(0)
    # ctrl.pid_x.setKd(0.0)
    ctrl.pid_x.setPoint(500.0)
    #ctrl.pid_x.setIntegrator(5000.0)
    #ctrl.pid_x.setDerivator(5000.0)

    # Set the y (left/right) controller
    ctrl.pid_y.setKp(10.0)
    # ctrl.pid_y.setKp(0.0)
    ctrl.pid_y.setKi(5000)
    ctrl.pid_y.setKd(0)
    ctrl.pid_y.setPoint(500.0)
    # ctrl.pid_y.setIntegrator(5000)
    # ctrl.pid_y.setDerivator(5000)

    # Set the z (altitude) controller
    ctrl.pid_z.setKp(0.0)
    ctrl.pid_z.setKi(0.0)
    ctrl.pid_z.setKd(0.0)
    ctrl.pid_z.setPoint(0.0)
    # ctrl.pid_z.setIntegrator(500)
    # ctrl.pid_z.setDerivator(500)

    # Disable hover mode
    qc.angular.x = 0.5
    qc.angular.y = 0.5

    # controller update values
    yaw_update = 0
    x_update   = 0
    y_update   = 0
    z_update   = 0

    # i = 0

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
                if (is_in_box(bbx_min, bbx_max, navdata.tag_y) and is_in_box(bby_min, bby_max, navdata.tag_x)):
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
                    # reached our landing altitude
                    rospy.loginfo("Going to secure mode")
                    land.goto_secure()

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
