#! /usr/bin/env python
"""
Makes the QC takeoff and increase altitude until it reaches a specified
altitude.
If the tag is lost, a timer is started and if the tag cannot be recovered
within a specified amount of time, it will enter the reacquisition mode. If the
tag is recovered within the specified amount of time, it will continue on with
its function (getting the QC to a specified height).

Created by: David Rojas
Date Created: April 11, 2016

Modified by: Josh Saunders
Date Modified: 4/21/2016
"""
# We're using ROS here
import rospy

# Python libraries
import math

# ROS message
from std_msgs.msg import String, Empty

# The classes that we're using
from Controller import Controller
from Navdata import navdata_info
from Takeoff import Takeoff

# The messages that we need
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata


state = 'nada'

def state_cb(msg):
    global state
    state = msg.data

# pub_transition = rospy.Publisher('/smach/transition', String, queue_size=1)
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



# if landed, takeoff
# TODO only works when max_altitudeGoal is 3000 mm or lower, need to fix
def main():
    speed = 1	 # m/s
    max_altitudeGoal = 2500  # mm
    timeout  = 10 # seconds
    takeoff  = Takeoff(speed, max_altitudeGoal, timeout)
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

    # To let us know that the mode is working
    rospy.loginfo("Started Takeoff Mode")

    while((state != 'takeoff')):
        # print takeoff.state
        rate.sleep()


    # To get this guy to take off! For some reason just calling this function
    # once does not work. This value (50) was determined experimentally
    i = 0
    while i < 50:
        takeoff.launch()
        i += 1
        rate.sleep()

    while not rospy.is_shutdown():
        # always update the altitude
        z_update = ctrl.pid_z.update(z_update)
        # We only want to execute these manuevers if we're in takeoff mode
        if takeoff.state == 'takeoff':
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

            if(takeoff.altitude < takeoff.max_altitudeGoal):
                rospy.loginfo("Go up!")
                takeoff.change_altitude(speed)
            elif(takeoff.altitude >= takeoff.max_altitudeGoal):
                speed = 0
                rospy.loginfo("Stop!")
                # takeoff.change_altitude(speed)
                # To change states, we publish the fact that we've
                # reached our takeoff altitude
                rospy.loginfo("Going to follow mode")
                takeoff.goto_follow()

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
