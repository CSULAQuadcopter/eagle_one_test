#! /usr/bin/env python
"""
Follow Mode Test
Written by: Josh Saunders
Date: 4/11/2016

Modified by: David Rojas
Date Modified: 5/16/16

This is to test the PID that controls the 4 Degrees Of Freedom (DOF) of the QC
"""

# We're using ROS here
import rospy

# ROS message
from std_msgs.msg import String

# The Takeoff class
from Follow import Follow

state = 'nada'

def state_cb(msg):
    global state
    state = msg.data

def main():
    ctrl.pid_x.setKp = 6
    ctrl.pid_x.setKi = .625
    ctrl.pid_x.setKd = 2.5
    ctrl.pid_y.setKp = 6
    ctrl.pid_y.setKi = .875
    ctrl.pid_y.setKd = 3.75

    follow = Follow(ctrl.pid_x.setkp, ctrl.pid_x.setKi, ctrl.pid_x.setKd, ctrl.pid_y.setkp, ctrl.pid_y.setKi, ctrl.pid_y.setKd)

    rate     = rospy.Rate(200) # 200Hz
    pub_ctrl = rospy.Publisher('cmd_vel', Twist, queue_size=100)

    qc      = Twist()
    navdata = navdata_info()
    ctrl    = Controller()

    while not rospy.is_shutdown():
        # always update the altitude
        z_update = ctrl.pid_z.update(z_update)
        # print("Theta %.2f"  % navdata.theta)
        # print("(%d, %d)"  % (navdata.tag_x, navdata.tag_y))

        if (navdata.tag_acquired):
            # print("Tag acquired %d" % i)
            # i += 1
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
