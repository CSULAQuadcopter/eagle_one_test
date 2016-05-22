#! /usr/bin/env python
'''
Follow Mode Test
Written by: Josh Saunders
Date: 4/11/2016

Modified by: David Rojas, Josh Saunders
Date Modified: 5/16/16, 5/21/2016

This is to test the PID that controls the 4 Degrees Of Freedom (DOF) of the QC
'''

# We're using ROS here
import rospy

# ROS message
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Custom classes
from Follow import Follow
from Navdata import navdata_info as nd

state = 'nada'

def state_cb(msg):
    global state
    state = msg.data

sub_state = rospy.Subscriber('/smach/state', String, state_cb, queue_size=1000)

def main():
    # pid_i = (kp, ki, kd, integrator, derivator, set_point)
    pid_x = (6, 0.625, 2.5, 0, 0, 500)
    pid_y = (6, 0.875, 3.75, 0, 0, 500)
    pid_z = (1,0,0,0,0,2.5)
    pid_theta = (1/260,0,0,0,0,0)

    # set the bounding box
    bbx = (375, 625)
    bby = (375, 625)
    bounding_box = True

    bbx_min, bbx_max = bbx
    bby_min, bby_max = bby

    yaw_min = 10
    yaw_max = 350

    ctrl = Follow(bbx, bby, pid_x, pid_y, pid_z, pid_theta)

    # controller update values
    yaw_update = 0
    x_update   = 0
    y_update   = 0
    z_update   = 0

    navdata = nd()

    # Twist commands
    qc = Twist()

    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        # always update the altitude
        z_update = ctrl.pid_z.update(z_update)
        # print("Theta %.2f"  % navdata.theta)
        # print("(%d, %d)"  % (navdata.tag_x, navdata.tag_y))

        if (navdata.tag_acquired):
            # If 10 < theta < 350 then let's rotate
            # if ((yaw_min < navdata.theta) and (navdata.theta < yaw_max)):
            if ((yaw_min < navdata.theta < yaw_max)):
                yaw_update  = ctrl.pid_theta.update(navdata.theta)
            else:
                yaw_update = 0


            # is_in_box(minimum, maximum, position)
            if (ctrl.is_in_box(bbx_min, bbx_max, navdata.tag_y) and ctrl.is_in_box(bby_min, bby_max, navdata.tag_x)):
                # If the QC is in the bounding box then we should enter 'Hover'
                # mode and just hang there
                x_update = 0
                y_update = 0
                # # qc.angular.x = 0.0
                # qc.angular.y = 0.0
                # print("In the Box")

            else:
                # It's not in the bounding box therefore we should update the PIDs
                x_update  = ctrl.pid_x.update(navdata.tag_x)
                y_update  = ctrl.pid_y.update(navdata.tag_y)
                # print("%.3f" % ctrl.pid_x.getError())

        # Make sure that we're not making any drastic updates
        # qc.angular.z = ctrl.pid_theta.avoid_drastic_corrections(yaw_update)
        # qc.linear.x  = ctrl.pid_x.avoid_drastic_corrections(x_update)
        # qc.linear.y  = ctrl.pid_y.avoid_drastic_corrections(y_update)
        # qc.linear.z  = ctrl.pid_z.avoid_drastic_corrections(z_update)

        qc.angular.z = yaw_update
        qc.linear.x  = x_update
        qc.linear.y  = y_update
        qc.linear.z  = z_update

        ctrl.pub_ctrl.publish(qc)
        rate.sleep()


if __name__=='__main__':
    main()
