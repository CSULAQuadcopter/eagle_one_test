#! /usr/bin/env python
import rospy

# We're using a thirdparty PID
from pid_controller import PID

# ROS messages
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

class Controller(object):
    def __init__(self):
        self.pid_x     = PID(0.1,0.001,0,0,0,500,-500)
        self.pid_y     = PID(0,0,0,0,0,500,-500)
        self.pid_z     = PID(0,0,0,0,0,500,-500)
        self.pid_theta = PID(0,0,0,0,0,500,-500)

        self.twist = Twist()
        # To disable hover
        self.twist.angular.x = 0.5
        self.twist.angular.y = 0.5

        # Tag information
        self.tag_acquired = False
        self.tag_x = 0
        self.tag_y = 0
        self.tag_length = 0
        self.tag_width = 0
        self.tag_theta = 0

        # ROSify this
        self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        self.sub_navdata = self.navdata_sub = rospy.Subscriber("ardrone/navdata",Navdata,self.navdata_callback)

    def navdata_callback(self, msg):
        # self.navdata = msg
        if(msg.tags_count > 1):
            self.tag_acquired = True
            self.tag_x = msg.tags_xc[0]
            self.tag_y = msg.tags_yc[0]
            self.theta = msg.tags_orientation[0]
        else:
            self.tag_acquired = False

    def publish_twist(self):
        self.pub_twist.publish(self.twist)

    def update(self, current_value_x=0, current_value_y=0, \
               current_value_z=0, current_value_theta=0):
        self.pid_x.update(current_value_x)
        self.pid_y.update(current_value_y)
        self.pid_z.update(current_value_z)
        self.pid_theta.update(current_value_theta)

    def set_goal(self, dof, new_goal):
        if dof == "linear.x":
            self.pid_x.setPoint(new_goal)
        elif dof == "linear.y":
            self.pid_y.setPoint(new_goal)
        elif dof == "linear.z":
            self.pid_z.setPoint(new_goal)
        elif dof == "angluar.z":
            self.pid_theta.setPoint(new_goal)
