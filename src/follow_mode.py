#! /usr/bin/env python
import rospy

# We're using a thirdparty PID
from pid_controller import PID
import math

from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

class Controller(object):
    def __init__(self, node_name):
        self.pid_x     = PID(0.1,0.001,0,0,0,500,-500)
        self.pid_y     = PID(0,0,0,0,0,500,-500)
        self.pid_z     = PID(0,0,0,0,0,500,-500)
        self.pid_theta = PID(0,0,0,0,0,500,-500)

        self.tag_acquired
        self.tag_x = 0
        self.tag_y = 0
        self.altitude = 0.0
        self.theta = 0.0

        self.twist = Twist()
        # To disable hover
        self.twist.angular.x = 0.5
        self.twist.angular.y = 0.5

        # ROSify this
        self.node = rospy.init_node(node_name)
        self.sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, self.navdata_callback)
        self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=100)

    def navdata_callback(self, msg):
        self.navdata = msg
        if(msg.tags_count > 1):
        	self.tag_acquired = True
            self.tag_x = msg.tags_xc[0]
            self.tag_y = msg.tags_yc[0]
            self.theta = msg.tags_orientation[0]
        else:
        	self.tag_acquired = False
            self.tag_x = msg.tags_xc[0]
            self.tag_y = msg.tags_yc[0]
            self.theta = msg.tags_orientation[0]

    def publish_twist():
        pub_twist.publish(twist)

    def update(self, current_value_x, current_value_y, \
               current_value_z, current_value_theta):
        self.pid_x.update(current_value_x)
        self.pid_y.update(current_value_y)
        self.pid_z.update(current_value_z)
        self.pid_theta.update(current_value_theta)

    
