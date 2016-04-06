#! /usr/bin/env python
import rospy

from Controller import Controller
from pid_controller import PID

import math

from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

# def main():
#     rospy.init_node('follow_mode_test_4dof')
#     rate = rospy.Rate(100) # 100Hz
#     qc = Twist()
#     # Disable hover mode
#     qc.angular.x = 0.5
#     qc.angular.y = 0.5
#
#     controller = Controller()
#
#     controller.set_goal("x", 500)
#
#     while not rospy.is_shutdown():
#         print("Tag x: %d" % controller.tag_x)
#         x_update = controller.pid_x.update(controller.tag_x)
#         controller.publish_twist()
#         rate.sleep()

#! /usr/bin/env python
import rospy

# We're using a thirdparty PID
from pid_controller import PID
import math

from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

# Simple proportional controller for the orientation
class YawControl(object):
    def __init__(self, max_theta, min_theta, rotation_rate):
        self.max_theta = max_theta
        self.min_theta = min_theta

        #  0 < rotation_rate <= 1.0
        self.rotation_rate = rotation_rate
        self.tag_aqcuired = False
        self.yaw = 0.0

        # rosify
        self.sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, self.yaw_callback)

    def check_yaw(self):
        if ((self.yaw < self.max_theta) and (self.yaw > self.min_theta)):
            if ((self.yaw < self.max_theta) and (self.yaw > 180)):
                return self.rotation_rate
            else:
                return -self.rotation_rate
        else:
            self.yaw = 0
            return 0

    def yaw_callback(self, msg):
        if(msg.tags_count > 0):
            self.yaw = msg.tags_orientation[0]
        else:
            # to stop the rotation of the qc when the tag is not acquired
            self.yaw = 0

# Simple proportional controller for the orientation
class XControl(object):
    def __init__(self, max_x, min_x, translation_rate):
        self.max = max_x
        self.min = min_x

        #  0 < translation_rate <= 1.0
        self.translation_rate = translation_rate
        self.tag_acquired = False
        self.x = 0.0

        # rosify
        self.sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, self.x_callback)

    def check_x(self):
        if self.tag_acquired:
            if ((self.x < self.max) and (self.x > self.min)):
                return 0
            elif self.x < self.min:
                return self.translation_rate
            else:
                return -self.translation_rate
        else:
            return 0

    def x_callback(self, msg):
        if(msg.tags_count > 0):
            self.tag_acquired = True
            self.x = int(msg.tags_xc[0] * 360 / 1000)
        else:
            # to stop the translation of the qc when the tag is not acquired
            self.tag_acquired = False

def main():
    rospy.init_node('follow_mode_test_4dof')
    rate = rospy.Rate(100) # 100Hz

    qc = Twist()

    pub_qc = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    # sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, navdata_callback)

    follow_yaw = YawControl(350, 10, 0.5)
    follow_x = XControl(270, 90, 0.5)

    while not rospy.is_shutdown():
        # print("Yaw: %f" % follow_yaw.yaw)
        qc.angular.z = follow_yaw.check_yaw()
        qc.linear.x = follow_x.check_x()
        pub_qc.publish(qc)
        rate.sleep()

if __name__=='__main__':
    main()
