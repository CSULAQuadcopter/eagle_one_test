#! /usr/bin/env python
import rospy

from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

class YawControl(object):
    def __init__(self, max_theta, min_theta, rotation_rate):
        self.max_theta = max_theta
        self.min_theta = min_theta

        #  0 < rotation_rate <= 1.0
        self.rotation_rate = rotation_rate

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
            return 0

    def yaw_callback(self, msg):
        if(msg.tags_count > 0):
            self.yaw = msg.tags_orientation[0]


def main():
    yaw = Twist()
    yaw.angular.x = 0.5
    yaw.angular.y = 0.5

    follow_yaw = YawControl(350, 10, 0.5)

    rospy.init_node('follow_mode')
    rate = rospy.Rate(100) # 100Hz
    pub_yaw = rospy.Publisher('cmd_vel', Twist, queue_size=100)

    while not rospy.is_shutdown():
        yaw.angular.z = follow_yaw.check_yaw()
        pub_yaw.publish(yaw)
        rate.sleep()


if __name__=='__main__':
    main()
