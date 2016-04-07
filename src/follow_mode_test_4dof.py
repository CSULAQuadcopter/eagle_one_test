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
        if ((self.yaw > self.max_theta) and (self.yaw < self.min_theta)):
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

# Simple proportional controller for the x translation
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
            if(self.x < self.min):
                # print("Below")
                return self.translation_rate
            elif(self.x > self.max):
                # print("Above")
                return -self.translation_rate
            else:
                # print("Within")
                return 0
        else:
            # print("No tag")
            return 0

    def x_callback(self, msg):
        if(msg.tags_count > 0):
            self.tag_acquired = True
            self.x = int(msg.tags_yc[0] * 360 / 1000)
        else:
            # to stop the translation of the qc when the tag is not acquired
            self.tag_acquired = False

# Simple proportional controller for the y translation
class YControl(object):
    def __init__(self, max_y, min_y, translation_rate):
        self.max = max_y
        self.min = min_y

        #  0 < translation_rate <= 1.0
        self.translation_rate = translation_rate
        self.tag_acquired = False
        self.y = 0.0

        # rosify
        self.sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, self.y_callback)

    def check_y(self):
        if self.tag_acquired:
            if(self.y < self.min):
                # print("Below")
                return self.translation_rate
            elif(self.y > self.max):
                # print("Above")
                return -self.translation_rate
            else:
                # print("Within")
                return 0
        else:
            # print("No tag")
            return 0

    def y_callback(self, msg):
        if(msg.tags_count > 0):
            self.tag_acquired = True
            self.y = int(msg.tags_xc[0] * 640 / 1000)
            if ((self.y < self.max) and (self.y > self.min)):
                return 0
            elif self.y < self.min:
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

# Simple proportional controller for the orientation
class ZControl(object):
    def __init__(self, max_z, min_z, translation_rate):
        self.max = max_z
        self.min = min_z

        #  0 < translation_rate <= 1.0
        self.translation_rate = translation_rate
        self.tag_acquired = False
        self.z = 0.0

        # rosify
        self.sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, self.z_callback)

    def check_z(self):
        if(self.z > self.max):
            return -self.translation_rate
        elif(self.z < self.min):
            return self.translation_rate
        else:
            # to stop the translation of the qc when the tag is not acquired
            return 0

    def z_callback(self, msg):
        self.z = msg.altd

class Window(object):
    def __init__(self):
        self.state = 'stop'
        self.sub_navdata = rospy.Subscriber('eagle_one/start', String, self.state_cb)

    def state_cb(self, msg):
        self.state = msg.data

def main():
    rospy.init_node('follow_mode_test_4dof')
    rate = rospy.Rate(100) # 100Hz

    w = Window()
    qc = Twist()

    # To disable hover mode
    qc.angular.x = 0.5
    qc.angular.y = 0.5

    pub_qc = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    # sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, navdata_callback)

    follow_yaw = YawControl(350, 10, 0.5)

    follow_x = XControl(205, 155, 0.1)
    follow_y = YControl(340, 300, 0.1)
    follow_z = ZControl(1250, 1150, 0.15)


    while (not (w.state == 'go') or (w.state == '')):
        print("Waiting...")

    # Hopefully this continues until we turn off the program in the control window
    while (not rospy.is_shutdown() and (w.state == 'go')):
        # print("Yaw: %f" % follow_yaw.yaw)
        print("Running...")
        qc.angular.z = follow_yaw.check_yaw()
        qc.linear.x = follow_x.check_x()

        qc.linear.y = follow_y.check_y()
        qc.linear.z = follow_z.check_z()

        pub_qc.publish(qc)
        rate.sleep()

if __name__=='__main__':
    main()
