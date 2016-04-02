#! /usr/bin/env python
import rospy

# We're using a thirdparty PID
from pid_controller import PID

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

class PositionControl(PID):
    def __init__(self, P=0.5, I=0.0, D=0.1, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min

        self.set_point=0.0
        self.error=0.0

    def avoid_drastic_corrections(self, controller_output):
        '''
            @input A float value
            @return If the value was bigger than 1.0 or -1.0, it will return 1.0 or -1.0,
            otherwise it returns the same value
        '''
        if math.fabs(controller_output) > 1.0:
            if controller_output > 1.0:
                controller_output = 1.0
            elif controller_output < -1.0:
                controller_output = -1.0
        return controller_output


tag_x = 0
tag_y = 0

def navdata_callback(msg):
    if(tags_count > 0):
        tag_x = msg.tags_xc[0]
        tag_y = msg.tags_yc[0]

def main():
    qc = Twist()
    # Disable hover mode
    qc.angular.x = 0.5
    qc.angular.y = 0.5

    follow_yaw = YawControl(350, 10, 0.5)

    pid_x = PositionControl()
    pid_y = PositionControl()

    # set the setpoints
    pid_x.setPoint(500)
    pid_y.setPoint(500)

    x_update = 0
    y_update = 0

    rospy.init_node('follow_mode')
    rate = rospy.Rate(100) # 100Hz
    pub_qc = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, navdata_callback)

    while not rospy.is_shutdown():
        x_update = pid_x.update(tag_x)
        y_update = pid_y.update(tag_y)
        qc.linear.x = pid_x.avoid_drastic_corrections(x_update)
        qc.linear.y = pid_x.avoid_drastic_corrections(x_update)
        qc.angular.z = follow_yaw.check_yaw()
        pub_qc.publish(qc)
        rate.sleep()


if __name__=='__main__':
    main()
