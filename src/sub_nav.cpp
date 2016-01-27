// This is to test what topics we're looking for in the /ardrone/navdata
// topic.

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ardrone_autonomy/Navdata.h>

// This is the callback to display the information received from the
// /ardrone/navdata topic
void imuCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    ROS_INFO("\nAltd: [%i]\nMotor1: [%i]\nMotor2: [%i]\nMotor3: [%i]\nMotor4: [%i]\nWind Speed: [%f]",
            msg->altd, msg->motor1, msg->motor2, msg->motor3, msg->motor4, msg->wind_speed);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_nav");

    ros::NodeHandle n;

    ros::Subscriber altd = n.subscribe("/ardrone/navdata", 1000, imuCallback);

    ros::spin();

    return 0;
}
