// This is to test what topics we're looking for in the /ardrone/navdata
// topic.

#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>

// This is the callback to display the information received from the
// /ardrone/navdata topic
void arNavDataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    ROS_INFO("\nAltd: [%i]", msg->altd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_nav");

    ros::NodeHandle n;

    ros::Subscriber altd = n.subscribe("/ardrone/navdata", 1000, arNavDataCallback);

    ros::spin();

    return 0;
}
