/*
Name: sub_nav

Testing how to subscribe to different topics that the QuadCopter publishes

Inputs: None
Outputs: altitude published on the /ardrone/navdata topic

Written by: Josh Saunders (jay3ss)
Date created: 1/26/2016
Modified by: Josh Saunders (jay3ss)
Date modified: 1/27/2016
*/


#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>

// This is the callback to display the information received from the
// /ardrone/navdata topic
// The standard template for the callback is:
//   callBackName     (const package_name::Message::ConstPtr& msg)
void arNavDataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    ROS_INFO("\nAltd: [%i]", msg->altd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_nav");
    ros::NodeHandle n;

    ros::Subscriber altd = n.subscribe("/ardrone/navdata", 1000, &arNavDataCallback);

    ros::spin();

    return 0;
}
