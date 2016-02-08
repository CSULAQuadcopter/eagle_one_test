/*
Name: sub_nav

Testing how to subscribe to different topics that the QuadCopter publishes

Written by: Josh Saunders (jay3ss)
Date created: 1/26/2016
Modified by: Josh Saunders (jay3ss)
Date modified: 2/8/2016
*/


#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <nav_msgs/Odometry.h>



void arOdometyCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::cout << "\nOdometry - Quaternion:\n";
    std::cout << "x: " << msg->pose.pose.orientation.w << "mm/s" << std::endl;
    std::cout << "y: " << msg->pose.pose.orientation.x << "mm/s" << std::endl;
    std::cout << "z: " << msg->pose.pose.orientation.y << "mm/s" << std::endl;
    std::cout << "z: " << msg->pose.pose.orientation.z << "mm/s" << std::endl;
}


//template<int>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_nav");
    ros::NodeHandle n;
    ros::Rate rate(50);

    ARNavData navdata;

    ros::Subscriber altd = n.subscribe("/ardrone/navdata", 1000, &ARNavData::callback, &navdata);
    //ros::Subscriber odom = n.subscribe("/ardrone/odometry", 1000, &arOdometyCallback);

    ros::spin();
    rate.sleep();
    return 0;
}
