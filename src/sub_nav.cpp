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
#include <nav_msgs/Odometry.h>

// This is the callback to display the information received from the
// /ardrone/navdata topic
// The standard template for the callback is:
//   callBackName     (const package_name::Message::ConstPtr& msg)
void arNavDataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    std::cout << "\nAltd: " << msg->altd << "mm" << std::endl;
    std::cout << "\n";
    std::cout << "Accelerations\n";
    std::cout << "ax: " << msg->ax << "g" << std::endl;
    std::cout << "ay: " << msg->ay << "g" << std::endl;
    std::cout << "az: " << msg->az << "g" << std::endl;

    std::cout << "\n";
    std::cout << "Velocities\n";
    std::cout << "vx: " << msg->vx << "mm/s" << std::endl;
    std::cout << "vy: " << msg->vy << "mm/s" << std::endl;
    std::cout << "vz: " << msg->vz << "mm/s" << std::endl;
}

void arOdometyCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::cout << "\nOdometry - Quaternion:\n";
    std::cout << "x: " << msg->pose.pose.orientation.w << "mm/s" << std::endl;
    std::cout << "y: " << msg->pose.pose.orientation.x << "mm/s" << std::endl;
    std::cout << "z: " << msg->pose.pose.orientation.y << "mm/s" << std::endl;
    std::cout << "z: " << msg->pose.pose.orientation.z << "mm/s" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_nav");
    ros::NodeHandle n;
    ros::Rate rate(50);

    ros::Subscriber altd = n.subscribe("/ardrone/navdata", 1000, &arNavDataCallback);
    //ros::Subscriber odom = n.subscribe("/ardrone/odometry", 1000, &arOdometyCallback);

    ros::spin();
    rate.sleep();
    return 0;
}
