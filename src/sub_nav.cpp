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
    ROS_INFO("\nAltd: %i\n", msg->altd);

    ROS_INFO("\n\nAccelerations\n\nax: %f\nay: %f\naz: %f\n\n",
             msg->ax, msg->ay, msg->az);

    ROS_INFO("\n\nVelocities\n\nvx: %f\nvy: %f\nvz: %f\n\n",msg->vx, msg->vy, msg->vz);

}

void arOdometyCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_INFO("\nOdometry - Quaternion:\n");
    ROS_INFO("\nw: %f\nx: %f\ny: %f\nz: %f",
             msg->pose.pose.orientation.w,
             msg->pose.pose.orientation.x,
             msg->pose.pose.orientation.y,
             msg->pose.pose.orientation.z);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_nav");
    ros::NodeHandle n;

    ros::Subscriber altd = n.subscribe("/ardrone/navdata", 1000, &arNavDataCallback);
    ros::Subscriber odom = n.subscribe("/ardrone/odometry", 1000, &arOdometyCallback);

    ros::spin();

    return 0;
}
