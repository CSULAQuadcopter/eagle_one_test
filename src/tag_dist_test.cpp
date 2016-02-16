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
#include <eagle_one_test/ARNavdata.h>
#include <eagle_one_test/Drone.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_nav");
    ros::NodeHandle n;
    ros::Rate rate(50);

    Drone qc;

    //ros::Subscriber altd = n.subscribe("/ardrone/navdata", 1000, &ARNavdata::callback, &navdata);
    ros::Subscriber tag_info = n.subscribe("/ardrone/navdata", 1000, &Drone::set_navdata, &qc);

    qc.print_tag_distance();

    ros::spin();
    rate.sleep();
    return 0;
}
