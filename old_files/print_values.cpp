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
#include <eagle_one_test/Odometry.h>
#include <string>
#include <iostream>

class Printer
{
public:
    Printer(){time_ = 0;}
    void get_cmd_vel(const geometry_msgs::Twist::ConstPtr& cmd_vel);
    double time_;
};

void Printer::get_cmd_vel(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    std::cout << time_ << "\t" << cmd_vel->angular.z << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_nav");
    ros::NodeHandle n;
    ros::Rate rate(30);

    // previous velocity
    double prev_vx = 0;
    double prev_vy = 0;

    geometry_msgs::Twist twist_msg;

    Drone qc;
    Odometry od;
    Printer p;

    double start_time = (double) ros::Time::now().toSec();
    //double run_time;

    ros::Subscriber vel_info = n.subscribe("/ardrone/navdata", 1000, &Drone::set_navdata, &qc);
    ros::Subscriber cmd_vel = n.subscribe("/cmd_vel", 1000, &Printer::get_cmd_vel, &p);
    //ros::Subscriber odom = n.subscribe("/ardrone/odometry", 1000, &Odometry::getOdomData, &od);
    //ros::Publisher follow = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    while(ros::ok())
    {
        p.time_ = (double) ros::Time::now().toSec() - start_time;

        //std::cout << run_time << /*"\t" << qc.getYaw() << */"\n";
        //follow.publish(twist_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
