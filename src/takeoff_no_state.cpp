/*
takeoff_forward_land

The QuadCopter (QC) will take off move forward and then land

Inputs
Outputs

Created by: Josh Saunders (jay3ss)
Date Created: 1/29/2016

Modified by: Josh Saunders (jay3ss)
Date Modified: 1/30/2016

Problems: Program is now hanging during simulation. Look into timers in Gazebo
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <nav_msgs/Odometry.h>

// Declare modes
#define SECURE  0
#define TAKEOFF 1
#define FLYING  2
#define LANDING 3

void arTakeoffCallBack(const std_msgs::Empty::ConstPtr& msg);
int arAltdCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);

int main(int argc, char** argv)
{
    std::cout << "Initializing ROS";
    ros::init(argc, argv, "takeoff_forward_land");

    std::cout << "Initializing ROS Messages";
    // empty -> for takeoff, land
    // flight_command -> flight maneuvers
    std_msgs::Empty empty;
    geometry_msgs::Twist flight_command;
    int queue = 1000;

    // Nodehandler, timer, publishers, subscribers
    std::cout << "Initializing ROS Node Handlers";
    ros::NodeHandle node;
    std::cout << "Initializing ROS Time Objects";
    //double time_start=(double)ros::Time::now().toSec();
    //double time_now=(double)ros::Time::now().toSec();
    // for simulation
    // For some reason the program now freezes in simulation
    // perhaps due to the ros::WallTime::now()
    double time_start=(double)ros::WallTime::now().toSec();
    double time_now=(double)ros::WallTime::now().toSec();

    std::cout << "Initializing ROS Publishers/Subscribers";
    ros::Publisher pub_takeoff = node.advertise<std_msgs::Empty>(
        "/ardrone/takeoff", queue);
    ros::Publisher pub_land = node.advertise<std_msgs::Empty>(
        "/ardrone/land", queue);
    ros::Publisher pub_flight = node.advertise<geometry_msgs::Twist>(
        "/cmd_vel", queue);
    ros::Subscriber sub_atld = node.subscribe<ardrone_autonomy::Navdata>(
                "ardrone_autonomy/Navdata", queue, &arAltdCallback);

    // We start off in secure mode
    int mode = SECURE;
    //ros::Rate rate(10); // Hz
    // for simulation
    ros::WallRate rate(10); // Hz

    mode = TAKEOFF;

    flight_command.linear.x=0.0;
	flight_command.linear.y=0.0;
	flight_command.linear.z=0.0;
	flight_command.angular.x=0.0;
	flight_command.angular.y=0.0;
	flight_command.angular.z=0.0;

    while(ros::ok())
    {
        // time_now=(double)ros::Time::now().toSec();
        // for simulation
        time_now=(double)ros::WallTime::now().toSec();
        if(time_now > (time_start + 10.0))
        {
            ROS_INFO("MODE: TAKEOFF");
            /*
            std::cout << "Time start: " << time_start << std::endl;
            std::cout << "Time now: " << time_now << std::endl;
            */
            pub_takeoff.publish(empty);
        }

        else if(time_now > (time_start + 20.0))
        {
            pub_flight.publish(flight_command);
            ROS_INFO("MODE: FLYING");
        }

        else if(time_now > (time_start + 80.0))
        {
            ROS_INFO("MODE: LANDING");
            pub_land.publish(empty);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void arTakeoffCallBack(const std_msgs::Empty::ConstPtr& msg)
{

}

int arAltdCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{

}
