/*
takeoff_forward_land

The QuadCopter (QC) will take off move forward and then land

Inputs
Outputs

Created by: Josh Saunders
Date Created: 1/29/2016

Modified by:
Date Modified:

*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
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
    ros::init(argc, argv, "takeoff_forward_land");
    // empty -> for takeoff, land
    // flight_command -> flight maneuvers
    std_msgs::Empty empty;
    geometry_msgs::Twist flight_command;
    int queue = 1000;

    // Nodehandler, timer, publishers, subscribers
    ros::NodeHandle node;
    double time_start=(double)ros::Time::now().toSec();
    double time_now=(double)ros::Time::now().toSec();
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
    ros::Rate rate(10); // Hz

    mode = TAKEOFF;

    flight_command.linear.x=0.0;
	flight_command.linear.y=0.0;
	flight_command.linear.z=0.0;
	flight_command.angular.x=0.0;
	flight_command.angular.y=0.0;
	flight_command.angular.z=0.0;

    while(ros::ok())
    {
        time_now=(double)ros::Time::now().toSec();
        switch(mode)
        {
            case TAKEOFF  :
            {
                ROS_INFO("MODE: TAKEOFF");
                pub_takeoff.publish(empty);
                if(time_now > time_start + 60.0)
                {
                    mode = FLYING;
                }
                break;
            }
            case FLYING   :
            {
                pub_flight.publish(flight_command);
                ROS_INFO("MODE: FLYING");
                if(time_now > time_start + 80.0)
                {
                    mode = LANDING;
                }
                break;
            }
            case LANDING  :
            {
                ROS_INFO("MODE: LANDING");
                pub_land.publish(empty);
                break;
            }
            default:
            {
                ROS_INFO("Quitting");
                exit(0);
            }
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
