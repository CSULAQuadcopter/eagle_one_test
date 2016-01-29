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

// Declare modes
#define SECURE  0
#define TAKEOFF 1
#define FLYING  2
#define LANDING 3

int main(int argc, char** argv)
{
    ros::init(argc, argv, "takeoff_forward_land");
    // empty -> for takeoff, land
    // flight_command -> flight maneuvers
    std_msgs::Empty empty;
    geometry_msgs::Twist flight_command;

    // We start off in secure mode
    int mode = SECURE;
    ros::Rate rate(200); // Hz

    mode = TAKEOFF;

    while(ros::ok())
    {
        switch(mode)
        {
            case SECURE   : break;
            case FLYING   : break;
            case LANDING  : break;
        }
    }

    return 0;
}
