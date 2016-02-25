#include <ros/ros.h>
#include <eagle_one_test/Pid.h>
#include <iostream>
#include <cstdlib>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pid_client");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<eagle_one_test::Pid>("pid", true);
    eagle_one_test::Pid srv;

    std::cout << "\nEnter a value for the error: ";
    std::cin >> srv.request.error;
    std::cin.ignore();

    std::cout << "\nEnter a value for the time: ";
    std::cin >>srv.request.time;
    std::cin.ignore();

    if(client.call(srv))
    {
        ROS_INFO("Update: %f", srv.response.update);
    }
    else
    {
        ROS_ERROR("Failed to call service pid");
        return 1;
    }

    return 0;
}
