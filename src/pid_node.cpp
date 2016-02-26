#include <ros/ros.h>
#include <eagle_one_test/Pid_send.h>
#include <eagle_one_test/Pid_receive.h>
#include <eagle_one_test/PIDController.h>

void PIDController::pid_set_param(const eagle_one_test::Pid_send::ConstPtr& data)
{
    current_error = data->error;
    last_time = data->last_time;
    current_time = data->current_time;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pid_node");
    ros::NodeHandle n;

    PIDController pid;
    eagle_one_test::Pid_receive pr;

    ros::Publisher pid_publisher = n.advertise<eagle_one_test::Pid_receive>("pid_receive", 1000);
    ros::Subscriber pid_subscriber = n.subscribe("pid_send", 1000, &PIDController::pid_set_param, &pid);

    ros::Rate rate(10);

    while(ros::ok())
    {
        pr.update = pid.p_update();
        ROS_INFO("\nReceived data:\n\tError: %f \n\tCurrent Time: %i \n\tLast Time: %i",                  pid.current_error, pid.current_time, pid.last_time);

        ROS_INFO("\nSending data:\n\tUpdate: %f", pr.update);

        pid_publisher.publish(pr);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
