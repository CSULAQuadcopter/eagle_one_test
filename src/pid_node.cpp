#include <ros/ros.h>
#include <eagle_one_test/Pid_send.h>
#include <eagle_one_test/Pid_receive.h>
#include <eagle_one_test/PIDController.h>

void Pid::pid_set_param(const eagle_one_test::Pid_send::ConstPtr& data)
{
    calcUpdate(data->error, data->current_time);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pid_node");
    ros::NodeHandle n;

    Pid pid;
    eagle_one_test::Pid_receive pr;
    pid.setKp(0.01);
    //pid.setKd(0.0001);

    ros::Publisher pid_publisher = n.advertise<eagle_one_test::Pid_receive>("pid_receive", 5);
    ros::Subscriber pid_subscriber = n.subscribe("pid_send", 5, &Pid::pid_set_param, &pid);

    ros::Rate rate(10);

    while(ros::ok())
    {
        pr.update = pid.getUpdate();
        //ROS_INFO("");

        ROS_INFO("\nSending data:\n\tUpdate: %f", pr.update);

        pid_publisher.publish(pr);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
