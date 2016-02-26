#include <ros/ros.h>
#include <eagle_one_test/Pid_send.h>
#include <eagle_one_test/Pid_receive.h>
#include <eagle_one_test/Drone.h>

void Drone::get_pid_update(const eagle_one_test::Pid_receive::ConstPtr& data)
{
    update = data->update;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "qc_node");
    ros::NodeHandle n;

    Drone qc;
    eagle_one_test::Pid_send pid;
    pid.error = 32;
    pid.last_time = 1;
    pid.current_time = 2;

    ros::Publisher qc_publisher = n.advertise<eagle_one_test::Pid_send>("pid_send", 1000);
    ros::Subscriber qc_subscriber = n.subscribe("pid_receive", 1000, &Drone::get_pid_update, &qc);

    ros::Rate rate(10);

    while(ros::ok())
    {
        ROS_INFO("\nSending data:\n\tError: %f \n\tCurrent Time: %i \n\tLast Time: %i",
                  pid.error, pid.current_time, pid.last_time);

        ROS_INFO("\nReceiving data:\n\tUpdate: %f", qc.update);

        qc_publisher.publish(pid);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
