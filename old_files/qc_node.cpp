#include <ros/ros.h>
#include <eagle_one_test/Pid_send.h>
#include <eagle_one_test/Pid_receive.h>
#include <eagle_one_test/Drone.h>
#include <geometry_msgs/Twist.h>

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
    pid.last_time = 0;
    pid.current_time = 2;

    geometry_msgs::Twist twist_msg;

    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;

    ros::Publisher qc_publisher = n.advertise<eagle_one_test::Pid_send>("pid_send", 5);
    ros::Subscriber qc_subscriber = n.subscribe("pid_receive", 5, &Drone::get_pid_update, &qc);
    ros::Subscriber tag_info = n.subscribe("/ardrone/navdata", 5, &Drone::set_navdata, &qc);
    ros::Publisher follow = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

    ros::Rate rate(10);

    while(ros::ok())
    {
        pid.error = qc.calcTagDistanceX(qc.getTagX());
        pid.current_time = (double) ros::Time::now().toSec();

        twist_msg.linear.x = qc.update;

        qc_publisher.publish(pid);
        follow.publish(twist_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
