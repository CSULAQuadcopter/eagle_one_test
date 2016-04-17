/*
Parker Conroy
Algorithmic Robotics Lab @ University of Utah

This program launches the AR Drone.
It is intended as a simple example for those starting with the AR Drone platform.
*/
#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

const double ALPHA = 0.0;
const double GAMMA_2 = 1.2743; // radians
const double ZETA = tan(GAMMA_2);
//#include <ardrone_autonomy/NavdataMessageDefinitions.h>

std_msgs::Empty emp_msg;

double getTagDistanceY(double y, double alt);

int main(int argc, char** argv)
{

	ROS_INFO("Flying AR.Drone 2.0");
	ros::init(argc, argv,"ARDrone_test");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
	ros::Publisher pub_takeoff;
	ros::Publisher pub_land;
	/* Message queue length is just 1 */
	pub_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
	pub_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1);

 	while (ros::ok())
	{
		double time_start=(double)ros::Time::now().toSec();
		/* Send command for five seconds*/
		while ((double)ros::Time::now().toSec()< time_start+8.0)
		{
			pub_takeoff.publish(emp_msg); /* launches the drone */
			ros::spinOnce();
			loop_rate.sleep();
		}//time loop
		ROS_INFO("AR.Drone 2.0 launched");
		/* Message queue length is just 1 */

		pub_land.publish(emp_msg);
		ros::spinOnce();
		loop_rate.sleep();

		ROS_INFO("AR.Drone 2.0 landed");

		exit(0);
	}//ros::ok loop

}//main



double getTagDistanceY(double y, double alt)
{
	double beta = atan(ZETA * (1 - y/500));
	return alt * tan(ALPHA + beta);
}
