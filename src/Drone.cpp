#include <cmath>
#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <nav_msgs/Odometry.h>
#include <eagle_one_test/Drone.h>
#include <ardrone_autonomy/Navdata.h>
#include <iostream>

Drone::Drone()
    :mode_(0)
{
    setTagCount(0);
    setTagX(0);
    setTagY(0);
    setYaw(0.0);
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
}
/*
Drone::Drone(ros::NodeHandle nh)
    :mode_(0),
     node(nh)
{
    setTagCount(0);
    setTagX(0);
    setTagY(0);
    setYaw(0.0);
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
    //publish = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    //timer = node.createTimer(ros::Duration(TIMER_TIMEOUT), boost::bind(&Drone::on_timer, this));
}
*/
Drone::~Drone()
{

}

void Drone::set_navdata(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    setTagCount(msg->tags_count);
    if(getTagCount() >= 1)
    {
        setTagX(msg->tags_xc[0]);
        setTagY(msg->tags_yc[0]);
        setAltd(msg->altd);
        setYaw(msg->tags_orientation[0]);
    }
}

void Drone::set_odometry(const nav_msgs::Odometry::ConstPtr& msg)
{

}

void Drone::print_tag_distance()
{
    double tag_x = calcTagDistanceX(getTagX());
    double tag_y = calcTagDistanceY(getTagY());
    std::cout << "Tag distance: ";
    if (getTagCount() >= 1)
    {
        std::cout << "("<< tag_x << ", " << tag_y << ") mm\n";
    }
    else
    {
        std::cout << "Unknown\n";
    }
}

void Drone::print_tag_x_distance()
{
    double tag_x = calcTagDistanceX(getTagX());
    std::cout << "Tag y distance: ";
    if (getTagCount() >= 1)
    {
        std::cout << tag_x << " mm\n";
    }
    else
    {
        std::cout << "Unknown\n";
    }
}

void Drone::print_tag_y_distance()
{
    double tag_y = calcTagDistanceY(getTagY());
    std::cout << "Tag y distance: ";
    if (getTagCount() >= 1)
    {
        std::cout << tag_y << " mm\n";
    }
    else
    {
        std::cout << "Unknown\n";
    }
}

/*
 * Calculates the x distance from the tag to the center of the quadcopter's
 * downward facing camera. Returns meters
 * d = z * tan(alpha + beta)
 * beta = atan(tan(gamma/2)(T' - S')/(S' - C1'))
 * alpha = roll (rad)
 * gamma = viewing angle of camera (rad)
 * T' = x coordinate of tag in image (px)
 * S' = center of image (500px)
 * C1' = x coordinate of edge of image (0px)
 * z = altitude (mm)
*/

double Drone::calcTagDistanceX(double x)
{
    double ALPHA = getRoll();
    double WIDTH_2 = 320.0;  // half the width of the image
    double GAMMA_2 = 0.5585; // gamma/2 (radians)

    // the coordinates are given in a 1000x1000 image but the camera is
    // 640x360 therefore scaling must done
    x *= 640.0/1000.0;

    double ZETA = tan(GAMMA_2);
    //double altd = getAltd();
    double beta = atan(ZETA * ((WIDTH_2 - x) / WIDTH_2));
	return getAltd() * tan(ALPHA + beta);
}

/*
 * Calculates the y distance from the tag to the center of the quadcopter's
 * downward facing camera. Returns meters
 * d = z * tan(alpha + beta)
 * beta = atan(tan(gamma/2)(S' - T')/(S' - C1'))
 * alpha = pitch (rad)
 * gamma = viewing angle of camera (rad)
 * T' = y coordinate of tag in image (px)
 * S' = center of image (500px)
 * C1' = y coordinate of edge of image (0px)
 * z = altitude (mm)
*/

double Drone::calcTagDistanceY(double y)
{
    double ALPHA = getPitch();
    double C1_PRIME = 0.0;
    double HEIGHT_2 = 180.0; // half the height of the image
    double GAMMA_2 = 0.5585; // gamma/2 (radians)

    // the coordinates are given in a 1000x1000 image but the camera is
    // 640x360 therefore scaling must done
    y *= 360.0/1000.0;

    double ZETA = tan(GAMMA_2);
    // double altd = getAltd();
    double beta = atan(ZETA * ((HEIGHT_2 - y) / HEIGHT_2));
	return (getAltd() * tan(ALPHA + beta));
}

double Drone::calcDistanceZ(double z)
{

}

double Drone::calcYawDistance(double yaw)
{
    double reference = 270.0;
    return (yaw * M_PI / 180.0);

}

void Drone::setMode(int m)
{
    mode_ = m;
}

int Drone::getMode()
{
    return mode_;
}

void Drone::setTagXDistance(int x)
{
    tag_x_distance_ = x;
}

int Drone::getTagXDistance()
{
    return tag_x_distance_;
}

void Drone::setTagYDistance(int y)
{
    tag_y_distance_= y;
}

int Drone::getTagYDistance()
{
    return tag_y_distance_;
}

void Drone::setTagZDistance(int z)
{
    tag_z_distance_ = z;
}

int Drone::getTagZDistance()
{
    return tag_z_distance_;
}
