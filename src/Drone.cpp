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
    setTagOrientation(0.0);
}

Drone::~Drone()
{

}

//
void Drone::set_navdata(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    setTagCount(msg->tags_count);
    if(getTagCount() >= 1)
    {
        setTagX(msg->tags_xc[0]);
        setTagY(msg->tags_yc[0]);
        setAltd(msg->altd);
        setTagOrientation(msg->tags_orientation[0]);
    }
}

void Drone::set_odometry(const nav_msgs::Odometry::ConstPtr& msg)
{

}

void Drone::print_tag_distance()
{
    double tag_y = calcTagDistanceY(getTagY());
    std::cout << "Tag y distance: ";
    if (getTagCount() >= 1)
    {
        std::cout << tag_y << "m\n";
    }
    else
    {
        std::cout << "Unknown\n";
    }
}

/*
 * Calculates the distance from the tag to the center of the quadcopter's
 * downward facing camera. d = tan(alpha + beta)
 * beta = z * atan(tan(gamma/2)(S' - T')/(S' - C1'))
 * alpha = pitch (rad)
 * gamma = viewing angle of camera (rad)
 * T' = y coordinate of tag in image (px)
 * S' = center of image (500px)
 * C1' = y coordinate of edge of image (0px)
 * z = altitude (mm)
*/

double Drone::calcTagDistanceY(double y)
{
    double ALPHA = 0.0;
    double C1_PRIME = 0;
    double S_PRIME = 500;
    double GAMMA_2 = 1.2743; // radians
    double ZETA = tan(GAMMA_2);
    double altd = getAltd();
    double beta = atan(ZETA * ((S_PRIME - y)/(S_PRIME - C1_PRIME)));
	return getAltd() * tan(ALPHA + beta);
}

/*
 * Calculates the distance from the tag to the center of the quadcopter's
 * downward facing camera. d = tan(alpha + beta)
 * beta = z * atan(tan(gamma/2)(S' - T')/(S' - C1'))
 * alpha = roll (rad)
 * gamma = viewing angle of camera (rad)
 * T' = x coordinate of tag in image (px)
 * S' = center of image (500px)
 * C1' = x coordinate of edge of image (0px)
 * z = altitude (mm)
*/

double Drone::calcTagDistanceX(double x)
{
    double ALPHA = 0.0;
    double C1_PRIME = 0;
    double S_PRIME = 500;
    double GAMMA_2 = 1.2743; // radians
    double ZETA = tan(GAMMA_2);
    double altd = getAltd();
    double beta = atan(ZETA * ((S_PRIME - x)/(S_PRIME - C1_PRIME)));
	return getAltd() * tan(ALPHA + beta);
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
