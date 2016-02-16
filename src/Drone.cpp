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
        std::cout << tag_y << "\n";
    }
    else
    {
        std::cout << "Unknown\n";
    }
}

double Drone::calcTagDistanceY(double y)
{
    double ALPHA = 0.0;
    double GAMMA_2 = 1.2743; // radians
    double ZETA = tan(GAMMA_2);
    double altd = getAltd();
    double beta = atan(ZETA * (1 - y/500));
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
