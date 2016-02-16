#include <cmath>
#include <eagle_one_test/Drone.h>

Drone::Drone()
    :mode_(0)
{
    setTagCount(0);
    setTagX(0);
    setTagY(0);
    setTagOrientation(0.0);
}

//
void Drone::set_navdata(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    setTagX(msg.tags_xc[0]);
    setTagY(msg.tags_yc[0]);
    setTagOrientation(msg.tags_orientation[0]);
}

void Drone::set_odometry(const nav_msgs::Odometry::ConstPtr& msg)
{

}

double Drone::getTagDistanceY(double y)
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

void Drone::setTagX(int x)
{
    tag_x_ = x;
}

int Drone::getTagX()
{
    return tag_x_;
}

void Drone::setTagY(int y)
{
    tag_y_ = y;
}

int Drone::getTagY()
{
    return tag_y_;
}

void Drone::setTagZ(int z)
{
    tag_z_ = z;
}

int Drone::getTagZ()
{
    return tag_z_;
}
