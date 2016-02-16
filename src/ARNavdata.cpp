#include <eagle_one_test/ARNavdata.h>
#include <ardrone_autonomy/Navdata.h>
#include <ros/ros.h>

ARNavdata::ARNavdata()
{
    vel_x_ = 0.0;
    vel_y_ = 0.0;
    vel_z_ = 0.0;
    acc_x_ = 0.0;
    acc_y_ = 0.0;
    acc_z_ = 0.0;
    rot_x_ = 0.0;
    rot_y_ = 0.0;
    rot_z_ = 0.0;
    altd_ = 0.0;
}
ARNavdata::ARNavdata(
    double vx, double vy, double vz,
    double ax, double ay, double az,
    double rx, double ry, double rz,
    double al)
{
    vel_x_ = vx;
    vel_y_ = vy;
    vel_z_ = vz;
    acc_x_ = ax;
    acc_y_ = ay;
    acc_z_ = az;
    rot_x_ = ax;
    rot_y_ = ay;
    rot_z_ = az;
    altd_ = al;

}

void ARNavdata::setVels(double vx, double vy, double vz)
{
    vel_x_ = vx;
    vel_y_ = vy;
    vel_z_ = vz;
}

void ARNavdata::setAccels(double ax, double ay, double az)
{
    acc_x_ = ax;
    acc_y_ = ay;
    acc_z_ = az;
}

void ARNavdata::setRotations(double rx, double ry, double rz)
{
    rot_x_ = rx;
    rot_y_ = ry;
    rot_z_ = rz;
}

void ARNavdata::setAltd(double alt)
{
    altd_ = alt;
}

double ARNavdata::getVx()
{
    return vel_x_;
}

double ARNavdata::getVy()
{
    return vel_y_;
}

double ARNavdata::getVz()
{
    return vel_z_;
}

double ARNavdata::getAx()
{
    return acc_x_;
}

double ARNavdata::getAy()
{
    return acc_y_;
}

double ARNavdata::getAz()
{
    return acc_z_;
}

double ARNavdata::getRx()
{
    return rot_x_;
}

double ARNavdata::getRy()
{
    return rot_y_;
}

double ARNavdata::getRz()
{
    return rot_z_;
}

double ARNavdata::getAltd()
{
    return altd_;
}

int ARNavdata::getTagCount()
{
    return tag_count_;
}

int ARNavdata::getTagX()
{
    return tag_xc_;
}

int ARNavdata::getTagY()
{
    return tag_yc_;
}

double ARNavdata::getTagOrientation()
{
    return tag_orientation_;
}
int ARNavdata::getTm()
{
    return tm_;
}
void ARNavdata::setTagCount(int count)
{
    tag_count_ = count;
}
void ARNavdata::setTagX(int tx)
{
    tag_xc_ = tx;
}
void ARNavdata::setTagY(int ty)
{
    tag_yc_ = ty;
}
void ARNavdata::setTagOrientation(double to)
{
    tag_orientation_ = to;
}
void ARNavdata::setTm(int t)
{
    tm_ = t;
}

void ARNavdata::callback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{}
