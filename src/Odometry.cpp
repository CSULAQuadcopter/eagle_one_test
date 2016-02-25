#include <cmath>
#include <eagle_one_test/Odometry.h>
#include <iostream>

Odometry::Odometry()
{
    setPoses(0.0, 0.0, 0.0);
    setOriention(0.0, 0.0, 0.0, 0.0);
    setLinear(0.0, 0.0, 0.0);
    setAngular(0.0, 0.0, 0.0);
    setFrameId("");
    setChildId("");
}
 /*
void Odometry::getOdomData(const nav_msgs::Odometry::ConstPtr& msg)
{

}
*/
void Odometry::setPoses(double x, double y, double z)
{
    pose_x_ = x;
    pose_y_ = y;
    pose_z_ = z;
}

void Odometry::setOriention(double w, double x, double y, double z)
{
    oriention_w_ = w;
    oriention_x_ = x;
    oriention_y_ = y;
    oriention_z_ = z;
}
void Odometry::setLinear(double x, double y, double z)
{
    linear_x_ = x;
    linear_y_ = y;
    linear_z_ = z;
}

void Odometry::setAngular(double roll, double pitch, double yaw)
{
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
}

void Odometry::setFrameId(std::string f_id)
{
    frame_id_ = f_id;
}

void Odometry::setChildId(std::string c_id)
{
    child_frame_id_ = c_id;
}

double Odometry::getPoseX()
{
    return pose_x_;
}

double Odometry::getPoseY()
{
    return pose_y_;
}

double Odometry::getPoseZ()
{
    return pose_z_;
}

double Odometry::getOrientationW()
{
    return oriention_w_;
}

double Odometry::getOrientationX()
{
    return oriention_x_;
}

double Odometry::getOrientationY()
{
    return oriention_y_;
}

double Odometry::getOrientationZ()
{
    return oriention_z_;
}

double Odometry::getLinearX()
{
    return linear_x_;
}

double Odometry::getLinearY()
{
    return linear_y_;
}

double Odometry::getLinearZ()
{
    return linear_z_;
}

double Odometry::getRoll()
{
    return roll_;
}

double Odometry::getPitch()
{
    return pitch_;
}

double Odometry::getYaw()
{
    return yaw_;
}
