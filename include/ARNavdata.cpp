#include "ARNavdata.h"

class ARNavData
{
public:
    // Constructor
    ARNavData(
        double vx = 0, double vy = 0, double vz = 0,
        double ax = 0, double ay = 0, double az = 0,
        double rx = 0, double ry = 0, double rz = 0,
        double al = 0)
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

    void setVels(double vx, double vy, double vz)
    {
        vel_x_ = vx;
        vel_y_ = vy;
        vel_z_ = vz;
    }

    void setAccels(double ax, double ay, double az)
    {
        acc_x_ = ax;
        acc_y_ = ay;
        acc_z_ = az;
    }

    void setRotations(rx, double ry, double rz)
    {
        rot_x_ = rx;
        rot_y_ = ry;
        rot_z_ = rz;
    }

    void setAltd(double alt)
    {
        altd_ = alt;
    }

    double getVx()
    {
        return vel_x_;
    }

    double getVy()
    {
        return vel_y_;
    }

    double getVz()
    {
        return vel_z_;
    }

    double getAx()
    {
        return acc_x_;
    }

    double getAy()
    {
        return acc_y_;
    }

    double getAz()
    {
        return acc_z_;
    }

    double getRx()
    {
        return rot_x_;
    }

    double getRy()
    {
        return rot_y_;
    }

    double getRz()
    {
        return rot_z_;
    }

    double getAltd()
    {
        return altd_;
    }

    void callback(const ardrone_autonomy::Navdata::ConstPtr& msg);
private:
    double vel_x_; double vel_y_; double vel_z_;
    double acc_x_; double acc_y_; double acc_z_;
    double rot_x_; double rot_y_; double rot_z_;
    double altd_;
};