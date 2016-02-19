#include <cmath>
#include <eagle_one_test/Odometry.h>
#include <iostream>

Odometry::Odometry()
{
    setPoses(0.0, 0.0, 0.0);
    setOriention(0.0, 0.0, 0.0, 0.0);
    setLinear(0.0, 0.0, 0.0);
    setAngular(0.0, 0.0, 0.0);
}

void Odometry::setPoses(double, double, double);    // poses: x, y, z (m)
void Odometry::setOriention(double, double,         // quaternion: w, x, y, z
                  double, double);
void Odometry::setLinear(double, double, double);   // linear velcoties: x, y, z (mm/s)
void Odometry::setAngular(double, double, double);  // angluar velcoties: x, y, z (rad/s)
void Odometry::setFrameId(std::string);
void Odometry::setChildId(std::string);
double Odometry::getPoseX();
double Odometry::getPoseY();
double Odometry::getPoseZ();
double Odometry::getOrientationW();
double Odometry::getOrientationX();
double Odometry::getOrientationY();
double Odometry::getOrientationZ();
double Odometry::getLinearX();
double Odometry::getLinearY();
double Odometry::getLinearZ();
double Odometry::getRoll();
double Odometry::getPitch();
double Odometry::getYaw();
