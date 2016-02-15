#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// include tf?

/*
Odometry

The driver calculates and publishes Odometry data by integrating velocity
estimates reported by the drone (which is based on optical flow). The data is
published as nav_msgs/Odometry messages to ardrone/odometry topic. The
corresponding TF transform is also published as odom -> base transformation.

*/
class Odometry
{
public:
    Odometry();
    Odometry(                       // all in cartesian
        double, double, double,     // poses: x, y, z (mm/s)
        double, double, double,     // quaternion: w, x, y, z
        double,
        double, double, double,     // linear velocities (mm/s)
        double, double, double);    // angular velocities (rad/s)

    void setPoses(double, double, double);    // poses: x, y, z (m)
    void setOriention(double, double,         // quaternion: w, x, y, z
                      double, double);
    void setLinear(double, double, double);   // linear velcoties: x, y, z (mm/s)
    void setAngular(double, double, double);  // angluar velcoties: x, y, z (rad/s)
    double getPoseX();
    double getPoseY();
    double getPoseZ();
    double getOrientationW();
    double getOrientationX();
    double getOrientationY();
    double getOrientationZ();
    double getLinearX();
    double getLinearY();
    double getLinearZ();
    double getAngularX();
    double getAngularY();
    double getAngularZ();

    void callback(const nav_msgs::Odometry::ConstPtr&);

private:
    double pose_x_; double pose_y_; double pose_z_;
    double oriention_w_; double oriention_x_;
    double oriention_y_; double oriention_z_;
    double linear_x_; double linear_y_; double linear_z_;
    double angular_x_; double angular_y_; double angular_z_;

    string frame_id;            // necessary for /tf topic
    string child_frame_id;      // necessary for /tf topic
};
#endif  /* ODOMETRY_H */
