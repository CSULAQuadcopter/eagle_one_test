/*
Name: sub_nav

Testing how to subscribe to different topics that the QuadCopter publishes

Inputs: None
Outputs: altitude published on the /ardrone/navdata topic

Written by: Josh Saunders (jay3ss)
Date created: 1/26/2016
Modified by: Josh Saunders (jay3ss)
Date modified: 1/27/2016
*/


#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <nav_msgs/Odometry.h>



void arOdometyCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::cout << "\nOdometry - Quaternion:\n";
    std::cout << "x: " << msg->pose.pose.orientation.w << "mm/s" << std::endl;
    std::cout << "y: " << msg->pose.pose.orientation.x << "mm/s" << std::endl;
    std::cout << "z: " << msg->pose.pose.orientation.y << "mm/s" << std::endl;
    std::cout << "z: " << msg->pose.pose.orientation.z << "mm/s" << std::endl;
}


// Functor: arNavData
//template<int>
class ARNavData
{
public:
    // Constructor
    ARNavData(
        double vx = 0, double vy = 0, double vz = 0,
        double ax = 0, double ay = 0, double az = 0,
        double al = 0)
    {
        vel_x_ = vx;
        vel_y_ = vy;
        vel_z_ = vz;
        acc_x_ = ax;
        acc_y_ = ay;
        acc_z_ = az;
        altd = al;

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

    void setAltd(double al)
    {
        altd = al;
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

    double getAltd()
    {
        return altd;
    }

    void callback(const ardrone_autonomy::Navdata::ConstPtr& msg);
private:
    double vel_x_; double vel_y_; double vel_z_;
    double acc_x_; double acc_y_; double acc_z_;
    double altd;
};

// This is the callback to display the information received from the
// /ardrone/navdata topic
// The standard template for the callback is:
//   callBackName     (const package_name::Message::ConstPtr& msg)
void ARNavData::callback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    this->setAltd(msg->altd);
    std::cout << "\nAltitude: " << this->getAltd() << "mm" << std::endl;
    std::cout << "\n";
    std::cout << "Accelerations\n";
    std::cout << "ax: " << msg->ax << "g" << std::endl;
    std::cout << "ay: " << msg->ay << "g" << std::endl;
    std::cout << "az: " << msg->az << "g" << std::endl;

    std::cout << "\n";
    std::cout << "Velocities\n";
    std::cout << "vx: " << msg->vx << "mm/s" << std::endl;
    std::cout << "vy: " << msg->vy << "mm/s" << std::endl;
    std::cout << "vz: " << msg->vz << "mm/s" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_nav");
    ros::NodeHandle n;
    ros::Rate rate(50);

    ARNavData navdata;

    ros::Subscriber altd = n.subscribe("/ardrone/navdata", 1000, &ARNavData::callback, &navdata);
    //ros::Subscriber odom = n.subscribe("/ardrone/odometry", 1000, &arOdometyCallback);

    //std::cout << "Test Potato: "  << navdata.av;

    ros::spin();
    rate.sleep();
    return 0;
}
