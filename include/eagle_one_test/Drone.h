#ifndef DRONE_H
#define DRONE_H

#include <ardrone_autonomy/Navdata.h>
#include <nav_msgs/Odometry.h>
#include <eagle_one_test/ARNavdata.h>
#include <ros/ros.h>

class Drone: public ARNavdata
{
public:
    Drone();
    ~Drone();

    void setMode(int);
    int getMode();
    void setTagXDistance(int);
    int getTagXDistance();
    void setTagYDistance(int);
    int getTagYDistance();
    void setTagZDistance(int);
    int getTagZDistance();

    void set_navdata(const ardrone_autonomy::Navdata::ConstPtr&);
    void set_odometry(const nav_msgs::Odometry::ConstPtr&);
    void print_tag_distance();

    void printTagDistanceY();
    double calcTagDistanceY(double y);
    void takeoff();
    void land();
    void emergency();
    void takeOff();
    void moveX(double);
    void moveY(double);
    void moveZ(double);
    void changeYaw(double);
    void takePicture();
    double follow(double);


private:
    int mode_;
    int tag_x_distance_;
    int tag_y_distance_;
    int tag_z_distance_;
};

#endif /* DRONE_H */
