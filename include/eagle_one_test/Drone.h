#ifndef DRONE_H
#define DRONE_H

#include <eagle_one_test/ARNavdata.h>

class Drone: public ARNavdata
{
public:
    Drone();
    ~Drone();

    void setMode(int);
    int getMode();
    void setTagX(int);
    int getTagX();
    void setTagY(int);
    int getTagY();
    void setTagZ(int);
    int getTagZ();

    void set_navdata(const ardrone_autonomy::Navdata::ConstPtr&);
    void set_odometry(const nav_msgs::Odometry::ConstPtr&);

    double getTagDistanceY(double y);
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
