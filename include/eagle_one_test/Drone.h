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
    int tag_x_;
    int tag_y_;
    int tag_z_;
};

#endif /* DRONE_H */
