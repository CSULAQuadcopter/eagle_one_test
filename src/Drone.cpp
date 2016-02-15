#include <eagle_one_test/Drone.h>

Drone::Drone()
    :mode_(0){}

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
