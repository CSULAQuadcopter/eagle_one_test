#ifndef DRONE_H
#define DRONE_H

#include <ardrone_autonomy/Navdata.h>
#include <nav_msgs/Odometry.h>
#include <eagle_one_test/ARNavdata.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#define TIMER_TIMEOUT (0.02F)   // (sec)

class Drone: public ARNavdata
{
public:
    Drone();
    Drone(ros::NodeHandle nh);
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
    void follow_tag(const geometry_msgs::Twist::ConstPtr&);

    void on_timer();

    void print_tag_distance();                 // Prints (x, y) of tag (mm, mm)
    void print_tag_x_distance();               // Prints x distance of tag (mm)
    void print_tag_y_distance();               // Prints y distance of tag (mm)

    double calcTagDistanceX(double x);         // tag z distance (mm)
    double calcTagDistanceY(double y);         // tag y distance (mm)
    double calcDistanceZ(double z);            // tag z distance (mm)
    double calcYawDistance(double yaw);        // yaw (rad)

    // Flight commands
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

    geometry_msgs::Twist twist_msg;     // velocities
    ros::NodeHandle node;
    ros::Timer timer;
    ros::Publisher publish;
};

#endif /* DRONE_H */
