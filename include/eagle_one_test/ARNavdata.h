#ifndef ARNAVDATA_H
#define ARNAVDATA_H

#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>

/*
ARNavdata
*/
class ARNavdata
{
public:
    // Constructors
    ARNavdata();
    ARNavdata(                      // all in cartesian
        double, double, double,     // velocities (mm/s)
        double, double, double,     // accelerations (g)
        double, double, double,     // rotations (degrees)
        double);                    // altd (mm)

    void setVels(double, double, double);       // velocities: x, y, z (mm/s)
    void setPrevVels();                         // velocities: x, y, z (mm/s)
    void setAccels(double, double, double);     // accelerations: x, y, z in (g)
    void setRotations(double, double, double);  // rotations: roll, pitch, yaw (rads)
    void setAltd(double);                       // altitude: (mm)
    double getVx();
    double getVy();
    double getVz();
    double getAx();
    double getAy();
    double getAz();
    double getPrevVx();
    double getPrevVy();
    double getPrevVz();
    double getRoll();    // Roll (degrees)
    double getPitch();   // Pitch (degrees)
    double getYaw();     // Yaw (degrees)
    double getAltd();
    int getTagCount();
    int getTagX();
    int getTagY();
    int getTm();
    void setTagCount(int);
    void setTagX(int);
    void setTagY(int);
    void setYaw(double);
    void setTm(int);

    void tag_x_pos(const ardrone_autonomy::Navdata::ConstPtr&);

private:
    double vel_x_; double vel_y_; double vel_z_;
    double prev_vel_x_; double prev_vel_y_; double prev_vel_z_;
    double acc_x_; double acc_y_; double acc_z_;
    double roll_; double pitch_; double yaw_;
    double altd_;

    // Tags in Vision Detection
    // TODO figure out how to implement the tags portion
    // As an array or just take the first values from the message array?

    // Only grab the first (and only?) tag from the array
    int tag_count_;
    // int tags_type_;
    int tag_xc_;
    int tag_yc_;
    // int tags_width_;
    // int tags_height_;
    double tag_orientation_;
    double tags_distance_;

    //time stamp
    double tm_;
};
#endif  /* ARNavdata_H */
