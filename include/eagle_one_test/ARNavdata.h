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
    void setAccels(double, double, double);     // accelerations: x, y, z in (g)
    void setRotations(double, double, double);  // rotations: x, y, z (degrees)
    void setAltd(double);                       // altitude: (cm)
    double getVx();
    double getVy();
    double getVz();
    double getAx();
    double getAy();
    double getAz();
    double getRx();     // Roll (degrees)
    double getRy();     // Pitch (degrees)
    double getRz();     // Yaw (degrees)
    double getAltd();

    void callback(const ardrone_autonomy::Navdata::ConstPtr&);

private:
    double vel_x_; double vel_y_; double vel_z_;
    double acc_x_; double acc_y_; double acc_z_;
    double rot_x_; double rot_y_; double rot_z_;
    double altd_;

    // Tags in Vision Detection
    // TODO figure out how to implement the tags portion
    // As an array or just take the first values from the message array?
    int tags_count;
    int tags_type[];
    int tags_xc[];
    int tags_yc[];
    int tags_width[];
    int tags_height[];
    float tags_orientation[];
    float tags_distance[];

    //time stamp
    float tm;
};
#endif  /* ARNavdata_H */
