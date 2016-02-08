#ifndef ARNAVDATA_H
#define ARNAVDATA_H

/*
ARNavData



*/
class ARNavData
{
public:
    // Constructor
    ARNavData(                      // all in cartesian
        double, double, double,     // velocities (mm/s)
        double, double, double,     // accelerations (g)
        double, double, double,     // rotations (degrees)
        double);                    // altd (mm)

    void setVels(double, double, double);       // velocities: x, y, z (mm/s)
    void setAccels(double, double, double);     // accelerations: x, y, z in (g)
    void setRotations(double, double, double);  // rotations: x, y, z (degrees)
    void setAltd(double);                       // altitude: (mm)
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
    void callback(const nav_msgs::Odometry::ConstPtr&);

private:
    double vel_x_; double vel_y_; double vel_z_;
    double acc_x_; double acc_y_; double acc_z_;
    double rot_x_; double rot_y_; double rot_z_;
    double altd_;
};
#endif  /* ARNAVDATA_H */
