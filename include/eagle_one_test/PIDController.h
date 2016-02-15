#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <ros/ros.h>

class PIDController
{
public:
    PIDController();
    PIDController();
    ~PIDController();

    double pid_update();
    double p_update();
    double i_update();
    double d_update();
    void pid_zeroize();

private:
    double proportional_gain;
    double integral_gain;
    double derivative_gain;
    double prev_error;
    double int_error;
    double windup_guard;
    double control;
    double dt;
};
#endif /* PIDCONTROLLER_H */
