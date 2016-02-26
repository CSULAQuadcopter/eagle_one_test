#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <ros/ros.h>
#include <eagle_one_test/Pid_send.h>
#include <eagle_one_test/Pid_receive.h>

class PIDController
{
public:
    //PIDController();
    PIDController();
    ~PIDController();

    double pid_update(double, double);
    double p_update();
    double i_update();
    double d_update();
    void pid_zeroize();

    void setKp(double);
    void setKi(double);
    void setKd(double);
    void setPrevError(double);
    void setIntError(double);
    void setWindUp(double);
    void setControl(double);
    void setDt(double);

    double getKp();
    double getKi();
    double getKd();
    double getPrevError();
    double getIntError();
    double getWindUp();
    double getControl();
    double getDt();

    double current_error;
    int last_time;
    int current_time;

    void pid_set_param(const eagle_one_test::Pid_send::ConstPtr& data);

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
