#include <eagle_one_test/PIDController.h>

PIDController::PIDController()
    :proportional_gain(0),
     integral_gain(0),
     derivative_gain(0),
     prev_error(0),
     int_error(0),
     windup_guard(0),
     control(0),
     dt(0)
     {}

PIDController::~PIDController()
{}

void PIDController::pid_zeroize()
{
    // set prev and integrated error to zero
    prev_error = 0;
    int_error = 0;
}

double PIDController::pid_update(double curr_error, double dt)
{
    double diff;
    double p_term;
    double i_term;
    double d_term;

    // integration with windup guarding
    int_error += (curr_error * dt);
    if (int_error < -(windup_guard))
        int_error = -(windup_guard);
    else if (int_error > windup_guard)
        int_error = windup_guard;

    // differentiation
    diff = ((curr_error - prev_error) / dt);

    // scaling
    p_term = (proportional_gain * curr_error);
    i_term = (integral_gain     * int_error);
    d_term = (derivative_gain   * diff);

    // summation of terms
    control = p_term + i_term + d_term;

    // save current error as previous error for next iteration
    prev_error = curr_error;

    return p_term + i_term + d_term;
}

double PIDController::p_update()
{
    return 32;
}

double PIDController::i_update()
{

}

double PIDController::d_update()
{

}


void PIDController::setKp(double kp)
{
    proportional_gain = kp;
}

void PIDController::setKi(double ki)
{
    integral_gain = ki;
}

void PIDController::setKd(double kd)
{
    derivative_gain = kd;
}

void PIDController::setPrevError(double)
{

}

void PIDController::setIntError(double)
{

}

void PIDController::setWindUp(double)
{

}

// void PIDController::setControl(double)
// {
//
// }

void PIDController::setDt(double ct)
{

}


double PIDController::getKp()
{

}

double PIDController::getKi()
{

}

double PIDController::getKd()
{

}

double PIDController::getPrevError()
{

}

double PIDController::getIntError()
{

}
double PIDController::getWindUp()
{

}

// double PIDController::getControl()
// {
//
// }

double PIDController::getDt()
{

}
