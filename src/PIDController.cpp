#include <eagle_one_test/PIDController.h>

PIDController::PIDController()
    :proportional_gain,
     integral_gain,
     derivative_gain,
     prev_error,
     int_error,
     windup_guard,
     control,
     dt
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
    pid->int_error += (curr_error * dt);
    if (pid->int_error < -(pid->windup_guard))
        pid->int_error = -(pid->windup_guard);
    else if (pid->int_error > pid->windup_guard)
        pid->int_error = pid->windup_guard;

    // differentiation
    diff = ((curr_error - pid->prev_error) / dt);

    // scaling
    p_term = (pid->proportional_gain * curr_error);
    i_term = (pid->integral_gain     * pid->int_error);
    d_term = (pid->derivative_gain   * diff);

    // summation of terms
    pid->control = p_term + i_term + d_term;

    // save current error as previous error for next iteration
    pid->prev_error = curr_error;
}

double PIDController::p_update(){}
double PIDController::i_update(){}
double PIDController::d_update(){}
