#! /usr/bin/env python
'''
Controller
4 Degree Of Freedom (DOF) controller.
Yaw controls the orientation
X controls forward/backward position
Y controls left/right position
Z control altitude

Written by: Josh Saunders
Created: 4/11/2016

Modified by: Josh Saunders
Date modified: 5/20/2016
'''

# We're using a thirdparty PID
from pid_controller import PID

class Controller(object):
    def __init__(self, pid_x, pid_y, pid_z, pid_theta, bounding_box=True):
        '''
        @param: pid_x is a tuple such that (kp, ki, kd, integrator, derivator,
                set_point)
        @param: pid_y is a tuple such that (kp, ki, kd, integrator, derivator,
                set_point)
        @param: pid_z is a tuple such that (kp, ki, kd, integrator, derivator,
                set_point)
        @param: pid_theta is a tuple such that (kp, ki, kd, integrator,
                derivator, set_point)
        @param: bounding_box is a boolean that will initially turn the bounding
                box on (True) or off (False). Default is True
        '''
        self.pid_x     = PID()
        self.pid_y     = PID()
        self.pid_z     = PID()
        self.pid_theta = PID()

        # Set gains
        self.pid_x.setKp(pid_x[0])
        self.pid_x.setKi(pid_x[1])
        self.pid_x.setKd(pid_x[2])
        self.pid_x.setIntegrator(pid_x[3])
        self.pid_x.setDerivator(pid_x[4])
        self.pid_x.setPoint(pid_x[5])

        self.pid_y.setKp(pid_y[0])
        self.pid_y.setKi(pid_y[1])
        self.pid_y.setKd(pid_y[2])
        self.pid_y.setIntegrator(pid_y[3])
        self.pid_y.setDerivator(pid_y[4])
        self.pid_y.setPoint(pid_y[5])

        self.pid_z.setKp(pid_z[0])
        self.pid_z.setKi(pid_z[1])
        self.pid_z.setKd(pid_z[2])
        self.pid_z.setIntegrator(pid_z[3])
        self.pid_z.setDerivator(pid_z[4])
        self.pid_z.setPoint(pid_z[5])

        self.pid_theta.setKp(pid_theta[0])
        self.pid_theta.setKi(pid_theta[1])
        self.pid_theta.setKd(pid_theta[2])
        self.pid_theta.setIntegrator(pid_theta[3])
        self.pid_theta.setDerivator(pid_theta[4])
        self.pid_theta.setPoint(pid_theta[5])

        # Should we use the bounding box?
        self.use_bounding_box = bounding_box

        def is_in_box(self, minimum, maximum, position):
            '''
            Checks if the position is within the given bounds
            '''
            if ((minimum < position) and (position < maximum)):
                # print("In box")
                return True
            else:
                # print("Out box")
                return False

        def update(self, values):
            '''
            This updates each controller and returns the updated values as a
            tuple

            @param: values is a tuple with the current value for each degree of
            freedom
            '''
            x_update = self.pid_x.update(values[0])
            y_update = self.pid_y.update(values[0])
            z_update = self.pid_z.update(values[0])
            theta_update = self.pid_theta.update(values[0])

            return (x_update, y_update, z_update, theta_update)
