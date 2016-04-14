#! /usr/bin/env python

# We're using a thirdparty PID
from pid_controller import PID

"""
Controller
Written by: Josh Saunders
Created: 4/11/2016

4 Degree Of Freedom (DOF) controller.
Yaw controls the orientation
X controls forward/backward position
Y controls left/right position
Z control altitude
"""

class Controller(object):
    def __init__(self):
        self.pid_x     = PID()
        self.pid_y     = PID()
        self.pid_z     = PID()
        self.pid_theta = PID()
        # self.pid_theta = YawControl(350, 10, -0.5)

    def update(self, current_value_x=0, current_value_y=0, \
               current_value_z=0, current_value_theta=0):
        self.pid_x.update(current_value_x)
        self.pid_y.update(current_value_y)
        self.pid_z.update(current_value_z)
        self.pid_theta.update(current_value_theta)

    def set_goal(self, dof, new_goal):
        if dof == "linear.x":
            self.pid_x.setPoint(new_goal)
        elif dof == "linear.y":
            self.pid_y.setPoint(new_goal)
        elif dof == "linear.z":
            self.pid_z.setPoint(new_goal)
        elif dof == "angluar.z":
            self.pid_theta.setPoint(new_goal)
