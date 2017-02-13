#!/usr/bin/env python2
from __future__ import division

import numpy as np

import matplotlib.pyplot as plt
#import sys
#sys.path.insert(0, '../motion_model/src/motion_model')
#import model.py as OdeModel
import motion_model.new_model as OdeModel

from controller_evasion import basetrajectory


class Trajectory:

    def __init__(self, base_trajectory_config):

        self.model = OdeModel.OdeModel()
        self.step = 30
        self.moment_max = 22.5
        self.servo_max = 0.35
        self.base_trajectory = basetrajectory.BaseTrajectory(base_trajectory_config)
        self.min_velocity = 5
        self.max_velocity = 23
        self.min_servo = 1
        self.max_servo = 3
        self.min_step = 5

    def lane_change_trajectory(self, conf_start, start_servo, start_moment):

        step = self.min_step
        solution = self.model.solve(x_start=conf_start, u=(0.0, 5),
                                    n_steps=step, dt=0.001)
        function = self.base_trajectory.trajectory_function(solution[:, 0])

        x = 0.88*step
        v = self.min_velocity
        u = self.min_servo
        velocity_max = 0
        max_servo = 0
        max_x = 0
        max_step = 0
        while(u < self.max_servo):

            while(v < self.max_velocity):

                while np.abs(solution[x:, 1].mean(0) - function[x:].mean(0)) < 0.0001:
                    step += 90
                    solution = self.model.solve(x_start=conf_start, u=((u/10), v),
                                     n_steps =step, dt=0.001)
                    function = self.base_trajectory.trajectory_function(solution[:, 0])
                    plt.plot(solution[:, 0], solution[:, 1], 'go')
                    if(max_x < solution[-1,0]):
                         velocity_max = v
                         max_servo= u
                         max_step=step
                         max_x= solution[-1, 0]
                         bestTryjectory= np.array(solution.shape, dtype=float)
                         bestTryjectory=solution

                v += 1
                step=5
                solution = self.model.solve(x_start=conf_start, u=(u, v),
                                    n_steps=step, dt=0.001)
                function = self.base_trajectory.trajectory_function(solution[:, 0])
            step=5
            v=10
            u += 0.1
            solution = self.model.solve(x_start=conf_start, u=(u, v),
                                    n_steps=step, dt=0.001)
            function = self.base_trajectory.trajectory_function(solution[:, 0])


        print "max is", max_x
        print "max_servo", max_servo
        print "max_vel", velocity_max
        print "max_step", max_step

        plt.plot(bestTryjectory[:, 0], bestTryjectory[:, 1], 'bo')
        return solution





