#!/usr/bin/env python2
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
import model as OdeModel
import basetrajectory

class Trajectory:

    def __init__(self):

        self.model = OdeModel.OdeModel()
        self.step = 30
        self.moment_max = 22.5
        self.servo_max = 0.35

    def lane_change_trajectory(self, conf_start, base_trajectory, start_servo, start_moment,flat_point):

        solution = self.model.solve(x_start=conf_start, u=(0, 2),
                                    n_steps=2, dt=0.001)

        while flat_point[0] > solution[-1, 0]:
            solution = self.model.solve(x_start=conf_start, u=(0, self.moment_max), n_steps=self.step, dt=0.001)
            plt.plot(solution[:, 0], solution[:, 1], 'go')
            conf_start = np.array([solution[-1, 0], solution[-1, 1],
                                   solution[-1, 2], solution[-1, 3], solution[-1, 4], solution[-1, 5], solution[-1, 6]])


        plt.plot(base_trajectory[:, 0], base_trajectory[:, 1], 'r')
        return solution
if __name__ == '__main__':

    conf = np.array([0.4, 3, 2])
    base_trajectory = basetrajectory.BaseTrajectory().lane_change_base_trajectory(conf, 0, 0.5, 300)
    flat_point = basetrajectory.BaseTrajectory().find_flat_part(base_trajectory)
    conf_start = np.array([0, 0, 0, 0, 0, 1, 0])
    #u=(0.35, 22.5)
    solution = Trajectory().lane_change_trajectory(conf_start, base_trajectory, 0, 22.5,flat_point)
    plt.show()


