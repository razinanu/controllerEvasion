#!/usr/bin/env python2
from __future__ import division
import numpy as np
import math as math
import matplotlib.pyplot as plt


class BaseTrajectory:
    """
    Parameters
    ----------
    trajectory_config : array
    config[0] : lateral_length
    the lateral length of trajectory
    config[1] : velocity
    config[2] : lateral_acceleration
    maximum lateral Acceleration of vehicle
    """
    def __init__(self, trajectory_config):

        self.inflection_error = 0.005
        self.flat_error = 0.001
        self.lateral_length = trajectory_config[0]
        self.velocity = trajectory_config[1]
        self.lateral_acceleration = trajectory_config[2]
        self.longitudinal = self.velocity * (math.sqrt((math.pi * self.lateral_length) / self.lateral_acceleration))
    """
    Parameters
    ----------
    start_point: Start point of trajectory
    fraction: float
    set the fraction of trajectory (max=1)
    velocity : float
       velocity during lane change
    lateral_acceleration:float
    step : integer
        Number of samples to generate.
    Returns
    -------
    trajectory : array
    """
    def lane_change_base_trajectory(self, start_point, fraction, step):

        inflection_point = np.zeros(shape=(2, 2), dtype=float)
        flat_part = np.zeros(shape=(1, 2), dtype=float)
        trajectory = np.zeros(shape=(step, 2), dtype=float)
        y = self.lateral_length / 2 * np.pi
        z = (2 * np.pi) / self.longitudinal
        c = 0
        j = 0
        t = 0
        """ start_point of trajetory  """
        for i in np.linspace(self.longitudinal*start_point, self.longitudinal*fraction, step):
            trajectory[c, 0] = i
            trajectory[c, 1] = y*z*i - y * np.sin(z * i)

            if np.abs(self.longitudinal/2-i) < self.inflection_error and j < 2:
                inflection_point[j, 0] = trajectory[c, 0]
                inflection_point[j, 1] = trajectory[c, 1]
                j += 1

            if(np.abs(trajectory[c, 1]-trajectory[c-1, 1]) < self.flat_error) and c > 0:
                flat_part.resize((t+1, 2))
                flat_part[t, 0] = trajectory[c, 0]
                flat_part[t, 1] = trajectory[c, 1]
                t += 1
            c += 1
        #
        #  plt.plot(trajectory[:, 0], trajectory[:, 1], 'r')
        # plt.plot(inflection_point[:, 0], inflection_point[:, 1], 'go')
        # plt.plot(flat_part[:, 0], flat_part[:, 1], 'bo')
        return trajectory

    def trajectory_function(self, x):
            y = self.lateral_length / 2 * np.pi
            z = (2 * np.pi) / self.longitudinal
            function = y*z*x - y * np.sin(z * x)
            plt.plot(x, function, 'r')
            return function

    def find_flat_part(self, trajectory):

        flat_point = np.array([0, 0], dtype=float)

        for i in range(0, trajectory.shape[0]):
            if np.abs(trajectory[i, 1]-trajectory[i+1, 1]) > self.flat_error:
                flat_point[0] = trajectory[i, 0]
                flat_point[1] = trajectory[i, 1]

                break
        return flat_point
if __name__ == '__main__':

    conf = np.array([0.4, 3, 2])
    trajectory = BaseTrajectory(conf).lane_change_base_trajectory(0, 1, 300)
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'r')
    plt.show()
    x = np.linspace(0, BaseTrajectory(conf).longitudinal)
    BaseTrajectory(conf).trajectory_function(x)
    plt.show()





