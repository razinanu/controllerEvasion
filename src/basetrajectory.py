#!/usr/bin/env python2
from __future__ import division
import numpy as np
import math as math
import matplotlib.pyplot as plt


class BaseTrajectory:

    def __init__(self):

        self.inflection_error = 0.005
        self.flat_error = 0.001
    """
    Parameters
    ----------
    config : array
    config[0] : lateral_length
    the lateral length of trajectory
    config[1] : velocity
    config[2] : lateral_acceleration
    maximum lateral Acceleration of vehicle
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
    def lane_change_base_trajectory(self, config, start_point, fraction, step):

        lateral_length = config[0]
        velocity = config[1]
        lateral_acceleration = config[2]
        longitudinal = velocity * (math.sqrt((math.pi * lateral_length) / lateral_acceleration))
        inflection_point = np.zeros(shape=(2, 2), dtype=float)
        flat_part = np.zeros(shape=(1, 2), dtype=float)
        trajectory = np.zeros(shape=(step, 2), dtype=float)
        y = lateral_length / 2 * np.pi
        z = (2 * np.pi) / longitudinal
        c = 0
        j = 0
        t = 0
        """ start_point of trajetory  """
        for i in np.linspace(longitudinal*start_point, longitudinal*fraction, step):
            trajectory[c, 0] = i
            trajectory[c, 1] = y*z*i - y * np.sin(z * i)
           # print "trajectory x: ", trajectory[c, 0], "trajectory y: ", trajectory[c, 1]
            if np.abs(longitudinal/2-i) < self.inflection_error and j < 2:
                inflection_point[j, 0] = trajectory[c, 0]
                inflection_point[j, 1] = trajectory[c, 1]
           # print "inflection x: ",self.inflection_point[j, 0], "inflection y: ", self.inflection_point[j, 1]
                j += 1

            if(np.abs(trajectory[c, 1]-trajectory[c-1, 1]) < self.flat_error) and c > 0:
                flat_part.resize((t+1, 2))
                flat_part[t, 0] = trajectory[c, 0]
                flat_part[t, 1] = trajectory[c, 1]
                t += 1
            c += 1
        # plt.plot(trajectory[:, 0], trajectory[:, 1], 'r')
        # plt.plot(inflection_point[:, 0], inflection_point[:, 1], 'go')
        # plt.plot(flat_part[:, 0], flat_part[:, 1], 'bo')
        return trajectory

    def find_flat_part(self, tajectory):

        flat_point = np.array([0, 0], dtype=float)

        for i in range(0, trajectory.shape[0]):
            if np.abs(trajectory[i, 1]-trajectory[i+1, 1]) > self.flat_error:
                flat_point[0] = trajectory[i, 0]
                print "x t :", trajectory[i, 0]
                flat_point[1] = trajectory[i, 1]
                print "x is ", flat_point[0]
                print "y is ", flat_point[1]
                break
        return flat_point
if __name__ == '__main__':

    conf = np.array([0.4, 3, 2])
    trajectory = BaseTrajectory().lane_change_base_trajectory(conf, 0, 1, 300)
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'r')
    flat_point = BaseTrajectory().find_flat_part(trajectory)
    plt.plot(flat_point[0], flat_point[1], 'go')
    plt.show()





