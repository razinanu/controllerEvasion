#!/usr/bin/env python2
from __future__ import division
import numpy as np
import math as math
import matplotlib.pyplot as plt
import model as OdeModel

class BaseTrajectory:
    """
    Parameters
    ----------
    lateral_length : float
       the lateral length of trajectory
    velocity : float
       velocity during lane change
    lateral_acceleration:float
     maximum lateral Acceleration of vehicle
    step : integer
        Number of samples to generate.
    Returns
    -------
    samples : ndarray
        `num` samples, equally spaced on a log scale.
    """
    def __init__(self, lateral_length, velocity, lateral_acceleration, step):
        self.lateral = lateral_length
        self.longitudinal = velocity * (math.sqrt((math.pi * lateral_length) / lateral_acceleration))
        self.step = step
        self.inflection_error = 0.005
        self.inflection_point = np.zeros(shape=(2, 2), dtype=float)
        self.flat_part = np.zeros(shape=(1, 2), dtype=float)
        self.flat_error = 0.001
    """
    Parameters
    ----------
    fraction: float
      set the fraction of trajectory (max=1)
    velocity : float
       velocity during lane change
    lateral_acceleration:float
     maximum lateral Acceleration of vehicle
    step : integer
        Number of samples to generate.
    Returns
    -------
    trajectory : array
    """
    def lane_change_trajectory(self, start_point, fraction):

        trajectory = np.zeros(shape=(self.step, 2), dtype=float)
        y = self.lateral / 2 * np.pi
        z = (2 * np.pi) / self.longitudinal
        c = 0
        j = 0
        t=0
        """ start_point of trajetory  """
        for i in np.linspace(self.longitudinal*start_point, self.longitudinal*fraction, self.step):
            trajectory[c, 0] = i
            trajectory[c, 1] = y*z*i - y * np.sin(z * i)
           # print "trajectory x: ", trajectory[c, 0], "trajectory y: ", trajectory[c, 1]
            if np.abs(self.longitudinal/2-i) < self.inflection_error and j < 2:
                self.inflection_point[j, 0] = trajectory[c, 0]
                self.inflection_point[j, 1] = trajectory[c, 1]
               # print "inflection x: ",self.inflection_point[j, 0], "inflection y: ", self.inflection_point[j, 1]
                j += 1

            if(np.abs(trajectory[c, 1]-trajectory[c-1, 1]) < self.flat_error) and c > 0:
                self.flat_part.resize((t+1, 2))
                self.flat_part[t, 0] = trajectory[c, 0]
                self.flat_part[t, 1] = trajectory[c, 1]
                t += 1
            c += 1
        plt.plot(trajectory[:, 0], trajectory[:, 1], 'r')
        plt.plot(self.inflection_point[:, 0], self.inflection_point[:, 1], 'go')
        plt.plot(self.flat_part[:, 0], self.flat_part[:, 1], 'bo')
        return trajectory

if __name__ == '__main__':

    trajectory = BaseTrajectory(0.4, 3, 2, 300).lane_change_trajectory(0, 1)
    plt.show()





