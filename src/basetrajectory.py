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
    def test_lane_change_base_trajectory(self, start_point, fraction, step, trajectory_config):
        longitudinal = trajectory_config[1] * (math.sqrt((math.pi * trajectory_config[0]) /trajectory_config[2]))
        inflection_point = np.zeros(shape=(2, 2), dtype=float)
        flat_part = np.zeros(shape=(1, 2), dtype=float)
        trajectory = np.zeros(shape=(step, 2), dtype=float)
        y = trajectory_config[0] / 2 * np.pi
        z = (2 * np.pi) / longitudinal
        c = 0
        j = 0
        t = 0
        """ start_point of trajetory  """
        for i in np.linspace(longitudinal*start_point, longitudinal*fraction, step):
            trajectory[c, 0] = i
            trajectory[c, 1] = y*z*i - y * np.sin(z * i)

            if np.abs(longitudinal/2-i) < self.inflection_error and j < 2:
                inflection_point[j, 0] = trajectory[c, 0]
                inflection_point[j, 1] = trajectory[c, 1]
                j += 1

            if(np.abs(trajectory[c, 1]-trajectory[c-1, 1]) < self.flat_error) and c > 0:
                flat_part.resize((t+1, 2))
                flat_part[t, 0] = trajectory[c, 0]
                flat_part[t, 1] = trajectory[c, 1]
                t += 1
            c += 1

        return trajectory
    """
    Parameters
    ----------
    x_start : array
       initial position of x and y.
    trajectory_config : array
    config[0] : float
        the lateral length of trajectory.
    config[1] : float
        velocity
    config[2] : float
        maximum lateral Acceleration of vehicle.
    step : integer
        Number of samples to generate.
    dt : float
        Size of spacing between samples
    Returns
    -------
    trajectory : array
    """
    def lane_change_base_trajectory(self, x_start, traj_config, step,dt):

        longitudinal_length = traj_config[1] * (math.sqrt((math.pi * traj_config[0]) / traj_config[2]))
        y = traj_config[0] / 2 * np.pi
        z = (2 * np.pi) / longitudinal_length
        max_step = self.get_max_step(traj_config,dt)
        steps = max_step if max_step < step else step
        # store trajectory
        traj = np.zeros(shape=(steps+1, 2))
        # set start point
        traj[0, :] = x_start

        for i in xrange(1, steps+1):
            traj[i, :] = np.array(self.traj_fcn(traj[i-1, :], dt, y, z))

        return traj

    def get_max_step(self, traj_config, dt):

        longitudinal_length = traj_config[1] * (math.sqrt((math.pi * traj_config[0]) / traj_config[2]))

        return int(longitudinal_length/dt)

    def traj_fcn(self, x, delta_t, y, z):

        x0 = x[0] + delta_t
        x1 = y*z*x0 - y * np.sin(z * x0)
        return [x0, x1]

    def find_flat_part(self, test_trajectory):

        flat_point = np.array([0, 0], dtype=float)

        for i in range(0, test_trajectory.shape[0]):
            if np.abs(test_trajectory[i, 1]-test_trajectory[i+1, 1]) > self.flat_error:
                flat_point[0] = test_trajectory[i, 0]
                flat_point[1] = test_trajectory[i, 1]

                break
        return flat_point
if __name__ == '__main__':

    trajectory_conf = np.array([0.4, 3, 2])
    conf_start = np.array([0, 0])
    trajectory = BaseTrajectory().lane_change_base_trajectory(conf_start, trajectory_conf, 800000, 0.00001)
    plt.plot(trajectory[:,0],trajectory[:,1], 'g')
    plt.show()





