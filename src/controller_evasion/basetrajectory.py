#!/usr/bin/env python2
from __future__ import division
import numpy as np
import math
import matplotlib.pyplot as plt
import bezierCurve as bz


class BaseTrajectory:
    '''
    Parameters
    ----------
    traj_config : array
    config[0] : float
        the lateral length of trajectory.
    config[1] : float
        velocity for lane change
    config[2] : float
        maximum lateral Acceleration of vehicle.
    '''
    def __init__(self, traj_config, dt):

        self.inflection_error = 0.005
        self.flat_error = 0.001
        self.longitudinal_length = traj_config[1] * (math.sqrt((2*math.pi * traj_config[0]) / traj_config[2]))
        self.first_const = traj_config[0] / (2 * math.pi)
        self.second_const = (2 * math.pi) / self.longitudinal_length
        self.max_step = int((0.5 * self.longitudinal_length)/dt)
        self.dt = dt
        self.lateral = traj_config[0]
        #TODO set the actuell velocity
        self.velocity = traj_config[1]

    '''
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
    '''
    def test_lane_change_base_trajectory(self, start_point, fraction, step, trajectory_config):
        longitudinal = trajectory_config[1] * (math.sqrt((math.pi * trajectory_config[0]) /trajectory_config[2]))
        inflection_point = np.zeros(shape=(2, 2), dtype=float)
        flat_part = np.zeros(shape=(1, 2), dtype=float)
        trajectory = np.zeros(shape=(step, 2), dtype=float)
        y = trajectory_config[0] / 2 * math.pi
        z = (2 * math.pi) / longitudinal
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
    step : integer
        Number of samples to generate.
    dt : float
        Size of spacing between samples
    Returns
    -------
    trajectory : array
    """
    def lane_change_base_trajectory(self, x_start, isSecond):

        vp = [bz.pt(0, 0), bz.pt(0.6, 0), bz.pt(1, 1)]
        traj1 = bz.demo(vp)

        steps = self.max_step
        # store trajectory
        traj = np.zeros(shape=(steps+1, 3))
        # set start point
        traj[0, :] = x_start
        for i in xrange(1, steps+1):
            traj[i, :] = np.array(self.traj_fcn(traj[i-1, :], self.dt, self.first_const, self.second_const))
        if isSecond:
            traj = self.second_half(traj)
        return traj

    def traj_fcn(self, x, delta_t, y, z):

        x0 = x[0] + delta_t
        x1 = self.first_const*self.second_const*x0 - self.first_const * np.sin(self.second_const * x0)
        dy = x1 - x[1]
        x2 = np.arctan2(dy, delta_t)
        return [x0, x1, x2]

    def second_half(self, trajectory):

        offset_x = trajectory[0, 0]
        offset_y = trajectory[0, 1]
        new_trajectory = trajectory
        for i, points in enumerate(trajectory[:, 0]):
            new_trajectory[i, 0] -= offset_x
            new_trajectory[i, 1] -= offset_y
        return new_trajectory

    def merging_base_trajectory(self, x_start):


        steps = self.max_step
        # store trajectory
        traj = np.zeros(shape=(steps+1, 3))
        # set start point
        traj[0, :] = x_start
        for i in xrange(1, steps+1):
            traj[i, :] = np.array(self.traj_merg_fcn(traj[i-1, :], self.dt, self.first_const, self.second_const))

        return traj

    def traj_merg_fcn(self, x, delta_t, y, z):

        x0 = x[0] + delta_t
        x1 = - (self.first_const*self.second_const*x0 - self.first_const * np.sin(self.second_const * x0))
        dy = x1 - x[1]
        x2 = np.arctan2(dy, delta_t)
        return [x0, x1, x2]

    def find_flat_part(self, test_trajectory):

        flat_point = np.array([0, 0], dtype=float)

        for i in range(0, test_trajectory.shape[0]):
            if np.abs(test_trajectory[i, 1]-test_trajectory[i+1, 1]) > self.flat_error:
                flat_point[0] = test_trajectory[i, 0]
                flat_point[1] = test_trajectory[i, 1]

                break
        return flat_point
if __name__ == '__main__':

    trajectory_conf = np.array([0.4, 1, 2])
    conf_start = np.array([0, 0, 0])
    #trajectory = BaseTrajectory(trajectory_conf, 0.01).lane_change_base_trajectory(conf_start, False)
    #plt.plot(trajectory[:, 0], trajectory[:, 1], 'r')
    # conf_start = trajectory[-1, :3]
    # trajectory = BaseTrajectory(trajectory_conf, 0.01).lane_change_base_trajectory(conf_start, True)
    trajectory = BaseTrajectory(trajectory_conf, 0.01).merging_base_trajectory(conf_start)
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'g')
    plt.show()





