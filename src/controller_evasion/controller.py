#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import numpy as np
from itertools import starmap

import matplotlib.pyplot as plt
from funcy import partial
from scipy import interpolate
import bezierCurve as bz


from common import ParticleFilter
from controller_evasion import basetrajectory
from motion_model import DataDrivenModel
from motion_model import kinematic
#from mpc import MPC
#from car import Car


class Controller(ParticleFilter):
    def __init__(self, NewModel, merging, num_particles=5):

        self.model = DataDrivenModel()
        #self.mpc = MPC_PF()
        #self.car = Car()

        x_dim = 2
        self.propagation_noise_var = [0.1, 0.1]
        self.propagation_noise = partial(lambda var, size:
                                         np.random.multivariate_normal([0] * x_dim, var * np.identity(x_dim), size),
                                         self.propagation_noise_var)

        self.observation_noise = lambda size: 0

        # TODO max velocity
        # TODO min steer for lane-change and merging
        # steer and speed
        if merging:
            # TODO set the value
            self.bounds = np.array([[-1, 1], [1, 1.5]])
        else:
            self.bounds = np.array([[0, 1], [1, 3]])
            #self.bounds = np.array([[-0.3, 0], [1, 1.5]])


        # initialize particle filter
        super(Controller, self).__init__(num_particles, x_dim, self.populate_fun,
                                         self.particle_propagation_fun, self.likelihood_fun,
                                         self.propagation_noise, self.observation_noise)

    def control(self, trajectory, model_conf, dt, steps, num_iterations=20):

        #self.lateral_offset = lateral_offset
        #self.long_offset = long_offset
        self.dt = dt
        self.steps = steps
        self.traj = trajectory

        self.solve_args = model_conf

        # it may be useful to NOT reinitialize all particles, if you want to follow a trajectory
        # because the last particle distribution is a good guess for the next
        self.initialize_particles()

        self.update_weights()

        for i in xrange(num_iterations):
            self.resample()
            self.propagate()
            self.update_weights()

        # get estimated values from particle distribution

        steer, speed = self.weighted_parameters()

        real_solution = self._solve(steer, speed)
        plt.plot(real_solution[:, 0], real_solution[:, 1], 'bo')

        solution = self._resample_solution(real_solution)
        #plt.plot(solution[:, 0], solution[:, 1], 'ro')

        print "final position x: ", solution[-1, 0], "final position y: ", solution[-1, 1]
        size = solution.shape[0]
        x_0 = solution[int(size/2), 0]
        y_0 = solution[int(size/2), 1]
        x_1 = solution[int(size/2)+1, 0]
        y_1 = solution[int(size/2)+1, 1]
        dx = x_1-x_0
        dy = y_1-y_0
        alpha = np.arctan2(dy, dx)
        v_x = speed * np.cos(alpha)
        time = solution[-1, 0] / v_x
        print "speed", speed, "steer", steer, "time", time

        return steer, speed, time

    def weighted_parameters(self):

        best = np.sum((self.particles.T * self.weights).T, axis=0)
        return best

    def _resample_solution(self, solution):

        interp_y = interpolate.interp1d(solution[:, 0], solution[:, 1], bounds_error=False)
        #interp_orientation = interpolate.interp1d(solution[:, 0], solution[:, 2], bounds_error=False)
        resampled = np.zeros_like(self.traj)

        for k, x in enumerate(self.traj[:, 0]):
            if x <= np.max(solution[:, 0]) and x >= np.min(solution[:, 0]):
                resampled[k, 0] = x
                resampled[k, 1] = interp_y(x)
                #resampled[k, 2] = interp_orientation(x)

        return resampled

    def populate_fun(self, acc):
        particle = [np.random.uniform(self.bounds[0, 0], self.bounds[0, 1], 1),
                    np.random.uniform(self.bounds[1, 0], self.bounds[1, 1], 1)]
        return particle, acc

    def particle_propagation_fun(self, particles):
        # select a few particles and teleport them to random positions
        # helpful if you get stuck in local minima
        stray = np.random.uniform(0, 1, particles.shape[0]) < 0.01
        n_stray = sum(stray)
        for i in xrange(2):
            particles[stray, i] = np.random.uniform(self.bounds[i, 0], self.bounds[i, 1], n_stray)

        # clip particles that are outside of parameter search space
        particles = np.clip(particles, self.bounds[:, 0], self.bounds[:, 1])

        return particles

    def propagate(self):
        self.particles = self.particle_propagation_fun(self.particles)
        self.particles += self.propagation_noise(self.num_particles)
        self.particles = np.clip(self.particles, self.bounds[:, 0], self.bounds[:, 1])

    def likelihood_fun(self, particles):
        errors = list(starmap(self._evaluate, particles))
        weights = -np.log(errors)

        return weights

    def _solve(self, steer, speed):

        solution = kinematic(self.steps,self.dt,0,0,0,speed,steer)
        self.solve_args[3] = speed
        # solution = self.model.solve(
        #     self.steps,
        #     self.dt,
        #     steer,
        #     speed,
        #     *self.solve_args)


        plt.plot(solution[:, 0], solution[:, 1], 'ro')

        return solution

    def calc_distance(self, xy):

        dxy = np.diff(xy, axis=0)
        ds = np.sum(dxy ** 2, axis=1) ** 0.5


        return np.sum(ds)

    def _evaluate(self, steer, speed):

        solution = self._solve(steer, speed)
        solution = self._resample_solution(solution)

        self.size = solution.shape[0]
        y_no_zero = np.trim_zeros(solution[:, 1])
        size = int(0.10 * y_no_zero.shape[0])
        lateral_distance = np.abs(np.mean(y_no_zero[-size:]))
        x_no_zero = np.trim_zeros(solution[:, 0])
        size = int(0.10 * x_no_zero.shape[0])
        long_distance = np.mean(x_no_zero[-size:])

        diff = self.traj - solution
        distances_squared = (np.square(diff[:, 0]) + np.square(diff[:, 1]))

        #upper_bound_y = (self.lateral_offset) - 0.1

        #lower_bound_y = 0.15
        #upper_bound_x = self.long_offset + 0.1

        #if lateral_distance < upper_bound_y and lateral_distance > lower_bound_y \
               # and long_distance < upper_bound_x:
            #error = np.sqrt(distances_squared).mean()
        #else:
        error = np.sqrt(distances_squared).mean()

        return error


if __name__ == '__main__':

    controller = Controller(DataDrivenModel, False)
    trajectory_conf = np.array([0.5, 1.5, 2])
    conf_start = np.array([0, 0, 0])
    #Model.solve(x_start = np.array([0,0,0,0,0,(start Gescheindigkeit in m/s),0]), u = (Servo im Bogenmass, Motordrehmoment), n_steps = 9000, dt = 0.001)
    conf_model = np.array([0, 0, 2, 0.8, 0, 0], dtype="float")
    vp = [bz.pt(0, 0), bz.pt(0.6, 0), bz.pt(1, 1)]
    traj = bz.demo(vp)
    #x, y, orientation, velocity, longitual_velocity, yaw_rate
    #self.solve_args[3] = speed
    steer = -0.23
    speed = 100
    step = 100
    dt = 0.01
    #controller.car.sensors.perception_trajectory.trajectory_arr = traj
    plt.show()

    #(steps, dt, control_steer, control_speed, x, y, orientation, velocity, longitual_velocity, yaw_rate)

    #long_offset = basetrajectory.BaseTrajectory(trajectory_conf, 0.01).longitudinal_length / 2
    #trajectory = basetrajectory.BaseTrajectory(trajectory_conf, 0.01).lane_change_base_trajectory(conf_start, False)
    #controller = Controller(NewModel,True)
    #trajectory = basetrajectory.BaseTrajectory(trajectory_conf, 0.01).merging_base_trajectory(conf_start)
    #steer, speed, time = controller.control(traj, conf_model, 0.01, 100)
    #plt.plot(traj[:, 0], traj[:, 1], 'g')
    #plt.show()
