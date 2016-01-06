#!/usr/bin/env python2
from __future__ import division
import numpy as np
import math as math
import matplotlib.pyplot as plt



class BaseTrajectory:

    def __init__(self, lateral_length, velocity, lateralAcceleration):
        self.lateral = lateral_length
        self.longitudinal = velocity * (math.sqrt((math.pi * self.lateral)/lateralAcceleration))
        print self.longitudinal

    def laneChangeTrajectory(self):

        x = np.linspace(0, self.longitudinal)
        y = self.lateral / 2 * np.pi
        print y
        z = (2 * np.pi) / self.longitudinal
        print z
        function = y*z*x - y * np.sin(z * x)
        plt.plot(x, function)
        plt.show()
    def mergeTarajectory(self):

        x = np.linspace(0, self.longitudinal)
        y = self.lateral / 2 * np.pi
        print y
        z = (2 * np.pi) / self.longitudinal
        print z
        function = self.lateral - (y*z*x - y * np.sin(z * x))
        plt.plot(x, function)
        plt.show()


if __name__ == '__main__':

    BaseTrajectory(0.4, 3, 2).laneChangeTrajectory()
    BaseTrajectory(0.4, 3, 2).mergeTarajectory()


