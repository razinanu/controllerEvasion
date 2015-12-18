#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt


class Base_Trajectory():



    def __init__(self, obstacle):
        self.obstacle = obstacle
        self.offSet = 2

        '''
      calculate the Coefficients
        '''
    def findParameter(self):

        xNames = ['a', 'b']
        A = np.zeros((2, 2), dtype=np.float64)
        b = np.zeros((2, ), dtype=np.float64)
        secondPoint = np.array([self.obstacle[0], self.obstacle[1] + self.offSet], dtype=np.float64)
        A[0, 0] = secondPoint[1]*secondPoint[1]*secondPoint[1]
        A[1, 0] = 6*secondPoint[1]
        A[0, 1] = secondPoint[1]*secondPoint[1]
        A[1, 1] = 2


        b[0] = secondPoint[0]

        x = np.linalg.solve(A, b)

        for xName, xValue in zip(xNames, x):
          print('%4s = % f' % (xName, xValue))
        z = np.linspace(1, 5)
        var = x[0]*z*z*z+x[1]*z*z
        plt.plot(z, var)
        plt.show()




if __name__ == '__main__':
    obstacle = (1, 2)
    Base_Trajectory(obstacle).findParameter()

