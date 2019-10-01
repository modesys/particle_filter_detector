# -*- coding: utf-8 -*-
# Subpixel interpolation for float odometry

import numpy as np

class newSubPx:
    
    def __init__(self):
        # coefficients for solving 1d quadratic of the form ax^2 + bx + c
        self.A1 = np.array([[1,-1, 1],
                            [0, 0, 1],
                            [1, 1, 1]])
        self.A1inv = np.linalg.pinv(self.A1)
        
        # coefficients for solving 2d quadratic of the form ax^2 + by^2 + cxy + dx + ey + f
        # I checked like 4 times its totally right
        self.A2 = np.array([[1, 1, 1,-1,-1, 1],
                            [0, 1, 0, 0,-1, 1],
                            [1, 1,-1, 1,-1, 1],
                            [1, 0, 0,-1, 0, 1],
                            [0, 0, 0, 0, 0, 1],
                            [1, 0, 0, 1, 0, 1],
                            [1, 1,-1,-1, 1, 1],
                            [0, 1, 0, 0, 1, 1],
                            [1, 1, 1, 1, 1, 1]])
        self.A2inv = np.linalg.pinv(self.A2)

    def max1d(self, patch):
        # solve for coefficients
        a,b,c = np.dot(self.A1inv, patch.reshape([3,1]))[:,0]
        # maximum is where derivative = 0
        x = -b / (2*a)
        # evaluate maximum
        return x, a*x*x + b*x + c
        
    def max2d(self, patch):
        a,b,c,d,e,f = np.dot(self.A2inv, patch.reshape([9,1]))[:,0]
        # derivatives for 2d result in a system of equations, solve that...
        x,y = np.dot(np.linalg.inv(np.array([[2*a,c],[c,2*b]])), np.array([[-d],[e]]))[:,0]
        return x, y, a*x*x + b*y*y + c*x*y + d*x + e*y + f
