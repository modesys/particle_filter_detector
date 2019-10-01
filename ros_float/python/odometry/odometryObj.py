# -*- coding: utf-8 -*-
# Odometry object for float odometry
# Filters speed, calculates framerate, etc.

import numpy as np

class newOdometry:
    def __init__(self, config):
        self.config = config.odometry
        
        # Kalman filter setup
        self.s = 0
        self.x = np.array([[self.s], [100.]])      # position, speed (mm and mm/s)
        self.P = np.array([[1.0, 1.0e4], [1.0e4, 1.0e8]]) # initial covariance, position known velocity unknown
        self.H = np.array([[1., 0.]])           # measurement matrix, only measurements of position
        self.sigma = 5. # process noise?
        self.Rcoef = 100. # measurement noise
        self.I = np.eye(2)
        self.last_t = 0
        

    def predict(self, t):
        dt = t - self.last_t
        F = np.array([[1., dt], [0., 1.]])
        G = np.array([[0.5*np.power(dt, 1.3333)], [np.power(dt, 0.6667)]])
        x = np.dot(F, self.x)
        Q = self.sigma * np.dot(G, G.T)
        P = np.dot(np.dot(F, self.P), F.T) + Q
        return(x[0,0], x[1,0], np.sqrt(P[0,0]), np.sqrt(P[1,1])) # position, speed, pos stdev, speed stdev

    def update(self, t, ds, sigma_ds):
        dt = t - self.last_t
        R = self.Rcoef * np.power(sigma_ds, 2) # measurement noise
        F = np.array([[1., dt], [0., 1.]])
        G = np.array([[0.5*np.power(dt, 1.3333)], [np.power(dt, 0.6667)]])
        xbar = np.dot(F, self.x)
        Q = self.sigma * np.dot(G, G.T)
        Pbar = np.dot(np.dot(F, self.P), F.T) + Q
        z = self.x[0,0] + ds
        y = z - np.dot(self.H, xbar)
        S = np.dot(np.dot(self.H, Pbar), self.H.T) + R
        K = np.dot(np.dot(Pbar, self.H.T), np.linalg.inv(S))
        self.x = xbar + np.dot(K, y)
        self.P = np.dot((self.I - np.dot(K, self.H)), Pbar)
        self.last_t = t
        return(self.x[0,0], self.x[1,0], np.sqrt(self.P[0,0]), np.sqrt(self.P[1,1])) # position, speed, pos stdev, speed stdev
