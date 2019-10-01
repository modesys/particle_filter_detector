# -*- coding: utf-8 -*-
# Rig object for float odometry
# Loads calibration, stereo rectifies, resizes, etc.

from scipy.io import loadmat
import numpy as np
import cv#, cv2


class newRig:
    
    def __init__(self, config):
        self.config = config.rig
        self.performance = config.performance
        
        cal_mat = loadmat(self.config.calib_file)
        self.size = (cal_mat.get('ny')[0,0], cal_mat.get('nx')[0,0])
        self.KK_right = cv.fromarray(cal_mat.get('KK_right').copy())
        self.KK_left = cv.fromarray(cal_mat.get('KK_left').copy())
        self.kc_right = cv.fromarray(cal_mat.get('kc_right').copy())
        self.kc_left = cv.fromarray(cal_mat.get('kc_left').copy())
        self.T = cv.fromarray(cal_mat.get('T').copy())
        self.R = cv.fromarray(cal_mat.get('R').copy())
        
        if self.performance.predownscale:
            self.size = (self.size[0]/2, self.size[1]/2)
            self.KK_left[0,0] /= 2.
            self.KK_left[1,1] /= 2.
            self.KK_left[0,2] /= 2.
            self.KK_left[1,2] /= 2.
            self.KK_right[0,0] /= 2.
            self.KK_right[1,1] /= 2.
            self.KK_right[0,2] /= 2.
            self.KK_right[1,2] /= 2.
    
        self.size_short = (self.size[0]/self.performance.short_factor, self.size[1])
        self.size_small = (self.size[0]/self.performance.small_factor, self.size[1]/self.performance.small_factor)
        
        self.R1 = cv.CreateMat(3, 3, cv.CV_64F)
        self.R2 = cv.CreateMat(3, 3, cv.CV_64F)
        self.P1 = cv.CreateMat(3, 4, cv.CV_64F)
        self.P2 = cv.CreateMat(3, 4, cv.CV_64F)
        self.Q = cv.CreateMat(4, 4, cv.CV_64F)
        
        self.rect_map1x = cv.CreateMat(self.size[0], self.size[1], cv.CV_32FC1)
        self.rect_map1y = cv.CreateMat(self.size[0], self.size[1], cv.CV_32FC1)
        self.rect_map2x = cv.CreateMat(self.size[0], self.size[1], cv.CV_32FC1)
        self.rect_map2y = cv.CreateMat(self.size[0], self.size[1], cv.CV_32FC1)
        
        cv.StereoRectify(self.KK_left, self.KK_right, self.kc_left, self.kc_right, self.size, self.R, self.T, self.R1, self.R2, self.P1, self.P2, self.Q) #, alpha=0
        cv.InitUndistortRectifyMap(self.KK_left, self.kc_left, self.R1, self.P1, self.rect_map1x, self.rect_map1y)
        cv.InitUndistortRectifyMap(self.KK_right, self.kc_right, self.R2, self.P2, self.rect_map2x, self.rect_map2y)
        
        self.rotscale_map = cv.CreateMat(2, 3, cv.CV_32FC1)
        
        self.alt = np.float32(np.zeros(self.size_short[0]))
    
    
    def stereoRectify1(self, im1, im1_r):
        cv.Remap(im1, im1_r, self.rect_map1x, self.rect_map1y, flags=cv.CV_INTER_LINEAR+cv.CV_WARP_FILL_OUTLIERS, fillval=0)
    
    def stereoRectify2(self, im2, im2_r)   : 
        cv.Remap(im2, im2_r, self.rect_map2x, self.rect_map2y, flags=cv.CV_INTER_LINEAR+cv.CV_WARP_FILL_OUTLIERS, fillval=0)
    
    def makeShort(self, im1, im1_short, im2, im2_short):
        cv.Resize(im1, im1_short, cv.CV_INTER_AREA)
        cv.Resize(im2, im2_short, cv.CV_INTER_AREA)
        
    def makeSmall(self, im, im_small):
        cv.Resize(im, im_small, cv.CV_INTER_AREA)
    
    def scaleRot(self, im_small, d_scale, d_heading):
        cv.GetRotationMatrix2D((self.P1[0,2]/self.performance.small_factor, self.P1[1,2]/self.performance.small_factor), -d_heading, d_scale, self.rotscale_map)
        #cv.GetRotationMatrix2D((self.size_small[1]/2, self.size_small[0]/2), -d_heading, d_scale, self.rotscale_map)
        #self.rotscale_map = cv.fromarray(cv2.getRotationMatrix2D((np.float32(self.size_small[1])/2,np.float32(self.size_small[0])/2), -d_heading, d_scale))
        
        #alpha = d_scale * np.cos(-d_heading * np.pi/180)
        #beta = d_scale * np.sin(-d_heading * np.pi/180)
        #self.rotscale_map[0,2] = (1-alpha)*680 - beta*512
        #self.rotscale_map[1,2] = beta*680 + (1-alpha)*512
        
        cv.WarpAffine(im_small, im_small, self.rotscale_map)
    
    def altFromDisparity(self, disparity):
        c = 0
        for i in range(self.size_short[0]):
            if np.isnan(disparity[i]) or np.isclose(disparity[i], np.float32(0.)):
                #c += 1
                #self.alt[i] = np.float32(0.)
                pass
            else:
                thisalt = -self.P2[0,3] / disparity[i]
                if thisalt < self.config.minalt or thisalt > self.config.maxalt:
                    pass
                else:
                    self.alt[c] = thisalt
                    c += 1
        return (self.alt, c)
    
    def distanceFromDisparity(self, disparity, alt):
        if disparity is not None:
            return alt * disparity / self.P1[0,0]
        else:
            return None
