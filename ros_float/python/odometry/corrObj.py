# -*- coding: utf-8 -*-
# Correlation object for float odometry
# Performs assorted correlations and related math

import cv, numpy as np, fftw3f as fftw
import subpxObj
#import scipy.ndimage.filters

class newCorr:    
    
    def __init__(self, config, rig):
        self.config = config.corr
        self.performance = config.performance
        self.rig = rig
        self.subpx = subpxObj.newSubPx()
        
        self.fft_size = 2
        while self.fft_size < self.rig.size_small[0] or self.fft_size < self.rig.size_small[1]:
            self.fft_size *= 2
        # mask for short images (altitude)
        a = np.hanning(self.rig.size_short[1])
        self.mask_stereo = np.float32(np.repeat([a], self.rig.size_short[0], axis=0))
        self.disparity = np.float32(np.zeros(self.rig.size_short[0]))
        
        # mask for small images (registration)
        a = np.hanning(self.rig.size_small[1])
        b = np.hanning(self.rig.size_small[0])
        #a = np.hanning(self.fft_size)
        #b = np.hanning(self.fft_size)
        #a = a[(self.fft_size-self.rig.size_small[1])/2:(self.fft_size+self.rig.size_small[1])/2]
        #b = b[(self.fft_size-self.rig.size_small[0])/2:(self.fft_size+self.rig.size_small[0])/2]
        self.mask_motion = np.float32(np.dot(np.transpose([b]), [a]))
        
        # high pass filter for fft
        a = np.linspace(-0.5, 0.5, self.fft_size+1)
        a = a[0:np.size(a)-1]
        Xetanu = np.dot(np.transpose([np.cos(np.pi * a)]), [np.cos(np.pi * a)])
        self.high_pass = np.float32((1 - Xetanu) * (2 - Xetanu))
        middle = self.high_pass < 1
        self.high_pass[middle] = np.sqrt(self.high_pass[middle])
        
        # log polar transform
        self.log_base = 10
        self.rho = np.logspace(np.log(1)/np.log(self.log_base), np.log(self.fft_size/2)/np.log(self.log_base), num=self.fft_size, base=self.log_base)
        c = (self.fft_size/2, self.fft_size/2)
        theta = np.linspace(0, np.pi, num=self.fft_size+1)
        self.theta = theta[0:np.size(theta)-1]
        x = np.dot(np.transpose([np.cos(self.theta)]), [self.rho]) + c[1]
        y = np.dot(np.transpose([np.sin(self.theta)]), [self.rho]) + c[0]
        self.logpolar_mapx = cv.fromarray(np.float32(x))
        self.logpolar_mapy = cv.fromarray(np.float32(y))
        
        # mask for log polar
        self.mask_lp = np.ones(x.shape)
        #a = np.hanning(self.fft_size)
        #self.mask_lp = np.float32(np.repeat([a], self.fft_size, axis=0))
        
        # memory allocation
        self.im_lp = cv.CreateMat(self.fft_size, self.fft_size, cv.CV_32FC1)
        
        # oh boy it's fftw time
        self.fftw_in = fftw.create_aligned_array((self.fft_size,self.fft_size), dtype='complex64')
        self.fftw_out = fftw.create_aligned_array((self.fft_size,self.fft_size), dtype='complex64')
        self.fftw_plan = fftw.planning.Plan(self.fftw_in, self.fftw_out, direction='forward', nthreads=self.performance.nthreads, flags=('measure','destroy input'))
        
        self.ifftw_in = fftw.create_aligned_array((self.fft_size,self.fft_size), dtype='complex64')
        self.ifftw_out = fftw.create_aligned_array((self.fft_size,self.fft_size), dtype='complex64')
        self.ifftw_plan = fftw.planning.Plan(self.ifftw_in, self.ifftw_out, direction='backward', nthreads=self.performance.nthreads, flags=('measure', 'destroy input'))
    
    def stereoCorr(self, im1_short, im2_short):
        im1_arr = np.asarray(im1_short) * self.mask_stereo
        im2_arr = np.asarray(im2_short) * self.mask_stereo
        
        im1_fft = np.fft.fft(im1_arr, axis=1)
        im2_fft = np.fft.fft(im2_arr, axis=1)
        
        fftr = im1_fft * np.conj(im2_fft)
        fftr_abs = np.abs(fftr)
        fftr_invalid = np.isnan(fftr_abs) + np.isinf(fftr_abs)
        fftr_abs[fftr_invalid] = np.finfo('float').max
        fftr_abs[np.where(fftr_abs==0)] = np.finfo('float').max
        fftr = fftr / fftr_abs
        corr = np.real(np.fft.ifft(fftr, axis=1))
        
        for i in range(self.rig.size_short[0]):
            self.disparity[i] = self.findPeak1D(corr[i,:], self.config.mincorr_stereo)
        return self.disparity
    
    def ffts(self, im_small):
        im_arr = np.asarray(im_small)
        im_arr = im_arr - im_arr.mean()
        im_arr = im_arr * self.mask_motion
        im_fft = np.fft.fftshift(np.fft.fft2(im_arr, (self.fft_size, self.fft_size))) # save for matching
        
        self.fftw_in[:] = 0.
        self.fftw_in[0:im_small.rows,0:im_small.cols] += im_arr
        self.fftw_plan.execute()
        im_fft = np.zeros(self.fftw_out.shape, self.fftw_out.dtype)
        im_fft += self.fftw_out
        im_fft = np.fft.fftshift(im_fft)
        
        im_fft_mag = np.abs(im_fft * self.high_pass)
        
        # log polar
        im_fft_im = cv.fromarray(np.float32(im_fft_mag))
        cv.Remap(im_fft_im, self.im_lp, self.logpolar_mapx, self.logpolar_mapy, cv.CV_INTER_LINEAR)
        im_lp = np.asarray(self.im_lp) * self.mask_lp
        #cv.SaveImage(filename, cv.fromarray(im_lp))
        im_lp = im_lp - np.mean(im_lp)        
        im_lp_fft = np.fft.fft2(im_lp, (self.fft_size, self.fft_size)) # save for matching
        
        self.fftw_in[:] = 0.
        self.fftw_in += im_lp
        self.fftw_plan.execute()
        im_lp_fft = np.zeros(self.fftw_out.shape, self.fftw_out.dtype)
        im_lp_fft += self.fftw_out
        
        return (im_fft, im_lp_fft)
    
    def motionCorr(self, last_im_fft, im_small, d_scale):
        im_arr = np.asarray(im_small)
        im_arr = im_arr - im_arr.mean()
        im_arr = im_arr * self.mask_motion
        
        im_fft = np.fft.fftshift(np.fft.fft2(im_arr, (self.fft_size, self.fft_size)))
        fftr = last_im_fft * np.conj(im_fft)
        fftr_abs = np.abs(fftr)
        fftr_invalid = np.isnan(fftr_abs) + np.isinf(fftr_abs)
        fftr_abs[fftr_invalid] = np.finfo('float').max
        fftr_abs[np.where(fftr_abs==0)] = np.finfo('float').max
        fftr = fftr / fftr_abs

        self.ifftw_in[:] = 0.
        self.ifftw_in += np.fft.ifftshift(fftr)
        self.ifftw_plan.execute()
        corr = np.zeros(self.ifftw_out.shape, 'float32')
        corr += np.real(self.ifftw_out) 
        
        (x,y,peakcorr) = self.findPeak2D(corr, self.config.mincorr_motion)
        if x is not None and y is not None:
            if x > self.fft_size/2:
                x = x - self.fft_size
            if y > self.fft_size/2:
                y = y - self.fft_size
            x = self.performance.small_factor * x #/ d_scale
            y = self.performance.small_factor * y #/ d_scale
            disp = np.sqrt(np.power(x,2) + np.power(y,2))
            return (disp, x, y, peakcorr)
        else:
            return (None, None, None, 0.)

            
    def rotScale(self, last_im_lp_fft, im_lp_fft):  
        fftr = last_im_lp_fft * np.conj(im_lp_fft)
        fftr_abs = np.abs(fftr)
        fftr_invalid = np.isnan(fftr_abs) + np.isinf(fftr_abs)
        fftr_abs[fftr_invalid] = np.finfo('float').max
        fftr_abs[np.where(fftr_abs==0)] = np.finfo('float').max
        fftr = fftr / fftr_abs
        
        self.ifftw_in[:] = 0.
        self.ifftw_in += fftr
        self.ifftw_plan.execute()
        corr = np.zeros(self.ifftw_out.shape, 'float32')
        corr += np.real(self.ifftw_out)
        
        #corr = np.real(np.fft.ifft2(fftr))
        
        (x,y,peakcorr) = self.findPeak2D(corr, self.config.mincorr_rotscale) # y angle, x scale
        if x is not None and y is not None:
            d_scale = self.getScale(np.float(x))
            d_rot = self.getRot(np.float(y))
        else:
            d_scale = None
            d_rot = None
     
        
        return (d_scale, d_rot, peakcorr)
    

    def findPeak1D(self, corr, mincorr):
        peak = np.argmax(corr)
        peakval = corr.max()
        xmax = corr.shape[0].__int__() - 1
        patch = np.asarray([corr[np.mod(peak-1,xmax)], corr[peak], corr[np.mod(peak+1,xmax)]])

        if np.count_nonzero(patch) > 0:
            (dx, peakval) = self.subpx.max1d(patch)
            peak = np.float(peak) + dx
        if peakval < mincorr:
            peak = None
        return peak
    
    def findPeak2D(self, corr, mincorr):
        #print(corr.max())
        peak = np.argmax(corr)
        (y, x) = np.unravel_index(peak, (self.fft_size, self.fft_size))
        
        ymax = corr.shape[0].__int__() - 1
        xmax = corr.shape[1].__int__() - 1
        
        patch = np.asarray([[corr[np.mod(y-1,ymax),np.mod(x-1,xmax)], corr[np.mod(y-1,ymax),x], corr[np.mod(y-1,ymax),np.mod(x+1,xmax)]],
                            [corr[y,np.mod(x-1,xmax)], corr[y,x], corr[y,np.mod(x+1,xmax)]],
                            [corr[np.mod(y+1,ymax),np.mod(x-1,xmax)], corr[np.mod(y+1,ymax),x], corr[np.mod(y+1,ymax),np.mod(x+1,xmax)]]])

        (dx, dy, peakval) = self.subpx.max2d(patch)
        x += dx
        y += dy
        
        if peakval > mincorr:
            return (x, y, peakval)
        else:
            return (None, None, 0.)
    
    def getScale(self, x):
        if x > self.fft_size/2:
            d_scale = np.power(self.log_base, (self.fft_size-x) / self.fft_size * np.log(self.fft_size/2)/np.log(self.log_base))
        else:
            d_scale = 1 / np.power(self.log_base, x / self.fft_size * np.log(self.fft_size/2)/np.log(self.log_base))
        return d_scale
    
    def getRot(self, y):
        if y > self.fft_size/2:
            d_rot = (y - self.fft_size) / (self.fft_size) * 180.
            #(y - 0/self.performance.small_factor - self.fft_size) / (self.fft_size - 1) * 180.
        else:
            d_rot = (y ) / (self.fft_size) * 180.
        return d_rot
