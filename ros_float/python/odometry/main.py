#!/usr/bin/python

# Float odometry and stuff
#svn version

import configObj, surveyObj, rigObj, corrObj, overlapObj, odometryObj
import numpy as np
import cv
#import subprocess
import lcmThread
#import time

config = configObj.load()
rig = rigObj.newRig(config)
survey = surveyObj.newSurvey(config, rig)
corr = corrObj.newCorr(config, rig)
overlap = overlapObj.newOverlap(config, rig)
odometry = odometryObj.newOdometry(config)

cam = lcmThread.lcmThread(config)

# log file
log_fid = open('/home/administrator/odometry2/log.txt', 'a')

# images
im1 = cv.CreateMat(rig.size[0], rig.size[1], cv.CV_32FC1)
im2 = cv.CreateMat(rig.size[0], rig.size[1], cv.CV_32FC1)
# short images (for altitude)
im1_short = cv.CreateMat(rig.size_short[0], rig.size_short[1], cv.CV_32FC1)
im2_short = cv.CreateMat(rig.size_short[0], rig.size_short[1], cv.CV_32FC1)
# small image (for matching)
im1_small = cv.CreateMat(rig.size_small[0], rig.size_small[1], cv.CV_32FC1)

#
alt = None
last_alt = None
last_std_alt = None
im_fft = None
last_im_fft = None
im_lp_fft = None
last_im_lp_fft = None
last_time = 0.


survey_time = 0.
bursting = False
burstend = 0.
nextburst = 0.
period = config.odometry.burst_period

image_period = period

cam.start()

while 1:
    try:
        distance = None
        (survey_time, left_file, right_file) = survey.getNextFilenames()
        
        survey.loadImages(left_file, right_file, im1, im2)
        
        # altitude
        rig.makeShort(im1, im1_short, im2, im2_short)
        disparity = corr.stereoCorr(im1_short, im2_short)
    
        # do altitude filtering stuff
        (alt, numalts) = rig.altFromDisparity(disparity)
        
        mean_alt = np.mean(alt[0:numalts])
        std_alt = np.std(alt[0:numalts])
        #print(mean_alt, std_alt, numalts)
        c = 0
        for i in range(numalts):
            if alt[i] < mean_alt + 3*std_alt and alt[i] > mean_alt - 3*std_alt:
                alt[c] = alt[i]
                c += 1
        numalts = c
        mean_alt = np.mean(alt[0:numalts])
        std_alt = np.std(alt[0:numalts])
        #print(mean_alt, std_alt, numalts)
        
        alt = mean_alt if numalts > .2*rig.size_short[0] else None
        if mean_alt < config.rig.minalt or mean_alt > config.rig.maxalt:
            mean_alt = None
        
        #print(alt, last_alt)
        
        # end altitude filtering stuff
        
        # caching stuff, see function description.
        rig.makeSmall(im1, im1_small)
        (im_fft, im_lp_fft) = corr.ffts(im1_small)
        
        dt = survey_time - last_time

        #print("dt %f" % dt)
        
        if dt > .9*config.odometry.burst_period and dt < 1.1*config.odometry.burst_period: # we're bursting!
            
            
            if alt is not None and last_alt is not None:
                # motion
                #this_im1_small = cv.CloneMat(im1_small)
                
                # last pair was good, do it
                if last_im_fft is not None and last_im_lp_fft is not None:
                    (d_scale, d_heading, peakcorr_rotscale) = corr.rotScale(last_im_lp_fft, im_lp_fft)
                    #print(d_scale, d_heading)
                    if d_scale is not None and d_heading is not None:
                        rig.scaleRot(im1_small, d_scale, d_heading)
                        (disparity, x_disp, y_disp, peakcorr_motion) = corr.motionCorr(last_im_fft, im1_small, d_scale)
                        if disparity is not None:
                            distance = rig.distanceFromDisparity(disparity, last_alt)
                            std_dist = rig.distanceFromDisparity(disparity, last_std_alt)
            #dt = survey_time - last_time
            if distance is None:
                speed_raw = 0
                (dist, speed, sigma_dist, sigma_speed) = odometry.predict(survey_time)
            else:
                if dt > 0.:
                    speed_raw = distance / dt
                else:
                    speed_raw = 0.
                ds = distance
                if ds < 5.0:
                    ds = 5.0
                if ds > 5000.0:
                    ds = 5000.0
                sigma_ds = std_dist
                
                (dist, speed, sigma_dist, sigma_speed) = odometry.update(survey_time, ds, sigma_ds)
                #print(survey_time, dist, speed, sigma_dist, sigma_speed)
                if config.analysis.compute_overlap:
                    pctoverlap = overlap.pctOverlap(x_disp, y_disp, d_heading, d_scale)
                else:
                    pctoverlap = 0.
                #print(survey_time, speed, speed_filt)
                #print(survey_time, dt, alt, distance, std_dist, peakcorr_rotscale, peakcorr_motion, pctoverlap)
            #print(survey_time, speed_raw, speed, sigma_speed)
            if distance is None:
                distance = 0.
            #print('%d %f %f %f %f %f' % (survey.imnum, survey_time, dt, speed_raw, speed, distance))
            print('%d %f %f %f %f' % (survey.imnum, survey_time, dt, alt, distance))
            log_fid.write('%d %f %f %f %f\n' % (survey.imnum, survey_time, dt, alt, distance))
    
           # last_time = survey_time
           # last_alt = alt
           ## last_std_alt = std_alt
           # last_im_fft = im_fft
           # last_im_lp_fft = im_lp_fft
        #time1 = time.time()
        #print(time1 - time0)

    
        # frame rate stuff
            (dist, speed, sigma_dist, sigma_speed) = odometry.predict(survey_time)
            period = config.odometry.image_spacing / (speed / 1000)
        
            cam.setPeriod(period)      
        #    print('Old period: %f, new period: %f' % (image_period, period))
            image_period = period
        #subprocess.call(['../cam-driverV2/bin/cam-attributes', '-m', '-s',  '--FrameRate', str(1./image_period)])
        last_time = survey_time
        last_alt = alt
        last_std_alt = std_alt
        last_im_fft = im_fft
        last_im_lp_fft = im_lp_fft
    except:
        pass
