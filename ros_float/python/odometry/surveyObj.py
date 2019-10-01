# -*- coding: utf-8 -*-
# Survey object for float odometry
#   Returns images and times.

import cv
from os import listdir
from re import match
from datetime import datetime
from time import sleep
#import numpy as np

class newSurvey:
    
    def __init__(self, config, rig):
        self.config = config.survey
        self.performance = config.performance
        self.rig = rig
        
        if self.config.is_antarctic:
            self.left_folder = self.config.survey_folder + '/a'
            self.left_prefix = 'aIMG'
            self.right_folder = self.config.survey_folder + '/b'
            self.right_prefix = 'bIMG'
        else:
            self.left_folder = self.config.survey_folder + ('/bw' if self.rig.config.bw_is_left else '/color')
            self.left_prefix = 'IMG' if self.rig.config.bw_is_left else 'cIMG'
            self.right_folder = self.config.survey_folder + ('/color' if self.rig.config.bw_is_left else '/bw')
            self.right_prefix = 'cIMG' if self.rig.config.bw_is_left else 'IMG'

        self.left_files = None
        self.right_files = None        
        
        self.format = '[abc]?IMG-([0-9]{4})([0-9]{2})([0-9]{2})-([0-9]{2})([0-9]{2})([0-9]{2})-([0-9]{6})-([0-9]+)\.tif'
        
        self.imnum = self.config.imnum_start - 1
        self.start_time = None
        
        if self.performance.predownscale:
            self.im1_half = cv.CreateMat(self.rig.size[0], self.rig.size[1], 0)
            self.im2_half = cv.CreateMat(self.rig.size[0], self.rig.size[1], 0)

    
    def getNextFilenames(self):
        
        self.imnum += 1
        left_filestruct = (None, None, None)
        right_filestruct = (None, None, None)

        if self.left_files is None or self.right_files is None:
        #    print('Building filelists.')
            self.buildFilelists()

        left_filestruct = self.getLeftFilestruct(self.imnum)
        right_filestruct = self.getRightFilestruct(left_filestruct)
        if left_filestruct[2] is None or right_filestruct[2] is None:
            print('Image number %d not found, rebuilding filelists.' % self.imnum)
            self.buildFilelists()
            left_filestruct = self.getLeftFilestruct(self.imnum)
            right_filestruct = self.getRightFilestruct(left_filestruct)
            while left_filestruct[2] is None or right_filestruct[2] is None:
                print('Image number %d not found, sleeping...' % self.imnum)
                sleep(0.5)
                self.buildFilelists()
                left_filestruct = self.getLeftFilestruct(self.imnum)
                right_filestruct = self.getRightFilestruct(left_filestruct)

        im_time = left_filestruct[1] + (right_filestruct[1] - left_filestruct[1])/2
        if self.start_time is None:
            self.start_time = im_time
        survey_time = (im_time - self.start_time).total_seconds()
        
        return (survey_time, left_filestruct[2], right_filestruct[2])
        
    def buildFilelists(self):
        filelist = listdir(self.left_folder)
        self.left_files = list()
        for filename in filelist:
            f_parsed = match(self.format, filename)
            if f_parsed is not None:
                #filenum = int(f_parsed.group(8))
                #filetime = datetime(int(f_parsed.group(1)), int(f_parsed.group(2)), int(f_parsed.group(3)), int(f_parsed.group(4)), int(f_parsed.group(5)), int(f_parsed.group(6)), int(f_parsed.group(7)))
                #filename = f_parsed.group(0)
                self.left_files.append((int(f_parsed.group(8)),
                                        datetime(int(f_parsed.group(1)), int(f_parsed.group(2)), int(f_parsed.group(3)), int(f_parsed.group(4)), int(f_parsed.group(5)), int(f_parsed.group(6)), int(f_parsed.group(7))),
                                        f_parsed.group(0)))
        filelist = listdir(self.right_folder)
        self.right_files = list()
        for filename in filelist:
            f_parsed = match(self.format, filename)
            if f_parsed is not None:
                self.right_files.append((int(f_parsed.group(8)),
                                         datetime(int(f_parsed.group(1)), int(f_parsed.group(2)), int(f_parsed.group(3)), int(f_parsed.group(4)), int(f_parsed.group(5)), int(f_parsed.group(6)), int(f_parsed.group(7))),
                                         f_parsed.group(0)))


    def getLeftFilestruct(self, imnum):
        if imnum is not None:
            for filestruct in self.left_files:
                if filestruct[0] == self.imnum:
                    return filestruct
        return (None, None, None)
    
    
    def getRightFilestruct(self, left_filestruct):
        if left_filestruct[1] is not None:
            for filestruct in self.right_files:
                if abs((filestruct[1] - left_filestruct[1]).total_seconds()) < self.config.max_imgtime_diff:
                    return filestruct
        return (None, None, None)
        
    def loadImages(self, left_file, right_file, im1, im2):
        images_loaded = False
        while not images_loaded:
            try:
                im1_int = cv.LoadImageM(self.left_folder + '/' + left_file, 0)
                im2_int = cv.LoadImageM(self.right_folder + '/' + right_file, 0)
                images_loaded = True
            except:
                sleep(0.5)

        if self.performance.predownscale:
            cv.Resize(im1_int, self.im1_half, cv.CV_INTER_AREA)
            cv.Resize(im2_int, self.im2_half, cv.CV_INTER_AREA)
            cv.ConvertScale(self.im1_half, im1, 1./255)
            cv.ConvertScale(self.im2_half, im2, 1./255)
        else:
            cv.ConvertScale(im1_int, im1, 1./255)
            cv.ConvertScale(im2_int, im2, 1./255)

        self.rig.stereoRectify1(im1, im1)
        self.rig.stereoRectify2(im2, im2)
        
        #cv.SaveImage('left.png', cv.fromarray(255.*np.asarray(im1)))
        #cv.SaveImage('right.png', cv.fromarray(255.*np.asarray(im2)))
