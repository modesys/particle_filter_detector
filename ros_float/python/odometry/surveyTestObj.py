# -*- coding: utf-8 -*-
# Survey object for float odometry
# This one picks images based on times and is useful for testing image scheduling
# Returns images and times.

import cv
from os import listdir
from re import match
from datetime import datetime, timedelta
from time import sleep

class newSurvey:
    
    def __init__(self, config, rig):
        self.config = config.survey
        self.performance = config.performance
        self.rig = rig
        
        self.image_period = config.odometry.burst_period
        self.next_time = datetime(1970, 1, 1, 0, 0, 0) - timedelta(0, self.image_period)
        
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

    
    def getNextFilenames(self):
        
        self.next_time += timedelta(0, self.image_period)        
        
        self.imnum += 1
        left_filestruct = (None, None, None)
        right_filestruct = (None, None, None)

        if self.left_files is None or self.right_files is None:
            print('Building filelists.')
            self.buildFilelists()

        while left_filestruct[2] is None or right_filestruct[2] is None:
            left_filestruct = self.getLeftFilestruct(self.next_time)
            right_filestruct = self.getRightFilestruct(left_filestruct)
            if left_filestruct[2] is None or right_filestruct[2] is None:
                print('Image number %d not found, rebuilding filelists.' % self.imnum)
                self.buildFilelists()
                left_filestruct = self.getLeftFilestruct(self.next_time)
                right_filestruct = self.getRightFilestruct(left_filestruct)
                if left_filestruct[2] is None or right_filestruct[2] is None:
                    print('Image number %d not found, sleeping...' % self.imnum)
                    sleep(self.performance.sleep_time)
                    self.buildFilelists()
                    left_filestruct = self.getLeftFilestruct(self.next_time)
                    right_filestruct = self.getRightFilestruct(left_filestruct)
                    if left_filestruct[2] is None or right_filestruct[2] is None:
                        print('Image number %d not found, giving up.' % self.imnum)
                        self.imnum += 1

        self.imnum = left_filestruct[0]
        im_time = left_filestruct[1] + (right_filestruct[1] - left_filestruct[1])/2
        if self.start_time is None:
            self.start_time = im_time
        survey_time = (im_time - self.start_time).total_seconds()
        
        
        if im_time - self.next_time > timedelta(1,0): # first image?
            print('first image')
            self.next_time = im_time + timedelta(0, self.image_period)
        #else:
            #print('adding %f seconds' % next_period)
            #self.next_time += timedelta(0, next_period)
        
        return (survey_time, left_filestruct[2], right_filestruct[2])
    
    def filelist_sort_key(self, fileobj):
        return fileobj[1]
    
    def buildFilelists(self):
        filelist = listdir(self.left_folder)
        self.left_files = list()
        for filename in filelist:
            f_parsed = match(self.format, filename)
            if f_parsed is not None:
                self.left_files.append((int(f_parsed.group(8)),
                                        datetime(int(f_parsed.group(1)), int(f_parsed.group(2)), int(f_parsed.group(3)), int(f_parsed.group(4)), int(f_parsed.group(5)), int(f_parsed.group(6)), int(f_parsed.group(7))),
                                        f_parsed.group(0)))
        self.left_files.sort(key=self.filelist_sort_key)
        filelist = listdir(self.right_folder)
        self.right_files = list()
        for filename in filelist:
            f_parsed = match(self.format, filename)
            if f_parsed is not None:
                self.right_files.append((int(f_parsed.group(8)),
                                         datetime(int(f_parsed.group(1)), int(f_parsed.group(2)), int(f_parsed.group(3)), int(f_parsed.group(4)), int(f_parsed.group(5)), int(f_parsed.group(6)), int(f_parsed.group(7))),
                                         f_parsed.group(0)))
        self.right_files.sort(key=self.filelist_sort_key)


    def getLeftFilestruct(self, next_time):
        if next_time is not None:
            for filestruct in self.left_files:
                #print(filestruct[1], next_time)
                if filestruct[1] >= next_time:
                    return filestruct
        return (None, None, None)
    
    
    def getRightFilestruct(self, left_filestruct):
        if left_filestruct[1] is not None:
            for filestruct in self.right_files:
                if abs((filestruct[1] - left_filestruct[1]).total_seconds()) < self.config.max_imgtime_diff:
                    return filestruct
        return (None, None, None)
        
    def loadImages(self, left_file, right_file, im1, im2):
        im1_int = cv.LoadImageM(self.left_folder + '/' + left_file, 0)
        im2_int = cv.LoadImageM(self.right_folder + '/' + right_file, 0)
        
        cv.ConvertScale(im1_int, im1, 1./255)
        cv.ConvertScale(im2_int, im2, 1./255)
        
        self.rig.stereoRectify1(im1, im1)
        self.rig.stereoRectify2(im2, im2)
        
        #cv.SaveImage('left.png', cv.fromarray(255.*np.asarray(im1)))
        #cv.SaveImage('right.png', cv.fromarray(255.*np.asarray(im2)))
