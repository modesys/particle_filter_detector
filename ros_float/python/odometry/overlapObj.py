# -*- coding: utf-8 -*-
# Overlap object for float odometry
#   Computes percent overlap

#import cv, numpy as np
#import scipy.ndimage.filters

import numpy as np
import cv
import shapely.geometry as shape

class newOverlap:
    
    def __init__(self, config, rig):
        self.config = config.analysis
        self.rig = rig
        
        x1 = np.linspace(0, rig.size[1], config.analysis.outline_ptsperside).tolist()
        y1 = np.linspace(0, rig.size[0], config.analysis.outline_ptsperside).tolist()
        
        points = []
        for x in x1:
            points.append([[x, 0.]])
        for y in y1:
            points.append([[rig.size[1], y]])
        x1.reverse()
        y1.reverse()
        for x in x1:
            points.append([[x, rig.size[0]]])
        for y in y1:
            points.append([[0., y]])
        
        self.points_raw = np.asarray(points)
        points_rect = cv.fromarray(self.points_raw.copy())
        
        if self.rig.config.match_left:
            cv.UndistortPoints(cv.fromarray(self.points_raw), points_rect, rig.KK_left, rig.kc_left, rig.R1, rig.P1)
            self.points_rect = (np.asarray(points_rect) - [rig.P1[0,2], rig.P1[1,2]])[:,0,:]
            self.points_raw = (self.points_raw[:,0,:] - [rig.KK_left[0,2], rig.KK_left[1,2]])
        else:
            cv.UndistortPoints(cv.fromarray(self.points_raw), points_rect, rig.KK_right, rig.kc_right, rig.R2, rig.P2)
            self.points_rect = (np.asarray(points_rect) - [rig.P2[0,2], rig.P2[1,2]])[:,0,:]
            self.points_raw = (self.points_raw[:,0,:] - [rig.KK_right[0,2], rig.KK_right[1,2]])
        
        self.shape_rect = shape.Polygon(self.points_rect)
        self.shape_raw = shape.Polygon(self.points_raw)
        self.shape_im = self.shape_rect.intersection(self.shape_raw)
        
    def pctOverlap(self, x, y, rot, scale):
        scale = 1
        x *= scale
        y *= scale
        theta = rot * np.pi / 180
        rot = np.asarray([[np.cos(theta),np.sin(theta)],[-np.sin(theta),np.cos(theta)]])
        newpts = np.dot(rot, self.points_rect.T).T
        newpts *= scale
        newpts += [x,y]
        newshape = shape.Polygon(newpts)
        sqpts = np.dot(rot, self.points_raw.T).T
        sqpts *= scale
        sqpts += [x,y]
        sqshape = shape.Polygon(newpts)
        imshape = newshape.intersection(sqshape)
        #return newshape.intersection(self.shape_rect).area / self.shape_rect.area
        return imshape.intersection(self.shape_im).area / self.shape_im.area
