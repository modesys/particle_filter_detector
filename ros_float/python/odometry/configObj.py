# -*- coding: utf-8 -*-
# Config parser for float odometry
from ConfigParser import SafeConfigParser

def load():
    class Anon: pass
    config = Anon()
    parser = SafeConfigParser()
    parser.read('/home/administrator/odometry2/config.ini')
    
    config.survey = Anon()
    config.survey.survey_folder = parser.get('survey', 'survey_folder')
    config.survey.imnum_start = parser.getint('survey', 'imnum_start')
    config.survey.max_imgtime_diff = parser.getfloat('survey', 'max_imgtime_diff')
    config.survey.is_antarctic = parser.getboolean('survey', 'is_antarctic')

    config.rig = Anon()
    config.rig.calib_file = parser.get('rig', 'calib_file')
    config.rig.bw_is_left = parser.getboolean('rig', 'bw_is_left')
    config.rig.match_left = parser.getboolean('rig', 'match_left')
    config.rig.minalt = parser.getfloat('rig', 'minalt')
    config.rig.maxalt = parser.getfloat('rig', 'maxalt')
    
    config.corr = Anon()
    config.corr.mincorr_stereo = parser.getfloat('corr', 'mincorr_stereo')
    config.corr.mincorr_rotscale = parser.getfloat('corr', 'mincorr_rotscale')
    config.corr.mincorr_motion = parser.getfloat('corr', 'mincorr_rotscale')
    
    config.performance = Anon()
    config.performance.predownscale = parser.getboolean('performance', 'predownscale')
    config.performance.nthreads = parser.getint('performance', 'nthreads')
    config.performance.short_factor = parser.getint('performance', 'short_factor')
    config.performance.small_factor = parser.getint('performance', 'small_factor')
    config.performance.sleep_time = parser.getfloat('performance', 'sleep_time')
    
    config.analysis = Anon()
    config.analysis.compute_overlap = parser.getboolean('analysis', 'compute_overlap')
    config.analysis.outline_ptsperside = parser.getint('analysis', 'outline_ptsperside')
    
    config.odometry = Anon()
    config.odometry.start_period = parser.getfloat('odometry', 'start_period')
    config.odometry.burst_mode = parser.getboolean('odometry', 'burst_mode')
    config.odometry.burst_delay = parser.getfloat('odometry', 'burst_delay')
    config.odometry.burst_period = parser.getfloat('odometry', 'burst_period')
    config.odometry.burst_num_ims = parser.getint('odometry', 'burst_num_ims')
    config.odometry.min_period = parser.getfloat('odometry', 'min_period')
    config.odometry.max_period = parser.getfloat('odometry', 'max_period')
    config.odometry.fixed_spacing = parser.getboolean('odometry', 'fixed_spacing')
    config.odometry.image_spacing = parser.getfloat('odometry', 'image_spacing')

    return config
