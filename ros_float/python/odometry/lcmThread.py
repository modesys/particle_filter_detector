# -*- coding: utf-8 -*-

# Thread to control the cameras. Doesn't actually use LCM for now...

import threading, time, subprocess

class lcmThread(threading.Thread):
    def __init__(self, config):
        threading.Thread.__init__(self)
        self.config = config
        self.exitFlag = False
        self.period = 3.0
        self.newperiod = 3.0
        self.starttime = time.time()
        self.nextburst = self.starttime
        
        
        #self.changeFramerate()
        #self.startCams()
    
    def run(self):
        while not self.exitFlag:
            if time.time() > self.nextburst:
                self.nextburst += self.config.odometry.burst_delay
                print("Entering Burst: %f" % self.config.odometry.burst_period)
                self.changeFramerate(1./self.config.odometry.burst_period)
                time.sleep(self.config.odometry.burst_period*self.config.odometry.burst_num_ims)
                print("Exiting Burst: %f" % self.period)
                self.changeFramerate(1./self.period)
            if self.newperiod < .95*self.period or self.newperiod > 1.05*self.period:
                self.changeFramerate(1./self.newperiod)
                self.period = self.newperiod
            time.sleep(0.1)
        print("Closing LCM thread.")
    
    def setPeriod(self, period):
        
        # min/max stuff
        if period < self.config.odometry.min_period:
            period = self.config.odometry.min_period
        if period > self.config.odometry.max_period:
            period = self.config.odometry.max_period
        self.newperiod = period
    
    def changeFramerate(self, newFramerate):
        subprocess.call(['/home/administrator/cam-driverV2/bin/cam-attributes', '-m', '-s', '--FrameRate', str(newFramerate)])
    
    def startCams(self):
        subprocess.call(['/home/administrator/cam-driverV2/bin/cam-attributes', '-c', '-s', '-O'])
        subprocess.call(['/home/administrator/cam-driverV2/bin/cam-attributes', '-m', '-s', '-O'])
