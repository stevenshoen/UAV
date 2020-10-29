#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 17 13:03:13 2020

@author: pi
"""

import time
#import math
import IMU
import datetime
#import os
import numpy as np
import math
import signal
import pickle



HIGHEST_INT = 32767
mag_limits = None

data = {} #store recent data aquires

mag_valid = False
G_GAIN = 0.07
last_read = datetime.datetime.now()
DOWN = np.array([0, 0, -1])
GRAVITY = 0.001 # nominal acc magnitude equivalent to 1G
dacc_dG = 0.001
data_dt = None








now = datetime.datetime.now()
data_dt = (now - last_read).microseconds/(1000000*1.0)





ACCx = IMU.readACCx()
ACCy = IMU.readACCy()
ACCz = IMU.readACCz()
GYRx = IMU.readGYRx()
GYRy = IMU.readGYRy()
GYRz = IMU.readGYRz()
MAGx = IMU.readMAGx()
MAGy = IMU.readMAGy()
MAGz = IMU.readMAGz()
acc, gyr, mag = np.array([[ACCx, ACCy, ACCz],
                 [GYRx, GYRy, GYRz],
                 [MAGx, MAGy, MAGz]], dtype=float) / HIGHEST_INT

gyr *= G_GAIN


acc_magnitude = np.linalg.norm(acc)
acc_hat = acc / acc_magnitude

mag_magnitude = np.linalg.norm(mag)
mag_hat = mag / mag_magnitude

gravity_hat = np.cross(mag_hat, DOWN) / np.linalg.norm(np.cross(mag_hat, DOWN))

gravity = GRAVITY * gravity_hat

adj_acc = acc - gravity

adj_acc_hat = adj_acc / np.linalg.norm(adj_acc)


#Calculate pitch and roll
pitch = np.arcsin(adj_acc_hat[0])
roll = -np.arcsin(adj_acc_hat[1] / np.cos(pitch))

        #magenometer
#Calculate heading
#        heading = self.trim_heading(math.atan2(MAGy,MAGx))

magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)
magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)

#yaw = trim_heading(np.arctan2(magYcomp,magXcomp))
last_read = now


        
        
        
        
        
        
        
        
        
        
        









