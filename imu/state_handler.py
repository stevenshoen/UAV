#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 22 12:06:29 2020

@author: pi
"""

from imu_handler import IMU_Handler
import numpy as np


h = IMU_Handler()
h.load_calibration()

print(h.cal_dict)
#
av_g, std_g, av_gyr, std_gyr = h.static_calibration_capture()
#h.dynamic_calibration_capture()
#print(h.data)
#
#h.calculate_mag_calibration(h.data)
#print(h.cal_dict)
h.save_calibration()


reading = h.get_reading()

attitude, acc_hat, mag_hat= reading

from utils import aircraft_viewer

width = 200
height = 200

aircraft_viewer.ProjectionViewer(width, height)

"""
find 
    earth down in body frame
    
"""
#heading = np.arctan2(mag_hat[0], mag_hat[1])
#
#
#alpha = np.atan2()
