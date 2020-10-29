#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 13 20:04:38 2020

@author: pi
"""
import numpy as np
from math import sin, cos

def get_quaternion(pitch, roll, yaw):
    cy = cos(yaw / 2)
    sy = sin(yaw / 2)
    cp = cos(pitch / 2)
    sp = sin(pitch / 2)
    cr = cos(roll / 2)
    sr = sin(roll / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])

class Robot(object):
    def __init__(self):
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

    def noise(self):
        return np.random.normal(0, 0.005, 3)
        
    def read_mag(self, t):
        pitch, roll, yaw = self.attitude(t)
        return np.array([np.sin(yaw), np.cos(yaw), 0.0])
        
    def read_acc(self, t):
        return self.ang_acc(t)
    
    def attitude(self, t):
        pitch = np.sin(t * 0.1)**2
        roll = 0.0
        yaw = np.cos(t * 0.1)**2
        return np.array([pitch, roll, yaw])
    
    def ang_velocity(self, t):
        w_pitch = 0.2 * np.cos(t * 0.1)
        w_roll = 0.0
        w_yaw = -0.2 * np.sin(t * 0.1)
        return np.array([w_pitch, w_roll, w_yaw])
    
    def ang_acc(self, t):
        a_pitch = -0.02 * np.sin(t * 0.1)
        a_roll = 0.0
        a_yaw = -0.02 * np.cos(t * 0.1)
        return np.array([a_pitch, a_roll, a_yaw])
        
        
        
    def state(self, t):
        q = get_quaternion(*self.attitude(t))
        gyro_bias = np.array([0, 0, 0])
        return np.concatenate([q, gyro_bias])
        
        
r = Robot()
for t in range(5):
    print(r.read_mag(t))
    print(r.read_acc(t))
    
#    print('############### t == ', t)
#    print(r.attitude(t))
#    print(r.ang_velocity(t))
#    print(r.ang_acc(t))
#    print(r.state(t))
#    q = get_quaternion(*r.attitude(t))
#    
#    print('q: ', q)