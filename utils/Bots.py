#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    measurements are 
        -magnetic field direction
        -linear acc
        -rot. vel
    state is:
        -rot. pos (attitude)
        -rot. vel
        -rot. acc
"""

import numpy as np
from math import sin, cos

class Robot(object):
    def __init__(self, init_state=np.array([1, 0, 0, 0, 1, 0], dtype=float)):
        
        self.state = init_state
        self.time = 0.0
        self.dt = 0.1
        self.Mprop = np.array([[1, 0, 0, 1 * self.dt, 0, 0],
                                [0, 1, 0, 0, 1 * self.dt, 0],
                                [0, 0, 0, 0, 0, 1 * self.dt],
                                [0, 0, 0, 1, 0, 0],
                                [0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 1]], dtype=float).T
        

        self.Macc_prop = np.array([[1, 0, 0, 0, 0, 0],
                                    [0, 1, 0, 0, 0, 0],
                                    [0, 0, 1, 0, 0, 0],
                                    [0, 0, 0, 1, 0, 0],
                                    [0, 0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 0, 1]], dtype=float)
    
    def time_step(self):
        """
        new_acc comes from the function
        
        new_state = old pos + old omega * dt + half omega dot **2
        new_omega is old omega plus omega dot * dt
        
        omega dot * dt is acc prop
        identity is prop?
        
        prop is 6x6 .. takes a state returns a state
        
        acc_prop is 3x6 .. takes an acc vec and gives state delta
        
        
#        """
        print('----')
        
        state = self.state.copy()
        dt = self.dt
        self.time += dt
        t = self.time
        
        print('xhat:', state)
        A = np.dot(state, self.Mprop.T)
        print(A)
        B = np.matmul(self.Macc_prop, A)

        print(B)
        
        
        new_state = A + B
        print('xhat:', new_state)
        
        
        self.state=new_state

        
        
r = Robot()
r.time_step()
#for t in range(5):
#    print(r.read_mag(t))
#    print(r.read_acc(t))
#        
#    def measure(self, t):
#        noisy_state = self.state(t).copy() + self.noise()
#        return noisy_state
#    
#    def attitude(self, t):
#        pitch = np.sin(t * 0.1)**2
#        roll = 0.0
#        yaw = np.cos(t * 0.1)**2
#        return np.array([pitch, roll, yaw])
#    
##    def ang_velocity(self, t):
##        w_pitch = 0.2 * np.cos(t * 0.1)
##        w_roll = 0.0
##        w_yaw = -0.2 * np.sin(t * 0.1)
##        return np.array([w_pitch, w_roll, w_yaw])
#        
#    def ang_acc(self, t):
#        return self.state[:3] * -1.0
#    
#    def noise(self):
#        #define the variance of the state
#        #use jacobian?
#        return np.random.normal(0, 0.005, 3)
#        
#    def state(self, t):
#        return np.array([self.attitude(t), self.ang_velocity(t), self.ang_acc])
#        
#    def read_mag(self, t):
#        pitch, roll, yaw = self.attitude(t)
#        return np.array([np.sin(yaw), np.cos(yaw), 0.0])
#        
#    def read_acc(self, t):
#        return self.ang_acc(t)
#    
#        
    

    

        
        
#        
#    def state(self, t):
#        q = get_quaternion(*self.attitude(t))
#        gyro_bias = np.array([0, 0, 0])
#        return np.concatenate([q, gyro_bias])
#    
#    def __init__(self, init_att, init_vel):
##        self.driving_func = driving_func
#        self.attitude = np.array(init_att)
#        self.omega = np.array(init_vel)
#        
#        self.time = 0
#        self.dt = 0.01
#        
##        self.acceleration = np.array([]).T
#        
#        self.A = np.array([[], [], [], [], [], [], []]) 
    
        
#class Measurement(object):
    
    


        

    
#    print('############### t == ', t)
#    print(r.attitude(t))
#    print(r.ang_velocity(t))
#    print(r.ang_acc(t))
#    print(r.state(t))
#    q = get_quaternion(*r.attitude(t))
#    
#    print('q: ', q)