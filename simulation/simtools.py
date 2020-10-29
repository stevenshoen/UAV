#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 28 10:09:23 2020

@author: pi
"""
import numpy as np
from scipy import interpolate

class Intercept(object):
    def __init__(self, position, heading, time):
        self.position = position
        self.heading = heading
        self.time = time
    
class Target(object):
    
    def __init__(self, pos=np.array([1000.0, 0.0, 1000.0])):
        self.initial_position = pos
        
        self.velocity = np.array([0.0, 15.0, 0.0])

    @property
    def position(self, t=0):
        return self.initial_position + self.velocity * t
    
    def extrapolate_path(self, t):
        return self.postition + velocity * t
#    def calc_intercept(self, pos, cur_time=0.0):
#        """
#        
#        this will be iterative .. the heart of the path_finding algo?
#        
#        repeat process when intercept time or location changes by a set amount?
#        
#        
#        
#        calc fastest path .. then adjust heading?
#        
#        """
#        t_pos = self.position + self.velocity * cur_time
#        distance = np.linalg.norm(t_pos - pos) 
#        
#        
#        hdg = self.heading
#        
#        
#        calc_cruise_time
#        
#        extrapolate_target_path
#        pass
#    

        
class MappedVariable:
    """
    
    wrapper for the linearization of a variable's gradient wrt params
    i.e. q_ele_coe
    
    gridmap will be interpolated
    
    queing measured_points
    
    velocity correction needs to calculate coes / m/s?
    
    needs to rescale and resize on the fly
    
    """
    def __init__(self, N=100):
        
        INITIAL_LIMITS = [-0.01, 0.01]
        self.default_value = 0.001
        
        self.map_len = N
        
        self.init_points = 10
                
        self.base_velocity = 50
        
        self.alpha_limits = INITIAL_LIMITS
        self.beta_limits = INITIAL_LIMITS
        
        self.measured_points = []

        self.func = None
        
    @property
    def alpha_range(self):
        return np.linspace(self.alpha_limits[0], self.alpha_limits[1], self.map_len)

    @property
    def beta_range(self):
        return np.linspace(self.beta_limits[0], self.beta_limits[1], self.map_len)


    @property
    def initialized(self):
            return not(type(self.func) == type(None))
    
    @property
    def isvalid(self):
            return len(self.measured_points) >= self.init_points
#    
    def limit_change(self, alpha, beta):
        changed = False
        if alpha < self.alpha_limits[0]:
            self.alpha_limits[0] = alpha
            changed = True
        elif alpha > self.alpha_limits[1]:
            self.alpha_limits[1] = alpha
            changed = True
        if beta < self.beta_limits[0]:
            self.beta_limits[0] = beta
            changed = True
        elif beta > self.beta_limits[1]:
            self.beta_limits[1] = beta
            changed = True
        return changed
        
    def update(self, eff, alpha, beta):
        
        r_alpha = alpha
        r_beta = beta
        
        self.measured_points.append(np.array([eff, alpha, beta]))
        
        if (not self.initialized) or self.limit_change(r_alpha, r_beta):
            if self.isvalid:
                self.func = self.build_func()
                
    def nearest_i(self, check_array, val):
        i = np.abs(np.asarray(check_array) - val).argmin()
        return i    
    
    def nearest(self, check_array, val):
        i = np.abs(np.asarray(check_array) - val).argmin()
        return check_array[i]    
       
    def build_func(self):
        pts = np.array(self.measured_points)
        alphas = self.alpha_range
#        print('alphas', alphas)
        betas = self.beta_range
        w = np.ndarray((len(alphas), len(betas)))
        
#        w = np.ndarray(())
        
        for i in range(len(pts)):
            alpha_i = self.nearest_i(alphas, pts[i][1])
#            print(alpha_i)
            beta_i = self.nearest_i(betas, pts[i][2])
            w[alpha_i, beta_i] = pts[i][0]
            
#        print('w', w)
        self.func = interpolate.interp2d(alphas, betas, w, kind='cubic')

    def estimate(self, alpha, beta):
        # value of x at params
        #TODO
        # v correction
        if self.initialized:
            x = self.func(alpha, beta)
            return x
        else:
            if len(self.measured_points) > 0:
                return self.measured_points[-1][0] #return the last
            else:
                return self.default_value
                
#        
#    def __call__(self):
#        #
                
        