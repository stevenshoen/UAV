#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 28 10:09:23 2020

@author: pi
"""
import numpy as np
from scipy import interpolate
from scipy.interpolate import RectBivariateSpline
import matplotlib.pyplot as plt
from state.aircraft_state import AircraftState
from state.position import EarthPosition
from state.velocity import BodyVelocity, NEDVelocity
from state.attitude import EulerAttitude
from time import perf_counter#, perf_counter_ns
"""
need to map Cl, Cn, Cm over 
"""
def trim_heading(hdg):
    if hdg < -np.pi:
        return hdg + 2 * np.pi
    elif hdg > np.pi:
        return hdg - 2 * np.pi
    else:
        return hdg

def trim_roll(roll):
    if roll < -np.pi:
        return roll + 2 * np.pi
    elif roll > np.pi:
        return roll - 2 * np.pi
    else:
        return roll
        
class FrequencyTimer(object):
    def __init__(self, systems, time = 0.0):
        self.time = time
        self.systems = systems
        # keyed by name
        # inside dict keys are: func to call, freq, last_called
    def update(self, t, measure=False):
#        print('t: ', t)
        resp = [np.nan for i in range(len(self.systems))]
        for sys in sorted(list(self.systems)):
            last = self.systems[sys]['last_called']
            if type(last) == type(None):
                self.systems[sys]['last_called'] = t
                self.systems[sys]['func']()                
            else:
                dt = 1 / self.systems[sys]['freq']
                if t - last >= dt:
                    self.systems[sys]['last_called'] = t
                    if measure: t0 = perf_counter()
                    self.systems[sys]['func']()
                    if measure: resp[sys] = perf_counter() - t0
                else:
                    if measure: resp[sys] = np.nan
        return resp
    
#def nav_ref():
#    print('nav called')
#def guide_ref():
#    print('guide called')
#def damper_ref():
#    print('damper called')
#    
#    
#nav_sys = {'func': nav_ref, 'freq': 1, 'last_called': None}
#guide_sys = {'func': guide_ref, 'freq': 2, 'last_called': None}
#damper_sys = {'func': damper_ref, 'freq': 10, 'last_called': None}
#
#sys = {'navigation': nav_sys,
#       'guidance': guide_sys,
#       'damper': damper_sys}
#       
#f = FrequencyTimer(sys)
#[f.update(i) for i in np.arange(0, 1, 0.1)]


class Mapper(object):
    def __init__(self, aircraft, N=8):
        self.N = N
        LIMITS = [-0.90, 0.90]
        self.aircraft = aircraft
        
        self.alpha_limits = LIMITS
        self.beta_limits = LIMITS
        
        self.CL_map = np.ndarray((len(self.alpha_range), len(self.beta_range)))
        self.CM_map = np.ndarray((len(self.alpha_range), len(self.beta_range)))
        self.CN_map = np.ndarray((len(self.alpha_range), len(self.beta_range)))
        
        self.CLc_map = np.ndarray((len(self.alpha_range), len(self.beta_range)))
        self.CMc_map = np.ndarray((len(self.alpha_range), len(self.beta_range)))
        self.CNc_map = np.ndarray((len(self.alpha_range), len(self.beta_range)))
                
        self.CL_inter = None
        self.CM_inter = None
        self.CN_inter = None
        
        self.CLc_inter = None
        self.CMc_inter = None
        self.CNc_inter = None
        
    def get_point(self):        
        control_delta = 0.001
        cur = self.controls['delta_aileron']
        dL = self.aircraft._calculate_control_lat_moments_coeffs(cur + control_delta) -\
        self.aircraft._calculate_control_lat_moments_coeffs(cur)
        self.dL_dail = dL / control_delta
        
    def build_maps(self):
        state = self.state()
        for a_i, b_i in zip(range(len(self.alpha_range)), range(len(self.beta_range))):
#        for a_i in range(len(self.alpha_range)):
#            for b_i in range(len(self.beta_range)):
                print(a_i, b_i)
                self.control_coeffs(a_i, b_i, state)
                self.body_coeffs(a_i, b_i, state)
                
        self.CL_inter = RectBivariateSpline(self.alpha_range, self.beta_range, self.CL_map)        
        self.CM_inter = RectBivariateSpline(self.alpha_range, self.beta_range, self.CM_map)        
        self.CN_inter = RectBivariateSpline(self.alpha_range, self.beta_range, self.CN_map)
        
        self.CLc_inter = RectBivariateSpline(self.alpha_range, self.beta_range, self.CLc_map)        
        self.CMc_inter = RectBivariateSpline(self.alpha_range, self.beta_range, self.CMc_map)        
        self.CNc_inter = RectBivariateSpline(self.alpha_range, self.beta_range, self.CNc_map)
        
    def state(self, x=0.0, y=0.0, theta=0.0, phi=0.0, psi=0.0, u=0.0, v=0.0, w=0.0):
        pos = EarthPosition(x, y, 1000)
        att = EulerAttitude(theta, phi, psi)
        vel = BodyVelocity(u=u, v=v, w=w, attitude=att)
        return AircraftState(pos, att, vel, angular_vel=None,
                 acceleration=None, angular_accel=None)

    def control_coeffs(self, alpha_i, beta_i, state):
        self.aircraft.alpha = self.alpha_range[alpha_i]
        self.aircraft.beta = self.beta_range[beta_i]
        CL, CN = self.aircraft._calculate_control_lat_moments_coeffs(state)        
        CM = self.aircraft._calculate_control_lon_moments_coeffs(state)
        self.CLc_map[alpha_i, beta_i] = CL
        self.CMc_map[alpha_i, beta_i] = CM
        self.CNc_map[alpha_i, beta_i] = CN
        return CL, CM, CN
        
    def body_coeffs(self, alpha_i, beta_i, state):
        self.aircraft.alpha = self.alpha_range[alpha_i]
        self.aircraft.beta = self.beta_range[beta_i]
        CL, CN = self.aircraft._calculate_body_lat_moments_coeffs(state)        
        CM = self.aircraft._calculate_body_lon_moments_coeffs(state)
        self.CL_map[alpha_i, beta_i] = CL
        self.CM_map[alpha_i, beta_i] = CM
        self.CN_map[alpha_i, beta_i] = CN
        return CL, CM, CN
                
    @property
    def alpha_range(self):
        return np.linspace(self.alpha_limits[0], self.alpha_limits[1], self.N)

    @property
    def beta_range(self):
        return np.linspace(self.beta_limits[0], self.beta_limits[1], self.N)


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
                





class Target(object):
    
    def __init__(self, pos=np.array([1000.0, -1000.0, -6000.0]), velocity=(0.0, 20.0, 0.0)):
        self.initial_position = EarthPosition(*pos)        
#        attitude = EulerAttitude(0, 0, 0)
#        self.velocity = NEDVelocity(*velocity, attitude)
        self.velocity = np.array(velocity)
        
    def position(self, t=0):
        return self.initial_position.earth_coordinates + (self.velocity * t)

    def distance_from(self, x, y, t=0):
        pos = self.position(t)
        return np.linalg.norm(np.array([pos[0]-x, pos[1]-y]))
    
    def heading_from(self, x, y, t=0):
        pos = self.position(t)
        x_, y_ = np.array([pos[0]-x, pos[1]-y])
#        print('x_', x_)
#        print('y_', y_)        
        return trim_heading(np.arctan2(y_, x_))   
        


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

        

                
#        
#    def __call__(self):
#        #
                
        