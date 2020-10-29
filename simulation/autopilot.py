#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 14:42:21 2020

@author: pi
"""

import numpy as np
#from scipy import interpolate

from signal_processing.PID_control import pd_signal
from simulation.simtools import Target, MappedVariable
from utils.constants import GRAVITY

class Controller:
    """
    monitor potential/kinetic energy
    
    given heading and altitude errors .. get desired rotation rates (PD)
    
    *add points to control matrix from roll rate dots and current controls
    
    given current rotation rates .. predict next rotation rates
    
    
    rotation error is the difference 
    
    calculate needed control delta for the correction
    
    set new controls
    
    
    """
    def __init__(self, sim, trimmed_controls):
        
        self.trimmed_controls = trimmed_controls
        self.controls = trimmed_controls
        self.control_names = list(trimmed_controls)
        self.sim = sim
        
        self.nominal_energy = sim.aircraft.mass * sim.aircraft.TAS + sim.aircraft.mass * -sim.system.full_state.position.z_earth * GRAVITY
        self.current_energy = self.nominal_energy
        
        self.update_time_delta = 0.1 #sec
        self.time = 0.0 # independent clock
        
        self.alpha = sim.aircraft.alpha
        
        self.initialized = False
        
        self.pitch_effection = MappedVariable()
        self.roll_effection = MappedVariable()
        
        roll_p = -0.01
        roll_d = -0.00001
        self.roll_pd = pd_signal(roll_p, roll_d)
        
        pitch_p = 0.5
        pitch_d = 0.01
        self.pitch_pd = pd_signal(pitch_p, pitch_d)
    
        thrust_p = 0.1
        thrust_d = 0.01
        self.thrust_pd = pd_signal(thrust_p, thrust_d)
        
        # for logging
        self.target = Target()
        self.target_body_dir = np.array([0, 0, 0])
        self.lead_distance = 100.0
        
        self.last_pitch_correction = 0.0
        self.last_roll_correction = 0.0
    
    def energy_monitor(self, full_state):
        k = self.mass * self.sim.aircraft.TAS #m * v
        p = self.mass * -self.sim.system.full_state.position.z_earth * GRAVITY #m * h
    
        curr_energy = k + p
        
        energy_error = self.nominal_energy - curr_energy
        error_dot = 0 # not implemented
        
        thrust_correction = self.thrust_pd(energy_error, error_dot)
        
        
        
    
    def update(self, time):
        if time < self.time + self.update_time_delta:
            return self.controls
        self.time = time
        print('updatin')
        """
        
        -init goes until effection is mapped
            sends slow variations.
            roll first
            
        
        
        -calc heading and alt errors
        
        -update effection map
        
        
        -apply correction
        
        
        """
        
        self.read_effection()
        
        hdg_error, alt_error = self.get_error_angles()
        
#        heading = self.sim.system.full_state.attitude.psi
        pitch = self.sim.system.full_state.attitude.theta
        pitch_rate = self.sim.system.full_state.angular_vel.q
        roll = self.sim.system.full_state.attitude.phi
        roll_rate = self.sim.system.full_state.angular_vel.p
        
        desired_pitch = self.alpha
        desired_pitch_rate = 0
        desired_roll = 0.0
        desired_roll_rate = 0.0
        
        pitch_error = pitch - desired_pitch
        pitch_error_dot = pitch_rate - desired_pitch_rate
        
        roll_error = roll - desired_roll
        roll_error_dot = roll_rate - desired_roll_rate
        
        roll_pd_sig = self.roll_pd(roll_error, roll_error_dot)
        pitch_pd_sig = self.pitch_pd(pitch_error, pitch_error_dot)
        
        pitch_correction = pitch_pd_sig
        roll_correction = roll_pd_sig
        
        self.controls['delta_elevator'] += pitch_correction
        self.controls['delta_aileron'] += roll_correction
        
        self.trim_controls()
        
        v = self.sim.aircraft.TAS
        alpha = self.sim.aircraft.alpha
        beta = self.sim.aircraft.beta
        pitch_eff = self.pitch_effection.estimate(alpha, beta)
        roll_eff = self.roll_effection.estimate(alpha, beta)
        
#        pitch_correction = pitch_eff * v**2
#        roll_correction = roll_eff * v**2 # [rad/rad]
#       
        
        self.last_pitch_correction = pitch_correction
        self.last_roll_correction = roll_correction
        
        return self.controls
    
    def control_deltas(self, controls):
        return {key:self.trimmed_controls[key] - controls[key] for key in list(controls)}
    
    def read_effection(self):
#        dcontrols = self.control_deltas(self.controls)
        
        d_ele = self.last_pitch_correction
        d_ail = self.last_roll_correction
        
        
        alpha = self.sim.aircraft.alpha
        beta = self.sim.aircraft.beta
        v = self.sim.aircraft.TAS
        
        
        
        p_dot = self.sim.system.full_state.angular_accel.p_dot #roll rate dot
        q_dot = self.sim.system.full_state.angular_accel.q_dot #pitch rate dot
#        r_dot = self.sim.system.full_state.angular_accel.r_dot
        
#        d_ele = dcontrols['delta_elevator']
#        d_ail = dcontrols['delta_aileron']
#        d_rud = dcontrols['delta_rudder']
        
        
        
#        print(d_ele, d_ail, d_rud)
#        print(p_dot, q_dot, r_dot)
        if d_ail != 0:
            roll_effect = p_dot / (d_ail * v**2)
            self.roll_effection.update(roll_effect, alpha, beta)
            
        if d_ele != 0:
            pitch_effect = q_dot / (d_ele * v**2)
            self.pitch_effection.update(pitch_effect, alpha, beta)

   
#    def get_current_controls(self, time):
#        
#        print('get controls :', self.controls)
#        return self.controls
    

    def null_update(self, time):
        return self.controls

    def get_error_angles(self):
        hdg_error = 0 
        alt_error = 0
        return hdg_error, alt_error
        

        
        
    def trim_controls(self, ctls=None):
        if  not ctls:
            ctls = self.controls
        for ctl in list(ctls):
            if ctl in list(self.sim.aircraft.control_limits):
                ctls[ctl] = np.clip(ctls[ctl], *self.sim.aircraft.control_limits[ctl])
        return ctls
                
#    
#class pd_input:
#        def __init__(self, p, d):
#            self.p = p
#            self.d = d
#            
#        def __call__(self, sig, sig_dot):
#            return self.p * sig + self.d * sig_dot
#        
#    class sine_input:
#        def __init__(self, low, high, freq):
#            self.low = low
#            self.high = high
#            self.freq = freq
#
#        def __call__(self, time):
#            x = np.cos(time * self.freq)
#            if x < 0:
#                return -x * self.low
#            else:
#                return x * self.high
#    
    
#              self.signals = {'pitch': self.pd_input(self.pitch_P, self.pitch_D),
#                        'roll': self.pd_input(self.roll_P, self.roll_D),
#                        'yaw': self.pd_input(self.yaw_P, self.yaw_D)}          
#    
#        self.p_ail_coe = 0.0
#        self.q_ele_coe = 0.0
#        self.r_rud_coe = 0.0
#        
#        
#        self.pitch_P = 0.02
#        self.pitch_D = 0.02
#        
#        self.roll_P = 0.005
#        self.roll_D = 0.005
#        
#        self.yaw_P = 0.0
#        self.yaw_D = 0.0
#        
#        self.target_roll = 0.0
#        self.target_pitch = 0.0481593
#        self.correction_limits = [[-np.deg2rad(5), np.deg2rad(5)], #Mx (roll) lims
#                              [-np.deg2rad(5), np.deg2rad(5)], #My (pitch)
#                              [-np.deg2rad(5), np.deg2rad(5)]] # Mz roll
                
