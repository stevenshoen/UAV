#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 26 13:26:42 2020

@author: pi
"""
import numpy as np
from utils.coordinates import hor2body

class Target(object):
    def __init__(self, pos=np.array([1000.0, 0.0, 1000.0])):
        self.position = pos
        self.velocity = np.array([0.0, 0.0, 0.0])
        
class Controller:
    class pd_input:
        def __init__(self, p, d):
            self.p = p
            self.d = d
            
        def __call__(self, sig, sig_dot):
            return self.p * sig + self.d * sig_dot
            
    class sine_input:
        def __init__(self, low, high, freq):
            self.low = low
            self.high = high
            self.freq = freq
            
        def __call__(self, time):
            x = np.cos(time * self.freq)
            if x < 0:
                return -x * self.low
            else:
                return x * self.high
            
    def __init__(self, sim, trimmed_controls):
        self.trimmed_controls = trimmed_controls
        self.controls = trimmed_controls
        self.control_names = list(trimmed_controls)
        self.sim = sim
        
        self.pitch_P = 0.02
        self.pitch_D = 0.02
        
        self.roll_P = 0.005
        self.roll_D = 0.005
        
        self.yaw_P = 0.0
        self.yaw_D = 0.0
        
        self.target_roll = 0.0
        self.target_pitch = 0.0481593
        self.correction_limits = [[-np.deg2rad(5), np.deg2rad(5)], #Mx (roll) lims
                              [-np.deg2rad(5), np.deg2rad(5)], #My (pitch)
                              [-np.deg2rad(5), np.deg2rad(5)]] # Mz roll
    
        self.signals = {'pitch': self.pd_input(self.pitch_P, self.pitch_D),
                        'roll': self.pd_input(self.roll_P, self.roll_D),
                        'yaw': self.pd_input(self.yaw_P, self.yaw_D)}
        # for logging
        self.target = Target()
        self.target_body_dir = np.array([0, 0, 0])
        self.lead_distance = 100.0
        
        
    def get_current_controls(self, time):
        return self.controls
    
    def find_angles(self):
        """
        get target world vector
        
        
        convert to body frame
        """
        
        
        target_pos = self.target.position
        cur_pos = np.array([self.sim.system.full_state.position.x_earth,
                            self.sim.system.full_state.position.y_earth,
                            -self.sim.system.full_state.position.z_earth])
            
        cur_vel = np.array([self.sim.system.full_state.velocity.v_north,
                            self.sim.system.full_state.velocity.v_east,
                            -self.sim.system.full_state.velocity.v_down])
            
        cur_world_dir = cur_vel / np.linalg.norm(cur_vel)
        target_world = target_pos - cur_pos
        print('target world:', target_world)
        target_world_dir = target_world / np.linalg.norm(target_world)
        
        lead_point = self.lead_distance * target_world_dir
        
        
        print('lead pt: ', lead_point)
        
        target_body = hor2body(lead_point,
                                     self.sim.system.full_state.attitude.theta,
                                     self.sim.system.full_state.attitude.phi,
                                     self.sim.system.full_state.attitude.psi)
#        
#        self.target_body_dir = target_body / np.linalg.norm(target_body)
        
#        heading = np.atan2()
        
#        heading_error = 
        
        
        
        
#        lead_world = (self.lead_distance * target_world_dir)
#            
#        cur_vel = np.array([self.sim.system.full_state.velocity.v_north,
#                            self.sim.system.full_state.velocity.v_east,
#                            self.sim.system.full_state.velocity.v_down])
#        cur_dir = cur_vel / np.linalg.norm(cur_vel)
#        target_body = hor2body(target_world,
#                                     self.sim.system.full_state.attitude.theta,
#                                     self.sim.system.full_state.attitude.phi,
#                                     self.sim.system.full_state.attitude.psi)
#        print('body target: ', body_frame_target)
        self.target_body_dir = target_body / np.linalg.norm(target_body)
        dx, dy, dz = self.target_body_dir
        
        alt_error = np.arctan2(dz, dx)
        heading_error = np.arctan2(dy, dx)
        
        print('dir --', self.target_body_dir)
        
        
    def state_estimate(self):
        dt = 0.1
        # u, v, w, p, q, r, theta, phi, psi, x_earth, y_earth, z_earth
#        u, v, w, p, q, r, theta, phi, psi, x_earth, y_earth, z_earth
        pitch_rate = self.sim.system.full_state.angular_vel.q
        q_dot = self.sim.system.full_state.angular_accel.q_dot
        roll_rate = self.sim.system.full_state.angular_vel.p
        p_dot = self.sim.system.full_state.angular_accel.p_dot
        yaw_rate = self.sim.system.full_state.angular_vel.r
        r_dot = self.sim.system.full_state.angular_accel.r_dot
        
        pitch = self.sim.system.full_state.attitude.theta
        roll = self.sim.system.full_state.attitude.phi
        yaw = self.sim.system.full_state.attitude.psi
        
        next_pitch = pitch + (pitch_rate + q_dot * dt) * dt
        next_roll = roll + (roll_rate + p_dot * dt) * dt
        next_yaw = yaw + (yaw_rate + r_dot * dt) * dt
        
        return next_pitch, next_roll, next_yaw
        
        
        desired_next_state = self.target_moments
        
        next_error = proj_state - desired_next_state
        inertia = np.array([self.sim.aircraft.inertia[0][0],
                            self.sim.aircraft.inertia[1][1],
                            self.sim.aircraft.inertia[2][2]])
            
        correction = -np.clip(next_error / inertia, *self.correction_limits)
        
        
#    def calc_effection(self):
#        controls = self.controls
#        state_dot = self.sim.system._state_vector_dot
#        
#        
#        effection = 
#        
        
        
    def update(self, time):
        """
        
        get new_state
        
        calc -
            last effection (delta moments_dot per control delta from center)
            
            
            
            grab desired next state
            
            solve for next control delta
            
            project next state
            
            
        
        """
        
        
        
        self.find_angles()
#        xb, yb, zb = self.target_body_dir

        
        pitch_rate = self.sim.system.full_state.angular_vel.q
        q_dot = self.sim.system.full_state.angular_accel.q_dot
        roll_rate = self.sim.system.full_state.angular_vel.p
        p_dot = self.sim.system.full_state.angular_accel.p_dot
        yaw_rate = self.sim.system.full_state.angular_vel.r
        
        pitch = self.sim.system.full_state.attitude.theta
        roll = self.sim.system.full_state.attitude.phi
        yaw = self.sim.system.full_state.attitude.psi
        
        roll_error = self.target_roll - roll
        roll_error_dot = -roll_rate
        
        pitch_error = self.target_pitch - pitch
        pitch_error_dot = -pitch_rate
        
        roll_response = self.signals['roll'](roll_error, roll_error_dot)
        
        pitch_response = self.signals['pitch'](pitch_error, pitch_error_dot)
        
#        print('#####################')
#        print(roll_response)
#        print(roll, roll_rate)
        
        
        ctls = self.controls.copy()
        
        ctls['delta_aileron'] += roll_response
        ctls['delta_elevator'] -= pitch_response
        
        ctls = self.trim_controls(ctls)
        
        self.controls = ctls
        
    def trim_controls(self, ctls):
        for ctl in list(ctls):
            if ctl in list(self.sim.aircraft.control_limits):
                ctls[ctl] = np.clip(ctls[ctl], *self.sim.aircraft.control_limits[ctl])
        return ctls
                
                   