#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 16 18:32:36 2020

@author: pi
"""
import copy
import operator
import numpy as np
import pandas as pd
import tqdm
import matplotlib.pyplot as plt
from pyfme.simulator import Simulation
from pyfme.environment.wind import NoWind
from pyfme.utils.coordinates import hor2body

class Target(object):
    def __init__(self, pos=np.array([1000.0, 0.0, 1000.0])):
        self.position = pos
        self.velocity = np.array([0.0, 0.0, 0.0])
    
    
    
class Wind(object):

    def __init__(self):
        # Wind velocity: FROM North to South, FROM East to West,
        # Wind velocity in the UPSIDE direction
        self.horizon = np.zeros([3], dtype=float)
        self.body = np.zeros([3], dtype=float)
        self.count = 0
        self.freq = .01
        self.mag = 0.50
        self.clip = 3.0
        self.steady = 5.0
        self.transient = 0.0
        
    def sin_wind(self):
        return np.array([self.steady + self.transient * np.sin(self.freq * self.count),
                  self.steady + self.transient * np.sin(self.freq * self.count),
                  0.0], dtype=float)
        
    def rand_wind(self):
        t = self.mag/2 * (np.random.random(3) - 1/2)
        self.transient = np.clip(self.transient + t, -self.clip, self.clip)
#        self.transient += self.mag * np.random.random(3)
#        self.transient = np.clip(self.transient, -self.clip, self.clip)
        return self.steady + self.transient  
    
    def update(self, state):
        self.body = self.rand_wind()
#        print('wind -', self.body)
#        self.count += 1
        
        pass


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
        
        print('get controls :', self.controls)
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
    def null_update(self, time):
        return self.trimmed_controls
    
    def update(self, time):
        self.controls = self.trimmed_controls
        print('controller update :', self.trimmed_controls)
        return self.trimmed_controls
        
    def update1(self, time):
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
                
                
                


class ModSim(Simulation):
    default_save_vars = {
#        'elev_CL': 'aircraft.elev_CL',
#        'elev_CD': 'aircraft.elev_CD',
#        'elev_CM': 'aircraft.elev_CM',
#        'rud_CY': 'aircraft.rud_CY',
#        'ail_Cl': 'aircraft.ail_Cl',
#        'rud_Cl': 'aircraft.rud_Cl',
#        'ail_CN': 'aircraft.ail_CN',
#        'rud_CN': 'aircraft.rud_CN',

        'time': 'system.time',
        'wind': 'environment.wind.body',
#        # environment
#        'temperature': 'environment.T',
#        'pressure': 'environment.p',
#        'rho': 'environment.rho',
#        'a': 'environment.a',
#        # aircraft
#        'Fx': 'aircraft.Fx',
#        'Fy': 'aircraft.Fy',
#        'Fz': 'aircraft.Fz',
#
#        'Mx': 'aircraft.Mx',
#        'My': 'aircraft.My',
#        'Mz': 'aircraft.Mz',
#
        'TAS': 'aircraft.TAS',
#        'Mach': 'aircraft.Mach',
#        'q_inf': 'aircraft.q_inf',
#
#        'alpha': 'aircraft.alpha',
#        'beta': 'aircraft.beta',

        'rudder': 'aircraft.delta_rudder',
        'aileron': 'aircraft.delta_aileron',
        'elevator': 'aircraft.delta_elevator',
        'thrust': 'aircraft.delta_t',
        # system
        'x_earth': 'system.full_state.position.x_earth',
        'y_earth': 'system.full_state.position.y_earth',
        'z_earth': 'system.full_state.position.z_earth',

        'height': 'system.full_state.position.height',

        'psi': 'system.full_state.attitude.psi',
        'theta': 'system.full_state.attitude.theta',
        'phi': 'system.full_state.attitude.phi',

#        'u': 'system.full_state.velocity.u',
#        'v': 'system.full_state.velocity.v',
#        'w': 'system.full_state.velocity.w',
#
#        'v_north': 'system.full_state.velocity.v_north',
#        'v_east': 'system.full_state.velocity.v_east',
#        'v_down': 'system.full_state.velocity.v_down',
#
#        'p': 'system.full_state.angular_vel.p',
#        'q': 'system.full_state.angular_vel.q',
#        'r': 'system.full_state.angular_vel.r'
    }
    
    def __init__(self, aircraft, system, environment, trimmed_controls, dt=0.01):
        """
        Simulation object
        
        Parameters
        ----------
        aircraft : Aircraft
            Aircraft model
        system : System
            System model
        environment : Environment
            Environment model.
        save_vars : dict, opt
            Dictionary containing the names of the variables to be saved and
            the object and attribute where it is calculated. If not given, the
            ones set in `_default_save_vars` are used.
        """
        self.system = copy.deepcopy(system)
        self.aircraft = copy.deepcopy(aircraft)
        self.environment = copy.deepcopy(environment)

        self.system.update_simulation = self.update

        self.controller = Controller(self, trimmed_controls=trimmed_controls)

        self.dt = dt

        self._save_vars = self.default_save_vars
        
        self.results = {name: [] for name in self._save_vars}

    @property
    def time(self):
        return self.system.time

    def update(self, time, state):
        self.environment.update(state)

        controls = self._get_current_controls(time)

        self.aircraft.calculate_forces_and_moments(
            state,
            self.environment,
            self.controller.controls
        )
        return self

    def propagate(self, time):
        """Run the simulation by integrating the system until time t.

        Parameters
        ----------
        time : float
            Final time of the simulation

        Notes
        -----
        The propagation relies on the dense output of the integration
        method, so that the number and length of the time steps is
        automatically chosen.
        """
        dt = self.dt
        half_dt = self.dt/2

        bar = tqdm.tqdm(total=time, desc='time', initial=self.system.time)
#        fig, axes = plt.subplots(1, 3)
        # To deal with floating point issues we cannot check equality to
        # final time to finish propagation
        time_plus_half_dt = time + half_dt
        print('running..')
        while self.system.time + dt < time_plus_half_dt:
            t = self.system.time
            self.environment.update(self.system.full_state)
#            controls = self._get_current_controls(t)
#            self.aircraft.controls = self._get_current_controls(t)
            self.controller.null_update(t)
            cur_controls = self.controller.controls
            self.aircraft._set_current_controls(cur_controls)
            print('--', self.aircraft.controls)
            print('--', cur_controls)
            self.aircraft.calculate_forces_and_moments(self.system.full_state,
                                                       self.environment, cur_controls)
            self.system.time_step(dt)
            self._save_time_step()
            bar.update(dt)
#            self.update_hud(axes)
#            plt.pause(0.05)
#        plt.show()
        bar.close()

        results = pd.DataFrame(self.results)
        results.set_index('time', inplace=True)

        return results

    def update_hud(self, axes):
        hud_ax, otro_ax, nutte_ax = axes
        
        phi = self.system.full_state.attitude.phi
        psi = self.system.full_state.attitude.psi
        theta = self.system.full_state.attitude.theta
#        print('phi ', phi)
        rise_line = np.cos(theta) * np.cos(phi)
        hud_ax.plot([[np.cos(phi), np.sin(phi)],
                     [-np.cos(phi), -np.sin(phi)]])
        
#        hud_ax.plot([np.cos(phi), np.sin(phi)])
#        hud_ax.plot([-np.cos(phi), -np.sin(phi)])
#        
        
#        roll_line = np.asin(1 / )
#        hz_line = 
        
#        self.controller.target_body_dir
        
#        hud_ax.scatter()
        plt.pause(0.05)
    def _save_time_step(self):
        """Saves the selected variables for the current system, environment
        and aircraft.
        """
        for var_name, value_pointer in self._save_vars.items():
            self.results[var_name].append(
                operator.attrgetter(value_pointer)(self)
            )

    def _get_current_controls(self, time):
        return self.controller.controls

#    def _get_current_controls(self, time):
#        """Get the control values for the current time step for the given
#        input functions.
#
#        Parameters
#        ----------
#        time : float
#            Current time value.
#
#        Returns
#        -------
#        controls : dict
#            Control value for each control
#
#        Notes
#        -----
#        Current controls are only a function of time in this kind of
#        simulation (predefined inputs). However, if the AP is active,
#        controls will be also function of the system state and environment.
#        """
#        c = {c_name: c_fun(time) for c_name, c_fun in self.controls.items()}
#        return c



