#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 14:42:21 2020

@author: pi
"""
import copy
import numpy as np
#from scipy import interpolate
from scipy.optimize import least_squares
from signal_processing.PID_control import pd_signal
from simulation.simtools import Target, FrequencyTimer #, MappedVariable
from utils.constants import GRAVITY
from utils.coordinates import body2hor, hor2body
from simulation.trimmer import trimming_cost_func

def trim_heading(hdg):
    if hdg < -np.pi:
        return hdg + 2 * np.pi
    elif hdg > np.pi:
        return hdg - 2 * np.pi
    else:
        return hdg

class Autopilot(object):
    _save_vars = {
            'waypoint_heading': 'controller.waypoint_heading',
        'waypoint_altitude': 'controller.waypoint_altitude',
        
        'guidance_roll_dot': 'controller.guidance_roll_dot',
        'guidance_roll': 'controller.guidance_roll',
        'guidance_heading': 'controller.guidance_heading',
        'guidance_climb': 'controller.guidance_climb',
        'guidance_climb_dot': 'controller.guidance_climb_dot',
        
        'pitch_correction': 'controller.pitch_correction',
        'roll_correction': 'controller.roll_correction',
        'yaw_correction': 'controller.yaw_correction',
        'thrust_correction': 'controller.thrust_correction',
        'pitch_err': 'controller.pitch_err',
        'pitch_rate_err': 'controller.pitch_rate_err',
        'roll_err': 'controller.roll_err',
        'roll_rate_err': 'controller.roll_rate_err',
        'yaw_err': 'controller.yaw_err',
        'yaw_rate_err': 'controller.yaw_rate_err',

        'pitch_response': 'controller.pitch_response',
        'roll_response': 'controller.roll_response',
        'yaw_response': 'controller.yaw_response',
        
        'navigation_time': 'controller.navigation_time',
        'guidance_time': 'controller.guidance_time',
        'damper_time': 'controller.damper_time',
        'response_time': 'controller.response_time'
        }
        
    def __init__(self, sim=None):
        if sim:
            self.attach_sim(sim)
        else: 
            self.sim = None
            self.aircraft = None
            self.guidance_alpha = 0.0
            self.guidance_theta = 0.0
        
        self.update_time_delta = 0.1 #sec
        self.time = 0.0 # independent clock

        nav_sys = {'func': self.intercept, 'freq': 1, 'last_called': None}
        guidance_sys = {'func': self.guidance, 'freq': 2, 'last_called': None}
        damper_sys = {'func': self.dampers2, 'freq': 20, 'last_called': None}
        response_sys = {'func': self.response, 'freq': 20, 'last_called': None}
        sys = {0: nav_sys,
               1: guidance_sys,
               2: damper_sys,
               3: response_sys}
               
        self.timer = FrequencyTimer(sys)
        
        self.navigation_time = 0.0
        self.guidance_time = 0.0
        self.damper_time = 0.0
        self.response_time = 0.0

        self.flightplan = None
        self.waypoint_heading = None
        self.waypoint_altitude = None
        #for logging
        self.pitch_correction = 0.0
        self.roll_correction = 0.0
        self.yaw_correction = 0.0
        self.thrust_correction = 0.0
        
        self.climb_to_climb_rate = 0.3 #bigger
        self.roll_to_roll_rate = 0.00001 #smaller
        self.heading_to_roll = 1.0 # made smaller changed sign
        
        self.guidance_distance = 100.0
        self.guidance_climb_limit = 1.0
        self.guidance_climb_dot_limit = 2.0
        self.guidance_heading_limit = np.deg2rad(10) 
        self.guidance_roll_limit = np.deg2rad(15) # clips the guidance roll
        self.guidance_roll_dot_limit = np.deg2rad(0.5) # clips guidance roll dot

        self.guidance_roll = 0.0
        self.guidance_roll_dot = 0.0
        self.guidance_heading = 0.0
        self.guidance_climb = 0.0
        self.guidance_climb_dot = 0.0
        self.pitch_response = 0.0
        self.roll_response = 0.0
        self.yaw_response = 0.0
              
        self.pitch_err = 0.0
        self.pitch_rate_err = 0.0
        self.roll_err = 0.0
        self.roll_rate_err = 0.0
        self.yaw_err = 0.0
        self.yaw_rate_err = 0.0

        pitch_p = 0.0001
        pitch_d = 0.0001
        self.pitch_pd = pd_signal(pitch_p, pitch_d)

        roll_p = -0.0001
        roll_d = -0.009
        self.roll_pd = pd_signal(roll_p, roll_d)

        roll_yaw_p = -0.01
        roll_yaw_d = -0.001
        self.roll_yaw_pd = pd_signal(roll_yaw_p, roll_yaw_d)

        yaw_p = -0.01
        yaw_d = -0.001
        self.yaw_pd = pd_signal(yaw_p, yaw_d)
        
        thrust_p = 0.01
        thrust_d = 0.005
        self.thrust_pd = pd_signal(thrust_p, thrust_d)
        
   
    def update(self):
        return self.controls
        
    def guidance(self):
        u, v, w, p, q, r, theta, phi, psi, x, y, z = self.sim.system.state_vector
        u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, theta_dot, phi_dot, psi_dot, x_dot, y_dot, z_dot = self.sim.system.state_vector_dot

        direction = np.array([u, v, w]) / np.linalg.norm(np.array([u, v, w]))
        climb_angle = np.arctan2(direction[2], direction[0])        
                
        guidance_climb_angle = np.arctan2(-z - self.waypoint_altitude, self.guidance_distance)
        climb_angle_err = climb_angle - guidance_climb_angle
        self.guidance_climb = np.clip(climb_angle_err, -self.guidance_climb_limit, self.guidance_climb_limit)
        self.guidance_climb_dot = np.clip(self.guidance_climb * self.climb_to_climb_rate, -self.guidance_climb_dot_limit, self.guidance_climb_dot_limit)
        
        vel_heading = np.arctan2(y_dot, x_dot) # world frame
        heading_err = -trim_heading(self.waypoint_heading - vel_heading)        
        self.guidance_heading = np.clip(heading_err, -self.guidance_heading_limit, self.guidance_heading_limit)
        
        desired_roll = np.clip(heading_err * -self.heading_to_roll, -self.guidance_roll_limit, self.guidance_roll_limit)
        self.guidance_roll = np.clip(desired_roll - self.sim.system.full_state.attitude.phi, -self.guidance_roll_limit, self.guidance_roll_limit)

        desired_roll_dot = (self.guidance_roll - phi) * self.roll_to_roll_rate
        self.guidance_roll_dot = np.clip(desired_roll_dot , -self.guidance_roll_dot_limit, self.guidance_roll_dot_limit)
        
        return self.guidance_climb, self.guidance_roll, self.guidance_climb_dot, self.guidance_roll_dot

    def dampers2(self):
        """
        take guidance positions
        return theta/phi errors rate errors (rads and rads/sec)
        
        """
#        vel_climb = -np.arctan2(self.sim.system.full_state.velocity.w, self.sim.system.full_state.velocity.u)
#        self.pitch_err = vel_climb - self.guidance_climb
        
        self.pitch_err = self.guidance_alpha - self.sim.system.full_state.attitude.theta
        self.pitch_rate_err = (self.pitch_err * self.climb_to_climb_rate) - self.sim.system.full_state.angular_vel.q
        
        self.roll_err = self.guidance_roll - self.sim.system.full_state.attitude.phi
        self.roll_rate_err = self.guidance_roll_dot - self.sim.system.full_state.angular_vel.p
        
        self.pitch_response = self.pitch_pd(self.pitch_err, self.pitch_rate_err)
        self.roll_response = self.roll_pd(self.roll_err, self.roll_rate_err)
        
        roll_yaw_response = self.roll_yaw_pd(self.roll_err, self.roll_rate_err)        
        yaw_response = self.yaw_pd(-self.guidance_heading, 0.0)
        
        self.yaw_response = roll_yaw_response + yaw_response
        
    def rate_dampers(self):
        """
        take guidance positions
        return theta/phi errors rate errors (rads and rads/sec)
        
        """
#        vel_climb = -np.arctan2(self.sim.system.full_state.velocity.w, self.sim.system.full_state.velocity.u)
#        self.pitch_err = vel_climb - self.guidance_climb
        
        self.pitch_err = self.guidance_alpha - self.sim.system.full_state.attitude.theta
        self.pitch_rate_err = (self.pitch_err * self.pitch_to_pitch_rate) - self.sim.system.full_state.angular_vel.q
        
        self.roll_err = self.guidance_roll - self.sim.system.full_state.attitude.phi
        self.roll_rate_err = self.guidance_roll_dot - self.sim.system.full_state.angular_vel.p
        
        self.pitch_response = self.pitch_pd(self.pitch_err, self.pitch_rate_err)
        self.roll_response = self.roll_pd(self.roll_err, self.roll_rate_err)
        
    def response(self):
        self.pitch_correction = -self.pitch_response
        self.roll_correction = self.roll_response
        self.yaw_correction = self.yaw_response
        
        self.controls['delta_elevator'] += self.pitch_correction
        self.controls['delta_aileron'] += self.roll_correction
        self.controls['delta_rudder'] += self.yaw_correction        
        
        
    def attach_sim(self, sim):
        if type(sim) == type(None):
            self.sim = None
            sim._save_vars.update(self._save_vars)
            self.control_limits = None
            self.aircraft = None
            self.guidance_alpha = 0.05
            self.guidance_theta = 0.0
        else:
            self.sim = sim
            sim._save_vars.update(self._save_vars)
            self.control_limits = sim.aircraft.control_limits
            self.aircraft = sim.aircraft        
            self.guidance_alpha = sim.aircraft.alpha
            self.guidance_theta = sim.system.full_state.attitude.theta
        
    def control_deltas(self, controls):
        return {key:self.trimmed_controls[key] - controls[key] for key in list(controls)}
    
    def null_update(self, time):
        return self.controls

    def trim_controls(self, ctls=None):
        if not ctls:
            ctls = self.controls
        for ctl in list(ctls):
            if ctl in list(self.sim.aircraft.control_limits):
                ctls[ctl] = np.clip(ctls[ctl], *self.sim.aircraft.control_limits[ctl])
        return ctls
    def energy_monitor(self):
        """
        sets guidance alpha based on:
            target speed / altitude
        
                
        
        force equations
        L D Y
        """
        default_alpha = 0.01
        
        self.guidance_alpha = default_alpha        
    def intercept(self):
        nom_speed = 65.0
        t_delta = 1.0
        correction = 0.8
        err = 2 * t_delta
        t = self.sim.system.time
        
        if type(self.t_interc) == type(None):
            dist = self.target.distance_from(self.sim.system.full_state.position.x_earth, self.sim.system.full_state.position.y_earth, t)
            self.t_interc = dist / nom_speed
#            
#        while err > t_delta:
#            dist_interc = self.target.distance_from(self.sim.system.full_state.position.x_earth, self.sim.system.full_state.position.y_earth, t + self.t_interc)
#            t_interc_ = dist_interc / nom_speed
#            err = self.t_interc - t_interc_
#            self.t_interc -= err * correction
        
        self.target_x , self.target_y, self.target_alt = self.target.position(t)
        self.x_interc, self.y_interc, self.alt_interc = self.target.position(self.t_interc)
        
    def target2body(self):        
        pos_interc = self.target.position(self.t_interc)
#        hdg = self.target.heading_from(self.sim.system.full_state.position.x_earth, self.sim.system.full_state.position.y_earth, self.t_interc)
        pos_interc[2] *= -1 # z_earth points down        
        vel = self.target.velocity._vel_NED        
        displace = pos_interc - self.sim.system.full_state.position.earth_coordinates
        target_vel = hor2body(vel, *self.sim.system.full_state.attitude.euler_angles)                
        target_pos = hor2body(displace, *self.sim.system.full_state.attitude.euler_angles)
        return target_pos, target_vel
        



        
class FlightplanAutopilot(Autopilot):    

        
    def __init__(self, trimmed_controls, sim=None):
        super().__init__(sim)

        self.trimmed_controls = trimmed_controls
        self.controls = trimmed_controls
        self.control_names = list(trimmed_controls)
        
        #setup
        self.ground_level = 500.0
        self.takeoff_speed = 60.0
        
        self.mode = 'takeoff'
        
        
        pitch_p = 0.05   # bigger
        pitch_d = 0.01   # smaller
        self.pitch_pd = pd_signal(pitch_p, pitch_d)

        roll_p = 0.001
        roll_d = 0.03 # bigger
        self.roll_pd = pd_signal(roll_p, roll_d)

        roll_yaw_p = 0.00
        roll_yaw_d = 0.00
        self.roll_yaw_pd = pd_signal(roll_yaw_p, roll_yaw_d)

        yaw_p = 0.01
        yaw_d = 0.01
        self.yaw_pd = pd_signal(yaw_p, yaw_d)
        
        thrust_p = 0.01
        thrust_d = 0.005
        self.thrust_pd = pd_signal(thrust_p, thrust_d)
        

        self.climb_to_climb_rate = 0.001
        self.roll_to_roll_rate = 0.00001 #smaller
        self.heading_to_roll = 1.2 # made bigger
        
        self.guidance_climb_limit = np.deg2rad(5)
        self.guidance_climb_dot_limit = np.deg2rad(1.0)
        self.guidance_heading_limit = np.deg2rad(10)
        self.guidance_roll_limit = np.deg2rad(20) # clips the chnage to guidance roll
        self.guidance_roll_dot_limit = np.deg2rad(1.0)
#        self.nominal_energy = self.system_energy()
#        self.current_energy = self.nominal_energy

        self.alpha = 0.05 #rolling (actual) alpha

    def intercept(self):
        print('flightplan running')
        guidance_distance = 50.0
        self.check_waypoint()        
        pt = self.flightplan.waypoints[self.flightplan.active_waypoint]
#        dist = pt.distance_from(self.sim.system.full_state.position.x_earth, self.sim.system.full_state.position.y_earth)
        u, v, w, p, q, r, theta, phi, psi, x, y, z = self.sim.system.state_vector
        u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, theta_dot, phi_dot, psi_dot, x_dot, y_dot, z_dot = self.sim.system.state_vector_dot
        self.waypoint_heading = pt.heading_from(self.sim.system.full_state.position.x_earth, self.sim.system.full_state.position.y_earth)
        self.waypoint_altitude = pt.height

    def check_waypoint(self):
        i = self.flightplan.active_waypoint
        dist = self.flightplan.waypoints[i].distance_from(self.sim.system.full_state.position.x_earth, self.sim.system.full_state.position.y_earth)
        if dist < self.flightplan.waypoint_radius:
            print('within ', dist, ' of waypoint ', i)
            self.flightplan.active_waypoint += 1
            print('###############################################')
            print('updated to waypoint ', self.flightplan.active_waypoint)
            print('###############################################')
        
    def update(self, time):
#        if time < self.time + self.update_time_delta:
#            return self.controls
        self.time = time
        print('updating controller for time: ', time)
#        energy_error, energy_error_dot = self.energy_monitor()
        times = self.timer.update(time, measure=False)
        self.navigation_time, self.guidance_time, self.damper_time, self.response_time = times
#        self.intercept() #sets waypoint vars
#        self.guidance() # sets guidance vars
#        self.dampers2() # pitch roll errors rate errors to unitless momentum ratio
#        self.response() # momentum ratio to control delta

        self.controls = self.trim_controls()
        return self.controls

    def system_energy(self):
        ke = (self.sim.aircraft.mass * self.sim.aircraft.TAS**2) / 2 #m * v
        pe = self.sim.aircraft.mass * (-self.sim.system.full_state.position.z_earth - self.ground_level) * GRAVITY #m * h
        return  ke + pe
        
    def energy_monitor(self):
        self.current_energy = self.system_energy()
        energy_error = (self.current_energy - self.nominal_energy) / self.nominal_energy
        error_dot = 0 # not implemented
        return energy_error, error_dot

     

class CoefAutopilot(Autopilot):
    """
    in this controller
    waypoint vars are determined at slower rate    


    outer loop is a function of target pos and velocity
        determine guidance theta and alpha
            energy monitor

        determine guidance phi
            from waypoint heading


            the inner loop (dampers) is a function of guidance angular position
                angular error and angular velocity error determined by input
                
                angular error and angular velocity error determined correction (ang acc)
                
                translate ang acc to control deltas
                
                outputs control deltas
    
    """
    _save_vars = {
        'dq_delev': 'controller.dq_delev',
        'dp_dail': 'controller.dp_dail',

        'target_x': 'controller.target_x',
        'target_y': 'controller.target_y',
        'target_alt': 'controller.target_alt',
        
        't_interc': 'controller.t_interc',        
        'x_interc': 'controller.x_interc',        
        'y_interc': 'controller.y_interc',        
        'alt_interc': 'controller.alt_interc'
        }
        
    def __init__(self, trimmed_controls, sim=None):
        super().__init__(sim)
        self.trimmed_controls = trimmed_controls
        self.controls = trimmed_controls
        self.control_names = list(trimmed_controls)

        pitch_p = -0.000005
        pitch_d = -0.00005
        self.pitch_pd = pd_signal(pitch_p, pitch_d)
        
#        roll_p = 0.00000000001
#        roll_d = 0.000001
        roll_p = 0.00000001
        roll_d = 0.0000001 # made smaller
        self.roll_pd = pd_signal(roll_p, roll_d)

        self.dq_delev = 0.0
        self.dp_dail = 0.0

        self.target = Target()
        self.target_x = None
        self.target_y = None
        self.target_alt = None
        
        self.t_interc = None
        self.x_interc = None
        self.y_interc = None
        self.alt_interc = None        
    def est_elev(self):
        control_delta = abs(self.pitch_correction)
        a = 0.3
        cur = self.controls['delta_elevator']
        dq_delev = (self.aircraft._calculate_control_lon_moments_coeffs(cur - control_delta) -\
        self.aircraft._calculate_control_lon_moments_coeffs(cur)) / self.sim.aircraft.inertia[1, 1]
        if dq_delev == 0:
            print('-skipping-', cur)
            dq_delev = 0.000001
        self.dq_delev = (self.dq_delev) * (1.0 - a) + (dq_delev) * a
        return self.dq_delev
        
    def est_ail(self):
        control_delta = abs(self.roll_correction)
        a = 0.3
        cur = self.controls['delta_aileron']
        dp_dail = (self.aircraft._calculate_control_lat_moments_coeffs(cur + control_delta) -\
        self.aircraft._calculate_control_lat_moments_coeffs(cur)) / self.sim.aircraft.inertia[0, 0]
        if dp_dail == 0:
            dp_dail = 0.000001
        self.dp_dail = (self.dp_dail) * (1.0 - a) + (dp_dail) * a
        return self.dp_dail
        
    def response(self):
        self.pitch_correction = self.pitch_response / self.est_elev()
        self.roll_correction = self.roll_response / self.est_ail()        
#        self.pitch_correction = np.clip(self.pitch_pd(self.pitch_err, self.pitch_rate_err), -pitch_correction_limit, pitch_correction_limit)
#        self.roll_correction = np.clip(self.roll_pd(self.roll_err, self.roll_rate_err), -roll_correction_limit, roll_correction_limit)
        




    def update(self, time):
        if time < self.time + self.update_time_delta:
            return self.controls
        self.time = time
        print('updating controller for time: ', time)        
        controls0 = copy.deepcopy(self.controls)
        
        body_CL = self.aircraft._calculate_body_lat_moments_coeffs()
        body_CM = self.aircraft._calculate_body_lon_moments_coeffs()
        body_moments = np.array([body_CL, body_CM])
        
        ctl_CL = self.aircraft._calculate_control_lat_moments_coeffs(controls0['delta_aileron'])
        ctl_CM = self.aircraft._calculate_control_lon_moments_coeffs(controls0['delta_elevator'])
        ctl_moments = np.array([ctl_CL, ctl_CM])
        total_moments = body_moments + ctl_moments
        
                
        self.intercept() #sets waypoint vars
        
        # world frame
        self.waypoint_heading = np.arctan2(self.y_interc - self.sim.system.full_state.position.y_earth, self.x_interc - self.sim.system.full_state.position.x_earth)
        print('wpt hdg:', self.waypoint_heading)
        
        # body frame
        self.guidance() # sets guidance vars
#        self.control



        self.rate_dampers() # pitch roll errors rate errors to unitless momentum ratio
        
        guidance_moments = np.array([self.roll_response, self.pitch_response])
        err_omega_dot = total_moments - guidance_moments
        print('err moments : ', err_omega_dot)
        
        self.response() # momentum ratio to control delta


        self.controls['delta_elevator'] += self.pitch_correction
        self.controls['delta_aileron'] += self.roll_correction
        
        self.controls = self.trim_controls()
        return self.controls

class LeastSquaresAutopilot(Autopilot):
    _save_vars = {
        'target_x': 'controller.target_x',
        'target_y': 'controller.target_y',
        'target_alt': 'controller.target_alt',
        
        't_interc': 'controller.t_interc',        
        'x_interc': 'controller.x_interc',        
        'y_interc': 'controller.y_interc',        
        'alt_interc': 'controller.alt_interc',        
        
        'waypoint_heading': 'controller.waypoint_heading',
        'waypoint_altitude': 'controller.waypoint_altitude',
        
        'guidance_roll_dot': 'controller.guidance_roll_dot',
        'guidance_roll': 'controller.guidance_roll',
        'guidance_heading': 'controller.guidance_heading',
        'guidance_climb': 'controller.guidance_climb',
        'guidance_climb_dot': 'controller.guidance_climb_dot',
        
        'pitch_correction': 'controller.pitch_correction',
        'roll_correction': 'controller.roll_correction',
        'yaw_correction': 'controller.yaw_correction',
        'thrust_correction': 'controller.thrust_correction',
        'pitch_err': 'controller.pitch_err',
        'pitch_rate_err': 'controller.pitch_rate_err',
        'roll_err': 'controller.roll_err',
        'roll_rate_err': 'controller.roll_rate_err',
        'yaw_err': 'controller.yaw_err',
        'yaw_rate_err': 'controller.yaw_rate_err'}
        
    def __init__(self, trimmed_controls, sim=None):
        super().__init__(sim)
        self.trimmed_controls = trimmed_controls
        self.controls = trimmed_controls
        self.control_names = list(trimmed_controls)

        pitch_p = 0.005
        pitch_d = 0.02
        self.pitch_pd = pd_signal(pitch_p, pitch_d)

        roll_p = -0.001
        roll_d = -0.001
        self.roll_pd = pd_signal(roll_p, roll_d)

        self.guidance_roll = 0.0
        self.guidance_roll_dot = 0.0
        self.guidance_heading = 0.0
        self.guidance_climb = 0.0
        self.guidance_climb_dot = 0.0
        self.waypoint_heading = 0.0
        self.waypoint_altitude = 0.0
        
        # for logging
        self.target = Target()
        self.target_x = None
        self.target_y = None
        self.target_alt = None
        
        self.t_interc = None
        self.x_interc = None
        self.y_interc = None
        self.alt_interc = None
        
    def update(self, time):
        if time < self.time + self.update_time_delta:
            return self.controls
        self.time = time
        print('updating controller for time: ', time)        
        
        verbose = 0
        exclude = []
        gamma = 0.0
        turn_rate = 0.0
        
        controls0 = copy.deepcopy(self.controls)
        state0 = copy.deepcopy(self.sim.system.full_state)
    
        environment0 = copy.deepcopy(self.sim.environment)
        aircraft0 = copy.deepcopy(self.sim.aircraft)
    
        environment0.update(state0)
    
        system0 = copy.deepcopy(self.sim.system)
    
        alpha0 = aircraft0.alpha
        beta0 = aircraft0.beta
        TAS0 = aircraft0.TAS
        aircraft0._calculate_aerodynamics_2(TAS0, alpha0, beta0, environment0)
    
        for control in aircraft0.controls:
            if control not in controls0:
                raise ValueError(
                    "Control {} not given in initial_controls: {}".format(
                        control, controls0)
                )
            else:
                aircraft0.controls[control] = controls0[control]

        controls_to_trim = list(aircraft0.controls.keys() - exclude)

        initial_guess = [alpha0, beta0]
        for control in controls_to_trim:
            initial_guess.append(controls0[control])

        lower_bounds = [-0.5, -0.25]  # Alpha and beta lower bounds.
        upper_bounds = [+0.5, +0.25]  # Alpha and beta upper bounds.
        for ii in controls_to_trim:
            lower_bounds.append(aircraft0.control_limits[ii][0])
            upper_bounds.append(aircraft0.control_limits[ii][1])
        bounds = (lower_bounds, upper_bounds)
    
        args = (system0, aircraft0, environment0, controls_to_trim, gamma, turn_rate)
    
        results = least_squares(trimming_cost_func,
                                x0=initial_guess,# alpha beta controls
                                args=args,
                                verbose=verbose,
                                bounds=bounds)
    
        # Residuals: last trim_function evaluation
#        u_dot, v_dot, w_dot, p_dot, q_dot, r_dot = results.fun
    
#        att = system.full_state.attitude
#        system.full_state.acceleration.update([u_dot, v_dot, w_dot], att)
#        system.full_state.angular_accel.update([p_dot, q_dot, r_dot], att)
    
        trimmed_controls = controls0
        for key, val in zip(controls_to_trim, results.x[2:]):
            trimmed_controls[key] = val
        self.controls = trimmed_controls
        print('controls == ', time, ' -- ', trimmed_controls)
        return self.controls

class FLSAutopilot(Autopilot):
    _save_vars = {
        'target_x': 'controller.target_x',
        'target_y': 'controller.target_y',
        'target_alt': 'controller.target_alt',
        
        't_interc': 'controller.t_interc',        
        'x_interc': 'controller.x_interc',        
        'y_interc': 'controller.y_interc',        
        'alt_interc': 'controller.alt_interc',        
        
        'waypoint_heading': 'controller.waypoint_heading',
        'waypoint_altitude': 'controller.waypoint_altitude',
        
        'guidance_roll_dot': 'controller.guidance_roll_dot',
        'guidance_roll': 'controller.guidance_roll',
        'guidance_heading': 'controller.guidance_heading',
        'guidance_climb': 'controller.guidance_climb',
        'guidance_climb_dot': 'controller.guidance_climb_dot',
        
        'pitch_correction': 'controller.pitch_correction',
        'roll_correction': 'controller.roll_correction',
        'yaw_correction': 'controller.yaw_correction',
        'thrust_correction': 'controller.thrust_correction',
        'pitch_err': 'controller.pitch_err',
        'pitch_rate_err': 'controller.pitch_rate_err',
        'roll_err': 'controller.roll_err',
        'roll_rate_err': 'controller.roll_rate_err',
        'yaw_err': 'controller.yaw_err',
        'yaw_rate_err': 'controller.yaw_rate_err'}
        
    def __init__(self, trimmed_controls, sim=None):
        super().__init__(sim)
        self.trimmed_controls = trimmed_controls
        self.controls = trimmed_controls
        self.control_names = list(trimmed_controls)

        pitch_p = 0.005
        pitch_d = 0.02
        self.pitch_pd = pd_signal(pitch_p, pitch_d)

        roll_p = -0.001
        roll_d = -0.001
        self.roll_pd = pd_signal(roll_p, roll_d)

        self.guidance_roll = 0.0
        self.guidance_roll_dot = 0.0
        self.guidance_heading = 0.0
        self.guidance_climb = 0.0
        self.guidance_climb_dot = 0.0
        self.waypoint_heading = 0.0
        self.waypoint_altitude = 0.0
        

#        self.lead_distance = 100.0
    def update(self, time):
        if time < self.time + self.update_time_delta:
            return self.controls
        self.time = time
        print('updating controller for time: ', time)

        self.intercept()
        self.guidance()
        self.rate_dampers()
        
        self.controls['delta_elevator'] += self.pitch_correction
        self.controls['delta_aileron'] += self.roll_correction
        self.controls['delta_t'] = self.thrust(time)

        self.trim_controls()
        return self.controls

    def intercept(self):
        nom_speed = 65.0
        t_delta = 1.0
        correction = 0.8
        err = 2 * t_delta
        t = self.sim.system.time
        
        if type(self.t_interc) == type(None):
            dist = self.target.distance_from(self.sim.system.full_state.position.x_earth, self.sim.system.full_state.position.y_earth, t)
            self.t_interc = dist / nom_speed        
        while err > t_delta:
            dist_interc = self.target.distance_from(self.sim.system.full_state.position.x_earth, self.sim.system.full_state.position.y_earth, t + self.t_interc)
            t_interc_ = dist_interc / nom_speed
            err = self.t_interc - t_interc_
            self.t_interc -= err * correction
        
        self.target_x , self.target_y, self.target_alt = self.target.position(t)
        self.x_interc, self.y_interc, self.alt_interc = self.target.position(self.t_interc)
        
    def guidance(self):
        guidance_distance = 100.0
        
        guidance_climb_step = np.deg2rad(.5)
        guidance_climb_limit = np.deg2rad(1)
        guidance_climb_rate_limit = np.deg2rad(0.04)
        climb_to_climb_rate = 0.001
        
#        roll_correction_limit = 0.0001 # clips the chnage to guidance roll
#        roll_limit = np.deg2rad(30) # clips desired roll
        roll_rate_limit = np.deg2rad(0.50) # clips guidance roll dot
        roll_to_roll_rate = 0.3  
        
        u, v, w, p, q, r, theta, phi, psi, x, y, z = self.sim.system.state_vector
        u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, theta_dot, phi_dot, psi_dot, x_dot, y_dot, z_dot = self.sim.system.state_vector_dot

        target_pos, target_vel = self.target2body()

        
#        target_climb = -np.arctan2(target_pos[2], target_pos[0])
        target_climb = -np.arctan2(target_pos[2], guidance_distance)
        
        vel_climb = -np.arctan2(w, u)
        vel_ang_err = np.clip(target_climb - vel_climb, -guidance_climb_step, guidance_climb_step)
        
        
        self.guidance_climb = np.clip(vel_ang_err, -guidance_climb_limit, guidance_climb_limit)
        
        
#        dif = np.clip(climb_err - self.guidance_climb, -guidance_climb_step, guidance_climb_step)
#
#        self.guidance_climb = np.clip(self.sim.system.full_state.attitude.theta + dif, -guidance_climb_limit, guidance_climb_limit)
#        
        desired_climb_dot = (self.guidance_climb - vel_climb) * climb_to_climb_rate
        self.guidance_climb_dot = np.clip(desired_climb_dot , -guidance_climb_rate_limit, guidance_climb_rate_limit)

        print('guidance ', self.guidance_roll)
        print('climb ', self.guidance_climb)

        self.guidance_roll = 0.0
        desired_roll_dot = (self.guidance_roll - phi) * roll_to_roll_rate
        self.guidance_roll_dot = np.clip(desired_roll_dot , -roll_rate_limit, roll_rate_limit)
        
#        return self.guidance_climb, self.guidance_roll, self.guidance_climb_dot, self.guidance_roll_dot

    def thrust(self, t):
        return 1.0

    def rate_dampers(self):
        roll_correction_limit = 0.003
        pitch_correction_limit = 0.01
        yaw_correction_limit = 0.001
        
        vel_climb = -np.arctan2(self.sim.system.full_state.velocity.w, self.sim.system.full_state.velocity.u)
        self.pitch_err = vel_climb - self.guidance_climb
        
#        self.pitch_err = self.sim.system.full_state.attitude.theta - self.guidance_climb
#        self.pitch_rate_err = self.sim.system.full_state.angular_vel.q - self.guidance_climb_dot
        
        self.roll_err = self.sim.system.full_state.attitude.phi - self.guidance_roll
        self.roll_rate_err = self.sim.system.full_state.angular_vel.p - self.guidance_roll
        
        self.pitch_correction = np.clip(self.pitch_pd(self.pitch_err, self.pitch_rate_err), -pitch_correction_limit, pitch_correction_limit)
        self.roll_correction = np.clip(self.roll_pd(self.roll_err, self.roll_rate_err), -roll_correction_limit, roll_correction_limit)
        
#        return self.pitch_correction, self.roll_correction
        
        
    def read_sensor(self):
        cur_pos = self.target.position(self.sim.system.time)
        x, y, z = hor2body(cur_pos, *self.sim.system.full_state.attitude._euler_angles)
        offset = np.linalg.norm(np.array([y, z]))        
        roll_angle = np.arctan2(-z, y)
#        climb_angle = np.arctan2(offset, x)
        climb_angle = np.arctan2(z, x)
        
        print('roll: ', roll_angle)
        print('climb: ', climb_angle)
         
def turn_coord_cons(turn_rate, alpha, beta, TAS, gamma=0):
    g0 = GRAVITY
    G = turn_rate * TAS / g0
    if abs(gamma) < 1e-8:
        phi = G * np.cos(beta) / (np.cos(alpha) - G * np.sin(alpha) * np.sin(beta))
        phi = np.arctan(phi)
    else:
        a = 1 - G * np.tan(alpha) * np.sin(beta)
        b = np.sin(gamma) / np.cos(beta)
        c = 1 + G ** 2 * np.cos(beta) ** 2
        sq = np.sqrt(c * (1 - b ** 2) + G ** 2 * np.sin(beta) ** 2)
        num = (a - b ** 2) + b * np.tan(alpha) * sq
        den = a ** 2 - b ** 2 * (1 + c * np.tan(alpha) ** 2)
        phi = np.arctan(G * np.cos(beta) / np.cos(alpha) * num / den)
    return phi      
    
def rate_of_climb_cons(gamma, alpha, beta, phi):
    """Calculates theta for the given ROC, wind angles, and roll angle.
    """
    a = np.cos(alpha) * np.cos(beta)
    b = np.sin(phi) * np.sin(beta) + np.cos(phi) * np.sin(alpha) * np.cos(beta)
    sq = np.sqrt(a ** 2 - np.sin(gamma) ** 2 + b ** 2)
    theta = (a * b + np.sin(gamma) * sq) / (a ** 2 - np.sin(gamma) ** 2)
    theta = np.arctan(theta)
    return theta

#    def response(self, state_vector, inertia, forces, moments):
#        Ix = inertia[0, 0]
#        Iy = inertia[1, 1]
#        Iz = inertia[2, 2]
#        Jxz = - inertia[0, 2]
#    
#        Fx, Fy, Fz = forces
#        L, M, N = moments
#    
#        u, v, w = state_vector[0:3]
#        p, q, r = state_vector[3:6]
#        
#        dp_dt = (L * Iz + N * Jxz - q * r * (Iz ** 2 - Iz * Iy + Jxz ** 2) +
#                 p * q * Jxz * (Ix + Iz - Iy)) / (Ix * Iz - Jxz ** 2)
#        dq_dt = (M + (Iz - Ix) * p * r - Jxz * (p ** 2 - r ** 2)) / Iy
#        dr_dt = (L * Jxz + N * Ix + p * q * (Ix ** 2 - Ix * Iy + Jxz ** 2) -
#                 q * r * Jxz * (Iz + Ix - Iy)) / (Ix * Iz - Jxz ** 2)
#        return np.array([dp_dt, dq_dt, dr_dt])