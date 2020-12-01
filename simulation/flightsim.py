#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 16 18:32:36 2020

@author: pi
"""
import copy
import operator
import pandas as pd
from simulation.simulator import Simulation

class TakeoffSim(Simulation):
    default_save_vars = {
        'time': 'system.time',
        'wind_x': 'environment.wind.x',
        'wind_y': 'environment.wind.y',
        'wind_z': 'environment.wind.z',
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

        'u': 'system.full_state.velocity.u',
        'v': 'system.full_state.velocity.v',
        'w': 'system.full_state.velocity.w',
#
#        'v_north': 'system.full_state.velocity.v_north',
#        'v_east': 'system.full_state.velocity.v_east',
#        'v_down': 'system.full_state.velocity.v_down',

        'p': 'system.full_state.angular_vel.p',
        'q': 'system.full_state.angular_vel.q',
        'r': 'system.full_state.angular_vel.r'
    }
    
    def __init__(self, aircraft, system, environment, controller, dt=0.01):
        self.system = copy.deepcopy(system)
        self.aircraft = copy.deepcopy(aircraft)
        self.environment = copy.deepcopy(environment)
        self.system.update_simulation = self.update
        self.dt = dt
        self._save_vars = self.default_save_vars

        self.controller = controller
        self.controller.attach_sim(self)
        
        self.results = {name: [] for name in self._save_vars}

    @property
    def time(self):
        return self.system.time

    def update(self, time, state):
        self.environment.update(state)
#        controls = self._get_current_controls(time)
        self.aircraft.calculate_forces_and_moments(
            state,
            self.environment,
            self.controller.controls
        )
        return self

    def propagate(self, time):
        dt = self.dt
        half_dt = self.dt/2

        time_plus_half_dt = time + half_dt
        print('running..')
        while self.system.time + dt < time_plus_half_dt:
            t = self.system.time
            
            self.environment.update(self.system.full_state)
            self.environment.wind.update(self.system.full_state, t) # redundant

            if time > self.controller.time + self.controller.update_time_delta:
                self.controller.update(t)
                self.controller.time = time

            cur_controls = self.controller.controls
            self.aircraft._set_current_controls(cur_controls)
#            print('--', self.aircraft.controls)
#            print('--', cur_controls)
            self.aircraft.calculate_forces_and_moments(self.system.full_state,
                                                       self.environment, cur_controls)
            self.system.time_step(dt)

            self._save_time_step()

        results = pd.DataFrame(self.results)
        results.set_index('time', inplace=True)

        return results

    def _save_time_step(self):
        for var_name, value_pointer in self._save_vars.items():
            self.results[var_name].append(
                operator.attrgetter(value_pointer)(self)
            )

    def _get_current_controls(self, time):
        return self.controller.controls
