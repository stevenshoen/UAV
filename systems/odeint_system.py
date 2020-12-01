# -*- coding: utf-8 -*-

from abc import abstractmethod

import numpy as np
from numpy import sin, cos

from scipy.integrate import odeint

from state.aircraft_state import AircraftState
from state.position import EarthPosition
from state.attitude import EulerAttitude
from state.state import (
    BodyVelocity,
    BodyAngularVelocity, BodyAcceleration, BodyAngularAcceleration
)



class OdeIntSystem(object):

    def __init__(self, t0, full_state, method='RK45', options=None):
        
        self.min_theta = full_state.attitude.theta # ground-wheel effect
        self.z_ground = full_state.position.z_earth
        self._time = t0
        
        x0 = self._get_state_vector_from_full_state(full_state)
        self.full_state = self._adapt_full_state_to_dynamic_system(full_state)

        self.update_simulation = None
        if options is None:
            options = {}
        self._state_vector = x0
        self._state_vector_dot = np.zeros_like(x0)


        self._method = method
        self._options = options
                
        
    @property
    def state_vector(self):
        return self._state_vector

    @property
    def state_vector_dot(self):
        return self._state_vector_dot



    @property
    def time(self):
        return self._time

    def time_step(self, dt):

        x0 = self.state_vector
        t_ini = self.time

        t_span = (t_ini, t_ini + dt)
        method = self._method

        # TODO: prepare to use jacobian in case it is defined
#        sol = solve_ivp(self.fun_wrapped, t_span, x0, method=method,
#                        **self._options)


#        sol = odeint(self.fun_wrapped, x0, t_span, args=self._options)
        sol = odeint(self.fun_wrapped, x0, t_span)
        
#        print('solution:', sol)
    


#        if sol.status == -1:
#            raise RuntimeError("Integration did not converge")
        self._time = t_ini + dt

#        print('sol:', sol)
        
        self._state_vector = np.array(sol[-1])
#        self._time = sol.t[-1]
#        self._state_vector = sol.y[:, -1]
        self._update_full_system_state_from_state(self.state_vector,
                                                  self.state_vector_dot)

        return self.full_state

    @abstractmethod
    def fun(self, x, t):
#
#        print('t:', t)
#
#        print('x:', x)
        
        self._update_full_system_state_from_state(x, self.state_vector_dot)
        updated_simulation = self.update_simulation(t, self.full_state)

        mass = updated_simulation.aircraft.mass
        inertia = updated_simulation.aircraft.inertia
        forces = updated_simulation.aircraft.total_forces
        moments = updated_simulation.aircraft.total_moments

        rv = _system_equations(t, x, mass, inertia, forces, moments)

        return rv
        
    def fun_wrapped(self, x, t):
        
#        print('t:', t)
#
#        print('x:', x)
        state_dot = self.fun(x, t)
        self._state_vector_dot = state_dot
        return state_dot
        
    def check_for_ground(self, state):
        if state[11] > self.z_ground:
            print('ground_collision', self.time)
            state[11] = self.z_ground
            
            if state[6] < self.min_theta:
                state[6] = self.min_theta
#            new_pos = EarthPosition(state.position.x_earth, state.position.y_earth, -self.z_ground)
#                
#            new_att = state.attitude
#            new_vel = BodyVelocity(u=state.velocity.u, v=state.velocity.v, w=0.0, attitude=new_att)
#            
#            state0 = AircraftState(new_pos, new_att, new_vel)
#
#            return state0
#        else:
#            print('no collision', state.position.z_earth)
        return state
        
    def _update_full_system_state_from_state(self, state, state_dot):

        if isinstance(self.z_ground, float):
            state = self.check_for_ground(state)
        
        
        self.full_state.position.update(state[9:12])
        self.full_state.attitude.update(state[6:9])
        att = self.full_state.attitude
        
        
        self.full_state.velocity.update(state[0:3], att)
        self.full_state.angular_vel.update(state[3:6], att)

        self.full_state.acceleration.update(state_dot[0:3], att)
        self.full_state.angular_accel.update(state_dot[3:6], att) 
        
    def _adapt_full_state_to_dynamic_system(self, full_state):

        
#        if isinstance(self.z_ground, float):
#            full_state = self.check_for_ground(full_state)

                
        pos = EarthPosition(full_state.position.x_earth,
                            full_state.position.y_earth,
                            
                            full_state.position.z_earth,
                            
                            full_state.position.lat,
                            full_state.position.lon)

        att = EulerAttitude(full_state.attitude.theta,
                            full_state.attitude.phi,
                            full_state.attitude.psi)

        vel = BodyVelocity(full_state.velocity.u,
                           full_state.velocity.v,
                           full_state.velocity.w,
                           att)

        ang_vel = BodyAngularVelocity(full_state.angular_vel.p,
                                      full_state.angular_vel.q,
                                      full_state.angular_vel.r,
                                      att)

        accel = BodyAcceleration(full_state.acceleration.u_dot,
                                 full_state.acceleration.v_dot,
                                 full_state.acceleration.w_dot,
                                 att)

        ang_accel = BodyAngularAcceleration(full_state.angular_accel.p_dot,
                                            full_state.angular_accel.q_dot,
                                            full_state.angular_accel.r_dot,
                                            att)

        full_state = AircraftState(pos, att, vel, ang_vel, accel, ang_accel)
        return full_state        

    def _get_state_vector_from_full_state(self, full_state):
        x0 = np.array(
            [
                full_state.velocity.u,
                full_state.velocity.v,
                full_state.velocity.w,
                full_state.angular_vel.p,
                full_state.angular_vel.q,
                full_state.angular_vel.r,
                full_state.attitude.theta,
                full_state.attitude.phi,
                full_state.attitude.psi,
                full_state.position.x_earth,
                full_state.position.y_earth,
                full_state.position.z_earth
            ]
        )
        return x0
        
    def steady_state_trim_fun(self, full_state, environment, aircraft,
                              controls):

        environment.update(full_state)
        aircraft.calculate_forces_and_moments(full_state, environment,
                                              controls)

        mass = aircraft.mass
        inertia = aircraft.inertia
        forces = aircraft.total_forces
        moments = aircraft.total_moments

        t0 = 0
        x0 = self._get_state_vector_from_full_state(full_state)

        rv = _system_equations(t0, x0, mass, inertia, forces, moments)
        return rv[:6]


def _system_equations(time, state_vector, mass, inertia, forces, moments):
    Ix = inertia[0, 0]
    Iy = inertia[1, 1]
    Iz = inertia[2, 2]
    Jxz = - inertia[0, 2]

    Fx, Fy, Fz = forces
    L, M, N = moments

    u, v, w = state_vector[0:3]
    p, q, r = state_vector[3:6]
    theta, phi, psi = state_vector[6:9]

    # Linear momentum equations
    du_dt = Fx / mass + r * v - q * w
    dv_dt = Fy / mass - r * u + p * w
    dw_dt = Fz / mass + q * u - p * v

    # Angular momentum equations
    dp_dt = (L * Iz + N * Jxz - q * r * (Iz ** 2 - Iz * Iy + Jxz ** 2) +
             p * q * Jxz * (Ix + Iz - Iy)) / (Ix * Iz - Jxz ** 2)
    dq_dt = (M + (Iz - Ix) * p * r - Jxz * (p ** 2 - r ** 2)) / Iy
    dr_dt = (L * Jxz + N * Ix + p * q * (Ix ** 2 - Ix * Iy + Jxz ** 2) -
             q * r * Jxz * (Iz + Ix - Iy)) / (Ix * Iz - Jxz ** 2)

    # Angular Kinematic equations
    dtheta_dt = q * cos(phi) - r * sin(phi)
    dphi_dt = p + (q * sin(phi) + r * cos(phi)) * np.tan(theta)
    dpsi_dt = (q * sin(phi) + r * cos(phi)) / cos(theta)

    # Linear kinematic equations
    dx_dt = (cos(theta) * cos(psi) * u +
             (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) * v +
             (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * w)
    dy_dt = (cos(theta) * sin(psi) * u +
             (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) * v +
             (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * w)
    dz_dt = -u * sin(theta) + v * sin(phi) * cos(theta) + w * cos(
        phi) * cos(theta)

    return np.array([du_dt, dv_dt, dw_dt, dp_dt, dq_dt, dr_dt, dtheta_dt,
                     dphi_dt, dpsi_dt, dx_dt, dy_dt, dz_dt])


