# -*- coding: utf-8 -*-
"""
Python Flight Mechanics Engine (PyFME).
Copyright (c) AeroPython Development Team.
Distributed under the terms of the MIT License.

Euler Flat Earth
----------------

Classical aircraft motion equations assuming no Earth rotation
inertial effects, representing attitude with Euler angles (not valid for
all-attitude flight) and integrating aircraft position in Earth axis (Flat
Earth).

try using np.linalg.solve(a x + b)

    state_vector : array_like, shape(9)
        Current value of absolute velocity and angular velocity, both
        expressed in body axes, euler angles and position in Earth axis.
        (u, v, w, p, q, r, theta, phi, psi, x, y, z)                                                   

    dstate_dt : array_like, shape(9)
        Derivative with respect to time of the state vector.
        Current value of absolute acceleration and angular acceleration,
        both expressed in body axes, Euler angles derivatives and velocity
        with respect to Earth Axis.
        (du_dt, dv_dt, dw_dt, dp_dt, dq_dt, dr_dt, dtheta_dt, dphi_dt,
        dpsi_dt, dx_dt, dy_dt, dz_dt)



"""

import numpy as np
from numpy import sin, cos

#from src.dynamic_system import AircraftDynamicSystem
from src.aircraft_state import AircraftState
from src.position import EarthPosition
from src.attitude import EulerAttitude
from src.velocity import BodyVelocity
from src.angular_velocity import BodyAngularVelocity
from src.acceleration import BodyAcceleration
from src.angular_acceleration import BodyAngularAcceleration
from src.euler_flat_earth import EulerFlatEarth

#from state import (
#    AircraftState, EarthPosition, EulerAttitude, BodyVelocity,
#    BodyAngularVelocity, BodyAcceleration, BodyAngularAcceleration
#)

class ModSystem(EulerFlatEarth):
#class EulerFlatEarth(AircraftDynamicSystem):
    """Euler Flat Earth Dynamic System.

    Classical aircraft motion equations assuming no Earth rotation, no Earth
    curvature and modelling attitude with Euler angles. Aircraft position is
    performed on Earth axis.
    """
    def __init__(self, t0, full_state, method='RK45', options=None):

        super().__init__(t0, full_state, method=method, options=options)

        self.update_simulation = None
    
    def time_step(self, dt):
        """Integrate the system from current time to t_end.

        Parameters
        ----------
        dt : float
            Time step.

        Returns
        -------
        y : ndarray, shape (n)
            Solution values at t_end.
        """

        x0 = self.state_vector
        t_ini = self.time
        print('time_step:', t_ini, t_ini + dt)
        t_span = (t_ini, t_ini + dt)
        method = self._method

        print('init_state: ', x0)


        # TODO: prepare to use jacobian in case it is defined
        sol = solve_ivp(self.fun_wrapped, t_span, x0, method=method,
                        **self._options)

        if sol.status == -1:
            raise RuntimeError(f"Integration did not converge at t={t_ini}")

        self._time = sol.t[-1]
        self._state_vector = sol.y[:, -1]

        return self._state_vector


    def time_step1(self, dt):
        """Integrate the system from current time to t_end.

        Parameters
        ----------
        dt : float
            Time step.

        Returns
        -------
        y : ndarray, shape (n)
            Solution values at t_end.
        """

        x0 = self.state_vector
        t_ini = self.time

        t_span = (t_ini, t_ini + dt)
        method = self._method

        # TODO: prepare to use jacobian in case it is defined
        sol = solve_ivp(self.fun_wrapped, t_span, x0, method=method,
                        **self._options)

        if sol.status == -1:
            raise RuntimeError(f"Integration did not converge at t={t_ini}")

        self._time = sol.t[-1]
        self._state_vector = sol.y[:, -1]

        return self._state_vector
    
    def fun(self, t, x):

        self._update_full_system_state_from_state(x, self.state_vector_dot)
        updated_simulation = self.update_simulation(t, self.full_state)

        mass = updated_simulation.aircraft.mass
        inertia = updated_simulation.aircraft.inertia
        forces = updated_simulation.aircraft.total_forces
        moments = updated_simulation.aircraft.total_moments

        rv = _system_equations(t, x, mass, inertia, forces, moments)

        return rv

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

    def _update_full_system_state_from_state(self, state, state_dot):

        self.full_state.position.update(state[9:12])
        self.full_state.attitude.update(state[6:9])
        att = self.full_state.attitude
        self.full_state.velocity.update(state[0:3], att)
        self.full_state.angular_vel.update(state[3:6], att)

        self.full_state.acceleration.update(state_dot[0:3], att)
        self.full_state.angular_accel.update(state_dot[3:6], att)

    def _adapt_full_state_to_dynamic_system(self, full_state):

        pos = EarthPosition(full_state.position.x_earth,
                            full_state.position.y_earth,
                            full_state.position.height,
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


# TODO: numba jit
def _system_equations(time, state_vector, mass, inertia, forces, moments):
    """Euler flat earth equations: linear momentum equations, angular momentum
    equations, angular kinematic equations, linear kinematic
    equations.

    Parameters
    ----------
    time : float
        Current time (s).
    state_vector : array_like, shape(9)
        Current value of absolute velocity and angular velocity, both
        expressed in body axes, euler angles and position in Earth axis.
        (u, v, w, p, q, r, theta, phi, psi, x, y, z)
         (m/s, m/s, m/s, rad/s, rad/s rad/s, rad, rad, rad, m, m ,m).
    mass : float
        Current mass of the aircraft (kg).
    inertia : array_like, shape(3, 3)
        3x3 tensor of inertia of the aircraft (kg * m2)
        Current equations assume that the aircraft has a symmetry plane
        (x_b - z_b), thus J_xy and J_yz must be null.
    forces : array_like, shape(3)
        3 dimensional vector containing the total total_forces (including
        gravity) in x_b, y_b, z_b axes (N).
    moments : array_like, shape(3)
        3 dimensional vector containing the total total_moments in x_b,
        y_b, z_b axes (N·m).

    Returns
    -------
    dstate_dt : array_like, shape(9)
        Derivative with respect to time of the state vector.
        Current value of absolute acceleration and angular acceleration,
        both expressed in body axes, Euler angles derivatives and velocity
        with respect to Earth Axis.
        (du_dt, dv_dt, dw_dt, dp_dt, dq_dt, dr_dt, dtheta_dt, dphi_dt,
        dpsi_dt, dx_dt, dy_dt, dz_dt)
        (m/s² , m/s², m/s², rad/s², rad/s², rad/s², rad/s, rad/s, rad/s,
        m/s, m/s, m/s).

    References
    ----------
    .. [1] B. Etkin, "Dynamics of Atmospheric Flight", Courier Corporation,
        p. 149 (5.8 The Flat-Earth Approximation), 2012.

    .. [2] M. A. Gómez Tierno y M. Pérez Cortés, "Mecánica del Vuelo",
        Garceta Grupo Editorial, pp.18-25 (Tema 2: Ecuaciones Generales del
        Moviemiento), 2012.

    """
    # Note definition of total_moments of inertia p.21 Gomez Tierno, et al
    # Mecánica de vuelo
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


            
            
            
            
            
from abc import abstractmethod
from scipy.integrate import solve_ivp


class DynamicSystem:
    """Dynamic system class to integrate initial value problems numerically.
    Serves as base for implementation of dynamic systems.

    Attributes
    ----------
    state_vector : ndarray
        State vector.
    time : float
        Current integration time.

    Methods
    -------
    integrate(t_end, t_eval=None, dense_output=True)
        Integrate the system from current time to t_end.
    fun(t, x)
        Dynamic system equations
    """

    def __init__(self, t0, x0, method='RK45', options=None):
        if options is None:
            options = {}
        self._state_vector = x0
        self._state_vector_dot = np.zeros_like(x0)
        self._time = t0

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

    def integrate(self, t_end, t_eval=None, dense_output=True):
        x0 = self.state_vector
        t_ini = self.time

        t_span = (t_ini, t_end)
        method = self._method

        # TODO: prepare to use jacobian in case it is defined
        sol = solve_ivp(self.fun, t_span, x0, method=method, t_eval=t_eval,
                        dense_output=dense_output, **self._options)

        self._time = sol.t[-1]
        self._state_vector = sol.y[:, -1]

        return sol.t, sol.y, sol

    def time_step(self, dt):
        """Integrate the system from current time to t_end.

        Parameters
        ----------
        dt : float
            Time step.

        Returns
        -------
        y : ndarray, shape (n)
            Solution values at t_end.
        """

        x0 = self.state_vector
        t_ini = self.time

        t_span = (t_ini, t_ini + dt)
        method = self._method

        # TODO: prepare to use jacobian in case it is defined
        sol = solve_ivp(self.fun_wrapped, t_span, x0, method=method,
                        **self._options)

        if sol.status == -1:
            raise RuntimeError(f"Integration did not converge at t={t_ini}")

        self._time = sol.t[-1]
        self._state_vector = sol.y[:, -1]

        return self._state_vector

    @abstractmethod
    def fun(self, t, x):
        raise NotImplementedError

    def fun_wrapped(self, t, x):
        # First way that comes to my mind in order to store the derivatives
        # that are useful for full_state calculation
        state_dot = self.fun(t, x)
        self._state_vector_dot = state_dot
        return state_dot


class AircraftDynamicSystem(DynamicSystem):

    def __init__(self, t0, full_state, method='RK45', options=None):

        x0 = self._get_state_vector_from_full_state(full_state)
        self.full_state = self._adapt_full_state_to_dynamic_system(full_state)

        super().__init__(t0, x0, method=method, options=options)

        self.update_simulation = None

    @abstractmethod
    def _adapt_full_state_to_dynamic_system(self, full_state):
        """Transforms the given state to the one used in the
        AircraftDynamicSystem in order to initialize dynamic's system
        initial state.
        For example, the full state given may be using quaternions for
        attitude representation, but the Aircraft dynamic system may
        propagate Euler angles.

        Parameters
        ----------
        full_state : AircraftState

        """
        raise NotImplementedError

    @abstractmethod
    def _update_full_system_state_from_state(self, state, state_dot):
        """Updates full system's state (AircraftState) based on the
        implemented dynamic's system state vector and derivative of system's
        state vector (output of system's equations dx/dt=f(x, t))

        Parameters
        ----------
        state : array_like
            State vector.
        state_dot : array_like
            Derivative of state vector.

        """
        raise NotImplementedError

    @abstractmethod
    def _get_state_vector_from_full_state(self, full_state):

        raise NotImplementedError

    @abstractmethod
    def steady_state_trim_fun(self, full_state, environment, aircraft,
                              controls):

        raise NotImplementedError

    def time_step(self, dt):

        super().time_step(dt)
        # Now self.state_vector and state_vector_dot are updated
        self._update_full_system_state_from_state(self.state_vector,
                                                  self.state_vector_dot)

        return self.full_state

            
            
            
            
            
            
            
            
            