#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 16 20:36:57 2020

@author: pi
"""

#from pyfme import aircrafts
from simulation.simcraft.cessna_172 import Cessna172
from utils.coordinates import wind2body
import numpy as np
from scipy.interpolate import RectBivariateSpline

#from src.aircraft import Aircraft
from utils.constants import slugft2_2_kgm2, lbs2kg
from abc import abstractmethod

from environment.anemometry import tas2cas, tas2eas, calculate_alpha_beta_TAS

class ElectricGlider(Cessna172):
    def __init__(self):
        super().__init__()
        # LDY are forces
        # lmn are moments
        self.ground_altitude = 500 #m
        
        self.lift = 0.0
        self.drag = 0.0
        #lateral
        
        self.elev_CL = 0.0
        self.elev_CD = 0.0
        self.elev_CM = 0.0
        
        # longitudinal
        
        self.rud_CY = 0.0
        self.ail_Cl = 0.0
        self.rud_Cl = 0.0
        self.ail_CN = 0.0
        self.rud_CN = 0.0

    def _calculate_control_lat_moments_coeffs(self, state):
        delta_aile = np.rad2deg(self.controls['delta_aileron'])  # deg
        delta_rud_RAD = self.controls['delta_rudder']  # rad
        alpha_DEG = np.rad2deg(self.alpha)  # deg

        Cl_delta_rud = np.interp(alpha_DEG, self.alpha_data, self.Cl_delta_rud_data)
        Cl_delta_aile_interp = np.interp(delta_aile, self.delta_aile_data, self.Cl_delta_aile_data)

        CN_delta_rud = np.interp(alpha_DEG, self.alpha_data, self.CN_delta_rud_data)
        CN_delta_aile_interp_ = RectBivariateSpline(self.delta_aile_data,
                                                    self.alpha_data,
                                                    self.CN_delta_aile_data)
        CN_delta_aile_interp = CN_delta_aile_interp_(delta_aile, alpha_DEG)[0, 0]

        Cl = (
            Cl_delta_aile_interp +
            0.075*Cl_delta_rud * delta_rud_RAD)
        # XXX: Tunned CN_delta_rud
        CN = (CN_delta_aile_interp +
            0.075*CN_delta_rud * delta_rud_RAD)
        return Cl, CN

    def _calculate_control_lon_moments_coeffs(self, state):
        delta_elev = np.rad2deg(self.controls['delta_elevator'])  # deg
        alpha_DEG = np.rad2deg(self.alpha)  # deg

        CM_alpha_interp = np.interp(alpha_DEG, self.alpha_data, self.CM_data)
        CM_delta_elev_interp = np.interp(delta_elev, self.delta_elev_data, self.CM_delta_elev_data)

        CM = (CM_alpha_interp + CM_delta_elev_interp)
        return CM
        
    def _calculate_body_lat_moments_coeffs(self, state):
        alpha_DEG = np.rad2deg(self.alpha)  # deg
        Cl_beta = np.interp(alpha_DEG, self.alpha_data, self.Cl_beta_data)
        CN_beta = np.interp(alpha_DEG, self.alpha_data, self.CN_beta_data)
        Cl = (0.1*Cl_beta * self.beta)
        CN = (CN_beta * self.beta)
        return Cl, CN

        
    def _calculate_body_lon_moments_coeffs(self, state):
        alpha_DEG = np.rad2deg(self.alpha)  # deg
        c = self.chord  # m
        V = self.TAS  # m/s
        alpha_dot = self.alpha_dot
        CM_alpha_interp = np.interp(alpha_DEG, self.alpha_data, self.CM_data)
        return CM_alpha_interp
        
    def _calculate_aero_lon_forces_moments_coeffs(self, state):
        delta_elev = np.rad2deg(self.controls['delta_elevator'])  # deg
        alpha_DEG = np.rad2deg(self.alpha)  # deg
        c = self.chord  # m
        V = self.TAS  # m/s
        p, q, r = (state.angular_vel.p, state.angular_vel.q,
                   state.angular_vel.r)  # rad/s
        alpha_dot = self.alpha_dot

        CD_alpha_interp = np.interp(alpha_DEG, self.alpha_data, self.CD_data)
        CD_delta_elev_interp_ = RectBivariateSpline(self.delta_elev_data,
                                                    self.alpha_data,
                                                    self.CD_delta_elev_data)
        CD_delta_elev_interp = CD_delta_elev_interp_(delta_elev, alpha_DEG)[0, 0]

        CL_alpha_interp = np.interp(alpha_DEG, self.alpha_data, self.CL_data)
        CL_alphadot = np.interp(alpha_DEG, self.alpha_data, self.CL_alphadot_data)
        CL_q = np.interp(alpha_DEG, self.alpha_data, self.CL_q_data)
        CL_delta_elev_interp = np.interp(delta_elev, self.delta_elev_data, self.CL_delta_elev_data)

        CM_alpha_interp = np.interp(alpha_DEG, self.alpha_data, self.CM_data)
        CM_q = np.interp(alpha_DEG, self.alpha_data, self.CM_q_data)
        CM_alphadot = np.interp(alpha_DEG, self.alpha_data, self.CM_alphadot_data)
        CM_delta_elev_interp = np.interp(delta_elev, self.delta_elev_data, self.CM_delta_elev_data)

        self.CL = (
            CL_alpha_interp +
            CL_delta_elev_interp +
            c/(2*V) * (CL_q * q + CL_alphadot * alpha_dot)
        )
        self.CD = CD_alpha_interp + CD_delta_elev_interp

        self.CM = (
            CM_alpha_interp +
            CM_delta_elev_interp +
            c/(2*V) * (2*CM_q * q + CM_alphadot * alpha_dot)
        )
        
#        self.elev_CL = CL_delta_elev_interp / delta_elev
#        self.elev_CD = CD_delta_elev_interp / delta_elev
#        self.elev_CM = CM_delta_elev_interp / delta_elev
        
        # FIXME: CM_q multiplicado por 2 hasta que alpha_dot pueda ser calculado

    def _calculate_aero_lat_forces_moments_coeffs(self, state):
        delta_aile = np.rad2deg(self.controls['delta_aileron'])  # deg
        delta_rud_RAD = self.controls['delta_rudder']  # rad
        alpha_DEG = np.rad2deg(self.alpha)  # deg
        b = self.span
        V = self.TAS
        p, q, r = state.angular_vel.p, state.angular_vel.q, state.angular_vel.r

        CY_beta = np.interp(alpha_DEG, self.alpha_data, self.CY_beta_data)
        CY_p = np.interp(alpha_DEG, self.alpha_data, self.CY_p_data)
        CY_r = np.interp(alpha_DEG, self.alpha_data, self.CY_r_data)
        CY_delta_rud = np.interp(alpha_DEG, self.alpha_data, self.CY_delta_rud_data)

        Cl_beta = np.interp(alpha_DEG, self.alpha_data, self.Cl_beta_data)
        Cl_p = np.interp(alpha_DEG, self.alpha_data, self.Cl_p_data)
        Cl_r = np.interp(alpha_DEG, self.alpha_data, self.Cl_r_data)
        Cl_delta_rud = np.interp(alpha_DEG, self.alpha_data, self.Cl_delta_rud_data)
        Cl_delta_aile_interp = np.interp(delta_aile, self.delta_aile_data, self.Cl_delta_aile_data)

        CN_beta = np.interp(alpha_DEG, self.alpha_data, self.CN_beta_data)
        CN_p = np.interp(alpha_DEG, self.alpha_data, self.CN_p_data)
        CN_r = np.interp(alpha_DEG, self.alpha_data, self.CN_r_data)
        CN_delta_rud = np.interp(alpha_DEG, self.alpha_data, self.CN_delta_rud_data)
        CN_delta_aile_interp_ = RectBivariateSpline(self.delta_aile_data,
                                                    self.alpha_data,
                                                    self.CN_delta_aile_data)
        CN_delta_aile_interp = CN_delta_aile_interp_(delta_aile, alpha_DEG)[0, 0]

        self.CY = (
            CY_beta * self.beta +
            CY_delta_rud * delta_rud_RAD +
            b/(2 * V) * (CY_p * p + CY_r * r)
        )
        # XXX: Tunned Cl_delta_rud
        self.Cl = (
            0.1*Cl_beta * self.beta +
            Cl_delta_aile_interp +
            0.075*Cl_delta_rud * delta_rud_RAD +
            b/(2 * V) * (Cl_p * p + Cl_r * r)
        )
        # XXX: Tunned CN_delta_rud
        self.CN = (
            CN_beta * self.beta +
            CN_delta_aile_interp +
            0.075*CN_delta_rud * delta_rud_RAD +
            b/(2 * V) * (CN_p * p + CN_r * r)
        )
        
#        self.rud_CY = (CY_delta_rud)
#        self.ail_Cl = Cl_delta_aile_interp / delta_aile
#        self.rud_Cl = 0.075*Cl_delta_rud
#        self.ail_CN = CN_delta_aile_interp / delta_aile
#        self.rud_CN = 0.075*CN_delta_rud

    def _calculate_aero_forces_moments(self, state):
        q = self.q_inf
        Sw = self.Sw
        c = self.chord
        b = self.span

        self._calculate_aero_lon_forces_moments_coeffs(state)
        self._calculate_aero_lat_forces_moments_coeffs(state)

        L = q * Sw * self.CL
        D = q * Sw * self.CD
        Y = q * Sw * self.CY
        l = q * Sw * b * self.Cl
        m = q * Sw * c * self.CM
        n = q * Sw * b * self.CN

        return L, D, Y, l, m, n

    def _calculate_thrust_forces_moments(self, environment):
        delta_t = self.controls['delta_t']
        rho = environment.rho
        V = self.TAS
        prop_rad = self.propeller_radius

        # In this model the throttle controls the revolutions of the propeller
        # linearly. Later on, a much detailed model will be included
        omega = np.interp(delta_t, self.delta_t_data, self.omega_data)  # rpm
        omega_RAD = (omega * 2 * np.pi) / 60.0  # rad/s

        # We calculate the relation between the thrust coefficient Ct and the
        # advance ratio J using the program JavaProp
        J = (np.pi * V) / (omega_RAD * prop_rad)  # non-dimensional
        Ct_interp = np.interp(J, self.J_data, self.Ct_data)  # non-dimensional

        T = (2/np.pi)**2 * rho * (omega_RAD * prop_rad)**2 * Ct_interp  # N

        # We will consider that the engine is aligned along the OX (body) axis
        Ft = np.array([T, 0, 0])

        return Ft

    
    def _calculate_aerodynamics_2(self, TAS, alpha, beta, environment):
        self.alpha, self.beta, self.TAS = alpha, beta, TAS
        # Setting velocities & dynamic pressure
        self.CAS = tas2cas(self.TAS, environment.p, environment.rho)
        self.EAS = tas2eas(self.TAS, environment.rho)
        self.Mach = self.TAS / environment.a
        self.q_inf = 0.5 * environment.rho * self.TAS ** 2

    def normal_force(self, z_earth, z_force):
        Fn = np.zeros(3)
        if -z_earth < self.ground_altitude:
            if z_force > 0:
                Fn = -z_force
        return Fn
    
    def calculate_forces_and_moments(self, state, environment, controls):
        # Update controls and aerodynamics
#        super().calculate_forces_and_moments(state, environment, controls)
        self._set_current_controls(controls)
        self._calculate_aerodynamics(state, environment)
        
        Ft = self._calculate_thrust_forces_moments(environment)
        L, D, Y, l, m, n = self._calculate_aero_forces_moments(state)
        self.lift = L
        self.drag = D
        Fg = environment.gravity_vector * self.mass

        Fa_wind = np.array([-D, Y, -L])
        Fa_body = wind2body(Fa_wind, self.alpha, self.beta)
        Fa = Fa_body
        
        Fn = self.normal_force(state.position.z_earth, (Ft + Fg + Fa)[2])

        self.total_forces = Ft + Fg + Fa + Fn
        self.total_moments = np.array([l, m, n])

        return self.total_forces, self.total_moments




