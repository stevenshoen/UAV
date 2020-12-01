# -*- coding: utf-8 -*-
"""
Created on Mon Nov  2 14:06:13 2020

@author: ubuntu
"""
import numpy as np
from scipy.interpolate import RectBivariateSpline

from simulation.simcraft.aircraft import Aircraft
from utils.constants import slugft2_2_kgm2, lbs2kg
from utils.coordinates import wind2body

#delta_aile = np.rad2deg(self.controls['delta_aileron']) 
alpha_data = np.array([-7.5, -5, -2.5, 0, 2.5, 5, 7.5, 10, 15, 17, 18, 19.5])

alpha_DEG = np.rad2deg(5) 
delta_elev = np.rad2deg(5) 
delta_aile = np.rad2deg(5) 
delta_aile_data = np.array([-15, -10, -5, -2.5, 0, 5, 10, 15, 20])
delta_elev_data = np.array([-26, -20, -10, -5, 0, 7.5, 15, 22.5, 28])

CN_delta_aile_data = np.array([[-0.004321, -0.002238, -0.0002783, 0.001645, 0.003699, 0.005861, 0.008099, 0.01038, 0.01397, 0.01483, 0.01512, 0.01539],
                                            [-0.003318, -0.001718, -0.0002137, 0.001263, 0.00284, 0.0045, 0.006218, 0.00797, 0.01072, 0.01138, 0.01161, 0.01181],
                                            [-0.002016, -0.001044, -0.000123, 0.0007675, 0.00173, 0.002735, 0.0038, 0.004844, 0.00652, 0.00692, 0.00706, 0.0072],
                                            [-0.00101, -0.000522, -0.0000649, 0.000384, 0.000863, 0.00137, 0.0019, 0.00242, 0.00326, 0.00346, 0.00353, 0.0036],
                                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                            [0.00101, 0.00052, 0.000065, -0.000384, -0.00086, -0.0014, -0.002, -0.002422, -0.00326, -0.00346, -0.00353, -0.0036],
                                            [0.00202, 0.001044, 0.00013, -0.0008, -0.00173, -0.002735, -0.0038, -0.004844, -0.00652, -0.00692, -0.00706, -0.0072],
                                            [0.00332, 0.00172, 0.000214, -0.001263, -0.00284, -0.0045, -0.00622, -0.008, -0.01072, -0.01138, -0.01161, -0.01181],
                                            [0.004321, 0.00224, 0.00028, -0.001645, -0.0037, -0.00586, -0.0081, -0.0104, -0.014, -0.01483, -0.01512, -0.0154]])

CM_delta_elev_data = np.array([0.3302, 0.3065, 0.2014, 0.1007, -0.0002, -0.1511, -0.2863, -0.3109, -0.345])

CN_delta_rud_data = (-1)*np.array([-0.211, -0.215, -0.218, -0.22, -0.224, -0.226, -0.228, -0.229, -0.23, -0.23, -0.23, -0.23])

Cl_delta_aile_data = np.array([-0.078052, -0.059926, -0.036422, -0.018211, 0, 0.018211, 0.036422, 0.059926, 0.078052])

Cl_delta_aile_interp = np.interp(delta_aile, delta_aile_data, Cl_delta_aile_data)


CM_delta_elev_interp = np.interp(delta_elev, delta_elev_data, CM_delta_elev_data)



CN_delta_rud = np.interp(alpha_DEG, alpha_data, CN_delta_rud_data)

CN_delta_aile_interp_ = RectBivariateSpline(delta_aile_data,
                                            alpha_data,
                                            CN_delta_aile_data)
                                            
CN_delta_aile_interp = CN_delta_aile_interp_(delta_aile, alpha_DEG)[0, 0]


"""
        p, q, r = state.angular_vel.p, state.angular_vel.q, state.angular_vel.r
        
q * Sw * b *
            Cl_delta_aile_interp +

q * Sw * c *
            CM_delta_elev_interp +
   
q * Sw * b *         
            CN_delta_aile_interp +
            0.075*CN_delta_rud * delta_rud_RAD +
            
            
        self.Cl = (
            0.1*Cl_beta * self.beta +
            Cl_delta_aile_interp +
            0.075*Cl_delta_rud * delta_rud_RAD +
            b/(2 * V) * (Cl_p * p + Cl_r * r)
        )


        self.CM = (
            CM_alpha_interp +
            CM_delta_elev_interp +
            c/(2*V) * (2*CM_q * q + CM_alphadot * alpha_dot)
        )
         
                self.CN = (
            CN_beta * self.beta +
            CN_delta_aile_interp +
            0.075*CN_delta_rud * delta_rud_RAD +
            b/(2 * V) * (CN_p * p + CN_r * r)
        )
       
                q = self.q_inf
        Sw = self.Sw
        c = self.chord
        b = self.span
        
                l = q * Sw * b * self.Cl
        m = q * Sw * c * self.CM
        n = q * Sw * b * self.CN
        
"""
        