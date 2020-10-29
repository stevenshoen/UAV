#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 24 15:10:30 2020

@author: pi

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

"""
import numpy as np

state_vector = np.array([ 4.99204665e+01,  2.25087059e-13,  2.81904629e+00, -8.38494073e-17,
       -3.45198836e-05,  1.05144341e-17,  5.64111600e-02, -4.13599707e-19,
        5.24872206e-20,  5.00000000e-01,  2.25090154e-15, -1.00000000e+03])
"""


need to propogate that
with ...
"""
m = np.zeros((12, 12))
n = np.identity(12)

m = np.array([[],
              [],
              []])
        