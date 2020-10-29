#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 11:33:32 2020

@author: pi
"""
import numpy as np
from scipy import interpolate


r_pi = 3.1

alphas = np.arange(-1, 1, .1) # meters
betas = np.arange(-1, 1, .1)
xx, yy = np.meshgrid(alphas, betas)

z = 


xx, yy = np.meshgrid(x, y)

z = np.sin(xx**2*yy*2)


known_points = [# x, y, ele
        [00, 0, 1],
        [3, 0, 1],
        ]

#w = np.ndarray((4, 3))

x = np.arange(4)
y = np.arange(4)

w = np.ndarray((4, 4))

for pt in known_points:
    w[pt[0], pt[1]] = pt[2]
    
func = interpolate.interp2d(x, y, w, kind='cubic')


     
