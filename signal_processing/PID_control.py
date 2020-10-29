#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 14:50:22 2020

@author: pi
"""

class pd_signal:
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