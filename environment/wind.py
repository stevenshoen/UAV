"""
Python Flight Mechanics Engine (PyFME).
Copyright (c) AeroPython Development Team.
Distributed under the terms of the MIT License.

Wind Models
-----------

"""
import numpy as np

class Wind(object):

    def __init__(self):
        # Wind velocity: FROM North to South, FROM East to West,
        # Wind velocity in the UPSIDE direction
        self.horizon = np.zeros([3], dtype=float)
        self.body = np.zeros([3], dtype=float)
        self.freq = [0.01, 0.001]
        self.mag = 0.2
        self.clip = 10.0
        self.steady = np.array([1.0, 1.0, 0.0])
        self.transient = np.array([3.0, 3.0, 0.0])
        self.max_transient = 3.0
    @property
    def x(self):
        return self.body[0]        
    @property
    def y(self):
        return self.body[1]
    @property
    def z(self):
        return self.body[2]
        
    def sin_wind(self, t):
        self.transient = np.array([self.max_transient * np.sin(self.freq[0] * t),
                  self.max_transient * np.sin(self.freq[1] * t),
                  0.0], dtype=float)
        return self.steady + self.transient
        
    def rand_wind(self):
        trans = np.append(self.mag/2 * (np.random.random(2) - 1/2), 0.0)
        self.transient = np.clip(self.transient + trans, -self.max_transient, self.max_transient)
#        self.transient += self.mag * np.random.random(3)
#        self.transient = np.clip(self.transient, -self.clip, self.clip)
        return np.clip(self.steady + self.transient, -self.clip, self.clip)
    
    def update(self, state, t=None):
        if type(t) == type(None): return
        self.body = self.rand_wind()
        
class NoWind(object):

    def __init__(self):
        # Wind velocity: FROM North to South, FROM East to West,
        # Wind velocity in the UPSIDE direction
        self.horizon = np.zeros([3], dtype=float)
        self.body = np.zeros([3], dtype=float)

    def update(self, state):
        pass
