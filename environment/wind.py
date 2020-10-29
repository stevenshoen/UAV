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
        self.count = 0
        self.freq = .01
        self.mag = 0.50
        self.clip = 3.0
        self.steady = 5.0
        self.transient = 0.0
        
    def sin_wind(self):
        return np.array([self.steady + self.transient * np.sin(self.freq * self.count),
                  self.steady + self.transient * np.sin(self.freq * self.count),
                  0.0], dtype=float)
        
    def rand_wind(self):
        t = self.mag/2 * (np.random.random(3) - 1/2)
        self.transient = np.clip(self.transient + t, -self.clip, self.clip)
#        self.transient += self.mag * np.random.random(3)
#        self.transient = np.clip(self.transient, -self.clip, self.clip)
        return self.steady + self.transient  
    
    def update(self, state):
        self.body = self.rand_wind()
#        print('wind -', self.body)
#        self.count += 1
        
        pass
    
class NoWind(object):

    def __init__(self):
        # Wind velocity: FROM North to South, FROM East to West,
        # Wind velocity in the UPSIDE direction
        self.horizon = np.zeros([3], dtype=float)
        self.body = np.zeros([3], dtype=float)

    def update(self, state):
        pass
