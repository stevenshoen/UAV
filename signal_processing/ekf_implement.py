#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 13 16:02:42 2020
https://www.thepoorengineer.com/en/ekf-impl/?singlepage=1#EKFimpl
@author: pi
"""

def update(self, a, m):
    tmp1 = np.linalg.inv(np.matmul(np.matmul(self.C, self.pBar), self.C.transpose()) + self.R)
    self.K = np.matmul(np.matmul(self.pBar, self.C.transpose()), tmp1)

    magGuass_B = self.getMagVector(m)
    accel_B = self.getAccelVector(a)

    measurement = np.concatenate((accel_B, magGuass_B), axis=0)
    self.xHat = self.xHatBar + np.matmul(self.K, measurement - self.yHatBar)
    self.xHat[0:4] = self.normalizeQuat(self.xHat[0:4])
    self.p = np.matmul(np.identity(7) - np.matmul(self.K, self.C), self.pBar)
    
def predict(self, w, dt):
    q = self.xHat[0:4]
    Sq = np.array([[-q[1], -q[2], -q[3]],
                   [ q[0], -q[3],  q[2]],
                   [ q[3],  q[0], -q[1]],
                   [-q[2],  q[1],  q[0]]])
    tmp1 = np.concatenate((np.identity(4), -dt / 2 * Sq), axis=1)
    tmp2 = np.concatenate((np.zeros((3, 4)), np.identity(3)), axis=1)
    self.A = np.concatenate((tmp1, tmp2), axis=0)
    self.B = np.concatenate((dt / 2 * Sq, np.zeros((3, 3))), axis=0)
    self.xHatBar = np.matmul(self.A, self.xHat) + np.matmul(self.B, np.array(w).transpose())
    self.xHatBar[0:4] = self.normalizeQuat(self.xHatBar[0:4])
    self.xHatPrev = self.xHat

    self.yHatBar = self.predictAccelMag()
    self.pBar = np.matmul(np.matmul(self.A, self.p), self.A.transpose()) + self.Q