import numpy as np
from scipy.spatial.transform import Rotation

def rad2deg(rad):
    return rad / np.pi * 180

def deg2rad(deg):
    return deg / 180 * np.pi

#def get_quaternion(pitch, roll, yaw): using scipy version
#    cy = cos(yaw / 2)
#    sy = sin(yaw / 2)
#    cp = cos(pitch / 2)
#    sp = sin(pitch / 2)
#    cr = cos(roll / 2)
#    sr = sin(roll / 2)
#    w = cr * cp * cy + sr * sp * sy
#    x = sr * cp * cy - cr * sp * sy
#    y = cr * sp * cy + sr * cp * sy
#    z = cr * cp * sy - sr * sp * cy
#    return np.array([w, x, y, z])

def getRotMat(q):
    c00 = q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2
    c01 = 2 * (q[1] * q[2] - q[0] * q[3])
    c02 = 2 * (q[1] * q[3] + q[0] * q[2])
    c10 = 2 * (q[1] * q[2] + q[0] * q[3])
    c11 = q[0] ** 2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2
    c12 = 2 * (q[2] * q[3] - q[0] * q[1])
    c20 = 2 * (q[1] * q[3] - q[0] * q[2])
    c21 = 2 * (q[2] * q[3] + q[0] * q[1])
    c22 = q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2

    rotMat = np.array([[c00, c01, c02], [c10, c11, c12], [c20, c21, c22]])
    return rotMat

def quatFromEuler(p, r, y):
    r = Rotation.from_euler('xyz', [p, r, y])
    return r.as_quat

def getEulerAngles(q):
    m = getRotMat(q)
    test = -m[2, 0]
    if test > 0.99999:
        yaw = 0
        pitch = np.pi / 2
        roll = np.arctan2(m[0, 1], m[0, 2])
    elif test < -0.99999:
        yaw = 0
        pitch = -np.pi / 2
        roll = np.arctan2(-m[0, 1], -m[0, 2])
    else:
        yaw = np.arctan2(m[1, 0], m[0, 0])
        pitch = np.arcsin(-m[2, 0])
        roll = np.arctan2(m[2, 1], m[2, 2])

    yaw = rad2deg(yaw)
    pitch = rad2deg(pitch)
    roll = rad2deg(roll)

    return yaw, pitch, roll

class Quaternion:
    def __init__(self):
        self.q = np.array([1, 0, 0, 0])  # Initial state of the quaternion

    def rotate(self, w, dt):
        q = self.q
        Sq = np.array([[-q[1], -q[2], -q[3]],
                       [q[0], -q[3], q[2]],
                       [q[3], q[0], -q[1]],
                       [-q[2], q[1], q[0]]])
        self.q = np.matmul(dt/2 * Sq, np.array(w).transpose()) + q

