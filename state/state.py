#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 26 12:57:35 2020

@author: pi
"""








from utils.coordinates import body2hor, hor2body

from abc import abstractmethod

import numpy as np


from utils.constants import EARTH_MEAN_RADIUS


class Position:
    """Position

    Attributes
    ----------

    geodetic_coordinates : ndarray, shape(3)
        (lat [rad], lon [rad], height [m])
    lat
    lon
    height
    geocentric_coordinates : ndarray, shape(3)
        (x_geo [m], y_geo [m], z_geo [m])
    x_geo
    y_geo
    z_geo
    earth_coordinates : ndarray, shape(3)
        (x_earth [m], y_earth [m], z_earth [m])
    x_earth
    y_earth
    z_earth
    """

    def __init__(self, geodetic, geocentric, earth):
        # Geodetic coordinates: (geodetic lat, lon, height above ellipsoid)
        self._geodetic_coordinates = np.asarray(geodetic, dtype=float)  # rad
        # Geocentric coordinates (rotating with Earth): (x_geo, y_geo, z_geo)
        self._geocentric_coordinates = np.asarray(geocentric, dtype=float)  # m
        # Earth coordinates (x_earth, y_earth, z_earth)
        self._earth_coordinates = np.asarray(earth, dtype=float)  # m

    @abstractmethod
    def update(self, coords):
        raise NotImplementedError

    @property
    def geocentric_coordinates(self):
        return self._geocentric_coordinates

    @property
    def x_geo(self):
        return self._geocentric_coordinates[0]

    @property
    def y_geo(self):
        return self._geocentric_coordinates[1]

    @property
    def z_geo(self):
        return self._geocentric_coordinates[2]

    @property
    def geodetic_coordinates(self):
        return self._geodetic_coordinates

    @property
    def lat(self):
        return self._geodetic_coordinates[0]

    @property
    def lon(self):
        return self._geodetic_coordinates[1]

    @property
    def height(self):
        return self._geodetic_coordinates[2]

    @property
    def earth_coordinates(self):
        return self._earth_coordinates

    @property
    def x_earth(self):
        return self._earth_coordinates[0]

    @property
    def y_earth(self):
        return self._earth_coordinates[1]

    @property
    def z_earth(self):
        return self._earth_coordinates[2]

    @property
    def value(self):
        """Only for testing purposes"""
        return np.hstack((self.earth_coordinates, self.geodetic_coordinates,
                          self.geodetic_coordinates))


class EarthPosition(Position):

    def __init__(self, x, y, height, lat=0, lon=0):
        # TODO: docstring
        earth = np.array([x, y, -height])
        # TODO: Assuming round earth use changes in x & y to calculate
        # new lat and lon. z_earth is -height:
        geodetic = np.array([lat, lon, height])  # m
        # TODO: make transformation from geodetic to geocentric:
        geocentric = np.zeros(3)  # m
        super().__init__(geodetic, geocentric, earth)

    def update(self, value):
        # Assuming round earth use changes in x & y to calculate
        # new lat and lon. z_earth is -height:
        delta_x, delta_y, _ = value - self.earth_coordinates
        delta_lat = delta_x / EARTH_MEAN_RADIUS
        delta_lon = delta_y / EARTH_MEAN_RADIUS
        self._geodetic_coordinates = \
            np.array([self.lat + delta_lat, self.lon + delta_lon, -value[2]])

        # TODO: make transformation from geodetic to geocentric:
        self._geocentric_coordinates = np.zeros(3)  # m

        # Update Earth coordinates with value
        self._earth_coordinates[:] = value

    def __repr__(self):
        rv = (f"x_e: {self.x_earth:.2f} m, y_e: {self.y_earth:.2f} m, "
              f"z_e: {self.z_earth:.2f} m")
        return rv


class GeodeticPosition(Position):

    def __init__(self, lat, lon, height, x_earth=0, y_earth=0):
        # TODO: docstring
        earth = np.array([x_earth, y_earth, -height])
        # TODO: Assuming round earth use changes in x & y to calculate
        # new lat and lon. z_earth is -height:
        geodetic = np.array([lat, lon, height])  # m
        # TODO: make transformation from geodetic to geocentric:
        geocentric = np.zeros(3)  # m
        super().__init__(geodetic, geocentric, earth)

    def update(self, value):
        # Assuming round earth use changes in x & y to calculate
        # new x, y from lat and lon. z_earth is -height
        delta_lat, delta_lon, _ = self.geodetic_coordinates - value
        dx_e = EARTH_MEAN_RADIUS * delta_lat
        dy_e = EARTH_MEAN_RADIUS * delta_lon
        self._earth_coordinates[:] = \
            np.array([self.x_earth + dx_e, self.y_earth + dy_e, -value[2]])

        # TODO: make transformation from geodetic to geocentric:
        self._geocentric_coordinates = np.zeros(3)  # m

        # Update geodetic coordinates with value
        self._geodetic_coordinates[:] = value

class Attitude:
    """Attitude

    Attributes
    ----------

    euler_angles : ndarray, shape(3)
        (theta [rad], phi [rad], psi [rad])
    theta
    phi
    psi
    quaternions : ndarray, shape(4)
        (q0, q1, q2, q3)
    q0
    q1
    q2
    q3
    """

    def __init__(self):
        # Euler angles (psi, theta, phi)
        self._euler_angles = np.zeros(3)  # rad
        # Quaternions (q0, q1, q2, q3)
        self._quaternions = np.zeros(4)

    @abstractmethod
    def update(self, value):
        raise NotImplementedError

    @property
    def euler_angles(self):
        return self._euler_angles

    @property
    def psi(self):
        return self._euler_angles[2]

    @property
    def theta(self):
        return self._euler_angles[0]

    @property
    def phi(self):
        return self._euler_angles[1]

    @property
    def quaternions(self):
        return self._quaternions

    @property
    def q0(self):
        return self._quaternions[0]

    @property
    def q1(self):
        return self._quaternions[1]

    @property
    def q2(self):
        return self._quaternions[2]

    @property
    def q3(self):
        return self._quaternions[3]

    @property
    def value(self):
        """Only for testing purposes"""
        return np.hstack((self.euler_angles, self.quaternions))


class EulerAttitude(Attitude):

    def __init__(self, theta, phi, psi):
        # TODO: docstring
        super().__init__()
        self.update(np.array([theta, phi, psi]))

    def update(self, value):
        self._euler_angles[:] = value
        # TODO: transform quaternions to Euler angles
        self._quaternions = np.zeros(4)

    def __repr__(self):
        rv = (f"theta: {self.theta:.3f} rad, phi: {self.phi:.3f} rad, "
              f"psi: {self.psi:.3f} rad")
        return rv


class QuaternionAttitude(Attitude):
    def __init__(self, q0, q1, q2, q3):
        # TODO: docstring
        super().__init__()
        self.update(np.array([q0, q1, q2, q3]))

    def update(self, value):
        self._quaternions[:] = value
        # TODO: transform quaternions to Euler angles
        self._euler_angles = np.zeros(3)

class Velocity:
    """Velocity

    Attributes
    ----------

    vel_body : ndarray, shape(3)
        (u [m/s], v [m/s], w [m/s])
    u
    v
    w
    vel_NED : ndarray, shape(3)
        (v_north [m/s], v_east [m/s], v_down [m/s])
    v_north
    v_east
    v_down
    """

    def __init__(self):
        # Body axis
        self._vel_body = np.zeros(3)  # m/s
        # Local horizon (NED)
        self._vel_NED = np.zeros(3)  # m/s

    @abstractmethod
    def update(self, coords, attitude):
        raise NotImplementedError

    @property
    def vel_body(self):
        return self._vel_body

    @property
    def u(self):
        return self.vel_body[0]

    @property
    def v(self):
        return self.vel_body[1]

    @property
    def w(self):
        return self.vel_body[2]

    @property
    def vel_NED(self):
        return self._vel_NED

    @property
    def v_north(self):
        return self._vel_NED[0]

    @property
    def v_east(self):
        return self._vel_NED[1]

    @property
    def v_down(self):
        return self._vel_NED[2]

    @property
    def value(self):
        """Only for testing purposes"""
        return np.hstack((self.vel_body, self.vel_NED))


class BodyVelocity(Velocity):

    def __init__(self, u, v, w, attitude):
        # TODO: docstring
        super().__init__()
        self.update(np.array([u, v, w]), attitude)

    def update(self, value, attitude):
        self._vel_body[:] = value
        self._vel_NED = body2hor(value,
                                 attitude.theta,
                                 attitude.phi,
                                 attitude.psi)  # m/s

    def __repr__(self):
        return f"u: {self.u:.2f} m/s, v: {self.v:.2f} m/s, w: {self.w:.2f} m/s"


class NEDVelocity(Velocity):
    def __init__(self, vn, ve, vd, attitude):
        # TODO: docstring
        super().__init__()
        self.update(np.array([vn, ve, vd]), attitude)

    def update(self, value, attitude):
        self._vel_NED[:] = value
        self._vel_body = hor2body(value,
                                  attitude.theta,
                                  attitude.phi,
                                  attitude.psi)  # m/s

    def __repr__(self):
        return (f"V_north: {self.v_north:.2f} m/s,"
                f"V_east: {self.v_east:.2f} m/s, "
                f"V_down: {self.v_down:.2f} m/s")


class AngularVelocity:
    """Angular velocity

    vel_ang : ndarray, shape(3)
        (p [rad/s], q [rad/s], r [rad/s])
    p
    q
    r
    euler_ang_rates : ndarray, shape(3)
        (theta_dot [rad/s], phi_dot [rad/s], psi_dot [rad/s])
    theta
    phi
    psi
    """

    def __init__(self):
        # ANGULAR VELOCITY: (p, q, r)
        self._vel_ang_body = np.zeros(3)  # rad/s
        # EULER ANGLE RATES (theta_dot, phi_dot, psi_dot)
        self._euler_ang_rate = np.zeros(3)  # rad/s

    @abstractmethod
    def update(self, coords, attitude):
        raise NotImplementedError

    @property
    def vel_ang_body(self):
        return self._vel_ang_body

    @property
    def p(self):
        return self._vel_ang_body[0]

    @property
    def q(self):
        return self._vel_ang_body[1]

    @property
    def r(self):
        return self._vel_ang_body[2]

    @property
    def euler_ang_rate(self):
        return self._euler_ang_rate

    @property
    def theta_dot(self):
        return self._euler_ang_rate[0]

    @property
    def phi_dot(self):
        return self._euler_ang_rate[1]

    @property
    def psi_dot(self):
        return self._euler_ang_rate[2]

    @property
    def value(self):
        """Only for testing purposes"""
        return np.hstack((self.vel_ang_body, self.euler_ang_rate))


class BodyAngularVelocity(AngularVelocity):

    def __init__(self, p, q, r, attitude):
        # TODO: docstring
        super().__init__()
        self.update(np.array([p, q, r]), attitude)

    def update(self, coords, attitude):
        self._vel_ang_body[:] = coords
        # TODO: transform angular velocity in body axis to euler angles
        # rates
        self._euler_ang_rate = np.zeros(3)  # rad/s

    def __repr__(self):
        return (f"P: {self.p:.2f} rad/s, "
                f"Q: {self.q:.2f} rad/s, "
                f"R: {self.r:.2f} rad/s")


class EulerAngularRates(AngularVelocity):

    def __init__(self, theta_dot, phi_dot, psi_dot, attitude):
        # TODO: docstring
        super().__init__()
        self.update(np.array([theta_dot, phi_dot, psi_dot]),
                    attitude)

    def update(self, coords, attitude):
        self._euler_ang_rate[:] = coords
        # TODO: transform euler angles rates to angular velocity in body
        #  axis
        self._vel_ang_body[:] = np.zeros(3)  # rad/s

class AngularAcceleration:
    """Angular Accelerations

    Attributes
    ----------
    accel_ang : ndarray, shape(3)
        (p_dot [rad/s²], q_dot [rad/s²], r_dot [rad/s²])
    p_dot
    q_dot
    r_dot
    euler_ang_acc : ndarray, shape(3)
        (theta_2dot [rad/s²], phi_2dot [rad/s²], psi_2dot [rad/s²])
    theta_2dot
    phi_2dot
    psi_2dot
    """

    def __init__(self):
        # ANGULAR VELOCITY: (p_dot, q_dot, r_dot)
        self._acc_ang_body = np.zeros(3)  # rad/s
        # EULER ANGLE RATES (theta_dot2, phi_dot2, psi_dot2)
        self._euler_ang_acc = np.zeros(3)  # rad/s

    @abstractmethod
    def update(self, coords, attitude):
        raise ValueError

    @property
    def acc_ang_body(self):
        return self._acc_ang_body

    @property
    def p_dot(self):
        return self._acc_ang_body[0]

    @property
    def q_dot(self):
        return self._acc_ang_body[1]

    @property
    def r_dot(self):
        return self._acc_ang_body[2]

    @property
    def euler_ang_acc(self):
        return self._euler_ang_acc

    @property
    def theta_2dot(self):
        return self._euler_ang_acc[0]

    @property
    def phi_2dot(self):
        return self._euler_ang_acc[1]

    @property
    def psi_2dot(self):
        return self._euler_ang_acc[2]

    @property
    def value(self):
        """Only for testing purposes"""
        return np.hstack((self.acc_ang_body, self.euler_ang_acc))


class BodyAngularAcceleration(AngularAcceleration):

    def __init__(self, p_dot, q_dot, r_dot, attitude):
        super().__init__()
        self.update(np.array([p_dot, q_dot, r_dot]), attitude)

    def update(self, coords, attitude):
        self._acc_ang_body[:] = coords
        # TODO: transform angular acc in body axis to euler angles
        # acc
        self._euler_ang_acc = np.zeros(3)  # rad/s

    def __repr__(self):
        rv = (f"P_dot: {self.p_dot:.2f} rad/s², "
              f"Q_dot: {self.q_dot:.2f} rad/s², "
              f"R_dot: {self.r_dot:.2f} rad/s²")
        return rv


class EulerAngularAcceleration(AngularAcceleration):

    def __init__(self, theta_dot, phi_dot, psi_dot, attitude):
        super().__init__()
        self.update(np.array([theta_dot, phi_dot, psi_dot]),
                    attitude)

    def update(self, coords, attitude):
        self._euler_ang_acc[:] = coords
        # TODO: transform euler angles acc to angular acceleration in body
        #  axis
        self._acc_ang_body[:] = np.zeros(3)  # rad/s



class Acceleration:
    """Acceleration

    Attributes
    ----------
    accel_body : ndarray, shape(3)
        (u_dot [m/s²], v_dot [m/s²], w_dot [m/s²])
    u_dot
    v_dot
    w_dot
    accel_NED : ndarray, shape(3)
        (VN_dot [m/s²], VE_dot [m/s²], VD_dot [m/s²])
    VN_dot
    VE_dot
    VD_dot
    """

    def __init__(self):
        # Body axis
        self._accel_body = np.zeros(3)  # m/s²
        # Local horizon (NED)
        self._accel_NED = np.zeros(3)  # m/s²

    @abstractmethod
    def update(self, coords, attitude):
        raise NotImplementedError

    @property
    def accel_body(self):
        return self._accel_body

    @property
    def u_dot(self):
        return self._accel_body[0]

    @property
    def v_dot(self):
        return self._accel_body[1]

    @property
    def w_dot(self):
        return self._accel_body[2]

    @property
    def accel_NED(self):
        return self._accel_NED

    @property
    def v_north_dot(self):
        return self._accel_NED[0]

    @property
    def v_east_dot(self):
        return self._accel_NED[1]

    @property
    def v_down_dot(self):
        return self._accel_NED[2]

    @property
    def value(self):
        """Only for testing purposes"""
        return np.hstack((self.accel_body, self.accel_NED))


class BodyAcceleration(Acceleration):

    def __init__(self, u_dot, v_dot, w_dot, attitude):
        super().__init__()
        self.update(np.array([u_dot, v_dot, w_dot]), attitude)

    def update(self, coords, attitude):
        self._accel_body[:] = coords
        self._accel_NED = body2hor(coords,
                                   attitude.theta,
                                   attitude.phi,
                                   attitude.psi)

    def __repr__(self):
        rv = (f"u_dot: {self.u_dot:.2f} m/s², v_dot: {self.v_dot:.2f} m/s², "
              f"w_dot: {self.u_dot:.2f} m/s²")
        return rv


class NEDAcceleration(Acceleration):

    def __init__(self, vn_dot, ve_dot, vd_dot, attitude):
        super().__init__()
        self.update(np.array([vn_dot, ve_dot, vd_dot]), attitude)

    def update(self, coords, attitude):
        self._accel_NED[:] = coords
        self._accel_body = hor2body(coords,
                                    attitude.theta,
                                    attitude.phi,
                                    attitude.psi)

    def __repr__(self):
        rv = (f"V_north_dot: {self.v_north_dot:.2f} m/s², "
              f"V_east_dot: {self.v_east_dot:.2f} m/s², "
              f"V_down_dot: {self.v_down_dot:.2f} m/s²")
        return rv










