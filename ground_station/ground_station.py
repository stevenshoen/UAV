#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from path_finding.bezier_path import *
import numpy as np
import matplotlib.pyplot as plt
from simulation.simtools import Target, Intercept


class GroundStation(object):
    """
    store
    wind map
    obstacle map
    elevation map
    
    
    receive
    aircraft state
    
    target state
        pos. vel. proj. path
        
    send
    intercept
        time pos hdg (aircraft body frame)
    
    
    """
    def __init__(self, sim, lat=0, lon=0, aircraft_altitude=1000, elevation=None, elevation_map=None, wind_map=None):
        
        self.GROUND_STATION_GPS_LOCATION = None
        self.GROUND_STATION_LOCAL_LOCATION = np.array([0.0, 0.0], dtype=float)
        
        self.sim = sim
        #earth position is in -z system
        
        self.position = np.array([sim.system.full_state.earth_x,
                        sim.system.full_state.earth_y,
                        -sim.system.full_state.earth_z])
        
        
        self.heading = sim.system.full_state.attitude.psi
        self.state = sim.system.full_state
        
        self.targets = [Target()]
        
        
        self.elevation_map = elevation_map
        self.wind_map = wind_map
    
    def cruise_time(self, d):
        pass
    def distance(self, pos):
        return np.linalg.norm(self.position - pos)
    
    
    def calc_intercept(self, target_i=0, cur_time=0.0):
        """
        
        this will be iterative .. the heart of the path_finding algo?
        
        repeat process when intercept time or location changes by a set amount?
        
        
        
        calc fastest path .. then adjust heading?
        
        """
        t = self.targets[i]

        distance = np.linalg.norm(t.position - pos) 
        
        if distance < 50.0:
            print('target in range')
            exit(0)
        
        
        
        hdg = self.heading
        
        
        calc_cruise_time
        
        extrapolate_target_path
        pass





"""Show the effect of the offset."""
start_x = 10.0  # [m]
start_y = 1.0  # [m]
start_yaw = np.radians(180.0)  # [rad]

end_x = -0.0  # [m]
end_y = -3.0  # [m]
end_yaw = np.radians(0.0)  # [rad]

#for offset in np.arange(1.0, 5.0, 1.0):
for offset in [1.0]:
    path, control_points = calc_4points_bezier_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)
    
    # Display the tangent, normal and radius of cruvature at a given point
    t = 0.86  # Number in [0, 1]
    x_target, y_target = bezier(t, control_points)
    derivatives_cp = bezier_derivatives_control_points(control_points, 2)
    point = bezier(t, control_points)
    dt = bezier(t, derivatives_cp[1])
    ddt = bezier(t, derivatives_cp[2])
    # Radius of curvature
    radius = 1 / curvature(dt[0], dt[1], ddt[0], ddt[1])
    # Normalize derivative
    dt /= np.linalg.norm(dt, 2)
    tangent = np.array([point, point + dt])
    normal = np.array([point, point + [- dt[1], dt[0]]])
    curvature_center = point + np.array([- dt[1], dt[0]]) * radius
    circle = plt.Circle(tuple(curvature_center), radius,
                        color=(0, 0.8, 0.8), fill=False, linewidth=1)
    
    assert path.T[0][0] == start_x, "path is invalid"
    assert path.T[1][0] == start_y, "path is invalid"
    assert path.T[0][-1] == end_x, "path is invalid"
    assert path.T[1][-1] == end_y, "path is invalid"

    if show_animation:  # pragma: no cover
        plt.plot(path.T[0], path.T[1], label="Offset=" + str(offset))

if show_animation:  # pragma: no cover
    plot_arrow(start_x, start_y, start_yaw)
    plot_arrow(end_x, end_y, end_yaw)
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.show()


#def main():
#    """Plot an example bezier curve."""
#    start_x = 10.0  # [m]
#    start_y = 1.0  # [m]
#    start_yaw = np.radians(180.0)  # [rad]
#
#    end_x = -0.0  # [m]
#    end_y = -3.0  # [m]
#    end_yaw = np.radians(-45.0)  # [rad]
#    offset = 3.0
#
#    path, control_points = calc_4points_bezier_path(
#        start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)
#
#    # Note: alternatively, instead of specifying start and end position
#    # you can directly define n control points and compute the path:
##    control_points = np.array([[5., 1.], [-2.78, 1.], [-11.5, -4.5], [-6., -8.]])
##    path = calc_bezier_path(control_points, n_points=100)
#
#    # Display the tangent, normal and radius of cruvature at a given point
#    t = 0.86  # Number in [0, 1]
#    x_target, y_target = bezier(t, control_points)
#    derivatives_cp = bezier_derivatives_control_points(control_points, 2)
#    point = bezier(t, control_points)
#    dt = bezier(t, derivatives_cp[1])
#    ddt = bezier(t, derivatives_cp[2])
#    # Radius of curvature
#    radius = 1 / curvature(dt[0], dt[1], ddt[0], ddt[1])
#    # Normalize derivative
#    dt /= np.linalg.norm(dt, 2)
#    tangent = np.array([point, point + dt])
#    normal = np.array([point, point + [- dt[1], dt[0]]])
#    curvature_center = point + np.array([- dt[1], dt[0]]) * radius
#    circle = plt.Circle(tuple(curvature_center), radius,
#                        color=(0, 0.8, 0.8), fill=False, linewidth=1)
#
#    assert path.T[0][0] == start_x, "path is invalid"
#    assert path.T[1][0] == start_y, "path is invalid"
#    assert path.T[0][-1] == end_x, "path is invalid"
#    assert path.T[1][-1] == end_y, "path is invalid"
#
#    if show_animation:  # pragma: no cover
#        fig, ax = plt.subplots()
#        ax.plot(path.T[0], path.T[1], label="Bezier Path")
#        ax.plot(control_points.T[0], control_points.T[1],
#                '--o', label="Control Points")
#        ax.plot(x_target, y_target)
#        ax.plot(tangent[:, 0], tangent[:, 1], label="Tangent")
#        ax.plot(normal[:, 0], normal[:, 1], label="Normal")
#        ax.add_artist(circle)
#        plot_arrow(start_x, start_y, start_yaw)
#        plot_arrow(end_x, end_y, end_yaw)
#        ax.legend()
#        ax.axis("equal")
#        ax.grid(True)
#        plt.show()