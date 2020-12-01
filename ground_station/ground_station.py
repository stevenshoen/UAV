#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#from path_finding.bezier_path import *
import numpy as np
import matplotlib.pyplot as plt

from path_finding.bezier_path import calc_4points_bezier_path, bezier_derivatives_control_points, bezier, curvature
from state.position import EarthPosition, GeodeticPosition
from utils.constants import EARTH_MEAN_RADIUS

def trim_heading(hdg):
    if hdg < -np.pi:
        return hdg + 2 * np.pi
    elif hdg > np.pi:
        return hdg - 2 * np.pi
    else:
        return hdg

class Scale(object):
    def __init__(self, lat, lon, altitude):

        self.lat0 = lat
        self.lon0 = lon
        self.factor = EARTH_MEAN_RADIUS + altitude # m / radian
    
    def distance(self, lat, lon):
        return np.sqrt(np.deg2rad(lat - self.lat0)**2 + np.deg2rad(lon - self.lon0)**2)   
    
    def __call__(self, lat, lon): # return local x and y
        x = np.deg2rad(lat - self.lat0) * self.factor
        # y points east
        y = np.deg2rad(lon - self.lon0) * self.factor
        return x, y
    
class FlightPlan(object):
    """

x is latitude
y is longitude
positive x north
positive y is east
hdg is from local x axis
    """
    def __init__(self, waypoints, active_waypoint=0):
        """
        set scale and move all waypoints into the x-y frame of the first
        
        """
        #        self.gs_position = GeodeticPosition(*gs_location, x_earth=0.0, y_earth=0.0) # (lat, lon , alt)
        #        ac_x, ac_y = self.scale(aircraft_pos.lat, aircraft_pos.lon)
        self.aircraft_position = waypoints[0]
        self.scale = Scale(self.aircraft_position.lat, self.aircraft_position.lon, self.aircraft_position.height)
        self.waypoint_radius = 100.0
        self.waypoints = []
        for pt in waypoints:
            print('waypoint from -->', *self.scale(pt.lat, pt.lon), pt.height)
            self.waypoints.append(EarthWaypoint(*self.scale(pt.lat, pt.lon), pt.height))

        self.active_waypoint = active_waypoint
    
class EarthWaypoint(EarthPosition):
    def __init__(self, x, y, height, hdg=0.0, lat=0.0, lon=0.0):
        self.heading = hdg
        super().__init__(x, y, height, lat, lon)
    def __repr__(self):
        rv = 'earth_x: ', self.earth_x, 'earth_y: ', self.earth_y, 'earth_z: ', self.earth_z,
        'heading: ', self.heading
        return rv        
    def distance_from(self, x, y):
        return np.linalg.norm(np.array([x-self.x_earth, y-self.y_earth]))
    
    def heading_from(self, x, y):
        x_, y_ = np.array([self.x_earth-x, self.y_earth-y])
#        print('x_', x_)
#        print('y_', y_)        
        return trim_heading(np.arctan2(y_, x_))   
    
class GeodeticWaypoint(GeodeticPosition):
    def __init__(self, lat, lon, height, hdg, x_earth=0.0, y_earth=0.0):
        self.heading = hdg
        super().__init__(lat, lon, height, x_earth=x_earth, y_earth=y_earth)
    def __repr__(self):
        rv = 'lat: ', self.earth_x, 'earth_y: ', self.earth_y, 'earth_z: ', self.earth_z,
        'heading: ', self.heading
        return rv            
    def distance_from(self, x, y):
        return np.linalg.norm(np.array([x-self.x_earth, y-self.y_earth]))
    
    def heading_from(self, x, y):
        x_, y_ = np.array([self.x_earth-x, self.y_earth-y])
#        print('x_', x_)
#        print('y_', y_)        
        return trim_heading(np.arctan2(y_, x_))
    
        
class Path(object):
    """
    builds times - and geometry on init
    
    after that need to recalc if vel profile changes
    
    """
    def __init__(self, wp1, wp2, vel_profile=None):
        self.beg_point = wp1
        self.end_point = wp2
        self.offset = 3.0
        self.N = 100
        self.velocity_profile = vel_profile
        
        self.path, self.control_points = calc_4points_bezier_path(
            wp1.loc[0], wp1.loc[1], wp1.heading, wp2.loc[0], wp2.loc[1], wp2.heading, self.offset, n=self.N)
        self.times = self.calc_times()
        
        self.geometries = [self.geometry(i) for i in range(self.N)]
        
    def geometry(self, i):  # Number in [0, 1]
        x_target, y_target = bezier(i, self.control_points)
        derivatives_cp = bezier_derivatives_control_points(self.control_points, 2)
        point = bezier(i, self.control_points)
        di = bezier(i, derivatives_cp[1])
        ddi = bezier(i, derivatives_cp[2])
        # Radius of curvature
        curv = curvature(di[0], di[1], ddi[0], ddi[1])
        radius = 1 / curv
        # Normalize derivative
        di /= np.linalg.norm(di, 2)
        tangent = np.array([point, point + di])
        normal = np.array([point, point + [- di[1], di[0]]])
#        curvature_center = point + np.array([- di[1], di[0]]) * radius
#        circle = plt.Circle(tuple(curvature_center), radius,
#                            color=(0, 0.8, 0.8), fill=False, linewidth=1)
        return curv, tangent, normal
        
    def calc_times(self, base_time = 0.0):
        num_pts = len(self.path)
        tt = self.total_time()
        dt = tt / float(num_pts)
        times = [i * dt + base_time for i in range(num_pts)]
        return times
    
    def total_time(self):
        t = 0.0
        for i in range(len(self.path) - 1):
            ds = np.sqrt((self.path[i][0] - self.path[i+1][0])**2 + (self.path[i][1] - self.path[i+1][1])**2)
            vel = self.velocity_profile(t)
            t += ds / vel
        return t
        
#class FlightPlan(object):
#    
#    """
#    velocity profile is ?
#    constant for now
#    """
#    def __init__(self, waypoints=[]):
#        self.waypoints = waypoints
#        self.nominal_velocity = 65
##        self.total_time = sum([FlightPath(self.waypoints[i], self.waypoints[i+1]).total_time(self.velocity_profile(self.waypoints[i], self.waypoints[i+1])) for i in range(len(self.waypoints)-1)])
##        self.flight_paths = self.make_paths()
#        self.active_waypoint = 0
#        
#    def velocity_profile(self, wp1, wp2):
#        def profile(t):
#            return self.nominal_velocity
#        return profile
#
#    def make_paths(self):
#        flight_paths = []
#        for i in range(len(self.waypoints) - 1):
#            wp1 = self.waypoints[i]
#            wp2 = self.waypoints[i + 1]
#            flight_paths.append(Path(wp1, wp2, self.velocity_profile(wp1, wp2)))
#        self.flight_paths = flight_paths
#        


        
#waypoints = [Waypoint([0.0, 0.0], 0.0, 500),
#             Waypoint([50.0, 50.0], np.pi/2, 500)]
#
##wp1 = Waypoint([0.0, 0.0], 0.0, 500)
##wp2 = Waypoint([50.0, 50.0], np.pi/2, 500)
#             
##f = FlightPath(wp1, wp2)
#fp = FlightPlan(waypoints)
#fp.make_paths()
#print('this')

#from simulation import plots
#plots.flight_plan_plot(fp)
#    
#class GroundStation(object):
#    """
#    store
#    wind map
#    obstacle map
#    elevation map
#    
#    
#    receive
#    aircraft state
#    
#    target state
#        pos. vel. proj. path
#        
#    send
#    intercept
#        time pos hdg (aircraft body frame)
#    
#    
#    """
#    def __init__(self, sim, lat=0, lon=0, aircraft_altitude=1000, elevation=None, elevation_map=None, wind_map=None):
#        
#        self.GROUND_STATION_GPS_LOCATION = None
#        self.GROUND_STATION_LOCAL_LOCATION = np.array([0.0, 0.0], dtype=float)
#        
#        self.sim = sim
#        #earth position is in -z system
#        
#        self.position = np.array([sim.system.full_state.earth_x,
#                        sim.system.full_state.earth_y,
#                        -sim.system.full_state.earth_z])
#        
#        
#        self.heading = sim.system.full_state.attitude.psi
#        self.state = sim.system.full_state
#        
#        self.targets = [Target()]
#        
#        
#        self.elevation_map = elevation_map
#        self.wind_map = wind_map
#    
#    def cruise_time(self, d):
#        pass
#    def distance(self, pos):
#        return np.linalg.norm(self.position - pos)
#    
#    
#    def calc_intercept(self, target_i=0, cur_time=0.0):
#        """
#        
#        this will be iterative .. the heart of the path_finding algo?
#        
#        repeat process when intercept time or location changes by a set amount?
#        
#        
#        
#        calc fastest path .. then adjust heading?
#        
#        """
#        t = self.targets[i]
#
#        distance = np.linalg.norm(t.position - pos) 
#        
#        if distance < 50.0:
#            print('target in range')
#            exit(0)
#        
#        
#        
#        hdg = self.heading
#        
#        
#        calc_cruise_time
#        
#        extrapolate_target_path
#        pass
#



#
#"""Show the effect of the offset."""
#start_x = 10.0  # [m]
#start_y = 1.0  # [m]
#start_yaw = np.radians(180.0)  # [rad]
#
#end_x = -0.0  # [m]
#end_y = -3.0  # [m]
#end_yaw = np.radians(0.0)  # [rad]
#
##for offset in np.arange(1.0, 5.0, 1.0):
#for offset in [1.0]:
#    path, control_points = calc_4points_bezier_path(
#        start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)
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
#        plt.plot(path.T[0], path.T[1], label="Offset=" + str(offset))

#if show_animation:  # pragma: no cover
#    plot_arrow(start_x, start_y, start_yaw)
#    plot_arrow(end_x, end_y, end_yaw)
#    plt.legend()
#    plt.axis("equal")
#    plt.grid(True)
#    plt.show()


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