# -*- coding: utf-8 -*-
"""
Created on Thu Feb 11 16:30:56 2016

@author: ubuntu
"""
import numpy as np
from simulation.simtools import Target, Intercept
from simulation import plots
from state.position import EarthPosition, GeodeticPosition

from ground_station.ground_station import World, EarthWaypoint, GeodeticWaypoint

# start location
gs_loc = (45.805, -108.6, 5000)
# define path
loc0 = (45.806, -108.606, 5000)
hdg0 = 0.0
loc1 = (45.675, -108.7715, 5000)
hdg1 = 0.54
wps = [GeodeticWaypoint(loc0, hdg0), # x_earth=0.0, y_earth=0.0),
       GeodeticWaypoint(loc1, hdg1)]
       
aircraft_hdg = hdg0
aircraft_loc = EarthPosition(0.0, 0.0, 6000, lat=45.8055, lon=-108.605)

w = World(gs_loc, wps, aircraft_loc, aircraft_hdg)

print('ground station at: ', w.gs_position.lat, w.gs_position.lon, w.gs_position.height)
print('ground station at: ', w.gs_position.x_earth, w.gs_position.y_earth, w.gs_position.height)
for i, pt in enumerate(w.waypoints):
    print('wp ', i+1)
    print('lat: ', pt.lat, ' lon: ', pt.lon, 'hdg: ', pt.heading)
    print('x: ', pt.x_earth, ' y: ', pt.y_earth, 'z: ', pt.z_earth)

#plots.flight_map(w)
"""
- make world

-make aircraft-environment

-get trim state

-make system(aircraft_state, pos/att from world)



-make simulation
    -make controller
        - load path from world
        - autostarts here



"""

#
#fp = FlightPlan(waypoints)
#fp.make_paths()


#drads = [] # m/s
#dtang = [] # m/s
#
#last_dt = fp.flight_paths[0].times[0]
#
#for j in range(len(fp.flight_paths)):
#    path0 = fp.flight_paths[j]
#    geo = path0.geometries
#    times = path0.times
#    
#    
#    
#    
#    for i in range(len(times))[1:]:
#        # radius tangent normal 
#        dt = times[i] - last_dt
#        drads.append((geo[i][0] - geo[i-1][0])/ dt)
#        tang_vecs = geo[i][1] - geo[i-1][1]
#        dtang_ = np.linalg.norm(tang_vecs[0] - tang_vecs[1])
#    
#    #    dtang_ = np.linalg.norm(geo[i][1] - geo[i-1][1])
#        dtang.append(dtang_/ dt)
#        
#        last_dt = dt
#
#
#
#
#
#
#
#
#plots.g_plot(fp)
#
#print('this')
#
#plots.flight_plan_plot(fp)