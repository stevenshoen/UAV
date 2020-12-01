#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 16 21:17:48 2020

@author: pi
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from utils.coordinates import body2hor, hor2body

def eff_plot(res):
    labs=[['dq_delev'],
        ['pitch_response'],
        ['dp_dail'],
        ['roll_response']]
       
    fig, axs = plt.subplots(len(labs), 1)
    for i, llist in enumerate(labs):
#        axs[i].set_ylabel(l)
        for l in llist:
            axs[i].plot(range(len(res[l])), res[l], label=l)
            axs[i].grid(True, which='major', axis='y')
#            res[l].plot(label=l, legend=True)
#            axs[i].set_ylabel(l)
        h, l = axs[i].get_legend_handles_labels()
        axs[i].legend(h, l, bbox_to_anchor=(0.95, 1.0))
        
def fls_plot(res_df):
    sensor_limits = [-np.deg2rad(20), np.deg2rad(20)]    
    pixel_limits = [-200, 200]    
    guide_distance = 100.0

    fig = plt.figure(figsize=plt.figaspect(1.0))
    plt.xlim(*pixel_limits)        
    plt.ylim(*pixel_limits)
    
    pos = np.array([np.array((res_df['x_earth'] - res_df['target_x']).dropna()),
            np.array((res_df['y_earth'] - res_df['target_y']).dropna()),
            np.array((res_df['height'] - res_df['target_alt']).dropna())]).T
    
    theta = np.array(res_df['theta'])
    phi = np.array(res_df['phi'])
    psi = np.array(res_df['psi'])
    
#    climb_cm = plt.get_cmap('Greys')
#    roll_cm = plt.get_cmap('Greys')
    
        
    guide_colors = np.linspace(0, 1, len(res_df['guidance_roll']))
    climb_cm = plt.cm.Blues(guide_colors)
    roll_cm = plt.cm.Reds(guide_colors)

    
    cntr = np.array([0, 0], dtype=float)
    
    for i in range(len(res_df.index)):
        roll = res_df['guidance_roll'].values[i]
        climb = res_df['guidance_climb'].values[i]
        climb_pt = np.array([0.0, np.tan(climb) * guide_distance])
        roll_pt = np.array([np.sin(roll) * guide_distance, np.cos(roll) * guide_distance])
        
        plt.plot([cntr[0], (cntr + climb_pt)[0]],
                  [cntr[1], (cntr + climb_pt)[1]],
                   color=climb_cm[i])
        
        plt.plot([cntr[0], (cntr + roll_pt)[0]], 
                  [cntr[1], (cntr + roll_pt)[1]], 
                   color=roll_cm[i])
        
    fls_target_pos = np.array([hor2body(pos[i], theta[i], phi[i], psi[i]) for i in range(len(pos))])
    
    fls_target_x = fls_target_pos[:, 0]
    fls_target_y = fls_target_pos[:, 1]
    fls_target_z = fls_target_pos[:, 2]
    
    dist = np.sqrt(fls_target_x**2 + fls_target_y**2 + fls_target_z**2)
    
    fls_target_x_ang = np.arctan2(fls_target_y, fls_target_x)
    fls_target_y_ang = -np.arctan2(fls_target_z, fls_target_x)
    
    fls_target_x_pix = np.clip(fls_target_x_ang * dist, *pixel_limits)
    fls_target_y_pix = np.clip(fls_target_y_ang * dist, *pixel_limits)
    
    cm = plt.get_cmap('Greys')
    colors = np.linspace(0, 1, len(fls_target_x_ang))
    #for j in range(len(fls_target_x_ang)):
    plt.scatter(fls_target_x_pix, fls_target_y_pix, color=colors, cmap=cm, marker='x')
    plt.grid(True, which='major')
    
    
def flightplan_map(fp, res_df, every=25):
#    fig, ax = plt.subplots()
    fig = plt.figure()
#    fig = plt.figure(figsize=plt.figaspect(1.0))
    ax = fig.gca()
    ax.invert_yaxis()
    ax.set_aspect('equal')
    for i, pt in enumerate(fp.waypoints):
        x = pt.x_earth
        y = pt.y_earth
#        h = pt.height
        hdg = pt.heading
        plt.plot(x, y, marker='H')
        label = "waypoint: " + str(i) + " /n heading: " + str(hdg)
        circle = plt.Circle((x, y), fp.waypoint_radius, color='orange', fill=False, alpha=0.5)
        ax.add_artist(circle)
        plt.annotate(label,
                     (x, y),
                    textcoords='offset points',
                    xytext=(0, 0),
                    ha='center')
    for i in res_df['waypoint_heading'].dropna().index[::every]:
        plot_arrow(res_df['x_earth'][i], res_df['y_earth'][i], res_df['waypoint_heading'][i], length=5.0, width=2.0, fc="b", ec="k")
    for i in res_df['guidance_heading'].dropna().index[::every]:
        plot_arrow(res_df['x_earth'][i], res_df['y_earth'][i], res_df['psi'][i] - res_df['guidance_heading'][i], length=2.0, width=1.0, fc="r", ec="k")
        
    plt.scatter(res_df['x_earth'], res_df['y_earth'], marker='.')

def target_flight_map(res):
    fig = plt.figure(figsize=plt.figaspect(1.0))
    plt.gca().invert_yaxis()
    plt.scatter(res['x_interc'], res['y_interc'], c='blue', marker='.')
    plt.scatter(res['target_x'], res['target_y'], c='red', marker='x')
    plt.scatter(res['x_earth'], res['y_earth'], c='black', marker='.')


def g_plot(fp):
    
    for path in fp.flight_paths:
        radii = []
        times = path.times
        for i in range(len(times)):
            radius, tangent, normal = path.geometry(i)
            radii.append(radius)
        print(radii)
        plt.plot(times, radii)
        
def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    """Plot arrow."""
#    if not isinstance(x, float):
#        for (ix, iy, iyaw) in zip(x, y, yaw):
#            plot_arrow(ix, iy, iyaw)
#    else:
    plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
              fc=fc, ec=ec, head_width=width, head_length=width, alpha = 0.5)
#    plt.plot(x, y)
        
def flight_plan_plot(flightplan):
    num_segs = len(flightplan.flight_paths)
    fig, ax = plt.subplots()
    for i in range(num_segs):
        path = flightplan.flight_paths[i].path
        control_points = flightplan.flight_paths[i].control_points
    
        
        ax.plot(path.T[0], path.T[1], label="Bezier Path")
        ax.plot(control_points.T[0], control_points.T[1], '--o', label="Control Points")
        
#        ax.plot(x_target, y_target)
#    ax.plot(tangent[:, 0], tangent[:, 1], label="Tangent")
#    ax.plot(normal[:, 0], normal[:, 1], label="Normal")
#    ax.add_artist(circle)
        plot_arrow(path.T[0][0], path.T[0][1], flightplan.flight_paths[i].beg_point.heading)
        plot_arrow(path.T[-1][0], path.T[-1][1], flightplan.flight_paths[i].end_point.heading)
    ax.legend()
    ax.axis("equal")
    ax.grid(True)
#    plt.show()    
#plt.ioff()
def effection_plot(mapped_variable):
    m = mapped_variable
    alphas = m.alpha_range
    betas = m.beta_range

    surface = np.matrix((len(alphas), len(betas)))
    
    for i, a in enumerate(alphas):
        
        for j, b in enumerate(betas):
            
            surface[i, j] = m.estimate(a, b)
#    
#    
#    for pt in m.measured_points:
#        a_i = m.nearest_i(alphas, pt[1])
#        b_i = m.nearest_i(betas, pt[2])
#        surface[a_i, b_i] =  pt[0]
#    
#    for a in alphas:
#        for b in betas:
#            surface[np.where(alphas == a), np.where(betas == b)] = m.estimate(a, b)
    print(surface)
    fig = plt.figure()
#    ax = fig.gca(projection='3d')
#    ax.plot_surface(aa, bb, surface)
    fig.matshow(surface)
#    ax.plot_surface(alphas, betas, surface)
    
def effection_plot2(mapped_variable):
    m = mapped_variable
    alphas = m.alpha_range
    betas = m.beta_range

    surface = np.ndarray((len(alphas), len(betas)))
    for a in alphas:
        for b in betas:
            surface[np.where(alphas == a), np.where(betas == b)] = m.estimate(a, b)
    print(surface)
    fig = plt.figure()
    ax = fig.gca(projection='3d')
#    ax.plot_surface(aa, bb, surface)
    ax.plot_surface(alphas, betas, surface)
    
def timer_plot(res):
    labs = [['navigation_time'],
            ['guidance_time'],
            ['damper_time'],
            ['response_time']]
    fig, axs = plt.subplots(len(labs), 1)
    for i, llist in enumerate(labs):
        for l in llist:
            axs[i].set_ylim(res[l].min(), res[l].max())
            print((res[l].min(), res[l].max()))
            axs[i].scatter(range(len(res[l])), res[l], label=l)
            axs[i].grid(True, which='major', axis='y')
            axs[i].set_ylabel(l)
            axs[i].autoscale(enable=True, axis='both', tight=True)
        h, l = axs[i].get_legend_handles_labels()
        axs[i].legend(h, l, bbox_to_anchor=(1.0, 0.9), loc='upper right')   
        
def environ_plot(res):
    labs=[['x_earth', 'y_earth'],
        ['wind_x', 'wind_y'],
        ['height', 'waypoint_altitude'],
        ['TAS'],
        ['thrust'],
        ['thrust_correction']]
    fig, axs = plt.subplots(len(labs), 1)
    for i, llist in enumerate(labs):
#        axs[i].set_ylabel(l)
        for l in llist:
            axs[i].plot(range(len(res[l])), res[l], label=l)
            axs[i].grid(True, which='major', axis='y')
#            res[l].plot(label=l, legend=True)
#            axs[i].set_ylabel(l)
        h, l = axs[i].get_legend_handles_labels()
        axs[i].legend(h, l, bbox_to_anchor=(0.95, 1.0))

def yaw_dampers_plot(res):
    labs = [['psi'],
            ['guidance_heading'],
            ['waypoint_heading'],
            ['yaw_correction'],
            ['yaw_err', 'yaw_rate_err'],
            ['rudder']]
    fig, axs = plt.subplots(len(labs), 1)
    for i, llist in enumerate(labs):
        for l in llist:
            axs[i].plot(range(len(res[l])), res[l], label=l)
            axs[i].grid(True, which='major', axis='y')
            axs[i].set_ylabel(l)
        h, l = axs[i].get_legend_handles_labels()
        axs[i].legend(h, l, bbox_to_anchor=(1.0, 0.9), loc='upper right')        

def roll_dampers_plot(res):
    labs = [['phi'],
            ['p'],
            ['aileron'],
            ['roll_err', 'roll_rate_err'],
            ['roll_correction'],
            ['guidance_roll'],
            ['psi'],
            ['guidance_heading'],
            ['waypoint_heading']
            ]
    fig, axs = plt.subplots(len(labs), 1)
    for i, llist in enumerate(labs):
        for l in llist:
            axs[i].plot(range(len(res[l])), res[l], label=l)
            axs[i].grid(True, which='major', axis='y')
            axs[i].set_ylabel(l)
        h, l = axs[i].get_legend_handles_labels()
        axs[i].legend(h, l, bbox_to_anchor=(1.0, 0.9), loc='upper right')        

def pitch_dampers_plot(res):
    labs = [['theta'],
            ['w'],
            ['elevator'],
            ['pitch_correction'],
            ['height', 'waypoint_altitude'],
            ['guidance_climb', 'guidance_climb_dot'],
            ['pitch_err', 'pitch_rate_err']]
    fig, axs = plt.subplots(len(labs), 1)
    for i, llist in enumerate(labs):
        for l in llist:
            axs[i].plot(range(len(res[l])), res[l], label=l)
            axs[i].grid(True, which='major', axis='y')
            axs[i].set_ylabel(l)
        h, l = axs[i].get_legend_handles_labels()
        axs[i].legend(h, l, bbox_to_anchor=(1.0, 0.9), loc='upper right')
        
def dampers_plot(res):
    labs = [['theta'],
            ['elevator'],
            ['pitch_correction'],
            ['pitch_err', 'pitch_rate_err'],
            ['phi'],
            ['aileron'],
            ['roll_correction'],
            ['roll_err', 'roll_rate_err'],
            ['psi'],
            ['yaw_correction'],
            ['rudder']]
#            ['psi', 'rudder']]

    fig, axs = plt.subplots(len(labs), 1)
    for i, llist in enumerate(labs):
#        axs[i].set_ylabel(l)
        for l in llist:
            axs[i].plot(range(len(res[l])), res[l], label=l)
            axs[i].grid(True, which='major', axis='y')
#            res[l].plot(label=l, legend=True)
            axs[i].set_ylabel(l)
        h, l = axs[i].get_legend_handles_labels()
        axs[i].legend(h, l, bbox_to_anchor=(1.0, 0.9), loc='upper right')
#            plt.legend()
#        plt.show()
    

        
def flight_plot(res, every=10):
#    fuselen = 15.0
#    wingspan = 15.0
    fig = plt.figure(figsize=plt.figaspect(1.0))
    ax=fig.gca(projection='3d')
    plt.gca().invert_yaxis()
    x, y, z, u, v, w, wu, wv, ww = [], [], [], [], [], [], [], [], []
    #ax.quiver(x, y, z, u, v, w, length=0.1, normalize=False)
    i_s = range(len(res[list(res)[0]]))[::every]
    
    for i in i_s:
    #for i in range(5):
        x.append(res['x_earth'][i])
        y.append(res['y_earth'][i])
        z.append(res['z_earth'][i])
        fuselen = res['u'][i]        
        wingspan = fuselen
        
        pt = np.array([res['x_earth'][i], res['y_earth'][i], res['z_earth'][i]])
        
        
        phi = res['phi'][i]
        psi = res['psi'][i]
        theta = res['theta'][i]

        u = res['u'][i]
        v = res['v'][i]
        w = res['w'][i]



        fuse_dir = body2hor(np.array([fuselen, 0, 0]), theta, phi, psi)
        
        wing_dir = body2hor(np.array([0, wingspan, 0]), theta, phi, psi)
        
        dx, dy, dz = body2hor(np.array([u, v, w]), theta, phi, psi)
        
    
        
    #    print(pt, fuse_dir)
#        plt.plot([pt[0], (pt - fuse_dir)[0]],
#                  [pt[1], (pt - fuse_dir)[1]],
#                  [pt[2], (pt - fuse_dir)[2]],
#                  color='green')
    #    print(pt, fuse_dir)
        plt.plot([(pt - 2*fuse_dir)[0], (pt + fuse_dir)[0]],
                  [(pt - 2*fuse_dir)[1], (pt + fuse_dir)[1]],
                  [(pt - 2*fuse_dir)[2], (pt + fuse_dir)[2]],
                  color='black',
                  label='fuselage',
                  linewidth=1.5)
    #    print(pt, wing_dir)
        plt.plot([(pt - wing_dir)[0], (pt + wing_dir)[0]],
                  [(pt - wing_dir)[1], (pt + wing_dir)[1]],
                  [(pt - wing_dir)[2], (pt + wing_dir)[2]],
                  color='blue',
                  label='wing',
                  linewidth=1.5)
                          
        plt.plot([pt[0], pt[0] + dx],
                  [pt[1], pt[1] + dy],
                  [pt[2], pt[2] + dz],
                  color='red',
                  label='velocity',
                  linewidth=1.5)
                  
    ax.legend(bbox_to_anchor=(0.0, 0.0))
        
#def rud_plot(res):
#    labs = ['rud_CY', 'rud_Cl', 'rud_CN']
#    fig, axs = plt.subplots(len(labs), 1)
#    for i, l in enumerate(labs):
#        axs[i].set_ylabel(l)
#        axs[i].plot(range(len(res[l])), res[l], label=l)
#        plt.legend()
#        
#def ail_plot(res):
#    labs = ['ail_Cl', 'ail_CN']
#    fig, axs = plt.subplots(len(labs), 1)
#    for i, l in enumerate(labs):
#        axs[i].set_ylabel(l)
#        axs[i].plot(range(len(res[l])), res[l], label=l)
#        plt.legend()
#
#        
#def elev_plot(res):
#    labs = ['elev_CL', 'elev_CD', 'elev_CM']
#    fig, axs = plt.subplots(len(labs), 1)
#    for i, l in enumerate(labs):
#        axs[i].set_ylabel(l)
#        axs[i].plot(range(len(res[l])), res[l], label=l)
#        plt.legend()
#
#def controls_plot(res):
#    labs = ['elevator', 'aileron', 'rudder', 'thrust']
#    fig, axs = plt.subplots(len(labs), 1)
#    for i, l in enumerate(labs):
#        axs[i].set_ylabel(l)
#        axs[i].plot(range(len(res[l])), res[l], label=l)
#        plt.legend()
##    plt.show()        