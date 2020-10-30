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
    
def environ_plot(res):
    w_xs = [res['wind'][i][0] for i in range(len(res['wind']))]
    w_ys = [res['wind'][i][1] for i in range(len(res['wind']))]
    fig, axs = plt.subplots(4, 1)
    
    axs[0].set_ylabel('wind')
    axs[0].plot(range(len(w_xs)), w_xs, label='wind_x')
    axs[0].plot(range(len(w_ys)), w_ys, label='wind_y')
    
#            axs[i].plot(range(len(res[l])), res[l], label=l)
    axs[2].set_ylabel('XY pos')
    axs[1].scatter(res['x_earth'], res['y_earth'])
    
    axs[2].set_ylabel('altitude')
    axs[2].plot(res['height'])
    
    axs[3].set_ylabel('TAS')
    axs[3].plot(res['TAS'])
#   

def telem_plot(res):
    
    labs = ['theta', 'phi', 'psi']
    fig, axs = plt.subplots(len(labs), 1, fig=plt.Figure('telem'))
    for i, l in enumerate(labs):
        axs[i].set_ylabel(l)
        axs[i].plot(range(len(res[l])), res[l], label=l)
        plt.legend()
#    plt.show()

def rud_plot(res):
    labs = ['rud_CY', 'rud_Cl', 'rud_CN']
    fig, axs = plt.subplots(len(labs), 1)
    for i, l in enumerate(labs):
        axs[i].set_ylabel(l)
        axs[i].plot(range(len(res[l])), res[l], label=l)
        plt.legend()
        
def ail_plot(res):
    labs = ['ail_Cl', 'ail_CN']
    fig, axs = plt.subplots(len(labs), 1)
    for i, l in enumerate(labs):
        axs[i].set_ylabel(l)
        axs[i].plot(range(len(res[l])), res[l], label=l)
        plt.legend()

        
def elev_plot(res):
    labs = ['elev_CL', 'elev_CD', 'elev_CM']
    fig, axs = plt.subplots(len(labs), 1)
    for i, l in enumerate(labs):
        axs[i].set_ylabel(l)
        axs[i].plot(range(len(res[l])), res[l], label=l)
        plt.legend()

def controls_plot(res):
    labs = ['elevator', 'aileron', 'rudder', 'thrust']
    fig, axs = plt.subplots(len(labs), 1)
    for i, l in enumerate(labs):
        axs[i].set_ylabel(l)
        axs[i].plot(range(len(res[l])), res[l], label=l)
        plt.legend()
#    plt.show()
        
        
def flight_plot(res, every=1):
    fig = plt.figure(figsize=plt.figaspect(1.0))
    ax = fig.gca(projection='3d')
    x, y, z, u, v, w, wu, wv, ww = [], [], [], [], [], [], [], [], []
    #ax.quiver(x, y, z, u, v, w, length=0.1, normalize=False)
    i_s = range(len(res[list(res)[0]]))
    
    for i in range(len(res[list(res)[0]]))[::every]:
    #for i in range(5):
        x.append(res['x_earth'][i])
        y.append(res['y_earth'][i])
        z.append(res['z_earth'][i])
        
        pt = np.array([res['x_earth'][i], res['y_earth'][i], res['z_earth'][i]])
        
        
        phi = res['phi'][i]
        psi = res['psi'][i]
        theta = res['theta'][i]
        
        fuse_dir = np.array([np.cos(psi) * np.cos(theta),
                             np.sin(psi) * np.cos(theta),
                             np.sin(theta)])
        
        fuse_dir = fuse_dir / np.linalg.norm(fuse_dir)
        
        wing_dir = np.array([np.sin(psi) * np.cos(phi),
                             np.cos(psi) * np.cos(phi),
                             np.sin(phi)])
        
        wing_dir = wing_dir / np.linalg.norm(wing_dir)
        
    #    print(pt, fuse_dir)
        plt.plot([pt[0], (pt + fuse_dir)[0]],
                  [pt[1], (pt + fuse_dir)[1]],
                  [pt[2], (pt + fuse_dir)[2]],
                  color='green')
    #    print(pt, fuse_dir)
        plt.plot([pt[0], (pt - 2*fuse_dir)[0]],
                  [pt[1], (pt - 2*fuse_dir)[1]],
                  [pt[2], (pt - 2*fuse_dir)[2]],
                  color='red')
    #    print(pt, wing_dir)
        plt.plot([pt[0], (pt + wing_dir)[0]],
                  [pt[1], (pt + wing_dir)[1]],
                  [pt[2], (pt + wing_dir)[2]],
                  color='blue')
        
#        print(pt, wing_dir)
        plt.plot([pt[0], (pt - wing_dir)[0]],
                  [pt[1], (pt - wing_dir)[1]],
                  [pt[2], (pt - wing_dir)[2]],
                  color='blue')        
        
        
        