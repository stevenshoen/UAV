# -*- coding: utf-8 -*-
"""
Created on Fri Nov 20 11:54:24 2020

@author: ubuntu
"""
fig = plt.figure(figsize=plt.figaspect(1.0))

sensor_limits = [-np.deg2rad(20), np.deg2rad(20)]

pos = np.array([np.array((res_df['x_earth'] - res_df['target_x']).dropna()),
        np.array((res_df['y_earth'] - res_df['target_y']).dropna()),
        np.array((res_df['height'] - res_df['target_alt']).dropna())]).T

theta = np.array(res_df['theta'])
phi = np.array(res_df['phi'])
psi = np.array(res_df['psi'])

    

fls_target_pos = np.array([hor2body(pos[i], theta[i], phi[i], psi[i]) for i in range(len(pos))])

fls_target_x = fls_target_pos[:, 0]
fls_target_y = fls_target_pos[:, 1]
fls_target_z = fls_target_pos[:, 2]

fls_target_x_ang = np.arctan2(fls_target_y, fls_target_x)
fls_target_y_ang = -np.arctan2(fls_target_z, fls_target_x)

#fls_target_x_ang = np.clip(fls_target_x_ang, sensor_limits[0], sensor_limits[1])
#fls_target_y_ang = np.clip(fls_target_y_ang, sensor_limits[0], sensor_limits[1])
cm = plt.get_cmap('Greys')
colors = np.linspace(0, 1, len(fls_target_x_ang))
#for j in range(len(fls_target_x_ang)):
plt.scatter(fls_target_x_ang, fls_target_y_ang, color=colors, cmap=cm, marker='x')

