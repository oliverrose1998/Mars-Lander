# -*- coding: utf-8 -*-
"""
Created on Mon Sep 25 15:51:14 2017

@author: Oliver Rose
"""
# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 1
v = 1

# simulation time, timestep and time
t_max = 1000
dt = 2
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = []
v_list = []


for t in t_array:

    # append current state to trajectories
    x_list.append(x)
    v_list.append(v)

# 1st and 2nd terms displacement
x_list[0] = x
x_list[1] = 2 * x_list[0] - dt**2 * k * x_list[0] / m

# 1st and 2nd terms velocity
v_list[0] = v
v_list[1] = (x_list[1] - x_list[0]) / dt

# Iterate for new position and velocity
for i in range (len(x_list)-2):
    
    x_list[i+2] = 2 * x_list[i+1] - x_list[i] - dt**2 * k * x_list[i+1] / m 
    v_list[i+2] = (x_list[i+1] - x_list[i]) / dt
    
    # convert trajectory lists into arrays, so they can be indexed more easily
x_array = np.array(x_list)
v_array = np.array(v_list)

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array, label='x (m)')
plt.plot(t_array, v_array, label='v (m/s)')
plt.legend()
plt.show()