# -*- coding: utf-8 -*-
"""
Created on Mon Sep 25 15:39:22 2017

@author: Oliver Rose
"""

# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
M = 6 * 10**24
r = np.array([10**7,0,0])
v = np.array([0,6328.070796,0])
a = np.zeros(3)
G = 6.67408 * 10**-11 

# function to normalise position vector
def normalize(v):
    norm=np.linalg.norm(v)
    if norm==0: 
       return v
    return v/norm

# simulation time, timestep and time
t_max = 10000
dt = 0.1
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
r_list = []
v_list = []

# Euler integration
for t in t_array:

    # append current state to trajectories
    r_list.append(r)
    v_list.append(v)
    
    # Unit postion vector r
    r_unit = normalize(r)
    
    # Magnitude postion vector r
    r_mag = np.linalg.norm(r)
    
    # calculate new position and velocity 
    a = - G * M * r_unit / (r_mag**2)
    r = r + dt * v
    v = v + dt * a
    
# convert trajectory lists into arrays, so they can be indexed more easily
r_array = np.array(r_list)
v_array = np.array(v_list)

x_array = np.zeros(len(r_array))
y_array = np.zeros(len(r_array))

for i in range (len(r_array)):
    x_array[i] = r_array[i][0]
    y_array[i] = r_array[i][1]

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.grid()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Path of orbit')
plt.plot(x_array, y_array)
plt.plot([0],[0],'ro')
plt.legend()
plt.show()