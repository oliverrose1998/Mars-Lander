# -*- coding: utf-8 -*-
"""
Created on Tue Oct  3 13:50:39 2017

@author: Oliver Rose
"""

# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# function to normalise position vector
def normalize(v):
    norm=np.linalg.norm(v)
    if norm==0: 
       return v
    return v/norm

# mass of the planet, gravitational constant, initial position and velocity
M = 6 * 10**24
r = np.array([10**7,0,0])
v = np.array([0,20000,0])
a = np.zeros(3)
G = 6.67408 * 10**-11 

# simulation time, timestep and time
t_max = 100000
dt = 1
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
r_list = [0]*len(t_array)
v_list = [0]*len(t_array)

# 1st and 2nd terms displacement
r_list[0] = r
r_list[1] = r + v * dt

# 1st and 2nd terms velocity
v_list[0] = v
v_list[1] = (r_list[1] - r_list[0]) / dt

# Iterate for new position and velocity
for i in range (len(r_list)-2):
    
    # Unit postion vector r
    r_unit = normalize(r_list[i+1])
    
    # Magnitude postion vector r
    r_mag = np.linalg.norm(r_list[i+1])
    
    r_list[i+2] = 2 * r_list[i+1] - r_list[i] - dt**2 * G * M * r_unit / (r_mag**2)
    v_list[i+2] = (r_list[i+1] - r_list[i]) / dt
    
# convert trajectory lists into arrays, so they can be indexed more easily
r_array = np.array(r_list)
v_array = np.array(v_list)

# empty arrays for x and y coordinates
x_array = np.zeros(len(r_array))
y_array = np.zeros(len(r_array))

# compiling x and y coordinates
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