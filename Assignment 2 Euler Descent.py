# -*- coding: utf-8 -*-
"""
Created on Mon Oct  2 17:27:33 2017

@author: Oliver Rose
"""

# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
M = 6 * 10**24
r = np.array([6*10**6,0,0])
v = np.zeros(3)
a = np.zeros(3)
G = 6.67408 * 10**-11 

# function to normalise position vector
def normalize(v):
    norm=np.linalg.norm(v)
    if norm==0: 
       return v
    return v/norm

# simulation time, timestep and time
t_max = 900
dt = 1
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

# create empty array of magnitudes
r_array_mag = np.zeros(len(r_array))
v_array_mag = np.zeros(len(r_array))

# convert array of vectors into array of magnitudes
for i in range (len(r_array)):
    r_array_mag[i] = np.linalg.norm(r_array[i])
    
for i in range (len(v_array)):
    v_array_mag[i] = np.linalg.norm(v_array[i])  


# plot graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()

# plot position-time
plt.subplot(211)
plt.plot(t_array, r_array_mag, label='x (m)')
plt.legend()

# plot velocity-time
plt.subplot(212)
plt.plot(t_array, v_array_mag, label='v (m/s)')
plt.legend()

plt.show()