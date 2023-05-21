from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime
import os

fig = plt.figure()
 
# syntax for 3-D projection
ax = plt.axes(projection ='3d')

def x_torus(t):
    r = 1
    r_big = 5
    return r_big*np.cos(np.pi*(t*0.5)/150)+r*np.cos(np.pi*(t*0.5)/25)*np.cos(np.pi*(t*0.5)/150) -(r_big+r)
def y_torus(t):
    r = 1
    r_big = 3
    b = 3
    return r_big*np.sin(np.pi*(t*0.5)/150)+r*np.cos(np.pi*(t*0.5)/25)*np.sin(np.pi*(t*0.5)/150) 
def z_torus(t):
    r = 1
    r_big = 3
    b = 3
    return r*np.sin(np.pi*(t*0.5)/25) +5


def x_spiral(t):
    return (4-(t*0.5)*0.15)*np.cos(np.pi*(t*0.5)/(100-0.3*(t*0.5))) - 4

def y_spiral(t):
    return (4-(t*0.5)*0.15)*np.sin(np.pi*(t*0.5)/(100-0.3*(t*0.5)))

def z_spiral(t):
    return 5.0


def x_line(t):
    return 1.33226763*10**(-16)*t+1.20000000*10**(-2)*t**2-1.20000000*10**(-4)*t**3


def x_oscillating_circle(t):
    return 5*np.cos(np.pi*(t*0.5)/100) - 5

def y_oscillating_circle(t):
    return 5*np.sin(np.pi*(t*0.5)/100)

def z_oscillating_circle(t):
    return 1*np.sin(np.pi*(t*0.5)/25) +5








t = int(60)
# plotting
ax.plot3D([x_line(i) for i in range(t)], [0 for i in range(t)], [5 for i in range(t)], 'red')
#ax.plot3D([x_spiral(i) for i in range(t)], [y_spiral(i) for i in range(t)], [z_spiral(i) for i in range(t)], 'red')
#ax.plot3D([x_torus(i) for i in range(t)], [y_torus(i) for i in range(t)], [z_torus(i) for i in range(t)], 'red')
#ax.plot3D([x_oscillating_circle(i) for i in range(t)], [y_oscillating_circle(i) for i in range(t)], [z_oscillating_circle(i) for i in range(t)], 'red')
