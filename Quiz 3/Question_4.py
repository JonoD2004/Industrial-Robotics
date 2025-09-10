import numpy as np
import matplotlib.pyplot as plt
import time
import swift
from spatialmath.base import *
from spatialmath import SE3
from roboticstoolbox import models, jtraj, trapezoidal
from spatialgeometry import Sphere, Arrow

# Useful variables
from math import pi

## Options                      
steps = 45 # Specify no. of steps
q1 = np.array([pi/10, pi/7, pi/5, pi/3, pi/4, pi/6])
q2 = np.array([-pi/6, -pi/3, -pi/4, -pi/8, -pi/7, -pi/10])

#trajectory - trap
s = trapezoidal(0, 1, steps).q
trap_matrix = np.empty((steps, 6))
trap_matrix[0, :] = q1
for i in range(1, steps):
    trap_matrix[i, :] = (1 - s[i]) * q1 + s[i] * q2

#velocity calc - trap
vel_trap = np.zeros([steps, 6])
for i in range(1, steps):
    vel_trap[i, :] = trap_matrix[i, :] - trap_matrix[i-1, :]

a = np.max(np.abs(vel_trap))
print ('maximum velocity: \n', a)