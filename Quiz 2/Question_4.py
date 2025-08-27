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
steps = 100 # Specify no. of steps

q1 = np.array([-0.4382, 0.2842, -0.7392, -3.142, -0.455, -2.703])
q2 = np.array([0.9113, -0.5942, -0.01781, 0, 0.0612, -0.9113])

q_matrixj = jtraj(q1, q2, steps).q

#velocity calc - quint
vel_quint = np.zeros([steps, 6])
for i in range(0, steps):
    vel_quint[i, :] = q_matrixj[i, :] - q_matrixj[i-1, :]

#trajectory - trap
s = trapezoidal(0, 1, steps).q
trap_matrix = np.empty((steps, 6))
for i in range(0, steps):
    trap_matrix[i, :] = (1 - s[i]) * q1 + s[i] * q2

#velocity calc - trap
vel_trap = np.zeros([steps, 6])
for i in range(0, steps):
    vel_trap[i, :] = trap_matrix[i, :] - trap_matrix[i-1, :]

a = np.abs(np.abs(vel_quint[:,0]) - np.abs(vel_trap[:,0]))
print ('velocity difference: \n', np.max(a))