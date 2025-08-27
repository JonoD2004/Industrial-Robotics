import numpy as np
import matplotlib.pyplot as plt
import time
import keyboard
import swift
from scipy import linalg
from spatialmath.base import plotvol3
from spatialmath import SE3
from spatialgeometry import Cylinder, Cuboid, Sphere
from roboticstoolbox import DHLink, DHRobot, jtraj
from ir_support import line_plane_intersection
from roboticstoolbox.models.DH import Sawyer
from ir_support.robots import DensoVS060

# Useful variables
from math import pi

l1 = DHLink(d=0.475, a=0.180, alpha=-pi/2, offset=0, qlim=np.radians([-170, 170]))
l2 = DHLink(d=0, a=0.385, alpha=0, offset=-pi/2, qlim=np.radians([-90, 135]))
l3 = DHLink(d=0, a=-0.100, alpha=pi/2, offset=pi/2, qlim=np.radians([-80, 165]))
l4 = DHLink(d=0.329 + 0.116, a=0, alpha=-pi/2, offset=0, qlim=np.radians([-185, 185]))
l5 = DHLink(d=0, a=0, alpha=pi/2, offset=0, qlim=np.radians([-120, 120]))
l6 = DHLink(d=0.09, a=0, alpha=0, offset=0, qlim=np.radians([-360, 360]))
robot = DHRobot([l1, l2, l3, l4, l5, l6], name='Denso VS060')

q1 = [0, 0, 0, 0, 0, 0]
q2 = [-0.1745, -0.6981, 2.094, 0.3491, 0.5236, 0.1745]

joint_trajectory = jtraj(q1, q2, 50)
translations = joint_trajectory.q

max_distance = 0
for q in translations:
    T = robot.fkine(q)
    laser = T @ SE3.Tz(6)
    position = laser.t
    distance = np.linalg.norm(position)
    if distance > max_distance:
        max_distance = distance
print("Maximum distance from base to end-effector:", max_distance)
