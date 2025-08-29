"""
Lab Assignment 1 - Dual Cobot Assembly Task
Author: <Jonathan Davied>
Date: <2025-09-03>
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import keyboard
from scipy import linalg
from spatialmath import SE3
from spatialmath.base import transl, trotx, troty, tr2rpy, r2q
from roboticstoolbox import DHLink, DHRobot
import swift
import spatialgeometry as geometry
import os
import asyncio
from ir_support import LinearUR3
# Ensure there's an event loop running (Python 3.10+ fix)
try:
    asyncio.get_running_loop()
except RuntimeError:
    asyncio.set_event_loop(asyncio.new_event_loop())

# launch the environment
env = swift.Swift()
env.launch()

#reading all appropriate files and setting up the robot
current_dir = os.path.dirname(os.path.abspath("/Users/minibeardman/Desktop/Industrial-Robotics/Assignment_1/Creating the Workspace/HalfSizedRedGreenBrick.stl"))
stl_path = os.path.join(current_dir, "HalfSizedRedGreenBrick.stl")
r1 = LinearUR3()
r1.base = r1.base * SE3(0.5,0.5,0) * SE3.Rz(pi/2)

#creation of the workspace
bricks = []
spacing = 0.2 
r1.add_to_env(env)

for i in range(3):      # rows
    for j in range(3):  # columns
        x_pos = i * spacing
        y_pos = j * spacing
        
        # alternate colors (red and green)
        color = (0.5, 0, 0, 1) if (i + j) % 2 == 0 else (0, 0.5, 0, 1)
        
        # brick pose: positioned + rotated 90° about z
        pose = SE3(x_pos, y_pos, 0) @ SE3.Rz(90, 'deg')
        
        brick = geometry.Mesh(
            stl_path,
            pose=pose,
            color=color
        )
        env.add(brick)
        bricks.append(brick)

env.step(0.01)


