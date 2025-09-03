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
from roboticstoolbox import jtraj
import roboticstoolbox as rtb
import swift
import spatialgeometry as geometry
import os
import asyncio
from ir_support import LinearUR3
from math import pi
import Calculations
import Movements
import Setup

# Each brick is 0.0667x * 0.1334y * 0.0334z
# but ive rotated it to be 0.1334x * 0.0667y * 0.0334z
   

if __name__ == "__main__":
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        asyncio.set_event_loop(asyncio.new_event_loop())

    # Launch the environment
    environment = swift.Swift()
    environment.launch()

    # Setup robot and bricks
    environment, robot = Setup.setup_robot(environment)
    Calculations.set_initial_q(robot)
    environment = Setup.setup_environment(environment)
    environment, bricks, poses = Setup.setup_bricks(environment, robot)

    # Wall poses to build the wall
    wall_poses = [
        SE3(-0.1334, 1, 0) @ SE3.Rz(pi / 2), SE3(0, 1, 0) @ SE3.Rz(pi / 2), SE3(0.1334, 1, 0) @ SE3.Rz(pi / 2), 
        SE3(-0.1334, 1, 0.0334) @ SE3.Rz(pi / 2), SE3(0, 1, 0.0334) @ SE3.Rz(pi / 2), SE3(0.1334, 1, 0.0334) @ SE3.Rz(pi / 2), 
        SE3(-0.1334, 1, 0.0668) @ SE3.Rz(pi / 2), SE3(0, 1, 0.0668) @ SE3.Rz(pi / 2), SE3(0.1334, 1, 0.0668) @ SE3.Rz(pi / 2)
    ]
    print("Wall poses created")

    # Pick-and-place each brick to its corresponding wall pose
    for i, brick in enumerate(bricks):
        target_pose = wall_poses[i]
        print(f"Placing brick {i} at wall position {i}")
        Movements.pick_and_place(robot, environment, brick, target_pose)

    environment.hold()