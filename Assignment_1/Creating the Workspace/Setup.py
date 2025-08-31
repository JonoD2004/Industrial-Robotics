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
from roboticstoolbox import DHLink, jtraj
import roboticstoolbox as rtb
import swift
import spatialgeometry as geometry
import os
import asyncio
from ir_support import LinearUR3
from math import pi
import Calculations
import Movements
import main


def setup_robot(environment):
    # Define robot
    robot = LinearUR3()
    """
    Setup the robot in the environment.

    Returns:
        link1 (q[0]) = 'slider_rail',
        link2 (q[1]) = 'shoulder_ur3',
        link3 (q[2]) = 'upperarm_ur3',
        link4 (q[3]) = 'forearm_ur3',
        link5 (q[4]) = 'wrist1_ur3',
        link6 (q[5]) = 'wrist2_ur3',
        link7 (q[6]) = 'wrist3_ur3'
    """
    robot.base = robot.base * SE3(0,0,0) * SE3.Ry(-pi/2)  # adjust base for better view and

    robot.add_to_env(environment)

    environment.step(1)
    return environment, robot

def setup_bricks(environment):

    def is_too_close(x, y, poses, min_x_spacing = 0.15, min_y_spacing = 0.05):
        """Check if new (x, y) is too close to existing bricks."""
        for pose in poses:
            existing_x, existing_y = pose.t[0], pose.t[1]

            too_close_x = abs(x - existing_x) < min_x_spacing
            too_close_y = abs(y - existing_y) < min_y_spacing

            if too_close_x:
                print(f"    Too close in X to brick at (x={existing_x:.2f})")
            if too_close_y:
                print(f"    Too close in Y to brick at (y={existing_y:.2f})")

            # Only reject if BOTH are too close
            if too_close_x and too_close_y:
                print("     Too close in BOTH X and Y. Please try again.")
                return True  # reject position
        return False      # valid

    current_dir = os.path.dirname(os.path.abspath(
        "/Users/minibeardman/Desktop/Industrial-Robotics/Assignment_1/Creating the Workspace/HalfSizedRedGreenBrick.stl"))
    stl_path = os.path.join(current_dir, "HalfSizedRedGreenBrick.stl")

    bricks = []
    poses = []
    num_bricks = 9
    x_pos_limit, x_neg_limit, y_pos_limit, y_neg_limit = Calculations.Calculate_Robot_Reach()

    # Create a grid of bricks in the workspace
    for i in range(num_bricks):
        print(f"\nBrick {i}:")

        # Keep trying until a valid position is found
        while True:
            x_position = float(input(f"  Enter x position (between {x_neg_limit} and {x_pos_limit}): "))
            y_position = float(input(f"  Enter y position (between {y_neg_limit} and {y_pos_limit}): "))

            if not is_too_close(x_position, y_position, poses):
                break  # valid position found

        # Alternate colors (red/green)
        color = (0.5, 0, 0, 1) if i % 2 == 0 else (0, 0.5, 0, 1)

        # Add brick
        pose = SE3(x_position, y_position, 0) @ SE3.Rz(pi / 2)
        brick = geometry.Mesh(stl_path, pose=pose, color=color)
        environment.add(brick)
        bricks.append(brick)
        poses.append(pose)

    print(f"Created {len(bricks)} bricks.")
    for idx, pose in enumerate(poses):
        xyz = pose.t
        rpy = tr2rpy(pose.R, unit="deg")
        print(f"Brick {idx:02d}: Pos (x={xyz[0]:.2f}, y={xyz[1]:.2f}, z={xyz[2]:.2f}) | RPY {rpy}")

    environment.step(1)
    return environment, bricks, poses