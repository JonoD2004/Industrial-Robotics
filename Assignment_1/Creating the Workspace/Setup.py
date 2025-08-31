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
        link1 = 'slider_rail',
        link2 = 'shoulder_ur3',
        link3 = 'upperarm_ur3',
        link4 = 'forearm_ur3',
        link5 = 'wrist1_ur3',
        link6 = 'wrist2_ur3',
        link7 = 'wrist3_ur3' 
    """
    robot.base = robot.base * SE3(0,0,0) * SE3.Ry(-pi/2)  # adjust base for better view and

    robot.add_to_env(environment)

    environment.step(1)
    return environment, robot

def setup_bricks(environment):
    current_dir = os.path.dirname(os.path.abspath("/Users/minibeardman/Desktop/Industrial-Robotics/Assignment_1/Creating the Workspace/HalfSizedRedGreenBrick.stl"))
    stl_path = os.path.join(current_dir, "HalfSizedRedGreenBrick.stl")
    #creation of the workspace
    bricks = []
    poses = []
    spacing = 0.15
    num_bricks = 9
    x_pos_limit, x_neg_limit, y_pos_limit, y_neg_limit = Calculations.Calculate_Robot_Reach()

    # Create a grid of bricks in the workspace
    for i in range(num_bricks):
        print(f"\nBrick {i}:")
        
        x_position = float(input(f"  Enter x position (between {x_neg_limit} and {x_pos_limit}): "))
        if x_position < x_neg_limit or x_position > x_pos_limit:
            raise ValueError(f"x position must be between {x_neg_limit} and {x_pos_limit}")
            continue
        
        y_position = float(input(f"  Enter y position (between {y_neg_limit} and {y_pos_limit}): "))
        if y_position < y_neg_limit or y_position > y_pos_limit:
            raise ValueError(f"y position must be between {y_neg_limit} and {y_pos_limit}")
            continue
        
        # alternate colors (red and green)
        color = (0.5, 0, 0, 1) if (i) % 2 == 0 else (0, 0.5, 0, 1)

        # brick pose: positioned + rotated 90° about z
        pose = SE3(x_position, y_position, 0) @ SE3.Rz(pi/2)
        brick = geometry.Mesh(stl_path, pose=pose, color=color)
        environment.add(brick)
        bricks.append(brick)
        poses.append(pose)
            
    print (f"Created {len(bricks)} bricks.")
    for idx, pose in enumerate(poses):
        xyz = pose.t
        rpy = tr2rpy(pose.R, unit="deg")
        print(f"Brick {idx:02d}: Pos (x={xyz[0]:.2f}, y={xyz[1]:.2f}, z={xyz[2]:.2f}) | RPY {rpy}")


    environment.step(1)
    return environment, bricks, poses