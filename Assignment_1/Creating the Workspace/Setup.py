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
    
    

    environment.step(0.001)
    return environment, robot

def setup_bricks(environment, robot):
    # Get robot limits
    x_pos_limit, x_neg_limit, y_pos_limit, y_neg_limit = Calculations.Calculate_Robot_Reach(robot)

    # === VISUALISE ROBOT REACH LIMITS ===
    # Create a semi-transparent rectangle showing robot workspace boundary
    workspace_pose = SE3((x_pos_limit + x_neg_limit) / 2,
                         (y_pos_limit + y_neg_limit) / 2,
                         0)   # center of limits
    workspace_size = (abs(x_pos_limit - x_neg_limit),
                      abs(y_pos_limit - y_neg_limit),
                      0.001)  # flat rectangle on ground
    workspace_box = geometry.Cuboid(workspace_size, pose=workspace_pose, color=(0, 0, 1, 0.1))  # blue transparent
    environment.add(workspace_box)

    # === VISUALISE FORBIDDEN ZONE ===
    forbidden_x_min, forbidden_x_max = (-0.25 - 0.1334), (0.25 + 0.1334)
    forbidden_y_min, forbidden_y_max = (-0.2 - 0.0667), (0.9 + 0.0667)
    forbidden_pose = SE3((forbidden_x_max + forbidden_x_min) / 2,
                         (forbidden_y_max + forbidden_y_min) / 2,
                         0)
    forbidden_size = (abs(forbidden_x_max - forbidden_x_min),
                      abs(forbidden_y_max - forbidden_y_min),
                      0.05)  # a little height so it's visible
    forbidden_box = geometry.Cuboid(forbidden_size, pose=forbidden_pose, color=(1, 0, 0, 0.3))  # red transparent
    environment.add(forbidden_box)

    # ========== VALIDATION CHECKS ==========
# Each brick is 0.0667x * 0.1334y * 0.0334z
    def is_invalid_position(x, y, poses, min_x_spacing=0.1334, min_y_spacing= 0.0667):

        # 1. Forbidden zone
        if forbidden_x_min <= x <= forbidden_x_max and forbidden_y_min <= y <= forbidden_y_max:
            print(f"    Position (x={x:.2f}, y={y:.2f}) is inside the ROBOT FORBIDDEN ZONE!")
            return True

        # 2. Outside limits
        if not (x_neg_limit <= x <= x_pos_limit):
            print(f"    Position (x={x:.2f}) is OUTSIDE X limits ({x_neg_limit:.2f}, {x_pos_limit:.2f})!")
            return True
        if not (y_neg_limit <= y <= y_pos_limit):
            print(f"    Position (y={y:.2f}) is OUTSIDE Y limits ({y_neg_limit:.2f}, {y_pos_limit:.2f})!")
            return True

        # 3. Too close to existing bricks
        for pose in poses:
            existing_x, existing_y = pose.t[0], pose.t[1]
            too_close_x = abs(x - existing_x) < min_x_spacing
            too_close_y = abs(y - existing_y) < min_y_spacing

            if too_close_x and too_close_y:
                print(f"    Too close to brick at (x={existing_x:.2f}, y={existing_y:.2f})")
                return True

        print (f"    Position (x={x:.2f}, y={y:.2f}) is valid.")
        return False  # valid

    # === BRICK CREATION ===
    current_dir = os.path.dirname(os.path.abspath("/Users/minibeardman/Desktop/Industrial-Robotics/Assignment_1/Creating the Workspace/HalfSizedRedGreenBrick.stl"))
    stl_path = os.path.join(current_dir, "HalfSizedRedGreenBrick.stl")

    bricks, poses = [], []
    num_bricks = 9

    for i in range(num_bricks):
        print(f"\nBrick {i}:")
        while True:
            print ("  Enter position for brick, outside the forbidden zone and within robot reach:")
            print (f"robot forbidden zone: between ({forbidden_x_min}, {forbidden_y_min}) and ({forbidden_x_max}, {forbidden_y_max})")
            x_position = float(input(f"  Enter x position (between {x_neg_limit} and {x_pos_limit}): "))
            y_position = float(input(f"  Enter y position (between {y_neg_limit} and {y_pos_limit}): "))

            if not is_invalid_position(x_position, y_position, poses):
                break  # valid

        colour = (0.5, 0, 0, 1) if i % 2 == 0 else (0, 0.5, 0, 1)
        pose = SE3(x_position, y_position, 0) @ SE3.Rz(pi / 2)
        brick = geometry.Mesh(stl_path, pose=pose, color=colour)
        environment.add(brick)
        bricks.append(brick)
        poses.append(pose)

    print(f"Created {len(bricks)} bricks.")
    environment.step(0.001)
    return environment, bricks, poses

def setup_environment(environment):
    #import walls for the building
    wall_1 = geometry.Cuboid(scale = (0.25, 4.75 ,3), pose = SE3(2, 1.5, 0), color = (0.5, 0, 0, 1) )
    environment.add(wall_1)
    wall_2 = geometry.Cuboid(scale = (0.25, 4.75 ,3), pose = SE3(-2, 1.5, 0), color = (0.5, 0, 0, 1) )
    environment.add(wall_2)
    wall_3 = geometry.Cuboid(scale = (3.75, 0.25, 3), pose = SE3 (0, 3.75, 0), color = (0.5, 0, 0, 1))
    environment.add(wall_3)
    wall_4 = geometry.Cuboid(scale = (3.75, 0.25, 3), pose = SE3 (0, -0.75, 0), color = (0.5, 0, 0, 1))
    environment.add(wall_4)
    
    #import table
    
    return environment

