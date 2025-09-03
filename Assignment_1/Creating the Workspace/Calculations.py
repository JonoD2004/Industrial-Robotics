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
import main
import Movements
import Setup

def Calculate_Inverse_Kinematics(robot, target_pose, environment, steps=1000):
    # Calculate inverse kinematics
    ik_solution = robot.ikine_LM(target_pose)
    
    if not ik_solution.success:
        raise ValueError(f"IK solution not found for pose:\n{target_pose}")

    q_goal = ik_solution.q
    return q_goal

def Calculate_Robot_Reach(robot, num_samples=2000, show_plot=True):
    q_min = robot.qlim[0, :]
    q_max = robot.qlim[1, :]

    x_vals, y_vals = [], []

    for _ in range(num_samples):
        # Random joint config (rails + arm joints)
        q_rand = [np.random.uniform(q_min[i], q_max[i]) for i in range(len(q_min))]
        T = robot.fkine(q_rand)

        x_vals.append(T.t[0])  # X position
        y_vals.append(T.t[1])  # Y position

    # Workspace limits
    x_neg_limit, x_pos_limit = min(x_vals), max(x_vals)
    y_neg_limit, y_pos_limit = min(y_vals), max(y_vals)

    print(f"Robot reach in X: {x_neg_limit:.2f} → {x_pos_limit:.2f}")
    print(f"Robot reach in Y: {y_neg_limit:.2f} → {y_pos_limit:.2f}")

    return x_pos_limit, x_neg_limit, y_pos_limit, y_neg_limit

def safe_pose(x, y, z, roll=pi):
    """
    Clamp x (rail) and z (floor) to keep robot within safe bounds.
    """
    rail_min, rail_max = 0.0, 0.8     # linear rail safe limits (meters)
    z_min, z_max = 0.05, 1.0          # don't let tool go through floor (z=0)

    y = np.clip(y, rail_min, rail_max)
    z = np.clip(z, z_min, z_max)

    return SE3(x, y, z) @ SE3.Rx(roll)

def set_initial_q(robot, q_init=None, enforce_limits=True, verbose=True):
    if q_init is None:
        q_init = np.array([-0.4, 0.0, -np.pi/2, 0.0, 0.0, 0.0, 0.0], dtype=float)
    else:
        q_init = np.asarray(q_init, dtype=float)
    if q_init.shape[0] != robot.n:
        raise ValueError(f"q_init must have length {robot.n} (got {q_init.shape[0]}).")
    q_applied = q_init.copy()
    if enforce_limits and hasattr(robot, "qlim") and robot.qlim is not None:
        low = robot.qlim[0, :]
        high = robot.qlim[1, :]
        q_clipped = np.clip(q_applied, low, high)
        if verbose and not np.allclose(q_clipped, q_applied):
            print("Note: initial q clipped to joint limits.")
        q_applied = q_clipped
    robot.q = q_applied
    return q_applied
