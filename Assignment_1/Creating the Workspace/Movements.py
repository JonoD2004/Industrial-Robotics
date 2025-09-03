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

def pick_and_place(robot, environment, brick, target_pose, steps=1000):
   # Ensure brick transform is SE3
    if isinstance(brick.T, np.ndarray):
        brick_pose = SE3(brick.T)
    else:
        brick_pose = brick.T
        
    if isinstance(target_pose, np.ndarray):
        target_pose = SE3(target_pose)

    # Now you can access .t safely
    grip_height = max(target_pose.t[2], 0.05)
    above_target = SE3(target_pose.t[0],
                       target_pose.t[1],
                       grip_height + 0.1) @ SE3.Rx(pi)

    # -----------------------------
    # Approach above brick
    grip_height = max(brick_pose.t[2], 0.05)
    above_brick = Calculations.safe_pose(brick_pose.t[0],
                                         brick_pose.t[1],
                                         grip_height + 0.1) @ SE3.Rx(pi)
    print(f"Brick raw: y={brick_pose.t[1]:.3f}, z={brick_pose.t[2]:.3f}")
    print(f"Clamped: y={above_brick.t[1]:.3f}, z={above_brick.t[2]:.3f}")
    print (f"move to position above brick: {above_brick}")
    q_goal = Calculations.Calculate_Inverse_Kinematics(robot, above_brick, environment)
    execute_trajectory(robot, environment, robot.q, q_goal, steps)

    # Move down to brick
    print ("reach down and collect brick")
    grip_height = max(brick_pose.t[2], 0.05)
    at_brick = Calculations.safe_pose(brick_pose.t[0], brick_pose.t[1], grip_height) @ SE3.Rx(pi)
    q_goal = Calculations.Calculate_Inverse_Kinematics(robot, at_brick, environment)
    execute_trajectory(robot, environment, robot.q, q_goal, steps)

    # "Grasp" brick (attach)
    print ("brick attach")
    attached_brick = brick  # tell execute_trajectory that this brick should move       

    # Lift brick
    print ("brick lift")
    execute_trajectory(robot, environment, robot.q, Calculations.Calculate_Inverse_Kinematics(robot, above_brick, environment), steps, attached=attached_brick)

    # Move above target
    print ("move over to wall position")
    grip_height = max(target_pose.t[2], 0.05)
    above_target = Calculations.safe_pose(target_pose.t[0], target_pose.t[1], target_pose.t[2] + 0.1) @ SE3.Rx(pi)
    q_goal = Calculations.Calculate_Inverse_Kinematics(robot, above_target, environment)
    execute_trajectory(robot, environment, robot.q, q_goal, steps, attached=attached_brick)

    # Lower to target
    print("prep brick ")
    q_goal = Calculations.Calculate_Inverse_Kinematics(robot, above_brick, environment)
    execute_trajectory(robot, environment, robot.q, q_goal, steps, attached=attached_brick)

    # Release brick
    print("place brick")
    attached_brick = None
    brick.T = target_pose
    environment.step(0.001)

    # Return to home
    q_home = np.zeros(robot.n)
    execute_trajectory(robot, environment, robot.q, q_home, steps)

    print("Pick-and-place completed!")
    for _ in range(20):
        print(". ")
        print(".. ")
        print("... ")
        environment.step(0.001)
    return environment
    
def execute_trajectory(robot, environment, q_start, q_goal, steps=1000, attached=None):
    traj = rtb.jtraj(q_start, q_goal, steps).q
    for q in traj:
        robot.q = q

        # If a brick is attached, follow the robot's end-effector pose
        if attached is not None:
            ee_pose = robot.fkine(robot.q)
            attached.T = ee_pose  # keep brick stuck to gripper

        environment.step(0.001)
        
def pick_and_place_multiple(robot, environment, bricks, wall_poses, steps=1000):
    if len(bricks) != len(wall_poses):
        raise ValueError("Number of bricks must match number of wall positions.")
    
    for i, brick in enumerate(bricks):
        target_pose = wall_poses[i]
        print(f"\nProcessing brick {i+1}/{len(bricks)}")
        pick_and_place(robot, environment, brick, target_pose, steps)
    
    print("\nAll bricks placed successfully!")
    return environment

def move_linear_rail(robot, environment, x_target, steps=1000):
    """
    Smoothly move the linear rail (joint 0) to x_target using a trajectory.
    """
    rail_min, rail_max = 0.0, 0.8
    x_target = np.clip(x_target, rail_min, rail_max)

    q_start = robot.q.copy()
    q_goal = robot.q.copy()
    q_goal[0] = x_target

    traj = rtb.jtraj(q_start, q_goal, steps).q
    for q in traj:
        robot.q = q
        environment.step(0.001)

    return x_target