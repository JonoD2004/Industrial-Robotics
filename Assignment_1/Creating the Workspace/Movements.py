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

def pick_and_place(robot, environment, target_pose, approach_height=0.1, steps=100):
   
    print("Pick-and-place completed!")
    
    
def build_wall(robot, environment, bricks, poses, wall_origin=SE3(0.7, 0.3, 0), spacing=(0.15, 0.15, 0.1), steps=100):
    
    print("Wall construction completed!")
    
    
def execute_trajectory(robot, environment, q_start, q_goal, steps=100, dt=0.2):
    traj = rtb.jtraj(q_start, q_goal, steps).q
    for q in traj:
        robot.q = q
        environment.step(0.05)