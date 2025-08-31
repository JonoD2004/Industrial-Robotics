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

def Calculate_Inverse_Kinematics(robot, target_pose):
    # Calculate inverse kinematics
    ik_solution = robot.ikine_LM(target_pose)

    if ik_solution.success:
        return ik_solution.q
    else:
        raise ValueError("IK solution not found")
    
def Calculate_Robot_Reach():
    x_neg_limit = 0.35
    x_pos_limit = 1
    y_neg_limit = 0
    y_pos_limit = 1

    print(f"the robot can reach between {x_neg_limit} and {x_pos_limit} in the x direction")
    print(f"the robot can reach between {y_neg_limit} and {y_pos_limit} in the y direction")
    return x_pos_limit, x_neg_limit, y_pos_limit, y_neg_limit