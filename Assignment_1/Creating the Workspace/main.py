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
   

if __name__ == "__main__":
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        asyncio.set_event_loop(asyncio.new_event_loop())

    # launch the environment
    environment = swift.Swift()
    environment.launch()
    #x_pos_limit, x_neg_limit, y_pos_limit, y_neg_limit = Calculations.Calculate_Robot_Reach()

    environment, robot = Setup.setup_robot(environment) # Setup the workspace and get robot,
    environment, bricks, poses = Setup.setup_bricks(environment) # Setup the bricks and get their poses

    environment.hold()
    pass