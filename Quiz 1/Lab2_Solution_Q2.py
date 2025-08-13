 # Require libraries
import numpy as np
import matplotlib.pyplot as plt
import time
import keyboard
from scipy import linalg
from spatialmath import SE3
from spatialmath.base import transl, trotx, troty, trotz, tr2rpy, r2q
from roboticstoolbox import DHLink, DHRobot
from ir_support import RobotCow, tranimate_custom, place_fence, orthogonalize_rotation

# Useful variables
from math import pi

def lab2_solution_run():
    plt.close("all")
    lab2_solution = Lab2Solution()
    lab2_solution.question2()



# ---------------------------------------------------------------------------------------#
class Lab2Solution:
    def __init__(self):
        # No generic initialisation required
        pass

    # ---------------------------------------------------------------------------------------# 
    def question2(self):
        base = np.eye(4)

        degrees = 39
        R1 = trotz(degrees, unit = 'deg')  # Rotate around Z axis by x degrees
        T1 = transl(37, 0, 0)  # Translate in X by 37
        
        translation = base @ R1 @ T1  # Apply rotation and translation to the base

        print ("Final Matrix:\n", SE3(translation).A)

        input('Finished Question 2, press Enter to continue\n')

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab2_solution_run()
    keyboard.unhook_all() 
    plt.close("all")
    time.sleep(0.5)       