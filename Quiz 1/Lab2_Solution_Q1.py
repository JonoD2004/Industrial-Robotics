# Require libraries
import numpy as np
import matplotlib.pyplot as plt
import time
import keyboard
from scipy import linalg
from spatialmath import SE3
from spatialmath.base import transl, trotx, troty, tr2rpy, r2q
from roboticstoolbox import DHLink, DHRobot
from ir_support import RobotCow, tranimate_custom, place_fence, orthogonalize_rotation

# Useful variables
from math import pi

def lab2_solution_run():
    plt.close("all")
    input("Press Enter to begin\n")
    lab2_solution = Lab2Solution()
    lab2_solution.question1()


class Lab2Solution:
    def __init__(self):
        pass

    def question1(self):
        """
        Question 1: Animate a UAV flying through a sequence of transforms.
        """

        print("TODO: Animate UAV transform sequence using tranimate_custom()")
        # 1.9) Encode the steps 1.1-1.8 in a ‘for’ loop and use the 'fps' option in ‘tranimate’ to speed up the animation
        for i in range (1):
            fps = 30
            # 1.1)  Start at the origin and move up to 10m off the ground (positive Z)
            origin = np.eye(4)
            Z1 = transl(0, 0, -8)  # Move to [0, 0, 10]
            tranimate_custom(origin, Z1, fps)  # Apply translation to the origin
            origin = Z1  # Apply translation to the origin

            # 1.3)  Move in the direction of global Y to [0,2,10]
            Y1 = transl(8, 8, 0)  # Move to [0, 2, 10]
            T3 = origin @ Y1  # Apply translation in Y direction
            tranimate_custom(origin, T3, fps)  # Animate the translation
            origin = T3  # Update the origin to the new position

            # 1.6) Move in the direction of global X to [2,2,10]
            X1 = transl(-5, 3, 2)  # Move to [2, 2, 10]
            T6 = origin @ X1  # Apply translation in X direction
            tranimate_custom(origin, T6, fps)  # Animate the translation
            origin = T6  # Update the origin to the new position
            
            print (f"Step {i+1}: Final Matrix:\n", SE3(origin).A)
            
        
        # 1.10)  Use the text tool from Week 1 Lab to plot in the left hand corner the RPY and quaternion value of the orientation at each step
            

        input("Press Enter to continue\n")
# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab2_solution_run()
    keyboard.unhook_all() 
    plt.close("all")
    time.sleep(0.5)       