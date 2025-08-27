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
    lab2_solution.lab_Question()


# ---------------------------------------------------------------------------------------#
class Lab2Solution:
    def __init__(self):
        # No generic initialisation required
        pass

    # ---------------------------------------------------------------------------------------#
    def lab_Question(self):
        plt.close()

        # 4.1 and 4.2: Define the DH Parameters to create the Kinematic model
        link1 = DHLink(d= 0.5, a= 0.3, alpha= pi/2, qlim= [-pi, pi]) 
        link2 = DHLink(d= 0, a= 1, alpha= 0, qlim= [-pi, pi]) 
        link3 = DHLink(d= 0, a= 0.2, alpha= -pi/2, qlim= [-pi, pi]) 
        link4 = DHLink(d= 0.7, a= 0, alpha= pi/2, qlim= [-pi, pi])
        robot = DHRobot([link1, link2, link3, link4], name= 'myRobot')
        workspace = [-3, 3, -3, 3, -3, 3]  # Define the workspace limits
        # q =  np.zeros([1,3]) # Initial joint angles = 0
        q = np.array([-0.7506, 0.5895, -1.8286,  2.5971])  # Initial joint angles as shown in Canvas
        
        
        

        try:
            options = {"eelength": 1.0, "jointaxislength": 0.5}
            robot.plot(q= q, limits= workspace,options=options)
        except Exception as e:
            robot.plot(q= q, limits= workspace)
            print ("pose final position\n", robot.fkine(q))

        input('Press Enter to finish the question\n')
        # 4.5 Get the joint limits
        print("joint limits: \n", robot.qlim)
# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab2_solution_run()
    keyboard.unhook_all() 
    plt.close("all")
    time.sleep(0.5)  