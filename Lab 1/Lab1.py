# Require libraries
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from spatialmath import SE2
from spatialmath.base import trplot2, plotvol2, tranimate2

# Useful variables
from math import pi

# ---------------------------------------------------------------------------------------#
def lab1_starter():
    """
    lab1_starter The 1st lab starter to create, plot and multiply transforms
      - Create a "New" function (no arguments) not a script or class (yet)
      - Clear and close all 
      - Make sure Robotics Toolbox is installed (check by 'pip show rvc3python')
      - Creating and plotting a 2D transform (x,y and orientation)
      - Creating 2nd transform with a different colour (hold on)
      - Multiplying 2nd transforms together to create a 3rd
      - Multiplying a transform by its inverse to get identity
      - Axis options (size, equal, grid)
      - One frame in another frame's 
    """
    # ---------------------------------------------------------------------------------------#
    ## PART 1: Creating and plotting a 2D transform

    # ---------------------------------------------------------------------------------------#
    ## Clear & close
    plt.close('all')
    ax = plt.subplot() # new figure for plotting

    # ---------------------------------------------------------------------------------------#
    ## Transform 1
    # To get transform data of a SE2 object, we use the .A operator
    # which will return a NDarray object (numpy n-dimension array)
    T1 = SE2(1, 2, 30*pi/180).A
    trplot2(T1, frame = "1", color = "b")
    print("T1=\n", SE2(T1))

    # ---------------------------------------------------------------------------------------#
    ## Transform 2
    T2 = SE2(2, 1, 0).A
    trplot2(T2, frame = "2", color = "r")
    print("T2=\n", SE2(T2))

    # ------------------------------------------------------------l---------------------------#
    ## Transform 3 is T1@T2
    T3 = T1 @ T2
    trplot2(T3, frame = "3", color = "g")
    print("T3=\n", SE2(T3))
    
    # ---------------------------------------------------------------------------------------#
    ## Axis
    plotvol2([-1, 6 , -1, 6], equal = True, grid = True)
    
    # ---------------------------------------------------------------------------------------#
    ## Move Transforms
    for i in np.arange(90, -0.5, -0.5): 
        # np.arange() function creates a range that includes the 
        # start value but excludes the end value, so we use -0.5 as 
        # the end value to include 0 in the sequence.
        
        # Update the figure
        ax.clear()
        plotvol2([-1, 6 ,-1, 6], equal = True, grid = True)
        
        T1 = SE2(1, 2, i*pi/180).A
        trplot2(T1, frame = "1", color = "b")    
        
        # T2 = SE2(i, 1, 0).A
        trplot2(T2, frame = "2", color = "r")
        
        T3 = T1@T2
        trplot2(T3, frame = "3", color = "g")

        plt.draw()
        plt.pause(0.01)


    # ---------------------------------------------------------------------------------------#
    ## PART 2: One frame in another frame's (roughly based on Canvas module 1.2)
    plt.close("all")

    T_0A = SE2(0,0,0).A; # For simplicity let's assume that the frame A is at the origin, but it doesn't need to be
    trplot2(T_0A, frame = 'T_A', color =  'r')

    # When working with matrix multiplication in Python objects created from numpy library
    # we use the '@' operator instead of '*'
    T_AB = T_0A @ SE2(1,1,-45*pi/180).A
    T_0B = T_0A @ T_AB
    trplot2(T_0B, frame = 'T_B', color =  'b')

    T_AC = SE2(3,-1,80*pi/180).A
    T_0C = T_0A @ T_AC
    trplot2(T_0C, frame = 'T_C', color =  'g')

    T_BD = SE2(1,1,45*pi/180).A
    T_0D = T_0A @ T_AB @ T_BD
    trplot2(T_0D, frame = 'T_D', color =  'y')

    # ---------------------------------------------------------------------------------------#
    ## If we wanted to determine the transform from frame C to frame D
    T_CD = linalg.inv(T_0C) @ T_0D

    # We can prove that this worked by transforming T_0C by T_CD to be T_0D 
    tranimate2(T_0C @ T_CD)
    
    # Note T_0C * T_CD is approximately equal to T_0D, i.e. it is very close
    # (less than eps). The difference is due to the way the inverse T_0C.inv()
    # is previously calculated.
    diff_value = (T_0C @ T_CD) - T_0D
    print(diff_value)
    check = diff_value < np.finfo(float).eps # Check that all parts of the diff matrix are less than the floating point error, eps
    print(check)

    plt.show()

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab1_starter()

