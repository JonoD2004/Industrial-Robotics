from spatialmath import SE2, SE3 # to create SE(n) objects
from spatialmath.base import * # for translation & rotation methods
from math import pi # pi constant
import numpy as np # numpy
import roboticstoolbox as rtb # general robotics toolbox module
from roboticstoolbox import DHLink, DHRobot # to create DH Robot

vecA = np.array([1, 2]) # create a vector A
vecB = np.array([2, 3]) # create a vector B

vecC = vecA + vecB # add vectors A and B
print("Vector C:", vecC) # print the result



