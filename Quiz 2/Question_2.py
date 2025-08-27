import numpy as np
from math import pi
from roboticstoolbox.models.DH import UR3
from spatialmath import SE3

robot = UR3()
robot.base = robot.base @ SE3(0, 0, 0.8)
q = [0, pi/10, 0, 0, 0, 0]
R = robot.fkine(q)
print("whole matrix = \n", R)

end_effector = R.t
print("End effector transformation matrix = \n", end_effector)