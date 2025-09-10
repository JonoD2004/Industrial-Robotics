import numpy as np
from roboticstoolbox import DHLink, DHRobot

l1 = DHLink(a=1)
l2 = DHLink(a=1)
l3 = DHLink(a=1)
l4 = DHLink(a=1)
l5 = DHLink(a=1)
robot = DHRobot([l1, l2, l3, l4, l5])

j1 = np.deg2rad (45)
j2 = np.deg2rad (-45)
j3 = np.deg2rad (40)
j4 = np.deg2rad (-45)
j5 = np.deg2rad (0)

q1 = [j1, j2, j3, j4, j5]

endeffector = robot.fkine(q1)
xyz = endeffector.t
print("End-effector xyz:\n", xyz)