import numpy as np
from math import pi
import roboticstoolbox as rtb
from spatialmath import SE3
left = rtb.models.DH.Baxter('left')
right = rtb.models.DH.Baxter('right')

left.base = SE3(0.064614, 0.25858, 0.119) * SE3.Rx(pi/4)
right.base = SE3(0.063534, -0.25966, 0.119) * SE3.Rx(-pi/4)

qLeft = [0,0,0,0,0,0,0]
qRight = [0,0,0,0,0,0,0]

a = left.fkine(qLeft)
b = right.fkine(qRight)
print("Left hand end-effector pose:\n", a)
print("Right hand end-effector pose:\n", b)

distance = np.linalg.norm(a.t - b.t) #.t isolates the translational part of the SE3 object
print("Distance between end-effectors:\n", distance)