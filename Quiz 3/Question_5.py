import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3

# Define Puma 560 parameters
a = [0, 0.4318, 0.0203, 0, 0, 0]
d = [0, 0, 0.15005, 0.4318, 0, 0]
alpha = [pi/2, 0, -pi/2, pi/2, -pi/2, 0]
qlim = [[-2.7925, 2.7925],
        [-0.7854, 3.9270],
        [-3.9270, 0.7854],
        [-1.9199, 2.9671],
        [-1.7453, 1.7453],
        [-4.6426, 4.6426]]

links = []
for i in range(6):
    link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim=qlim[i])
    links.append(link)

# Build the robot
p560 = rtb.DHRobot(links, name="Puma560")

T = SE3(0.6,0.1,0.1) #desired end-effector pose

qn = [0, 0.78539816, 3.14159265, 0, 0.78539816, 0] #initial guess

sol = p560.ikine_LM(T, q0=qn, mask=[1, 1, 1, 0, 0, 0]) 

print("Solution q[0] =", sol.q[0])
print("solution q[1] =", sol.q[1])
print("solution q[2] =", sol.q[2])
print("solution q[3] =", sol.q[3])
print("solution q[4] =", sol.q[4])
print("solution q[5] =", sol.q[5])