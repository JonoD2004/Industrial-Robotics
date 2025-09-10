import numpy as np
from math import pi
from roboticstoolbox import DHRobot, DHLink
from ir_support import line_plane_intersection

# Define the robot links
l1 = DHLink(d=1, a=0, alpha=np.pi/2, offset=0)
l2 = DHLink(d=0, a=2, alpha=0, offset=0)
l3 = DHLink(d=0, a=3, alpha=0, offset=0)

# Create the robot
R3 = DHRobot([l1, l2, l3], name="3-link robot")

# Joint configuration
q = [-pi/12, 0, 0]

# Compute transforms
T_all = R3.fkine_all(q)

# Extract joint positions
p0 = np.array([0, 0, 0])                  # base
p1 = T_all[0].t                           # after link 1
p2 = T_all[1].t                           # after link 2 (second joint center)
p3 = T_all[2].t                           # end effector

# Define the wall plane: x = 3.1
plane_normal = [1, 0, 0]   # normal vector to wall
plane_point = [3.1, 0, 0]  # a point on the wall

# Compute intersection of line (p2 -> p3) with plane
intersection, *_ = line_plane_intersection(
    plane_normal, plane_point, p2, p3
)

print("Intersection point:", intersection)