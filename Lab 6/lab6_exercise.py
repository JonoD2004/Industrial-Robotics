""" 
Lab 6 Exercise: Range Measurements, Point Clouds, and Collision Checking (Python version)
Make sure you've reviewed the Learning Module and Lab Starter video before attempting these exercises.
""" 

# Required libraries
import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
from spatialmath.base import rotz, trotx, roty, transl, angvec2tr
from ir_support import EllipsoidRobot, schunk_UTS_v2_0, line_plane_intersection, make_ellipsoid
from roboticstoolbox import DHLink, DHRobot, models, jtraj, trapezoidal
import keyboard, time

# Useful variables
from math import pi, radians

# ---------------------------------------------------------------------------------------#
class Lab6Solution:
    def __init__(self):
        # No generic initialisation required
        pass

    # ========== Question 1: Convert range measurements to a point cloud ==========
    def question1(self):
        """
        Questions 1: Convert range measurements to a point cloud
        """    
        plt.close('all')
        
        # 1.1) Load and animate the Schunk robot at pose [0, pi/2, 0, 0, 0, 0]
        robot = schunk_UTS_v2_0()
        q = [0,pi/2,0,0,0,0]
        fig = robot.plot(q, limits = [-3,3,-3,3,0,3])
        fig.ax.set_box_aspect([1,1,0.5])
        line_h = []
        verts = []


        # 1.2) Assume the LTM ray is aligned with the z-axis of the end effector.
        # Plot a red line 1.9594m long starting from the end effector along the z-axis.
        # Add a red '*' at the tip of the ray.
        tr = robot.fkine(q).A
        start_p = tr[:3,3]         
        dist = 1.9594
        
        end_p = tr[:3,3] + dist * tr[:3,2]
        line_h.append(fig.ax.plot([start_p[0], end_p[0]], [start_p[1], end_p[1]], [start_p[2], end_p[2]], 'r'))
        fig.ax.plot(end_p[0], end_p[1], end_p[2], 'r*')
        verts.append(end_p)

        # 1.3) Repeat the above ray projection for the following poses and measured distances:
        # Pose 1: [pi/10, pi/2, 0, 0, 0, 0] with LTM reading 2.4861m
        # Pose 2: [-pi/10, 5*pi/12, 0, 0, 0, 0] with LTM reading 1.9132m
        # Save the endpoints as for later use.

        # 1.4) Use the three measured points to compute the triangle normal:
        # a) normal = unit vector of (v1 - v2) × (v2 - v3)
        # b) create a meshgrid in Y-Z from -2 to 2 by 0.1; set X = 0
        # c) rotate this flat Y-Z plane so that it lies in the triangle's plane:
        #    - base normal is [-1, 0, 0]
        #    - rotation axis = cross(base_normal, triangle_normal) to rotate the base plane around
        #    - angle = arccos(dot(base_normal, triangle_normal)) how much to rotate the base plane to make it match the triangle plane
        #    - make a transform to do that rotation using angvec2tr(rotation_radians, rotation_axis)
        #    - plot a surface normal from inside the triangle plane
        #    - apply transformation and shift the plane to the triangle center
        # d) clamp any points with Z < 0 to Z = 0
        # e) plot the surface using ax.plot_surface

        # 1.5) Fix the robot at pose [0, pi/2, 0, 0, 0, 0]
        # Sweep the LTM from -20° to 20° in 1° increments around the end effector X-axis
        # For each ray:
        #   - rotate the max range ray about the local X-axis by θx
        #   - transform this rotated vector into world coordinates using the robot's fkine(q)
        #   - check intersection with wall plane using: intersect_p, check = line_plane_intersection(triangle_normal, triangle_point, start_p, ray_end)
        #   - check intersection with the ground (z=0)
        #   - keep the closest intersection point
        # Store and plot all the points to visualise the LRF sweep.

        # 1.6) Repeat 1.5, but now also sweep the LTM about the Y-axis
        # This gives a 2D sweep: 41 readings in X × 41 readings in Y = 1681 points
        # You are simulating a 3D LiDAR like a Velodyne Puck
        # Check and record the same intersection logic for each combined rotation
        # Plot all resulting scan points

    # ========== Question 2: Collision Detection with Ellipsoids ==========
    def question2(self):
        """
        Question 2: Ellipsoid and Point collision checking
        """
        plt.close('all')
        # 2.1) Create an ellipsoid centered at the origin with radii (rx=3, ry=2, rz=1)
        # Use np.linspace and np.meshgrid to create a grid and parametrize the surface

        # 2.2) Plot the surface and make it translucent so you will be able to see points inside it. 
        # (consider using matplotlib's plot_surface and set_alpha)

        # 2.3) Create a cube centered at [2, 0, -0.5] with side 1.5m using six surfaces
        # You can build it from six meshgrids or combine rotated faces

        # 2.4) Check which cube points are inside the ellipsoid
        # Use the ellipsoid algebraic distance equation:
        # ((x - xc)/rx)^2 + ((y - yc)/ry)^2 + ((z - zc)/rz)^2 < 1
        # or use the get_algebraic_dist() function provided

        # 2.5) Move the ellipsoid center to [1, 1, 1] and recompute how many points are inside

        # 2.6) Instead of moving the ellipsoid, transform the cube points with inverse translation
        # Use homogeneous coordinates and matrix multiplication

        # 2.7) Now apply both translation and rotation (about X by 45°) to the ellipsoid
        # Again, apply the inverse of this transform to the cube points

        # 2.8) Create a 3-link planar robot with three ellipsoids centered at the origin
        # Plot the robot and attach ellipsoids to each link

        # 2.9) (Bonus) For a given pose, use the FK of the robot to transform cube points into end-effector frame
        # Then check which are inside the EE ellipsoid

        # 2.10) (Bonus) Repeat this process for all ellipsoids on the robot links

# ========== Question 3: Joint Interpolation vs RMRC ==========
    def question3(self):
        """
        Question 3: Joint Interpolation vs RMRC
        """

        # 3.1) Create a 2-link planar robot 

        # 3.2) Define two transformation targets:
        # e.g., T1 = [[I, [1.5, 1, 0]], [0 0 0 1]], T2 = [[I, [1.5, -1, 0]], [0 0 0 1]]

        # 3.3) Solve IK to get joint angles q1 and q2 for T1 and T2

        # 3.4) Use linear interpolation between q1 and q2 using jtraj

        # 3.5) Use 50 steps to move from A to B with Resolved Motion Rate Control (RMRC)

        # 3.6) Define start and end points: x1 to x2 in XY plane that match the location of T1 to T2 

        # 3.7) Create a matrix of waypoints that interpolates between the start and end points using LSPB or linear blending

        # 3.8) Create an empty matrix of joint angles that we will fill in later

        # 3.9) Use your preferred inverse kinematics solver to determine the joint angles for the first point in the trajectory 

        # 3.10) Solve RMRC: at each timestep, compute xdot and use J⁻¹ xdot = qdot to move the end-effector from x1 to x2.

        # 3.11) Plot the RMRC trajectory and compare it to the trajectory taken during regular joint interpolation.

# ---------------------------------------------------------------------------------------#
def get_algebraic_dist(points, center_point, radii):
    """
    Determine the algebraic distance given a set of points and the center 
    point and radii of an elipsoid
    
    Args:
    
        points (array-like) (many*(2||3||6) double) x,y,z cartesian point
    
        center_point (array-like) (1 * 3 double) xc,yc,zc of an ellipsoid
    
        radii (1 * 3 double) a,b,c of an ellipsoid
    
    Returns:
    (Nx1) algebraic_dist (many*1 double) algebraic distance for the ellipsoid
    """
    return np.sum(((points - center_point)/radii)**2, axis= 1)

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab6_solution = Lab6Solution()
    lab6_solution.question1()
    lab6_solution.question2()
    lab6_solution.question3()
    keyboard.unhook_all() 
    plt.close("all")
    time.sleep(0.5)
