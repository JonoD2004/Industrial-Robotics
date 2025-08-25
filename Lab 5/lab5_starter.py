# Require libraries
import numpy as np
import matplotlib.pyplot as plt
from spatialmath.base import *
from roboticstoolbox import DHLink, DHRobot
from ir_support import line_plane_intersection

# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
class Lab5Starter:
    def __init__(self):
        plt.close('all')
        self.part1()
        # self.part2()
    
    @staticmethod
    def part1():
        """
        Part1: Collision checking between a 1DOF robot and a sphere
        """
        plt.close('all')

        # Changed sphere plot to allow for either point cloud or triangle mesh based on value of a boolean variable, 'make_point_cloud'.
        make_point_cloud = False
        make_triangle_mesh = True
        
        ## Create 1-link robot
        link1 =  DHLink(d= 0, a= 1, alpha= 0, qlim= [-pi, pi])
        robot = DHRobot([link1], name = 'my_robot')
        
        plt.figure()
        q = 0
        workspace = [-0.5, 1.5, -0.5, 1.5, -1, 1]
        fig = robot.plot(q, limits= workspace, fig= plt.gcf())
        
        ax = plt.gca()
        ## Create sphere
        sphere_center = np.array([1, 1, 0])
        radius = 0.5
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = sphere_center[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = sphere_center[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = sphere_center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        
        ## Plot it
        if make_point_cloud:
            # Plot point cloud
            points = np.column_stack((x.flatten(), y.flatten(), z.flatten()))
            ax.plot(points[:, 0], points[:, 1], points[:, 2], 'r.')
        
        if make_triangle_mesh:
            # Triangle mesh plot
            ax.plot_surface(x, y, z, cmap='viridis', edgecolor='none', alpha = 0.5)
        
        ## Move robot
        for q in np.arange(0,pi/2,pi/180):
            robot.q = q
            fig.step(0.05)

            ## Change code to choose the behaviour 
            # (1) Do this to move through obstacles with a message
            Lab5Starter.check_collision(robot,sphere_center,radius)
            # OR (2) stop the movement when an obstacle is detected
            if Lab5Starter.check_collision(robot,sphere_center,radius) == 1:
                print('UNSAFE: Robot stopped')
                break

        input('Press Enter to continue\n')
    
    @staticmethod
    def part2():
        """
        Part2
        - As well as the way shown in the previous part, there is also
        another function called line_plane_intersection.   
        """
        plt.close('all')
        # A plane can be defined with the following point and normal vector
        plane_normal = [-1,0,0]
        plane_point = [1.5,0,0]

        # Then if we have a line (perhaps a robot's link) represented by two points:
        line_start_point = [-0.5,0,0]
        line_end_point = [3.5,0,0]

        # Then we can use the function to calculate the point of
        # intersection between the line (line) and plane (obstacle) 
        intersection_point,check = line_plane_intersection(plane_normal, plane_point, line_start_point,line_end_point)
        
        # The returned values and their means are as follows:
        # (1) intersection_point, which shows the xyz point where the line intersects the plane
        # intersection_point
        
        # (2) check intersects the plane check, which is defined as follows:
        # check 
        # check == 0 if there is no intersection
        # check == 1 if there is a line plane intersection between the two points
        # check == 2 if the segment lies in the plane (always intersecting)
        # check == 3 if there is intersection point which lies outside line segment
        
        # We can visualise this as follows by first creating and
        # plotting a plane, which conforms to the previously defined planePoint and planeNormal
        y, z = np.meshgrid(np.arange(-2, 2 + 0.1, 0.1),
                                    np.arange(-2, 2 + 0.1, 0.1))
        x = np.full_like(y, 1.5)

        plt.figure().add_subplot(projection= '3d')
        ax = plt.gca()
        ax.plot_surface(x, y, z)

        # Then plot the start and end point in green and red, respectively.
        ax.plot(line_start_point[0], line_start_point[1], line_start_point[2], 'g*')
        ax.plot(line_end_point[0], line_end_point[1], line_end_point[2], 'r*')
        ax.plot([line_start_point[0], line_end_point[0]], [line_start_point[1], line_end_point[1]], [line_start_point[2], line_end_point[2]], 'k')
        ax.plot(intersection_point[0], intersection_point[1], intersection_point[2], 'k*', markersize=10)

        input("Enter to continue\n")

    # ---------------------------------------------------------------------------------------#
    @staticmethod
    def check_collision(robot, sphere_center, radius):
        """
        CheckCollision
        Checks for collisions with a sphere and can be modified to return an
        is_collision result
        """
        tr = robot.fkine(robot.q).A
        squared_diff = (transl(tr) - np.array(sphere_center)) **2
        ee_to_center_dist = np.sqrt(np.sum(squared_diff))
        if ee_to_center_dist <= radius:
            print("Oh no a collision!")
            is_collision = 1
        else:
            print(f"SAFE: End effector to sphere centre distance {ee_to_center_dist}m is more than the sphere radius {radius}m")
            is_collision = 0
        
        return is_collision

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    Lab5Starter()