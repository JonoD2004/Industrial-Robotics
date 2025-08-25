# Some required libraries
import numpy as np
import time
from itertools import combinations
import threading
from typing import List
from spatialmath.base import *
from roboticstoolbox import jtraj, Robot, models
from ir_support import RectangularPrism, line_plane_intersection, CylindricalDHRobotPlot

from spatialgeometry import Cylinder, Sphere, Cuboid
from spatialmath import SE3
import swift

# Useful variables
from math import pi

# ---------------------------------------------------------------------------------------------------------------------------------------------#
class Lab5Exercises:
    def __init__(self):
        print(f"Welcome to Lab 5! Questions 2 + 3 Starting Point")
        self.stop_event = threading.Event()         # Event to end teach mode when 'enter' pressed


    # 2.4) Using your understanding of forward kinematics write a function that you can pass in vector of joint angles, q, 
    # to represent a joint state of the arm, and it will return a 4x4x4 matrix which contains:
    # TR(:,:,1) = arm.base
    # TR(:,:,2) = Transform at end of link 1
    # TR(:,:,2) = Transform at end of link 2
    # TR(:,:,3) = arm.fkine(q)
    # Hint: Look at the "fkine_all()" function in the Robotics Toolbox
    def get_link_poses(self, robot:Robot, q=None)->List[np.ndarray]|np.ndarray:
        """
        Args:
            q: robot joint angles
            robot:  seriallink robot model

        Returns:
            transforms - list of transforms of robot joints
        """
        if q is None:
            return ...
        return ...
    
    # 2.7) Complete this method for section 2.7 (more details given at that point in skeleton)
    # Use the docstring to determine the intended function arguments
    def is_collision(self, ):
        """
        Function to determine if any link of a robot is in collision
        with some object in the environment, based on the location of
        its mesh properties (faces, vertices, face normals)

        Args:
            robot: seriallink robot model
            q_matrix: 2D array (matrix) of robot joint states (e.g. a joint trajectory)
            faces: faces of the mesh object checking collision
            vertices: vertices of the mesh object checking collision
            face_normals: face normals of the mesh object checking collision
            env: Swift environment to plot intersection points (if desired)

        Returns:
            result (boolean): True (collision), False (no collision)
        """
        pass


    def question2(self):
        """
        2) Simple collision checking for 3-link planar robot
        """
        # Create a Swift Environment to run the exercise
        env = swift.Swift()
        env.launch(realtime=True)

        # 2.1) Create a 3 link planar robot with all 3 links having a = 1m, leave the base at eye(4).
        # Hint: Use robot = DHRobot([link1, link2, link3], name="my_robot") where link = DHLink(d=..., a=..., alpha=..., offset=..., qlim=[lower, upper]) for each link
        link1 = ...
        link2 = ...
        link3 = ...
        robot = ...

        # Creating simple cylindrical geometry for each link of the DHRobot (but this time all links will be blue)
        cyl_viz = CylindricalDHRobotPlot(robot, cylinder_radius=0.05, color="blue")
        blaster_robot = cyl_viz.create_cylinders()
        env.add(robot)                                          # Add robot to Swift environment

        # 2.2) Put a cube with sides 1.5m in the environment that is centered at [2,0,-0.5].
        # Get the vertices, faces and face_normals of a cube mesh using RectangularPrism Class in ir_support
        cube_length = ...
        lwh = [cube_length, cube_length, cube_length]           # Python list containing the desired length (X), width (Y), height (Z) of the cuboid object
        centre = [..., ..., ...]                                # Python list containing XYZ centre of cuboid
        prism = Cuboid(scale=lwh, color=[0.0, 1.0, 0.0, 0.5])   # Set colour to green, but with some transparency to see through it (RGBA)
        prism.T = ...                                           # Define a 4x4 matrix encoding the pose of the prism (reference is cube centre)
        env.add(prism)                                          # Add prism to Swift environment
        env.set_camera_pose([1, 2, 2], [1.5, 0, 0])             # Set camera pose (position, look-at)

        # Get the prism's mesh properties (vertices, faces, face normals) using RectangularPrism class from ir_support
        # Note: RectangularPrism first 3 parameters are length (along X), width (along Y), height (along Z)
        vertices, faces, face_normals = RectangularPrism(lwh[...], ..., ..., center=...).get_data()
        input("Press enter to continue, and then enter again to exit teach mode\n")

        # 2.3) Use teach and note when the links of the robot can collide with 4 of the planes:
        # Plane 1: point [1.25,0,-0.5] normal [-1,0,0]
        # Plane 2: point [2,0.75,-0.5] normal [0,1,0]
        # Plane 3: point [2,-0.75,-0.5] normal [0,-1,0]
        # Plane 4: point [2.75,0,-0.5] normal [1,0,0]

        # Create a 'teach' mode in Swift using sliders. If you prefer the matplotlib teach implementation, refer to Lab 4
        # Note: Degrees are used for the slider as they are whole numbers, compared to radians
        slider_joint1 = swift.Slider(cb=lambda value: self.slider_callback(value, 0, robot), min=-180, max=180, step=1, value=-90, desc='Joint 1', unit='°')
        slider_joint2 = swift.Slider(cb=lambda value: self.slider_callback(value, 1, robot), min=-180, max=180, step=1, value=0, desc='Joint 2', unit='°')
        slider_joint3 = swift.Slider(cb=lambda value: self.slider_callback(value, 2, robot), min=-180, max=180, step=1, value=0, desc='Joint 3', unit='°')
        env.add(slider_joint1)
        env.add(slider_joint2)
        env.add(slider_joint3)
        # Extension: Try adding some labels to show the XYZ and Roll, Pitch, Yaw.
        # To create labels: use swift.Label(text), can change text through label.desc property
        # To get XYZ: use forward kinematics
        # To get roll, pitch, yaw: use the tr2rpy() function
        # You can also try updating the slider_callback function to have the labels automatically update as the sliders change!


        # Continuously update the teach figure while it is being used (every 0.05s)
        input_thread = threading.Thread(target=self.wait_for_enter)
        input_thread.start()
        while not self.stop_event.is_set():
            # 2.4) Complete method at top of class
            tr = self.get_link_poses(..., ...)

            # 2.5) Use line_plane_intersection in ir_support to check if any of the links gets into collision with the cube 
            # (i.e. the link n intersects with any of the 4 planes of the cube when q = [0,0,0]. 
            # HINT: Note how the link n can be represented in simple fashion as a line from position TR[n][0:3,3] to TR[n+1][0:3,3]
            # The below skeleton code is provided to help guide you

            # Go through each link and triangle face
            for i in range(np.size(tr,2)-1):
                for j, face in enumerate(...):
                    # Line plane intersection requires a point on the plane being tested, access the first vertex of the current face being tested
                    vert_on_plane = ...[face][...]
                    # Use the line_plane_intersection function (if in VS Code, right click function and then 'Go To Definition' to read about input parameters)
                    # You want to access the jth face_normal, as it will correspond with the face being checked
                    intersect_p, check = line_plane_intersection(..., 
                                                                ..., 
                                                                ..., 
                                                                ...)
                    # list of all triangle combination in a face
                    triangle_list  = np.array(list(combinations(face,3)),dtype= int)

                    # Check if a line-plane intersection occured within the line segment passed to the 'line_plane_intersection' function
                    # HINT: Look at the 'line_plane_intersection' function and determine which return variable and value corresponds to this outcome
                    if ... == ...:
                        # Loop through the list of triangles. For each triangle, check if the intersection point lies inside the triangle using:
                        # is_intersection_point_inside_triangle(<intersection point>, <vertices of triangle>)
                        # HINT: Note how we obtain the vertices of each face, it is a similar method
                        # It could be a good idea to run in debug mode to check the data and structure of faces, vertices, triangles (or use print statements)
                        for triangle in ...:
                            pass                # Replace this with the above logic

                            # If the intersection point lies in the triangle, plot a red sphere in the environment
                            # Set its pose to be a 4x4 matrix with the XYZ position of the intersection point, then break 
                            # (if there is one collision, we don't need to find more)


            env.step(0.01)

        input_thread.join()

        # Reset the Swift environment to remove spheres and teach sliders - have to re-add robot and prism
        env.reset()
        env.add(robot)
        env.add(prism)
        env.set_camera_pose([-1.5, 0, 2], [2, 0, 0])  # (position, look-at)

        # 2.6) Get a trajectory from q1 to q2 and pick a value for steps such that the size of each step is less than ±1 degree.
        # Hint: Look at the step size in degrees using: (np.diff(np.rad2deg(jtraj(q1,q2,steps).q), also remember negative degrees!
        # (consider the absolute value?)
        q1 = [-pi/4,0,0]
        q2 = [pi/4,0,0]
        steps = 2         # Start with a low number of steps
        while np.any(... < ..., axis=0):
            # Increment the number of steps for higher fidelity movement, as the difference in joint positions for some steps was >1 or <-1
            steps = ...   

        # Given the number of steps is now known, create the trajectory matrix
        q_matrix = ...

        # 2.7) Check each of the joint states in the trajectory to work out which ones are in collision by writing a function
        # Return a logical vector of size steps which contains 0 = no collision (safe) and 1 = yes collision (Unsafe). 
        # You may like to use this structure:
        result = [True for _ in range(steps)]       # Create a Python list of size 'steps', where every value is default to 'True'
        for i,q in enumerate(q_matrix):
            robot.q = q
            env.step(0.05)
            # HINT: 'is_collision' should function the same as the logic of section 2.5, but for a matrix of joint vectors, not a single joint state
            result[i] = self.is_collision(robot, [q], faces, vertices, face_normals, env=...)

        input("Press enter to close Question 2 Switt environment")
        env.close()     # Close Swift browser window/environment

    
    def question3(self):
        """
        3) Basic collision avoidance for 3-link planar robot
        """
        # Create a Swift Environment to run the exercise
        env = swift.Swift()
        env.launch(realtime=True)

        # Create a 3-link planar robot (we will use the model from the toolbox on this occasion)
        robot = models.DH.Planar3()
        # Creating simple cylindrical geometry for each link of the DHRobot (but this time all links will be blue)
        cyl_viz = CylindricalDHRobotPlot(robot, cylinder_radius=0.05, color="blue")
        blaster_robot = cyl_viz.create_cylinders()
        env.add(robot)                                          # Add robot to Swift environment

        # Re-create the cube from question 2
        cube_length = 1.5
        lwh = [cube_length, cube_length, cube_length]           # Python list containing the desired length, width, height of the cuboid object
        centre = [2, 0, -0.5]                                   # Python list containing XYZ centre of cuboid
        prism = Cuboid(scale=lwh, color=[0.0, 1.0, 0.0, 0.5])   # Set colour to green, but with some transparency to see through it (RGBA)
        prism.T = transl(centre)                                # Define a 4x4 matrix encoding the pose of the prism (reference is cube centre)
        env.add(prism)                                          # Add prism to Swift environment    
        # Get the prism's mesh properties (vertices, faces, face normals) using RectangularPrism class from ir_support
        vertices, faces, face_normals = RectangularPrism(lwh[1], lwh[0], lwh[2], center=centre).get_data()

        # ------------------------------------------------------------------------------------------------------------------------------------------ #

        # 3) Determine a path from pose q1 = [-pi/4,0,0] degrees to q2 = [pi/4,0,0] that doesn’t collide with the cube from previous question.
        q1 = [-pi/4, 0, 0]
        q2 = [pi/4, 0, 0]
        q_waypoints = [q1]        # Python list to hold waypoints between q1 and q2 (inclusive) - start with q1 and add q2 at the end

        # Use the following 3 methods to make a path going between waypoints:
        # 3.1) Method 1: Manually determine intermediate joint states that are not in collision with the cube using teach. 
        # HINT: Refer to Question 2 on how to create a 'teach' mode in Swift, or Lab 4 for 'teach' in Matplotlib
        # E.g. q_waypoints.append(robot.ikine_LM(transl(x,y,z), q0=q_waypoints[-1], mask= [1,1,0,0,0,0]).q)

        # 3.2) Method 2: Manually determine Cartesian points (i.e. [x,y,z] points) that the end effector could follow such that the end effector 
        # does not go inside the cube
        # E.g. q_waypoints.append(robot.ikine_LM(transl(1.5,-1,0), q0=q_waypoints[-1], mask= [1,1,0,0,0,0]).q)

        # 3.3, 3.4, 3.5) 
        # 3.3) Method 3: Now, iteratively, randomly and automatically pick a pose within the joint angle bounds (primitive RRT)
        # q = (2 * np.random.rand(1, 6) - 1) * pi, or refer to Question 1.7
        # 3.4) Then interpolate between current pose and this new pose. If all the results are equal to 0 then the path is collision-free
        # 3.5) At each step try and connect from the current joint state to the final goal state


        # Keep upon concatenating the joint trajectory until you can reach the goal.
        result = [True for _ in range(len(q_waypoints))]       # Create a Python list of size 'steps', where every value is default to 'True'
        for i,q in enumerate(q_waypoints):
            robot.q = q
            env.step(0.05)
            result[i] = self.is_collision(robot, [q], faces, vertices, face_normals, env=...)      


        # You can visually inspect if the robot seems to go through the cube as well, although this is purely for testing
        for i in range(len(q_waypoints)-1):
            q_matrix = jtraj(q_waypoints[i], q_waypoints[i+1], 20)
            for q in q_matrix:
                robot.q = q
                env.step(0.05)

        input("Press enter to close Question 3 Switt environment")
        env.close()

        
    def slider_callback(self, value, index, robot):
        """
        This function is a callback from the Swift Slider elements. It uses the value
        returned by Swift, and encodes the robot joint that the slider is representing
        to update the robot's pose.
        """
        # Convert degrees to radians (slider in degrees, but robot joint in radians)
        radians = value * pi/180

        # Update robot.q (robot joints)
        robot.q[index] = radians

        # Extension: If you created labels for XYZ, RPY, pass them to the function
        # and update them here.


    def wait_for_enter(self):
        '''
        Helper threaded function to detect keypress without needing keyboard library
        '''
        try:
            #print("Press Enter to stop.\n")
            input()
        except EOFError:
            pass
        self.stop_event.set()


##########################################################################################################################
# Helper Functions for Collision
def is_intersection_point_inside_triangle(intersect_p, triangle_verts):
    u = triangle_verts[1, :] - triangle_verts[0, :]
    v = triangle_verts[2, :] - triangle_verts[0, :]

    uu = np.dot(u, u)
    uv = np.dot(u, v)
    vv = np.dot(v, v)

    w = intersect_p - triangle_verts[0, :]
    wu = np.dot(w, u)
    wv = np.dot(w, v)

    D = uv * uv - uu * vv

    # Get and test parametric coords (s and t)
    s = (uv * wv - vv * wu) / D
    if s < 0.0 or s > 1.0:  # intersect_p is outside Triangle
        return 0

    t = (uv * wu - uu * wv) / D
    if t < 0.0 or (s + t) > 1.0:  # intersect_p is outside Triangle
        return False

    return True  # intersect_p is in Triangle


# Main block (uncomment questions that you want to run)
if __name__ == "__main__":
    exercise = Lab5Exercises()
    exercise.question2()
    # exercise.question3()