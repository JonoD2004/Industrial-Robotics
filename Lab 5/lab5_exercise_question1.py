# Required libraries
import numpy as np
import time
from spatialmath.base import *
from roboticstoolbox import jtraj, Robot
from ir_support import schunk_UTS_v2_0, check_intersections, SwiftUFOFleet, CylindricalDHRobotPlot

from spatialgeometry import Cylinder
from spatialmath import SE3
import swift

# Useful variables
from math import pi

# ---------------------------------------------------------------------------------------------------------------------------------------------#
class Lab5Exercises:
    def __init__(self):
        print(f"Welcome to Lab 5! Question 1 Starting Point")


    # 1.7) Create a function called get_goal_joint_state (must take the parameters “blaster_robot” and “ufo_fleet” and return “goal_joint_state”)
    def placeholder_name(self, blaster_robot: Robot, ufo_fleet: SwiftUFOFleet):
        """
        Generate a 'goal joint state' for a robot to look towards an object in 3D space

        Here is one solution that randomly picks a pose with some basic logic
        """
        goal_joint_state = [q + (np.random.random() - 0.5) * 20 * np.pi / 180 for q in blaster_robot.q]
        ee_tr = blaster_robot.fkine(goal_joint_state)
        
        # Ensure the Z component of the Z axis is positive (pointing upwards), and the Z component of the point is above 1 (approx mid height)
        while ee_tr.A[2, 2] < 0.1 or ee_tr.A[2, 3] < 1:
            goal_joint_state = np.array(blaster_robot.q) + (np.random.rand(1, 6) - 0.5) * 20 * np.pi / 180
            ee_tr = blaster_robot.fkine(goal_joint_state)
            #print(f'trying again')
        
        return ...
    

    def question1(self):
        """
        1) Help the Robot Blaster save the planet from UFOs!
        """
        # Create a Swift Environment to run the exercise
        env = swift.Swift()
        env.launch(realtime=True)
        env.set_camera_pose([8, 8, 10], [0, 0, 4])  # Set camera (position, look-at)

        # 1.2) Create and plot a UFO Fleet of 10 ships
        ufo_fleet = SwiftUFOFleet(env, 10)

        # 1.3) Create and plot the blaster robot. Note this is the actual “Grit Blasting” robot working on the Sydney Harbor Bridge
        blaster_robot = schunk_UTS_v2_0()
        # Create simple cylindrical geometry for the Schunk robot (this should work for all DHRobots with valid DH parameters)
        cyl_viz = CylindricalDHRobotPlot(blaster_robot, cylinder_radius=0.05, multicolor=True)
        blaster_robot = cyl_viz.create_cylinders()
        blaster_robot.q = np.zeros([1, 6])                           # Set initial joint state as all zeros
        qlim = np.transpose(blaster_robot.qlim)                      # Get the joint limits of the robot
        env.add(blaster_robot)                                       # Add robot to swift environment

        # 1.4) Plot a "blast" cylinder coming out of the end effector (assume it is the Z-axis of the end effector).
        # Make the cylinder 10m long with a radius of 0.01m. 
        # Note the frame of reference is in the middle of the cylinder, so you will need to translate it along the Z-axis 
        # to ensure the end is extruding from the end-effector.
        end_effector_tr = blaster_robot.fkine(np.zeros([1,6]))
        beam = Cylinder(radius=..., length=..., color='#F8B500', pose=end_effector_tr @ SE3(0, 0, 5))
        env.add(beam)

        # 1.5) Now plot a "scoreboard" in Swift using Label
        current_score = 0
        text_h = f'Score: {current_score} after 0 seconds'
        label = swift.Label(text_h)
        env.add(label)
        
        start_time = time.time()        # Start timer to track duration of loop

        # 1.6) Add the following "while loop" to iteratively call a function to aim laser at UFOs
        while not ufo_fleet.is_destroy_all():
            ufo_fleet.step()
            # Get the goal joint state 
            goal_joint_state = self.get_goal_joint_state(blaster_robot, ufo_fleet)
            # Fix goal pose back to a small step away from the min/max joint limits
            fix_index_min = np.where(goal_joint_state < qlim[:, 0])[0]
            if len(fix_index_min) > 0:
                for index in fix_index_min:
                    goal_joint_state[index] = qlim[index, 0] + 10 * np.pi / 180
            fix_index_max = np.where(qlim[:,1] < goal_joint_state)[0]
            if len(fix_index_max) > 0:
                for index in fix_index_max:
                    goal_joint_state[index] = qlim[index, 1] - 10 * np.pi / 180
            
            # Get a trajectory from the current joint state to the goal joint state, use 5 steps (or experiment and note time vs fidelity)
            # Hint: If unsure about jtraj, reference Lab 4 Exercise/Solution
            joint_trajectory = jtraj(..., ..., ...).q
            for q in joint_trajectory:
                blaster_robot.q = q
                env.step(0.01)
                end_effector_tr = blaster_robot.fkine(...)     # Can use .fkine(blaster_robot.q) or .fkine(goal_joint_state) - slightly different

                # Get global pose of beam (halfway between end-effector and end of beam which is 10m long)
                beam.T = end_effector_tr @ SE3(0, 0, ...)
                # Get start and end points of beam to perform line-plane intersection for collision detection
                cone_ends = [end_effector_tr.A[0:3, 3], (end_effector_tr @ SE3(0,0,10)).A[0:3, 3]]

                ufo_hit_index = check_intersections(end_effector_tr.A, cone_ends, ufo_fleet)
                ufo_fleet.set_hit(ufo_hit_index)
                current_score = current_score + len(ufo_hit_index)

                duration = round(time.time() - start_time, 2)
                label.desc = f'Score: {current_score} after {duration} seconds'     # Update Swift scoreboard label

                env.step(0.05)

        print(f'Done!')
        input(f'Press enter when done')


        # 1.7) Create a function called get_goal_joint_state which is called by the while loop above. 
        # The function must take the parameters “blaster_robot” and “ufo_fleet” and use these to determine and return “goal_joint_state”

        # 1.8) Currently inside 'get_goal_joint_state; is one solution that randomly picks a pose 2. 
        # After running it 200 times the results were as follows: average time = 463 secs.

        # 1.9) Write a better solution that uses inverse kinematics like in earlier exercises, such that you get consistently faster 
        # results than the above random method.

        env.close()     # Close Swift browser window/environment


# Main block
if __name__ == "__main__":
    exercise = Lab5Exercises()
    exercise.question1()