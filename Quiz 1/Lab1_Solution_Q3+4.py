import numpy as np
import matplotlib.image as mpimg
from scipy import linalg
import matplotlib.pyplot as plt
from spatialmath import SE2
from spatialmath.base import trplot2 
from pathlib import Path
from ir_support import functions as ir
from math import pi   

def lab1_solution_run():
    plt.close("all")
    lab1_solution = Lab1Solution()

    # Question 3: Check distance before moving, with Car2 starting at (1,1)
    dist_start = lab1_solution.questions3_and_4(
        questions=[3],
        car1_start=SE2(300, 550, 0),
        car2_start=SE2(1, 1, 0), #change this for starting position
        max_steps=0  # no movement, just starting positions
    )
    print(f"Q3 distance at start: {dist_start[0]:.2f} meters")

    # Question 3 & 4: Given start poses, after some partial steps, what's the y in car1's frame?
    car1_start_pose = SE2(300, 550, 0)
    car2_start_pose = SE2(508, 300, pi/2)  # 90 degrees in radians

    steps_car1 = 7
    steps_car2 = 5

    y_values = lab1_solution.questions3_and_4(
        questions=[3],
        car1_start=car1_start_pose,
        car2_start=car2_start_pose,
        max_steps=max(steps_car1, steps_car2),
        steps_to_report=(steps_car1, steps_car2),
        return_relative_y=True
    )
    print(f"Q3 y of car2 in car1's frame after steps: {y_values[0]:.2f}")

    # Now run the full visual for both questions 3 and 4
    lab1_solution.questions3_and_4(
        questions=[3,4],
        car1_start=car1_start_pose,
        car2_start=car2_start_pose,
        max_steps=360
    )

class Lab1Solution:
    def __init__(self):
        # Load the track image
        image_path = Path(__file__).parent / "Lab1CircularRaceTrack.jpg"
        self.img = mpimg.imread(str(image_path))

        # Track constants - please don’t change these, as per instructions
        self.RADIUS_OUTER = (550 - 66)/2
        self.RADIUS_INNER = (500 - 125)/2
        self.total_steps = 360

        # Pre-calc transforms for each step (move and turn for each car)
        self.car1_move_tr = SE2((2 * pi * self.RADIUS_OUTER)/self.total_steps, 0, 0)
        self.car1_turn_tr = SE2(0, 0, -2*pi/self.total_steps)
        self.car2_move_tr = SE2((2 * pi * self.RADIUS_INNER)/self.total_steps, 0, 0)
        self.car2_turn_tr = SE2(0, 0, 2*pi/self.total_steps)

        self.next_question = False

    def questions3_and_4(self, questions=[3,4], car1_start=SE2(300, 550, 0), car2_start=SE2(300, 125, 0),
                        max_steps=360, steps_to_report=None, return_relative_y=False):
        """
        This function runs the simulation and plots the cars on the track.

        Params:
        - questions: list of question numbers (3 and/or 4)
        - car1_start, car2_start: initial poses of the cars (SE2 objects)
        - max_steps: how many steps to simulate (0 means no movement, just show start)
        - steps_to_report: tuple with number of steps for car1 and car2 to report relative y for Q3
        - return_relative_y: if True, returns the y-coordinate of car2 in car1’s frame after steps_to_report

        Returns:
        - list of y values if return_relative_y is True
        - or list of distances if max_steps==0 for question 3
        """
        relative_y_results = []
        distances = []

        for question in questions:
            self.fig = plt.figure()
            self.fig.canvas.manager.set_window_title('Question ' + str(question))
            self.fig.canvas.mpl_connect('close_event', self.on_close)

            if question == 4:
                plt.subplot(1, 2, 1)

            plt.imshow(self.img)

            car1_tr = car1_start
            car2_tr = car2_start

            if question == 4:
                plt.subplot(1,2,2)
                plt.xlabel('Timestep')
                plt.ylabel("Distance between cars")

            dist = np.zeros(max_steps if max_steps>0 else 1)

            for i in range(max_steps):
                # Update positions only if we are actually stepping
                if max_steps > 0:
                    car1_tr = ir.clean_SE2(car1_tr * self.car1_move_tr * self.car1_turn_tr)
                    car2_tr = ir.clean_SE2(car2_tr * self.car2_move_tr * self.car2_turn_tr)

                if question == 4:
                    plt.subplot(1,2,1)

                plt.cla()
                plt.imshow(self.img)

                trplot2(car1_tr.A, frame='1', color='b', length=50, width=0.05)
                trplot2(car2_tr.A, frame='2', color='r', length=50, width=0.05)

                if question == 4:
                    plt.subplot(1, 2, 2)
                    dist[i] = linalg.norm(car1_tr.t - car2_tr.t)
                    plt.plot(range(1, i+2), dist[:i+1], 'b-')
                    plt.xlabel('Timestep')
                    plt.ylabel("Distance between cars")

                plt.draw()
                plt.pause(0.01)

                if self.next_question:
                    self.next_question = False
                    break

            # For Q3: if we want the relative y after partial steps
            if question == 3 and return_relative_y and steps_to_report is not None:
                steps_car1, steps_car2 = steps_to_report
                car1_tr_partial = car1_start
                car2_tr_partial = car2_start

                for i in range(max(steps_car1, steps_car2)):
                    if i < steps_car1:
                        car1_tr_partial = ir.clean_SE2(car1_tr_partial * self.car1_move_tr * self.car1_turn_tr)
                    if i < steps_car2:
                        car2_tr_partial = ir.clean_SE2(car2_tr_partial * self.car2_move_tr * self.car2_turn_tr)

                car2_in_car1 = car1_tr_partial.inv() * car2_tr_partial
                relative_y_results.append(round(car2_in_car1.t[1], 2))

            # For Q3 with no movement, just return starting distance
            if question == 3 and max_steps == 0:
                dist_init = linalg.norm(car1_start.t - car2_start.t)
                distances.append(dist_init)

        if return_relative_y:
            return relative_y_results
        if distances:
            return distances

    def on_close(self, event):
        # This is just so closing the figure window moves on to the next question
        self.next_question = True

if __name__ == "__main__":
    lab1_solution_run()