import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from spatialmath import SE2
from spatialmath.base import trplot2 
from pathlib import Path
from ir_support import functions as ir
from math import pi   

class Lab1Solution:
    def __init__(self):
        image_path = Path(__file__).parent / "Lab1CircularRaceTrack.jpg"
        self.img = mpimg.imread(str(image_path))

        self.fig = None
        self.next_question = False

        self.RADIUS_OUTER = (550 - 66)/2
        self.RADIUS_INNER = (500 - 125)/2

    def question2(self):
        self.fig = plt.figure()
        # Comment this out if you don't have on_close defined
        # self.fig.canvas.mpl_connect('close_event', self.on_close)
        self.fig.canvas.manager.set_window_title('Question 2')
        plt.imshow(self.img)
        
        car1_tr = SE2(300, 550, 0)
        trplot2(car1_tr.A, frame='1', color='b', length=50, width=0.05)

        total_steps = 360
        iterations = 106

        car1_move_tr = SE2((2 * pi * self.RADIUS_OUTER)/total_steps, 0, 0)
        car1_turn_tr = SE2(0, 0, -2 * pi / total_steps)

        for _ in range(iterations):
            plt.cla()
            plt.imshow(self.img)

            car1_tr = ir.clean_SE2(car1_tr * car1_move_tr * car1_turn_tr)
            trplot2(car1_tr.A, frame='1', color='b', length=50, width=0.05)

            message = '\n'.join(['  '.join([f"{val:.2g}" for val in row]) for row in car1_tr.A])
            plt.text(10, 50, message, fontsize=10, color=[.6, .2, .6])

            plt.draw()
            plt.pause(0.01)

            if self.next_question:
                self.next_question = False
                break

        y_pos_pixels = car1_tr.t[1]
        print(f"Exact Y position after {iterations} iterations (pixels): {y_pos_pixels}")

def lab1_solution_run():
    plt.close("all")
    lab1_solution = Lab1Solution()
    lab1_solution.question2()
    plt.show()

if __name__ == "__main__":
    lab1_solution_run()