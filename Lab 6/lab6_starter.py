# Require libraries
import numpy as np
import matplotlib.pyplot as plt
from spatialmath.base import *
from ir_support import schunk_UTS_v2_0, line_plane_intersection, make_ellipsoid

# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
class Lab6Starter:
    def __init__(self):
        plt.close('all')

        self.p_normal = np.array([0, 1, 0])  # Normal of the plane
        self.p_point = np.array([0, 1.55, 0])  # Point on the plane
        
        self.fig = None # Figure handle

        self.normal_line_h = []  # Line plot handles
        self.orient_line_h = []
        self.approach_line_h = []
        
        self.normal_point_h = None  # Point plot handles
        self.orient_point_h = None
        self.approach_point_h = None
        
        self.intersect_p_h = []  # Handles for the intersection point
        self.ray = None  # Handle for the ray from the end effector to the intersection point
        self.ray_length = 10  # Length of the ray from the end effector
        
        self.default_q = [0, np.pi/2, 0, 0, 0, 0]  # Robot start pose
        self.tr_line_length = 0.1  # Length of the lines used for the legs of the tr plot
        self.jacob_elipsoid_h = []  # Handle for the jacobian ellipsoid plot
        self.colors = ['red', 'green', 'blue', 'orange', 'purple', 'yellow'] # Color for each joint
        self.bar_handle = [] # Handle for the bar plot
        
        self._add_robot_and_plane()
        self._plot_approach_vector_intersection()

    # ---------------------------------------------------------------------------------------#
    def test1(self):
        """
        Test the class
        """
        self._reset()
        self._create_jacobian_plot()
        # Move joint 1, 1 degree at a time and plot intersections
        for q1 in np.arange(-pi/8, pi/8 + pi/180, pi/180):
            self.robot.q[0] = q1
            self._plot_approach_vector_intersection()
            # self._plot_tr()  
            # self._visualise_jacobian(self.robot.q)
            self.fig.step(0.05)  

    # ---------------------------------------------------------------------------------------# 
    def test2(self):
        """
        Test the class
        """
        self._reset()
        self._create_jacobian_plot()
        # Move joint 1 and 5 to sweep and tilt, 5 degrees at a time and plot intersections
        step_size = 5 * pi/180
        for q1 in np.arange(-pi/8, pi/8 + step_size, step_size):
            for q5 in np.arange(-pi/8, pi/8 + step_size, step_size):
                self.robot.q[0] = q1
                self.robot.q[4] = q5
                self._plot_approach_vector_intersection()
                self._plot_tr()
                # self._visualise_jacobian(self.robot.q)
                self.fig.step(0.05)

    # ---------------------------------------------------------------------------------------#
    def _add_robot_and_plane(self):
        self.robot = schunk_UTS_v2_0() # robot hanlde
        self.fig = self.robot.plot(self.default_q, limits= [-1.5,1.5,-1.5,1.5,0,1.5]) 
        ax = self.fig.ax   
        ax.set_box_aspect([1,1,0.5])
        ellipsoid_lengths = np.array([1 if length == 0 else 0 for length in self.p_normal])
        
        _,_ = make_ellipsoid(ellipsoid_lengths, self.p_point, v = np.linspace(0, np.pi/2, 50), ax= ax) # specify v range as np.pi/2 to draw the upper half only

    # ---------------------------------------------------------------------------------------# 
    def _create_jacobian_plot(self):
        plt.figure(num='Jacobian visualisation')
        ax = plt.subplot()
        ax.set_xlim(0,6.5)
        ax.set_ylim(-1,1)
        ax.axvline(x=2)
        ax.axvline(x=4)
        ax.set_xticks([1,3,5])
        ax.set_xticklabels(['x', 'y', 'z'])
        ax.set_xlabel('End effector cartesian movement in base frame')
        ax.set_ylabel('Change in (m) for a given positive delta q')
        for i, color in enumerate(self.colors):
            ax.bar(i, 0, color = color, label = f'Joint{i+1}')
        ax.legend(loc='lower right')

    # ---------------------------------------------------------------------------------------# 
    def _visualise_jacobian(self,q_input):
        """
        Visualise jacobian
        """
        q = np.array(q_input)
        translation_jacobian = np.zeros([3,6])
        for joint in range(6):
            small_delta = np.zeros([1,6])
            small_delta[0,joint] = pi/180
            tr_plus = self.robot.fkine(q + small_delta).A
            tr_minus = self.robot.fkine(q - small_delta).A
            translation_jacobian[:,joint] = tr_plus[:3,3] - tr_minus[:3,3]
        # Normalise x,y,z so highest value = 1
        translation_jacobian = translation_jacobian/np.max(np.abs(translation_jacobian))

        plt.figure(num='Jacobian visualisation')
        ax = plt.subplot()
        try:
            for bar in self.bar_handle:
                bar.remove()
            self.bar_handle = []
        except:
            pass
        bar_position = [1, 3 , 5] 
        for i in range(6):
            for index,val in enumerate(translation_jacobian[:,i]):
                self.bar_handle.append(ax.bar(bar_position[index], val, width=0.5, color=self.colors[i]))

        try:
            self.jacob_elipsoid_h.remove()
            self.jacob_elipsoid_h = []
        except:
            pass
        tr = self.robot.fkine(q).A
        ellipsoid_matrix = translation_jacobian @ translation_jacobian.T
        self.jacob_elipsoid_h,_ = make_ellipsoid(ellipsoid_matrix, tr[:3,3], color= 'b', 
                                                 v = np.linspace(0, np.pi, 50), ax = self.fig.ax)  # specify v range as np.pi/2 to draw the upper half only
        
    # ---------------------------------------------------------------------------------------# 
    def _plot_approach_vector_intersection(self):
        tr = self.robot.fkine(self.robot.q).A
        start_p = tr[:3,3]
        end_p = tr[:3,3] + self.ray_length * tr[:3,2]
        
        # Plot the intersection point
        intersect_p, _ = line_plane_intersection(self.p_normal, self.p_point, start_p, end_p)
        self.intersect_p_h.append(self.fig.ax.plot(intersect_p[0],
                                                   intersect_p[1],
                                                   intersect_p[2], 'c*'))
        
        # Plot the ray from the end effector to the intersection point
        if self.ray is not None:
            for line in self.ray:
                line.remove()
        self.ray = self.fig.ax.plot([start_p[0], intersect_p[0]],
                                    [start_p[1], intersect_p[1]],
                                    [start_p[2], intersect_p[2]], 'b')

    # ---------------------------------------------------------------------------------------# 
    def _plot_tr(self):
        tr = self.robot.fkine(self.robot.q).A

        # The axes for the coordinate frame on the end effector are called 
        # "normal", "orient" and "approach" for the local X,Y,Z axes of the coordinate frame
        start_p = tr[0:3,3]
        normal_end_p = start_p + self.tr_line_length * tr[:3,0]
        orient_end_p = start_p + self.tr_line_length * tr[:3,1]
        approach_end_p = start_p + self.tr_line_length * tr[:3,2]

        # Plot normal orient and approach with red, green and blue
        self.normal_line_h.append(self.fig.ax.plot([start_p[0], normal_end_p[0]],
                                                   [start_p[1], normal_end_p[1]],
                                                   [start_p[2], normal_end_p[2]], 'r'))
        self.orient_line_h.append(self.fig.ax.plot([start_p[0], orient_end_p[0]],
                                                   [start_p[1], orient_end_p[1]],
                                                   [start_p[2], orient_end_p[2]], 'g'))
        self.approach_line_h.append(self.fig.ax.plot([start_p[0], approach_end_p[0]],
                                                   [start_p[1], approach_end_p[1]],
                                                   [start_p[2], approach_end_p[2]], 'b'))
        
    # ---------------------------------------------------------------------------------------# 
    def _reset(self):
        self._delete_intersection_plot()
        self._delete_tr_plots()
        self.robot.q = self.default_q
        self.fig.step(0.01)
        plt.figure(num='Jacobian visualisation').gca().cla()
        
    # ---------------------------------------------------------------------------------------# 
    def _delete_intersection_plot(self):
        for points in self.intersect_p_h:
            for point in points:
                point.remove()
        self.intersect_p_h = []

    # ---------------------------------------------------------------------------------------# 
    def _delete_tr_plots(self):
        for points in self.normal_line_h:
            for point in points:
                point.remove()
        
        for points in self.orient_line_h:
            for point in points:
                point.remove()

        for points in self.approach_line_h:
            for point in points:
                point.remove()

        self.normal_line_h = []
        self.orient_line_h = []
        self.approach_line_h = []
    
# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab6_starter = Lab6Starter()
    
    input('Enter to start test 1\n')
    lab6_starter.test1()

    input('Enter to start test 2\n')
    lab6_starter.test2()
    lab6_starter.fig.hold()