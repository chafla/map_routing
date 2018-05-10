import numpy as np
import matplotlib.path as mpath
import matplotlib.pyplot as plt
import math

import rospy
from lidar_navigation.msg import Contour
# Map is 30x30


start = (0, 15)
end = (25, 25)
grid = np.zeros((30, 30))
path = np.zeros((30, 30))


class Bug(object):

    def __init__(self, start_point, target_point, move_step, turn_direction="left"):
        self.start = start_point
        self.target = target_point
        self.current_point = start_point
        self.move_step = move_step
        self._tracking_wall = False
        self._current_tracking_direction = None
        self.turn_direction = turn_direction

    def move(self):
        while self.current_point != self.target:
            if self._tracking_wall:
                # check if we now have a clear shot at the goal from our current point
                closest_point = self.closest_point(*self.current_point)
                slope_to_closest = self.get_slope(closest_point, self.current_point)
                slope = -1 * 1 / (slope_to_closest[0] / slope_to_closest[1])
            else:
                # Head in the direction of the current path
                slope = (self.target[0] - self.current_point[0]), (self.target[1] - self.current_point[1])
            # theta of path is theta of the slope, we just reduce the distance
            path_theta = math.degrees(math.atan2(slope[1], slope[0]))
            # Move linearly towards the point: step is rcostheta or rsintheta, where r is step
            x_step = self.move_step * math.cos(path_theta)
            y_step = self.move_step * math.sin(path_theta)

            self.current_point = (self.current_point[0] + x_step, self.current_point[1] + y_step)
            self.update_plot()

    def space_is_occupied(self, x, y):
        """
        point: (x, y) tuple
        :return: Boolean: True if space occupied, else False
        """
        pass

    def update_plot(self):
        """
        Update the plot with the bot's current position
        :return:
        """

    def get_slope(self, point_a, point_b):
        # Form: (dy, dx)
        return point_b[1] - point_a[1], point_b[0] - point_a[0]

    def get_closest_point(self, x, y):
        """
        Return the slope from the current point to the closest occupied point on the graph
        :return: (dy, dx)
        """

        # TODO Optimize this

        radius = self.move_step
        closest_point = (0, 0)
        closest_point_dist = 0
        for cur_x in range(x - radius, x + radius):
            for cur_y in range(y - radius, y + radius):
                if self.space_is_occupied(cur_x, cur_y):
                    dist = math.sqrt((cur_x - x) ** 2 + (cur_y - y) ** 2)
                    if dist < closest_point_dist:
                        closest_point_dist = dist
                        closest_point = (cur_x, cur_y)

        return closest_point


class Graph(object):

    def __init__(self):
        self.figure, self.axes = plt.subplots()
