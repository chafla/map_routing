#!/usr/bin/python
"""Navigator node to bring a robot around the map"""

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Float32
import map_handler
from threading import Lock, Event, Thread
import numpy as np
from matplotlib import pyplot as plt
import cv2
from map_handler import MapHandler, WALL_COLOR, UNEXPLORED_TERRAIN_COLOR, VALID_PATH_COLOR, CLEAR_TERRAIN_MAP_COLOR


GMAP_TILE_FULL = 100
GMAP_TILE_UNDISCOVERED = -1
GMAP_TILE_EMPTY = 0


class Navigator(object):

    def __init__(self):
        rospy.init_node("map_navigator")
        self.mode = None  # Current navigation mode
        self.map_array = None  # Constructed map as an array
        self._pure_pursuit_pid = None
        self._adj_graph = {}

        self._target_node = None
        self._cur_node = None
        self.cur_position = None
        self._has_read_map_data = False

        self._raw_map = None  # Map pulled from ROS
        self._raw_odom = None
        # Corner values for map so that we don't have to check more values than we need
        # min_x, min_y, max_x, max_y
        self._map_corner_values = (-1, 10e50, -1, 10e50)

        self.rate = rospy.Rate(20)

        self._cur_twist = None

        self._map_data_lock = Lock()
        self._odom_data_lock = Lock()
        self._twist_data_lock = Lock()

        self._is_active = Event()  # Fires when we want to close all loops gracefully

        self._map_params = {
            "resolution": -1,
            "origin": (-1, -1),  # we're given a third dimension but we'll discard it later
            "inverted": False,
            "occupied_thresh": -1,  # Should be 255 * occupied_thresh
            "free_thresh": -1,
            "width": -1,
            "height": -1,
        }

        rospy.Subscriber("/map", OccupancyGrid, self.on_map_data)  # rospy subscriber
        rospy.Subscriber("/odom", Odometry, self.on_odom_data)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)

    def on_map_data(self, map_msg):
        """
        Update the current map array to reflect new map data from ROS.
        Check to see if it is identical to the previous one before acting.

        Identical can most likely be estimated by whether or not the rows and cols one pixel off of the cropped image's
        border are still empty

        If we don't have any map data, this should fire off init_map

        We don't want to do too many complicated operations here, but instead just get the data that we need and pass it
        off.

        Map data comes in as an int8
        """

        map_metadata = map_msg.info
        map_array = map_msg.data

        if not self._has_read_map_data:
            # self._has_read_map_data = True
            self.init_map(map_msg)


        # Write out the data so we can figure out how it looks

        # print(map_msg.data)
        self.rate.sleep()

        pass

    def _map_to_img(self, map_array):
        """Take in the int8[] that we get from ros and convert it to an image format that we can handle"""

    def init_map(self, map_msg):
        """
        Initialize the map based on the data passed in (what exists in the .yaml file), putting the resulting data into
        self._map_params

        In the map returned, we tend to get three values:
        -1, 100, and 0, each of which correspond to the probability they're full
        e.g. 0 is open space.
        """
        # Init the metadata
        metadata = map_msg.info
        print(metadata)
        self._map_params["resolution"] = float(metadata.resolution)
        self._map_params["width"] = int(metadata.width)
        self._map_params["height"] = int(metadata.height)
        self._map_params["origin"] = (metadata.origin.position.x,
                                      metadata.origin.position.y)

        # Create the map in a way that we can handle
        new_map_array = np.zeros((self._map_params["height"],
                                 self._map_params["width"]),
                                 np.uint8)

        ros_map_data = map_msg.data

        orig_map_arr_idx = 0

        for i in range(metadata.height):
            for j in range(metadata.width):
                # Use extra factor to scale value from 0-100 to 0-255
                ros_map_pixel_value = ros_map_data[orig_map_arr_idx]
                # todo clean this up
                if ros_map_pixel_value == 100:
                    ros_map_pixel_value = WALL_COLOR
                elif ros_map_pixel_value == -1:
                    ros_map_pixel_value = 75
                else:
                    ros_map_pixel_value = CLEAR_TERRAIN_MAP_COLOR
                new_map_array[i][j] = ros_map_pixel_value
                orig_map_arr_idx += 1

        cv2.imshow("aaaaa", new_map_array)
        cv2.waitKey(0)
        with open("asdf.txt", "a") as outfile:
            for row in new_map_array:
                outfile.write(str(row) + "\n")
            for row in ros_map_data:
                outfile.write(str(row) + "\n")

    def on_odom_data(self, odom_msg):
        """
        Update the current odom data and create a relative reference within the map
        """
        pass

    def twist_pub_thread(self):
        """
        Publish twist values as they are generated
        """

    def update_map_thread(self):
        """
        Looping thread which locks on the map data, and calculates a path from our current point to our target point if
        we have one.
        """

    def stop(self):
        """
        Set the _is_active() event, causing all running threads to fall through, and sends a "stop" twist value.
        """

    def run(self):
        """
        Start up threads
        """

        rospy.spin()


if __name__ == '__main__':

    nav = Navigator()
    nav.run()
