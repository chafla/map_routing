#!/usr/bin/python
"""Navigator node to bring a robot around the map"""

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Float32
from tf2_msgs.msg import TFMessage
import tf
import map_handler
from threading import Lock, Event, Thread
import numpy as np
from matplotlib import pyplot as plt
import cv2
from map_handler import MapHandler, WALL_COLOR, UNEXPLORED_TERRAIN_COLOR, VALID_PATH_COLOR, CLEAR_TERRAIN_MAP_COLOR
from time import sleep, time
import math


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
        self._img_painted = None

        self._robot_visited_tiles = set()

        # TODO The current offset provided by the TF data
        # Whenever we reference a set of points that we have saved, they should always be stored directly as they are
        # but then updated with the offset when read.
        self._cur_offset = None

        self._target_node = None
        self._cur_node = None
        self.cur_position = None
        self._has_read_map_data = False

        self._origin_coords = (0, 0)

        self._raw_map = None  # Map pulled from ROS
        self._raw_odom = None
        # Corner values for map so that we don't have to check more values than we need
        # min_x, min_y, max_x, max_y
        self._outermost_map_values = (-1, 10e50, -1, 10e50)

        self.rate = rospy.Rate(20)

        self._cur_twist = None
        self._cur_target_path = None  # The a* map path

        self._map_data_lock = Lock()
        self._odom_data_lock = Lock()
        self._twist_data_lock = Lock()
        self._robot_visited_path_lock = Lock()

        # self._display_window = cv2.namedWindow("aaaaa")

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
        rospy.Subscriber("/tf", TFMessage, self.on_tf_msg)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)

    @property
    def row_offset(self):
        return int(self._cur_offset[1] / self._map_params["resolution"])

    @property
    def col_offset(self):
        return int(self._cur_offset[0] / self._map_params["resolution"])

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

        # if not self._has_read_map_data:
            # self._has_read_map_data = True
        self.init_map(map_msg)


        # Write out the data so we can figure out how it looks

        # print(map_msg.data)
        # self.rate.sleep()

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
        init_ts = time()
        # Init the metadata
        metadata = map_msg.info
        print(metadata)
        self._map_params["resolution"] = float(metadata.resolution)
        self._map_params["width"] = int(metadata.width)
        self._map_params["height"] = int(metadata.height)
        self._map_params["origin"] = (metadata.origin.position.x,
                                      metadata.origin.position.y)

        # Get the origin row and column into the frame of reference of the resolution, as position comes in in terms of
        # meters (which the simulation is actually reflecting).
        origin_row = int(metadata.origin.position.x / metadata.resolution)
        origin_col = int(-metadata.origin.position.y / metadata.resolution)
        self._origin_coords = (origin_row, origin_col)
        # origin appears to be offset from the center of the map so if it's -10
        # it should be height / 2 + origin.y width / 2 + origin.x

        # self._origin_coords = (int((metadata.height / 2) + metadata.origin.position.y),
        #                        int((metadata.width / 2) + metadata.origin.position.x))

        # Create the map in a way that we can handle
        new_map_array = np.zeros((self._map_params["height"],
                                 self._map_params["width"]),
                                 np.uint8)

        ros_map_data = map_msg.data
        print(self._origin_coords)

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

        new_map_array[origin_row][origin_col] = 175
        print("Map array == old: {}".format(np.all(new_map_array == self.map_array)))

        self.map_array = np.copy(new_map_array)

        new_map_array[self.cur_position[0]][self.cur_position[1]] = 200

        # TODO cropping speeds things up immensely but it then throws off all of our other coordinates
        # Should set min coords/max coords to search for
        # Seriously. It's a matter of ~1.6s to generate a graph for the cropped coords but 27sx for uncropped

        crop_ts = time()
        # cropped_img = MapHandler.crop_to_contents(np.copy(new_map_array), 75)
        self._outermost_map_values = MapHandler.get_outermost_coords(new_map_array, 75)
        print("Generating graph. Cropping took {}s to complete.".format(time() - crop_ts))
        graph_ts = time()

        # TODO this sometimes fails if we don't have a map tha's full enough
        self._adj_graph = MapHandler.create_graph(new_map_array, 2, self._outermost_map_values)

        print("Painting. Graph took {}s to generate.".format(time() - graph_ts))
        self._img_painted = MapHandler.get_node_pixels(new_map_array, 2, self._outermost_map_values)

        if self.cur_position is not None:
            pathfinding_ts = time()
            print("Pathfinding.")
            path = MapHandler.astar(self._img_painted, self._adj_graph, self._adj_graph.get(self.cur_position),
                                    target_unexplored=True, original_image=new_map_array,
                                    outermost_coords=self._outermost_map_values,
                                    unexplored_terrain_color=75)
            for tile in path:
                self._img_painted[tile.row][tile.col] = 5

            self._img_painted = MapHandler.upscale_img(self._img_painted, fx=3, fy=3)

            print("Pathfinding complete. Took {}s.".format(time() - pathfinding_ts))





        # with self._robot_visited_path_lock:
        #     visited = self._robot_visited_tiles

        # for adj_row, adj_col in visited:
            # print(ad, c)
            # adj_row = int(r - (self._cur_offset[1] / self._map_params["resolution"]))
            # adj_col = int(c + (self._cur_offset[0] / self._map_params["resolution"]))
            # print(adj_row, adj_col)
            # Adjust for rotation
            # print(self.row_offset, self.col_offset)
            # adj_row, adj_col = self._rotate_point(adj_row,
            #                                       adj_col, self._cur_offset[2])
            #
            # adj_row += self.row_offset
            # adj_col += self.col_offset
            # print(adj_row, adj_col)
            # adj_row = int(adj_row + (self._cur_offset[1] / self._map_params["resolution"]))
            # adj_col = int(adj_col + (self._cur_offset[0] / self._map_params["resolution"]))
            #
            # print(adj_row, adj_col)
            # new_map_array[adj_row][adj_col] = 173

        # cv2.imshow("aaaaa", new_map_array)
        # cv2.waitKey(1)

        print("Time to process: {}".format(time() - init_ts))
        # with open("asdf.txt", "a") as outfile:
        #     for row in new_map_array:
        #         outfile.write(str(row) + "\n")
        #     for row in ros_map_data:
        #         outfile.write(str(row) + "\n")

    def _rotate_point(self, x, y, theta):
        # https://stackoverflow.com/questions/2259476/rotating-a-point-about-another-point-2d
        # print("Theta: ", theta)
        sin = math.sin(theta)
        cos = math.cos(theta)

        # Translate back to origin
        # print(y)
        y += self._origin_coords[0]
        x -= self._origin_coords[1]

        # Rotate
        x_rot = x * cos - y * sin
        y_rot = x * sin + y * cos

        y_rot -= self._origin_coords[0]
        x_rot += self._origin_coords[1]

        return int(x_rot), int(y_rot)

    def on_odom_data(self, odom_msg):
        """
        Update the current odom data and create a relative reference within the map.

        TODO What currently happens is that we take note of all the tiles as we cross them but don't reflect
        map transformations, meaning that the "tiles" that we think we have crossed are pushed out of alignment with
        there they 'should' be. If we can figure out how the map transforms happen, we can work out how to update our
        existing points
        """
        cur_position_point = odom_msg.pose.pose.position

        position_row = int(cur_position_point.y / self._map_params["resolution"]) - self._origin_coords[0]
        position_col = int(cur_position_point.x / self._map_params["resolution"]) + self._origin_coords[1]

        self.cur_position = (position_row, position_col)
        # print("cur position: ", self.cur_position)

        with self._robot_visited_path_lock:
            self._robot_visited_tiles.add(self.cur_position)

        # if self.map_array is None:
        #     return
        #
        # map_arr = np.copy(self.map_array)
        #
        # for r, c in self._robot_visited_tiles:
        #     map_arr[r][c] = 173
        #
        # cv2.imshow("aaaaa", map_arr)
        # cv2.waitKey(1)
        #
        # sleep(0.25)

        # self.rate.sleep()

        # print(position_row, position_col)

    def on_tf_msg(self, msg):
        """
        Take in transform messages for the map to update our existing frames of reference, such as with our visited
        tiles.

        As the map updates, it sometimes changes its orientation due to the robot getting a better understanding of
        where it is. The TF messages coming in seem to reflect an offset which currently appears to be the shift for
        all points.
        """
        # the tfmessage has an array of transforms so let's look for the one that has a map

        for raw_tf in msg.transforms:
            if str(raw_tf.header.frame_id) == "map":
                # print(raw_tf)
                translation = raw_tf.transform.translation
                rotation = raw_tf.transform.rotation

                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [rotation.x, rotation.y,
                     rotation.z, rotation.w])

                # https://answers.ros.org/question/141366/convert-the-yaw-euler-angle-into-into-the-range-0-360/
                # yaw_degrees = yaw * 180.0 / pi

                # We do 2pi - offset because we need to rotate the path in relation to the map.
                self._cur_offset = (translation.x, translation.y, 2 * math.pi - yaw)
                # with self._robot_visited_path_lock:
                #     updated_visited_tiles = set()
                #     for tile in self._robot_visited_tiles:
                #         # self._robot_visited_tiles.remove(tile)
                #         updated_visited_tiles.add((int(tile[0] - (translation.y / self._map_params["resolution"])),
                #                                    int(tile[1] + (translation.x / self._map_params["resolution"]))))
                #     self._robot_visited_tiles = updated_visited_tiles

                with self._robot_visited_path_lock:
                    visited = self._robot_visited_tiles

                if self.map_array is None:
                    return

                map_arr = np.copy(self.map_array)
                # print(self.map_array)

                for adj_row, adj_col in visited.copy():
                    # print(ad, c)
                    # adj_row = int(r - (self._cur_offset[1] / self._map_params["resolution"]))
                    # adj_col = int(c + (self._cur_offset[0] / self._map_params["resolution"]))
                    # print(adj_row, adj_col)
                    # Adjust for rotation
                    # print(self.row_offset, self.col_offset)
                    adj_row_rot, adj_col_rot = self._rotate_point(adj_row,
                                                          adj_col, self._cur_offset[2])
                    #
                    # print(self._cur_offset)
                    map_arr[adj_row_rot][adj_col_rot] = 150
                    # adj_row += self.row_offset
                    # adj_col += self.col_offset
                    # # print(adj_row, adj_col)
                    # # adj_row = int(adj_row + (self._cur_offset[1] / self._map_params["resolution"]))
                    # # adj_col = int(adj_col + (self._cur_offset[0] / self._map_params["resolution"]))
                    # #
                    # # print(adj_row, adj_col)
                    # map_arr[adj_row][adj_col] = 173

                if self._img_painted is not None:
                    cv2.imshow("painted", self._img_painted)

                cv2.imshow("aaaaa", map_arr)
                cv2.waitKey(1)


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
