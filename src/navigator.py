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
import traceback
import numpy as np
from matplotlib import pyplot as plt
import cv2
from map_handler import MapHandler, WALL_COLOR, UNEXPLORED_TERRAIN_COLOR, VALID_PATH_COLOR, CLEAR_TERRAIN_MAP_COLOR
from time import sleep, time
import math
from sys import stderr

# Constants for the values that gmapping sets the values to in the int8 returned over the /map topic
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

        # Increasing wall distance has a very adverse effect on performance
        self.wall_distance = 4

        self.path_following_radius = 1

        self._robot_visited_tiles = set()

        # TODO The current offset provided by the TF data
        # Whenever we reference a set of points that we have saved, they should always be stored directly as they are
        # but then updated with the offset when read.
        # in (x, y) form
        self._cur_offset = None

        self._target_node = None
        self._cur_node = None
        self.cur_position = None  # (r, c)
        self.cur_orientation = None  # theta
        self._has_read_map_data = False

        self._origin_coords = (0, 0)

        self._raw_map_msg = None  # Map pulled from ROS
        self._raw_odom = None
        # Corner values for map so that we don't have to check more values than we need
        # min_x, min_y, max_x, max_y
        self._outermost_map_values = (-1, 10e50, -1, 10e50)

        self.rate = rospy.Rate(20)

        self._cur_twist = None
        self._cur_target_path = None  # The a* map path
        self._target_path_visited = None  # Tiles along the a* path
        self._cur_target_path_timestamp = None

        self.turn_p = 0.1
        self.vel_p = 0.05

        self._map_data_lock = Lock()
        self._odom_data_lock = Lock()
        self._twist_data_lock = Lock()
        self._robot_visited_path_lock = Lock()

        # self._display_window = cv2.namedWindow("aaaaa")

        self._interrupt_event = Event()  # Fires when we want to close all loops gracefully

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

        self.path_process_thread = Thread(target=self.process_path)
        self.map_handling_thread = Thread(target=self.init_map)
        self.twist_publisher_thread = Thread(target=self.twist_pub_runner)

        # Setting a thread as a daemon means that it dies when the main process does too
        self.path_process_thread.daemon = True
        self.map_handling_thread.daemon = True
        self.twist_publisher_thread.daemon = True

    @property
    def row_offset(self):
        return int(self._cur_offset[1] / self._map_params["resolution"])

    @property
    def col_offset(self):
        return int(self._cur_offset[0] / self._map_params["resolution"])

    def paint_img_painted(self, row, col, color):
        """Img painted is scaled so all attempts to draw on it will not work"""

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
        # Pass off the map processing to another thread since ROS subscribers only use one thread
        with self._map_data_lock:
            self._raw_map_msg = map_msg

    def _map_to_img(self, map_array):
        """Take in the int8[] that we get from ros and convert it to an image format that we can handle"""

    def init_map(self):
        """
        Initialize the map based on the data passed in (what exists in the .yaml file), putting the resulting data into
        self._map_params

        In the map returned, we tend to get three values:
        -1, 100, and 0, each of which correspond to the probability they're full
        e.g. 0 is open space.
        """

        while not self._interrupt_event.is_set():
            try:
                with self._map_data_lock:
                    if self._raw_map_msg is None:
                        sleep(0.5)
                        print("SLeeping in init map")
                        continue
                    else:
                        map_msg = self._raw_map_msg
                        self._raw_map_msg = None

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

                crop_ts = time()
                # cropped_img = MapHandler.crop_to_contents(np.copy(new_map_array), 75)
                # Get the outermost pixels so that we can constrain our search area.
                # Dealing with the full map changes these times from 1-3s to over 27s.

                # Seriously. Adding a minimum value to create_graph cut the time by 13s.
                self._outermost_map_values = MapHandler.get_outermost_coords(new_map_array, 75)
                print("Generating graph. Cropping took {}s to complete.".format(time() - crop_ts))
                graph_ts = time()

                # TODO this sometimes fails if we don't have a map tha's full enough
                self._adj_graph = MapHandler.create_graph(new_map_array, self.wall_distance, self._outermost_map_values)

                print("Painting. Graph took {}s to generate.".format(time() - graph_ts))
                _img_painted_raw = MapHandler.get_node_pixels(new_map_array, self.wall_distance, self._outermost_map_values)

                self._img_painted = np.copy(_img_painted_raw)

                if self.cur_position is not None:
                    pathfinding_ts = time()
                    print("Pathfinding.")
                    path = MapHandler.astar(_img_painted_raw, self._adj_graph, self._adj_graph.get(self.cur_position),
                                            target_unexplored=True, original_image=new_map_array,
                                            outermost_coords=self._outermost_map_values,
                                            unexplored_terrain_color=75)

                    # Comes in end-to-start, let's reverse it
                    path = list(reversed(path))

                    for tile in path:
                        self._img_painted[tile.row][tile.col] = 5

                    with self._robot_visited_path_lock:
                        self._cur_target_path = path
                        self._cur_target_path_timestamp = time()

                    print("Pathfinding complete. Took {}s.".format(time() - pathfinding_ts))
            finally:
                self.rate.sleep()




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

        # print("Time to process: {}".format(time() - init_ts))
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

    def _check_boundaries(self, new_image):
        """
        Check the spaces just outside those that we previously had marked as our boundaries to see if we need to recrop.
        """

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

        # Get our rotation for navigation purposes
        # This one seems to be in rads

        orientation = odom_msg.pose.pose.orientation
        self.cur_orientation = tf.transformations.euler_from_quaternion([orientation.x, orientation.y,
                                                                        orientation.z, orientation.w])[2]
        # print("cur position: ", self.cur_position)
        # print("Cur orientation: {}".format(self.cur_orientation))

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
                    # print(adj_row, adj_col)
                    # adj_row = int(adj_row + (self._cur_offset[1] / self._map_params["resolution"]))
                    # adj_col = int(adj_col + (self._cur_offset[0] / self._map_params["resolution"]))
                    #
                    # print(adj_row, adj_col)
                    # map_arr[adj_row][adj_col] = 173

                if self._img_painted is not None:
                    cv2.imshow("painted", MapHandler.upscale_img(self._img_painted, fx=3, fy=3))

                cv2.imshow("aaaaa", map_arr)
                cv2.waitKey(1)

    def process_path(self):
        """
        Publish twist values as they are generated.
        We should act on path basically as long as len(path) > 0. What we want to do go through each path node in order,
        seeing as how it should be generated in order from first to last. Then, each time this runs through, set each
        value on the path within a given radius as visited, then pure pursuit to the next closest path tile.

        todo it tends to get stuck when it's at about 180 degrees from its target path.
        """
        while not self._interrupt_event.is_set():
            try:
                # TODO this access should be locked
                # with self._robot_visited_path_lock:
                path = self._cur_target_path

                # Get the closest point that isn't within the radius.
                cur_path_point = None
                cur_path_point_dist = None

                if not path:
                    # We want to stop it if we have any sort of invalid path
                    twist = Twist()
                    twist.linear.x = 0
                    twist.angular.z = 0
                    self.twist_pub.publish(twist)
                    continue

                # self.map_array = cv2.circle(self.map_array, (self.cur_position[1], -self.cur_position[0]),
                #                             int(self.path_following_radius / self._map_params["resolution"]), 150)

                print("Path: ", path)
                for point in path:  # note: points in path are of type Node
                    point_dist = MapHandler.get_distance_from_coords(point.row, point.col,
                                                                     self.cur_position[0], self.cur_position[1]) * self._map_params["resolution"]

                    print("point dist: ", point_dist)

                    if point_dist >= self.path_following_radius:
                        cur_path_point = point.coords
                        cur_path_point_dist = point_dist
                        break

                    else:
                        self._cur_target_path.remove(point)
                        self.map_array[point.row][point.col] = 0
                        self._img_painted[point.row][point.col] = 230
                        pass

                else:
                    # We don't have a path for some reason, or it's too short.
                    # Either way let's not move
                    continue

                velocity = min(self.vel_p * cur_path_point_dist, 0.5)

                # Let's draw a point out somewhere
                # We don't really care where as long as it is along the line

                target_point = (int(5 * math.sin(self.cur_orientation) + self.cur_position[0]),
                                int(5 * math.cos(self.cur_orientation) + self.cur_position[1]))

                print("Target point: ", target_point)
                print("Cur point: ", self.cur_position)
                print("Cur path point: ", cur_path_point)

                self.map_array[target_point[0]][target_point[1]] = 25
                self.map_array[cur_path_point[0]][cur_path_point[1]] = 35

                # We need three lines:

                # a is cur_path_point_dist

                # b
                dist_to_target = MapHandler.get_distance_from_coords(target_point[0], target_point[1],
                                                                     self.cur_position[0], self.cur_position[1]) * self._map_params["resolution"]

                # c
                far_line_dist = MapHandler.get_distance_from_coords(target_point[0], target_point[1],
                                                                    cur_path_point[0], cur_path_point[1]) * self._map_params["resolution"]

                print("a", cur_path_point_dist)
                print("b", dist_to_target)
                print("c", far_line_dist)

                # sigma = arccos((a^2 + b^2 - c^2)/(2ab))
                acos_body = (cur_path_point_dist ** 2 + dist_to_target ** 2 - far_line_dist ** 2) / (2 * cur_path_point_dist * dist_to_target)

                # Theta is the angle between our line of sight and line to the point
                try:
                    theta = math.acos(acos_body)
                except ValueError:  # Domain error for cosine
                    print >> stderr, "acos body: {}".format(acos_body)

                # We now have to figure out if the point is to the robot's left or right so we know which way to turn
                # theta isn't enough since it's just the raw angle: always positive
                print("Turn p * theta:", self.turn_p * theta)
                turn_amount = min(self.turn_p * theta, 0.5)

                # TODO The issue is that vel scales with theta so when it wraps around we tend to go crazy
                if turn_amount >= 0.5 or theta > math.pi / 2:
                    velocity = 0

                # An issue that we tend to run into is that when theta is about equal to 180, the robot gets stuck since
                # it constantly fluctuates between turn left hard and turn right hard without any forward velocity.

                # A solution to this may be to just rotate in one direction when theta is greater than pi / 4.
                # It's a little hacky but it might do

                if theta > math.pi / 2:
                    print("Theta is too large ({}), forcing rotation.".format(theta))
                    turn_amount = 0.5
                else:

                    print("theta: ", theta)

                    print("theta (degrees):", (theta * 360) / (2 * math.pi))

                    point_to_check_right = (int(5 * math.cos(self.cur_orientation + theta) + self.cur_position[0]),
                                            int(5 * math.sin(self.cur_orientation + theta) + self.cur_position[1]))

                    point_to_check_left = (int(5 * math.cos(self.cur_orientation - theta) + self.cur_position[0]),
                                           int(5 * math.sin(self.cur_orientation - theta) + self.cur_position[1]))

                    print("cur orientation + theta: {}".format(self.cur_orientation + theta))

                    print("point to check left: {}\nright: {}".format(point_to_check_left, point_to_check_right))

                    print("target", cur_path_point)



                    if MapHandler.distance_from_coords_tuple(point_to_check_right, cur_path_point) > MapHandler.distance_from_coords_tuple(point_to_check_left, cur_path_point):

                        print("clockwise")
                        turn_amount *= -1

                    else:

                        print("counterclockwise")

                    self.map_array[point_to_check_right[0]][point_to_check_right[1]] = 45
                    self.map_array[point_to_check_left[0]][point_to_check_left[1]] = 65

                twist = Twist()

                twist.linear.x = velocity
                twist.angular.z = turn_amount

                with self._twist_data_lock:
                    self._cur_twist = twist

                print("Twist", velocity, turn_amount)






                # law of cosines to find the angle of the thing





                # Get the slope to the target point, then get a normal line to that point
                # Find the intersection of that normal line and a line that's straight along line of sight of the robot
                #
            except KeyboardInterrupt:
                self.stop()
            except Exception:
                print("Exception occured in process_path:")
                traceback.print_exc()

            finally:
                # self.rate.sleep()
                sleep(0.25)

        print("process_path thread exiting.")

    def twist_pub_runner(self):
        while not self._interrupt_event.is_set():
            try:
                with self._twist_data_lock:
                    twist = self._cur_twist
                    if twist is None:
                        continue

                self.twist_pub.publish(twist)
            finally:
                self.rate.sleep()

    def update_map_thread(self):
        """
        Looping thread which locks on the map data, and calculates a path from our current point to our target point if
        we have one.
        """

    def stop(self):
        """
        Set the _is_active() event, causing all running threads to fall through, and sends a "stop" twist value.
        """
        print("Stopping...")
        zero_twist = Twist()
        zero_twist.angular.z = 0
        zero_twist.linear.x = 0
        self.twist_pub.publish(zero_twist)
        sleep(1)  # Wait for the twist thing to publish before we kill everything
        self._interrupt_event.set()
        self.path_process_thread.join()

    def run(self):
        """
        Start up threads
        """
        self.path_process_thread.start()
        self.map_handling_thread.start()
        self.twist_publisher_thread.start()


if __name__ == '__main__':

    nav = Navigator()
    try:
        nav.run()
        rospy.spin()
    except KeyboardInterrupt:
        nav.stop()
