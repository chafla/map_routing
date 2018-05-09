"""Navigator node to bring a robot around the map"""

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
import map_handler
from threading import Lock, Event, Thread
from map_handler import MapHandler


class Navigator(object):

    def __init__(self):
        self.mode = None  # Current navigation mode
        self.map_array = None  # Constructed map as an array
        self._pure_pursuit_pid = None
        self._adj_graph = {}

        self._target_node = None
        self._cur_node = None
        self.cur_position = None

        rospy.Subscriber("/map", OccupancyGrid, self.on_map_data)  # rospy subscriber
        rospy.Subscriber("/odom", Odometry, self.on_odom_data)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        self._raw_map = None  # Map pulled from ROS
        self._raw_odom = None

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
        }

    def on_map_data(self, map_msg):
        """
        Update the current map array to reflect new map data from ROS.
        Check to see if it is identical to the previous one before acting.

        Identical can most likely be estimated by whether or not the rows and cols one pixel off of the cropped image's
        border are still empty

        If we don't have any map data, this should fire off init_map

        We don't want to do too many complicated operations here, but instead just get the data that we need and pass it
        off
        """

        # Write out the data so we can figure out how it looks

        print(map_msg.data)
        self.rate.sleep()


        pass

    def init_map(self):
        """
        Initialize the map based on the data passed in (what exists in the .yaml file), putting the resulting data into
        self._map_params
        """

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
    pass
