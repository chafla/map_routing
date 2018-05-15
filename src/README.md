# Map Routing

Little test-bed ROS project of mine to process the maps returned by the implementation of gmapper in the turtlebot3 package (turtlebot3_slam).

Note that since this is entirely a testing project and *very much* a work-in-progress, it's very rough around the edges.

# Usage

Project should be cloned to `~/catkin_ws/src` or wherever you keep your catkin projects.

```bash
# Bring up the world and mapping
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
$ roslaunch turtlebot3_slam turtlebot3_slam.launch
$ rosrun map_routing navigator.py
```


