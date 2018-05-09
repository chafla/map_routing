import cv2
import yaml
import numpy as np
import json
import math
import traceback
from Queue import PriorityQueue
from time import time, sleep
from sys import stderr

# MAP_NAME = "map_tb_world"
MAP_NAME = "double_zz_map"
VALID_PATH_COLOR = 50
CLEAR_TERRAIN_MAP_COLOR = 254  # reee
UNEXPLORED_TERRAIN_COLOR = 205
WALL_COLOR = 0
MAP_DISPLAY_SCALING_FACTOR = 7


# TODO Make BFS pathfinding work to discover the map in the first place


class Node(object):

    def __init__(self, row, col, width=1, height=1, neighbors=None):
        self.row = row
        self.col = col
        self.width = width
        self.height = height
        self.dist_from_wall = -1
        self.dist_from_start = 1e50
        # Neighbors should be unique
        self.neighbors = set(neighbors) if neighbors is not None else set()
        self.coords = (row, col)

    def __cmp__(self, other):
        # Strictly for use with a priority queue
        assert isinstance(other, Node)
        return self.dist_from_start - other.dist_from_start

    def __eq__(self, other):
        if not isinstance(other, Node):
            raise TypeError("Expected Node, got {}".format(type(other).__name__))
        return self.row == other.row and self.col == other.col

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return "Node(r{}, c{})".format(self.row, self.col, self.height, len(self.neighbors))

    def __repr__(self):
        return "<{}>".format(self.__str__())

    def distance_to(self, other):
        if isinstance(other, Node):
            r = other.row
            c = other.col
        elif isinstance(other, tuple):
            r, c = other
        else:
            raise TypeError("Expected tuple or Node")

        return math.sqrt((self.row - r) ** 2 + (self.col - c) ** 2)

    def to_dict(self):
        return {
            "r": self.row,
            "c": self.col,
            "w": self.width,
            "h": self.height,
            "neighbors": ["{},{}".format(i.row, i.col) for i in self.neighbors]
        }

    # TODO Finish from_dict()
    # We'd need to leave it up to whatever is calling from_dict to handle the nodes

    # @classmethod
    # def from_dict(cls, serialized_dict):
    #     raw_neighbors = serialized_dict["neighbors"]
    #     neighbor_nodes = set()
    #     for node_coords in raw_neighbors:
    #         # Stored as a list of coord pairs
    #         # TODO we want to cache nodes that we know exist and then go back in and fill them
    #
    #         for r, c in raw_neighbors.split(","):
    #             neighbor_nodes.add(Node(r, c))
    #
    #     return cls(serialized_dict["r"],
    #                serialized_dict["c"],
    #                serialized_dict["w"],
    #                serialized_dict["h"],
    #                neighbor_nodes)

    def add_neighbors(self, nodes):
        self.neighbors.update(nodes)


class MapHandler(object):

    def __init__(self):
        pass

    @staticmethod
    def crop_to_contents(image):
        height, width = image.shape
        # Crop map

        min_x = 10e50
        min_y = 10e50
        max_x = -1
        max_y = 10e50

        for row in range(height):
            last_row_occupied = False
            for col in range(width):

                # the color of the pixel is just the grayscale intensity (0-255)
                intensity = image[row][col]
                if intensity != 205:  # The value for unexplored space
                    min_x = min(col, min_x)
                    min_y = min(row, min_y)
                    max_x = max(col, max_x)
                    # Check if the row we're checking is occupied
                    # if it is, we'll set it below
                    last_row_occupied = True

            if not last_row_occupied and min_x < 10e50 and min_y < 10e50:
                max_y = min(row, max_y)
        image = image[min_y:max_y, min_x:max_x]
        return image

    @staticmethod
    def _dist(x1, y1, x2, y2):
        ret = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return ret if ret != 0 else 1

    @staticmethod
    def _get_dist_heuristic(image, wall_distance, row, col):
        for i in range(row - wall_distance, row + wall_distance + 1):
            if not 0 < i < image.shape[0]:
                continue
            for j in range(col - wall_distance, col + wall_distance + 1):
                if not 0 < j < image.shape[1]:
                    continue
                # if i == row and j == col:
                #     continue
                if image[i][j] == 0:  # if we've found that it's within range of a black pixel
                    dist = 255.0 * (float(wall_distance) / float(MapHandler._dist(row, col, i, j)))
                    return dist
        return VALID_PATH_COLOR

    @staticmethod
    def _check_surrounding_px(image, wall_distance, row, col):
        for i in range(row - wall_distance, row + wall_distance + 1):
            for j in range(col - wall_distance, col + wall_distance + 1):
                if image[i][j] == 0:  # if it's a black pixel
                    return False
        return True

    @staticmethod
    def get_node_pixels(image, wall_distance):
        """Get the pixels which are a given distance away from any other pixels"""
        img_tmp = np.copy(image)
        h, w = img_tmp.shape
        for row in range(h - 1):
            for col in range(w - 1):
                # watch for out of range
                # The -1 and +1 in _check_surrounding_px are because otherwise it stops short
                # The != 0 is there in case it's a wall pixel
                if wall_distance < col < w - wall_distance - 1 and wall_distance < row < h - wall_distance - 1 and \
                        img_tmp[row][col] != WALL_COLOR:
                    # if MapHandler._check_surrounding_px(img_tmp, wall_dist, row, col):
                    #     # img_tmp[row][col] = VALID_PATH_COLOR
                    img_tmp[row][col] = MapHandler._get_dist_heuristic(img_tmp, wall_distance, row, col)
        return img_tmp

    @staticmethod
    def create_graph(image, wall_distance):
        """Create nodes for every path tile on the graph. Each node holds its own neighbors."""
        map_graph = {}
        # img_tmp = np.copy(img)

        # Go over once to identify nodes
        h, w = image.shape  # HEIGHT - WIDTH
        print(image.shape)
        for row in range(h):
            for col in range(w):
                # print(w)
                # watch for out of range
                # The -1 and +1 in _check_surrounding_px are because otherwise it stops short
                if 0 < col < w + 1 and 0 < row < h + 1 and image[row, col] != WALL_COLOR:
                    dist = MapHandler._get_dist_heuristic(image, wall_distance, row, col)
                    if (row, col) == (53, 2):
                        print("Aaaaa")
                    new_node = Node(row, col)
                    # TODO When we go in here there's a chance not all neighbors have been generated
                    # neighbors = MapHandler.get_neighbor_coords(new_node, img)
                    # new_node.add_neighbors(MapHandler.get_neighbors_from_img(new_node, img, graph))
                    new_node.dist_from_wall = dist
                    map_graph[(row, col)] = new_node

        # get neighbors: requires going back over after it's been generated
        for coords, node in map_graph.iteritems():
            # node = graph[(row, col)]
            node.add_neighbors(MapHandler.get_neighbors_from_img(node, image, map_graph, CLEAR_TERRAIN_MAP_COLOR))

        return map_graph

    @staticmethod
    def get_first_path_tile(graph_map, image):
        h, w = image.shape
        print(image.shape)

        # Get the first tile
        for row in range(h - 1):  # For oob
            for col in range(w - 1):
                if image[row][col] == UNEXPLORED_TERRAIN_COLOR:
                    return graph_map[(row, col)]

    @staticmethod
    def bfs(image, graph_map, starting_node, target_node):
        """BFS to find all nodes' neighbors"""
        image_tmp = np.copy(image)
        queue = []
        visited = {}
        cur_node = starting_node if starting_node is not None else MapHandler.get_first_path_tile(graph_map, image_tmp)

        # cur_node = MapHandler.get_first_path_tile(graph_map, img)

        queue.append(cur_node)

        while len(queue) > 0:
            cur_node = queue.pop(0)

            if cur_node == target_node:
                break

            for neighbor_node in MapHandler.get_neighbors_from_img(cur_node, image_tmp, graph_map,
                                                                   UNEXPLORED_TERRAIN_COLOR):
                if neighbor_node in visited:
                    continue

                cur_node.add_neighbors([neighbor_node])
                # cur_node_neighbors.neighbors.append(neighbor_node)
                visited[neighbor_node] = cur_node
                queue.append(neighbor_node)

        path_back = []

        while cur_node != starting_node:
            path_back.append(cur_node)
            image_tmp[cur_node.row][cur_node.col] = 25

            cv2.imshow("Path", image_tmp)
            cur_node = visited[cur_node]

        if len(path_back) == 1:
            # Single element path
            print("Could not find a path.")
            return []

        print("Dummy")
        return path_back

        # No check to see if we're at goal since we'll break when the queue size is 0

    @staticmethod
    def astar(image, graph_map, starting_node, target_node=None,
              step_through_finished_path=False, step_through_explored_nodes=False, target_unexplored=False,
              original_image=None):
        """
        More clever a-star to go through the maze.
        Rather than just pulling the pixels which are a safe distance from the wall, we will instead calculate nodes
        for every pixel that isn't a wall and then try to navigate. Our heuristic will reflect the distance
        to the nearest wall (as our target, the turtlebot3, has a nonzero physical size).
        If target_unexplored is True and target_node is None, unknown tiles will be targeted, and
        original_image *must* be passed in.
        """
        graph_keys = graph_map.keys()
        assert starting_node.coords in graph_keys
        if target_node is None:
            assert target_unexplored
            assert original_image is not None
        else:
            assert not target_unexplored
            assert target_node.coords in graph_keys
        # assert target_node is not None and target_node.coords in graph_keys
        assert ((target_node is None and target_unexplored is True) or
                target_node is not None and target_unexplored is False)
        image_tmp = np.copy(image)
        queue = PriorityQueue()  # TODO This doesn't seem to be clearing correctly
        parents = {}
        visited = set()
        cur_node = starting_node if starting_node is not None else MapHandler.get_first_path_tile(graph_map, image_tmp)
        cur_node.dist_from_start = 0

        queue.put(cur_node)

        while visited != graph_keys and queue.qsize() > 0:
            cur_node = queue.get()
            # print("Cur node is {}".format(cur_node))

            if step_through_explored_nodes:
                image_tmp[cur_node.row][cur_node.col] = 254 * 255.0 / max(float(cur_node.dist_from_start), 0.1)
                MapHandler.show_img(image_tmp)
                sleep(0.001)
                cv2.waitKey(1)

            if cur_node in visited:
                continue

            elif target_unexplored and MapHandler.get_neighbor_coords(cur_node, original_image,
                                                                      UNEXPLORED_TERRAIN_COLOR):
                print("Found unexplored terrain.")
                break

            elif not target_unexplored and cur_node == target_node:
                break

            if len(cur_node.neighbors) == 0:
                raise RuntimeWarning("Node {} was searched but has no neighbors.".format(cur_node))

            # neighbors = cur_node.neighbors

            for neighbor_node in cur_node.neighbors:
                # if neighbor_node in parents:
                #     continue

                # cur_node.add_neighbors([neighbor_node])
                # cur_node_neighbors.neighbors.append(neighbor_node)
                new_dist_from_start = cur_node.dist_from_start + neighbor_node.dist_from_wall

                if new_dist_from_start < neighbor_node.dist_from_start:
                    neighbor_node.dist_from_start = new_dist_from_start
                    # cur_node.add_neighbors([neighbor_node])
                    # cur_node_neighbors.neighbors.append(neighbor_node)
                    parents[neighbor_node] = cur_node
                    queue.put(neighbor_node)

            visited.add(cur_node)

        path_back = []

        # cur_node = target_node

        if step_through_explored_nodes:
            cv2.destroyWindow("Explored")

        print("Tracing back")
        while cur_node != starting_node:
            # print(cur_node)
            path_back.append(cur_node)
            image_tmp[cur_node.row][cur_node.col] = 0
            cur_node = parents[cur_node]
            if step_through_finished_path:

                new_img = cv2.resize(np.copy(image_tmp), None, fx=MAP_DISPLAY_SCALING_FACTOR,
                                     fy=MAP_DISPLAY_SCALING_FACTOR, interpolation=cv2.INTER_AREA)
                cv2.imshow("Path", new_img)
                sleep(0.05)
                cv2.waitKey(1)

        if len(path_back) == 1:
            # Single element path
            print("Could not find a path.")
            return []

        image_tmp = cv2.resize(image_tmp, None, fx=MAP_DISPLAY_SCALING_FACTOR,
                               fy=MAP_DISPLAY_SCALING_FACTOR, interpolation=cv2.INTER_AREA)
        cv2.imshow("Path", image_tmp)
        return path_back

    @staticmethod
    def upscale_img(image, fx=MAP_DISPLAY_SCALING_FACTOR, fy=MAP_DISPLAY_SCALING_FACTOR):
        return cv2.resize(image, None, fx=fx, fy=fy, interpolation=cv2.INTER_AREA)

    @staticmethod
    def get_neighbor_coords(node, image, target_color=UNEXPLORED_TERRAIN_COLOR):
        """Return the coordinates of all valid neighbor path tiles"""
        neighbors = []
        print(image[node.row, node.col])
        if node.col > 0 and image[node.row][node.col - 1] == target_color:
            neighbors.append((node.row, node.col - 1))

        if node.col < image.shape[1] and image[node.row][node.col + 1] == target_color:
            neighbors.append((node.row, node.col + 1))

        if node.row > 0 and image[node.row - 1][node.col] == target_color:
            neighbors.append((node.row - 1, node.col))

        if node.row < image.shape[0] and image[node.row + 1][node.col] == target_color:
            neighbors.append((node.row + 1, node.col))

        return neighbors

    @staticmethod
    def get_neighbors_from_img(node, image, adj_graph, target_color):
        """Get the neighbor nodes based on the map."""
        # Messy but grabs neighbors efficiently
        # print(img.size)
        neighbors = []
        # print(img[node.row, node.col])
        if node.col > 0 and image[node.row][node.col - 1] == target_color:
            neighbors.append(adj_graph[(node.row, node.col - 1)])

        if node.col < image.shape[1] - 1 and image[node.row][node.col + 1] == target_color:
            neighbors.append(adj_graph[(node.row, node.col + 1)])

        if node.row > 0 and image[node.row - 1][node.col] == target_color:
            neighbors.append(adj_graph[(node.row - 1, node.col)])

        if node.row < image.shape[0] - 1 and image[node.row + 1][node.col] == target_color:
            neighbors.append(adj_graph[(node.row + 1, node.col)])

        return neighbors

    @staticmethod
    def get_graph_neighbors(node, image, map_graph):
        # Messy but grabs neighbors efficiently
        # print(img.size)
        neighbors = []
        print(image[node.row, node.col])
        if node.col > 0 and image[node.row][node.col - 1] == UNEXPLORED_TERRAIN_COLOR:
            neighbors.append(map_graph[(node.row, node.col - 1)])

        if node.col < image.shape[1] and image[node.row][node.col + 1] == UNEXPLORED_TERRAIN_COLOR:
            neighbors.append(map_graph[(node.row, node.col + 1)])

        if node.row > 0 and image[node.row - 1][node.col] == UNEXPLORED_TERRAIN_COLOR:
            neighbors.append(map_graph[(node.row - 1, node.col)])

        if node.row < image.shape[0] and image[node.row + 1][node.col] == UNEXPLORED_TERRAIN_COLOR:
            neighbors.append(map_graph[(node.row + 1, node.col)])

        return neighbors

    @staticmethod
    def graph_to_json(adj_graph, filepath="graph.json"):
        """
        Serialize and save a graph to json
        Format:
        {


        """
        output_dict = {}
        for coords, node in adj_graph.iteritems():
            output_dict["{},{}".format(coords[0], coords[1])] = node.to_dict()

        with open(filepath, "w") as f:
            json.dump(output_dict, f, indent=4)

    # @staticmethod
    # def graph_from_json(filepath):
    #     graph = {}
    #     with open(filepath, "r") as f:
    #         data_dict = json.load(f)
    #
    #     for coords, node_data in data_dict.iteritems():
    #         coords_split = coords.split(",")
    #         r = int(coords_split[0])
    #         c = int(coords_split[1])
    #         graph[(r, c)] = Node.from_dict(node_data)
    #
    #     return graph

    @staticmethod
    def cartesian_to_row_major(x, y):
        return y, x

    @staticmethod
    def show_img(image, rescale=True, window_name="Path"):
        temp_image = np.copy(image)
        if rescale:
            temp_image = MapHandler.upscale_img(temp_image)
        cv2.imshow(window_name, temp_image)


if __name__ == '__main__':

    wall_dist = 2

    # TODO Make this more modular

    with open("maps/{}.yaml".format(MAP_NAME)) as map_yaml:
        map_data = yaml.load(map_yaml)

    img = cv2.imread("maps/{}.pgm".format(MAP_NAME), 0)

    cv2.namedWindow("Path")

    time_init = time()
    img_cropped = MapHandler.crop_to_contents(img)
    print("Cropped")
    print("Time: {}".format(time() - time_init))
    # MapHandler.show_img(img_cropped, window_name="Cropped")
    img_painted = MapHandler.get_node_pixels(img_cropped, wall_dist)
    print("Found valid tiles")
    print("Time: {}".format(time() - time_init))
    # Generate an adjacency graph which contains all the nodes and their neighbors
    graph = MapHandler.create_graph(img_cropped, wall_dist)

    print("graph generated")
    # cv2.imshow("afafa", img_painted)
    last_start_point = (0, 0)
    last_target_point = (0, 0)
    # Without this control variable, it tends to get stuck in infinite loops when
    # something is clicked while pathing is taking place
    actively_pathing = False

    def on_mouse_click(event, x, y, *_):
        global last_start_point, last_target_point, actively_pathing
        y /= MAP_DISPLAY_SCALING_FACTOR
        x /= MAP_DISPLAY_SCALING_FACTOR

        if event == cv2.EVENT_LBUTTONDOWN:
            # First click
            if last_start_point == (0, 0) and not actively_pathing:
                last_start_point = MapHandler.cartesian_to_row_major(x, y)

                img_tmp = np.copy(img_painted)
                img_tmp[y][x] = 0
                cv2.imshow("Path", MapHandler.upscale_img(np.copy(img_tmp)))
                print("Setting start to ({}, {})".format(x, y))

            # second click
            elif last_target_point == (0, 0) and not actively_pathing:
                last_target_point = MapHandler.cartesian_to_row_major(x, y)
                print("Setting target to ({}, {})".format(x, y))
                # reverse them since it prints as it backtracks
                actively_pathing = True
                try:
                    print("Attempting to pathfind...")
                    # TODO determine why this keeps getting stuck in inf loops
                    MapHandler.astar(img_painted,
                                     graph,
                                     graph[last_target_point],
                                     graph[last_start_point],
                                     # target_unexplored=True,
                                     # original_image=img_cropped),
                                     step_through_finished_path=True,
                                     step_through_explored_nodes=True,
                                     )

                    print("Finished pathfinding.")
                except (KeyError, RuntimeError):
                    print >> stderr, "Ignoring exception:"
                    traceback.print_exc(file=stderr)
                    # print("{}: {}".format(type(e).__name__, e))
                    # last_target_point or last_start_point probs aren't in the graph
                    print("Invalid point(s) selected!")
                    last_start_point = (0, 0)
                    last_target_point = (0, 0)
                    MapHandler.show_img(img_painted)
                finally:
                    actively_pathing = False

            # reset it
            elif not actively_pathing:
                print("Resetting: setting start")
                # cv2.destroyWindow("Path")
                last_start_point = (0, 0)
                last_target_point = (0, 0)
                img_tmp = np.copy(img_painted)
                # img_tmp[y][x] = 0
                MapHandler.show_img(img_tmp)

    cv2.imshow("Path", MapHandler.upscale_img(np.copy(img_painted)))
    cv2.setMouseCallback("Path", on_mouse_click)

    # cv2.setMouseCallback("Path", release_callback)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
