import cv2
import yaml
import numpy as np
import json
import math
# import time
from Queue import PriorityQueue

from time import time, sleep

MAP_NAME = "double_zz_map"
CLEAR_TERRAIN_COLOR = 254  # reee
VALID_PATH_COLOR = 75
WALL_COLOR = 255

with open("maps/{}.yaml".format(MAP_NAME)) as map_yaml:
    map_data = yaml.load(map_yaml)

img = cv2.imread("maps/{}.pgm".format(MAP_NAME), 0)
# cv2.imshow("asdf", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


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

    @classmethod
    def from_dict(cls, serialized_dict):
        raw_neighbors = serialized_dict["neighbors"]
        neighbor_nodes = set()
        for node_coords in raw_neighbors:
            # Stored as a list of coord pairs
            # we want to cache nodes that we know exist and then go back in and fill them

            for r, c in raw_neighbors.split(","):
                neighbor_nodes.add(Node(r, c))

        return cls(serialized_dict["r"],
                   serialized_dict["c"],
                   serialized_dict["w"],
                   serialized_dict["h"],
                   neighbor_nodes)

    def add_neighbors(self, nodes):
        self.neighbors.update(nodes)


class MapHandler(object):

    def __init__(self):
        pass

    @staticmethod
    def crop_to_contents(img):
        width, height = img.shape
        # Crop map

        min_x = min_y = None
        max_x = -1
        max_y = 10e50

        for row in range(height):
            last_row_occupied = False
            for col in range(width):

                # the color of the pixel is just the grayscale intensity (0-255)
                intensity = img[row][col]
                if intensity != 205:  # The value for unexplored space
                    last_row_occupied = True

                    if min_x is None and min_y is None:  # Looking for the top and left bounds
                        min_x = col
                        min_y = row
                    # Have to do this here so that it's still accounted for properly
                    max_x = max(col, max_x)

            if not last_row_occupied and min_x is not None and min_y is not None:
                max_y = min(row, max_y)
        img = img[min_y:max_y, min_x:max_x]
        return img

    @staticmethod
    def _dist(x1, y1, x2, y2):
        ret = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return ret if ret != 0 else 1

    @staticmethod
    def _get_dist_heuristic(img, wall_dist, row, col):
        for i in range(row - wall_dist, row + wall_dist + 1):
            if not 0 < i < img.shape[0]:
                continue
            for j in range(col - wall_dist, col + wall_dist + 1):
                if not 0 < j < img.shape[1]:
                    continue
                # if i == row and j == col:
                #     continue
                if img[i][j] == 0:  # if we've found that it's within range of a black pixel
                    dist = 255.0 * (float(wall_dist) / float(MapHandler._dist(row, col, i, j)))
                    return dist
        return VALID_PATH_COLOR

    @staticmethod
    def _check_surrounding_px(img, wall_dist, row, col):
        for i in range(row - wall_dist, row + wall_dist + 1):
            for j in range(col - wall_dist, col + wall_dist + 1):
                if img[i][j] == 0:  # if it's a black pixel
                    return False
        return True

    @staticmethod
    def get_node_pixels(img, wall_dist):
        """Get the pixels which are a given distance away from any other pixels"""
        img_tmp = np.copy(img)
        w, h = img_tmp.shape
        for row in range(h):
            for col in range(w):
                # watch for out of range
                # The -1 and +1 in _check_surrounding_px are because otherwise it stops short
                if wall_dist < col < w - wall_dist - 1 and wall_dist < row < h - wall_dist - 1 and img_tmp[row][col] != 0:
                    # if MapHandler._check_surrounding_px(img_tmp, wall_dist, row, col):
                    #     # img_tmp[row][col] = VALID_PATH_COLOR
                    img_tmp[row][col] = MapHandler._get_dist_heuristic(img_tmp, wall_dist, row, col)
        return img_tmp

    @staticmethod
    def create_graph(img, wall_dist):
        """Create nodes for every path tile on the graph. Each node holds its own neighbors."""
        graph = {}
        # img_tmp = np.copy(img)

        # Go over once to identify nodes
        h, w = img.shape  # HEIGHT - WIDTH
        print(img.shape)
        for row in range(h):
            for col in range(w):
                # print(w)
                # watch for out of range
                # The -1 and +1 in _check_surrounding_px are because otherwise it stops short
                if 0 < col < w + 1 and 0 < row < h + 1:
                    dist = MapHandler._get_dist_heuristic(img, wall_dist, row, col)
                    if (row, col) == (53, 2):
                        print("Aaaaa")
                    new_node = Node(row, col)
                    # TODO When we go in here there's a chance not all neighbors have been generated
                    # neighbors = MapHandler.get_neighbor_coords(new_node, img)
                    # new_node.add_neighbors(MapHandler.get_neighbors_from_img(new_node, img, graph))
                    new_node.dist_from_wall = dist
                    graph[(row, col)] = new_node

        # get neighbors: requires going back over after it's been generated
        for coords, node in graph.iteritems():
            # node = graph[(row, col)]
            node.add_neighbors(MapHandler.get_neighbors_from_img(node, img, graph, CLEAR_TERRAIN_COLOR))

        return graph

    @staticmethod
    def get_first_path_tile(graph, img):
        h, w = img.shape
        print(img.shape)

        # Get the first tile
        for row in range(h - 1):  # For oob
            for col in range(w - 1):
                if img[row][col] == VALID_PATH_COLOR:
                    return graph[(row, col)]

    @staticmethod
    def bfs(image, graph_map, starting_node, target_node, node_width=1, node_height=1):
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

            for neighbor_node in MapHandler.get_neighbors_from_img(cur_node, image_tmp, graph_map):
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
        # image_tmp = cv2.resize(image_tmp, None, fx=3, fy=3)
        # cv2.imshow("Path", image_tmp)
        return path_back

        # No check to see if we're at goal since we'll break when the queue size is 0

    @staticmethod
    def astar(image, graph_map, starting_node, target_node, node_width=1, node_height=1, show_progress=False):
        """
        More clever a-star to go through the maze.
        Rather than just pulling the pixels which are a safe distance from the wall, we will instead calculate nodes
        for every pixel that isn't a wall and then try to navigate. Our heuristic will reflect the distance
        to the nearest wall (as our target, the turtlebot3, has a nonzero physical size)."""
        graph_keys = graph_map.keys()
        assert starting_node.coords in graph_keys and target_node.coords in graph_keys
        image_tmp = np.copy(image)
        queue = PriorityQueue()
        parents = {}
        visited = set()
        cur_node = starting_node if starting_node is not None else MapHandler.get_first_path_tile(graph_map, image_tmp)

        cur_node.dist_from_start = 0

        queue.put(cur_node)

        while len(parents) != len(graph):
            cur_node = queue.get()

            if cur_node in visited:
                continue

            if cur_node == target_node:
                break

            print(cur_node)

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

        cur_node = target_node

        print("Tracing back")
        while cur_node != starting_node:
            print(cur_node)
            path_back.append(cur_node)
            image_tmp[cur_node.row][cur_node.col] = 0
            cur_node = parents[cur_node]
            if show_progress:

                new_img = cv2.resize(image_tmp, None, fx=3, fy=3, interpolation=cv2.INTER_AREA)
                cv2.imshow("Path", new_img)
                sleep(0.05)
                cv2.waitKey(1)

        if len(path_back) == 1:
            # Single element path
            print("Could not find a path.")
            return []

        print("Dummy")
        image_tmp = cv2.resize(image_tmp, None, fx=3, fy=3, interpolation=cv2.INTER_AREA)
        cv2.imshow("Path", image_tmp)
        return path_back

    @staticmethod
    def upscale_img(img, fx=3, fy=3):
        return cv2.resize(img, None, fx=fx, fy=fy, interpolation=cv2.INTER_AREA)

    @staticmethod
    def get_neighbor_coords(node, img, target_color=VALID_PATH_COLOR):
        """Return the coordinates of all valid neighbor path tiles"""
        neighbors = []
        print(img[node.row, node.col])
        if node.col > 0 and img[node.row][node.col - 1] == target_color:
            neighbors.append((node.row, node.col - 1))

        if node.col < img.shape[1] and img[node.row][node.col + 1] == target_color:
            neighbors.append((node.row, node.col + 1))

        if node.row > 0 and img[node.row - 1][node.col] == target_color:
            neighbors.append((node.row - 1, node.col))

        if node.row < img.shape[0] and img[node.row + 1][node.col] == target_color:
            neighbors.append((node.row + 1, node.col))

        return neighbors

    @staticmethod
    def get_neighbors_from_img(node, img, graph, target_color=VALID_PATH_COLOR):
        """Get the neighbor nodes based on the map."""
        # Messy but grabs neighbors efficiently
        # print(img.size)
        neighbors = []
        # print(img[node.row, node.col])
        if node.col > 0 and img[node.row][node.col - 1] == target_color:
            neighbors.append(graph[(node.row, node.col - 1)])

        if node.col < img.shape[1] - 1 and img[node.row][node.col + 1] == target_color:
            neighbors.append(graph[(node.row, node.col + 1)])

        if node.row > 0 and img[node.row - 1][node.col] == target_color:
            neighbors.append(graph[(node.row - 1, node.col)])

        if node.row < img.shape[0] - 1 and img[node.row + 1][node.col] == target_color:
            neighbors.append(graph[(node.row + 1, node.col)])

        return neighbors

    @staticmethod
    def get_graph_neighbors(node, img, graph):
        # Messy but grabs neighbors efficiently
        # print(img.size)
        neighbors = []
        print(img[node.row, node.col])
        if node.col > 0 and img[node.row][node.col - 1] == VALID_PATH_COLOR:
            neighbors.append(graph[(node.row, node.col - 1)])

        if node.col < img.shape[1] and img[node.row][node.col + 1] == VALID_PATH_COLOR:
            neighbors.append(graph[(node.row, node.col + 1)])

        if node.row > 0 and img[node.row - 1][node.col] == VALID_PATH_COLOR:
            neighbors.append(graph[(node.row - 1, node.col)])

        if node.row < img.shape[0] and img[node.row + 1][node.col] == VALID_PATH_COLOR:
            neighbors.append(graph[(node.row + 1, node.col)])

        return neighbors

    @staticmethod
    def graph_to_json(graph, filepath="graph.json"):
        """
        Serialize and save a graph to json
        Format:
        {


        """
        output_dict = {}
        for coords, node in graph.iteritems():
            output_dict["{},{}".format(coords[0], coords[1])] = node.to_dict()

        with open(filepath, "w") as f:
            json.dump(output_dict, f, indent=4)

    @staticmethod
    def graph_from_json(filepath):
        graph = {}
        with open(filepath, "r") as f:
            data_dict = json.load(f)

        for coords, node_data in data_dict.iteritems():
            coords_split = coords.split(",")
            r = int(coords_split[0])
            c = int(coords_split[1])
            graph[(r, c)] = Node.from_dict(node_data)

        return graph


if __name__ == '__main__':

    time_init = time()
    img_cropped = MapHandler.crop_to_contents(img)
    print("Cropped")
    print("Time: {}".format(time() - time_init))
    img_painted = MapHandler.get_node_pixels(img_cropped, 2)
    print("Found valid tiles")
    print("Time: {}".format(time() - time_init))
    # img_painted = cv2.resize()
    graph = MapHandler.create_graph(img_cropped, 2)
    # cv2.imshow("cropped", img_cropped)
    # print(graph)
    # path = MapHandler.bfs(img_painted, graph, Node(55, 13), Node(7, 4))
    path = MapHandler.astar(img_painted, graph, graph.get((6, 54)), graph.get((55, 55)), show_progress=True)

    # MapHandler.graph_to_json(graph, "test.json")
    #
    # loaded_graph = MapHandler.graph_from_json("test.json")
    # MapHandler.astar(img_painted, loaded_graph, graph.get((6, 54)), graph.get((55, 55)))
    # # img_painted = cv2.resize(img_painted, None, fx=3, fy=3)
    #
    # print("Path:", path)
    # print(path)

    # cv2.imshow("afafa", img_painted)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
