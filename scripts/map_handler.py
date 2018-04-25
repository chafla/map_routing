import cv2
import yaml
import numpy as np
import json
import math

from time import time

MAP_NAME = "double_zz_map"
VALID_PATH_COLOR = 75

with open("maps/{}.yaml".format(MAP_NAME)) as map_yaml:
    map_data = yaml.load(map_yaml)

img = cv2.imread("maps/{}.pgm".format(MAP_NAME), 0)
# cv2.imshow("asdf", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


class Node(object):

    def __init__(self, row, col, width=1, height=1, neighbors=None):
        self.row = row
        self.col = col
        self.width = width
        self.height = height
        # Neighbors should be unique
        self.neighbors = set(neighbors) if neighbors is not None else set()

    def __eq__(self, other):
        return self.row == other.row and self.col == other.col

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
            "neighbors": list(self.neighbors)
        }

    @classmethod
    def from_dict(cls, serialized_dict):
        return cls(serialized_dict["r"],
                   serialized_dict["c"],
                   serialized_dict["w"],
                   serialized_dict["h"],
                   serialized_dict["neighbors"])

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
                if wall_dist < col < w - wall_dist - 1 and wall_dist < row < h - wall_dist - 1:
                    if MapHandler._check_surrounding_px(img_tmp, wall_dist, row, col):
                        img_tmp[row][col] = VALID_PATH_COLOR

        return img_tmp

    @staticmethod
    def create_graph(img, wall_dist):
        """Create nodes for every path tile on the graph. Each node holds its own neighbors."""
        graph = {}
        # img_tmp = np.copy(img)

        # Go over once to identify nodes
        w, h = img.shape
        for row in range(h):
            for col in range(w):
                # watch for out of range
                # The -1 and +1 in _check_surrounding_px are because otherwise it stops short
                if wall_dist < col < w - wall_dist - 1 and wall_dist < row < h - wall_dist - 1:
                    if (row, col) == (56, 13):
                        print("56, 13 processed")
                    if MapHandler._check_surrounding_px(img, wall_dist, row, col):
                        new_node = Node(row, col)
                        new_node.add_neighbors(MapHandler.get_neighbors(new_node, img, graph))

                        graph[(row, col)] = new_node
        return graph

    @staticmethod
    def get_first_path_tile(graph, img):
        w, h = img.shape
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

            for neighbor_node in MapHandler.get_neighbors(cur_node, image_tmp, graph_map):
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
            cur_node = visited[cur_node]

        if len(path_back) == 1:
            # Single element path
            print("Could not find a path.")
            return []

        print("Dummy")
        image_tmp = cv2.resize(image_tmp, None, fx=3, fy=3)
        cv2.imshow("Path", image_tmp)
        return path_back

        # No check to see if we're at goal since we'll break when the queue size is 0

    @staticmethod
    def astar(image, graph_map, starting_node, target_node, node_width=1, node_height=1):
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

            for neighbor_node in MapHandler.get_neighbors(cur_node, image_tmp, graph_map):
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
            cur_node = visited[cur_node]

        if len(path_back) == 1:
            # Single element path
            print("Could not find a path.")
            return []

        print("Dummy")
        image_tmp = cv2.resize(image_tmp, None, fx=3, fy=3)
        cv2.imshow("Path", image_tmp)
        return path_back

    @staticmethod
    def get_neighbors(node, img, graph):
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
            json.dump(output_dict, f)

    @staticmethod
    def graph_from_json(filepath):
        graph = {}
        with open(filepath, "r") as f:
            data_dict = json.load(f)

        for coords, node_data in data_dict:
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
    graph = MapHandler.create_graph(img_cropped, 2)
    cv2.imshow("cropped", img_cropped)
    print(graph)
    path = MapHandler.bfs(img_painted, graph, Node(55, 13), Node(7, 4))
    # img_painted = cv2.resize(img_painted, None, fx=3, fy=3)

    print("Path:", path)
    # print(path)

    cv2.imshow("afafa", img_painted)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
