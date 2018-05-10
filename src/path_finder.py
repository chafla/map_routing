from Queue import PriorityQueue
import numpy as np

VALID_PATH_COLOR = 75

class PathFinder(object):

    def __init__(self, map_image):
        self.img = map_image
        self._map_graph = {}
        self.bfs_queue = []
        self.priority_queue = PriorityQueue()


        self.generate_graph()

    def generate_graph(self):
        img = np.zeros(self.img.shape)

    @staticmethod
    def crop_image(img):
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
                # yuck
                if img[i][j] == 0:  # if it's a black pixel
                    return False
        return True

    def get_node_pixels(self, img, wall_dist):
        """Get the pixels which are a given distance away from any other pixels"""
        img_tmp = np.copy(img)
        w, h = img_tmp.shape
        for row in range(h):
            for col in range(w):
                # watch for out of range
                # The -1 and +1 in _check_surrounding_px are because otherwise it stops short
                if wall_dist < col < w - wall_dist - 1 and wall_dist < row < h - wall_dist - 1:
                    if self._check_surrounding_px(img_tmp, wall_dist, row, col):
                        img_tmp[row][col] = VALID_PATH_COLOR

        return img_tmp

    def explore(self, map_image):
        visited = {}
        queue = []

        pass

    def djikstra(self, start, target):
        pass


