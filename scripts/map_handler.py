import cv2
import yaml
import numpy as np

from time import time

MAP_NAME = "double_zz_map"

with open("maps/{}.yaml".format(MAP_NAME)) as map_yaml:
    map_data = yaml.load(map_yaml)

img = cv2.imread("maps/{}.pgm".format(MAP_NAME), 0)
# cv2.imshow("asdf", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


class Node(object):

    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height


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
                # yuck
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
                        img_tmp[row][col] = 75

        return img_tmp

    @staticmethod
    def gen_nodes(node_width, node_height):
        pass


if __name__ == '__main__':

    time_init = time()
    img_cropped = MapHandler.crop_to_contents(img)
    img_painted = MapHandler.get_node_pixels(img_cropped, 2)
    print("Time: {}".format(time() - time_init))
    img_painted = cv2.resize(img_painted, None, fx=3, fy=3)

    cv2.imshow("afafa", img_painted)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
