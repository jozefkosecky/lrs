from pprint import pprint
from collections import deque
import copy
import re
import numpy as np
from PIL import Image
import io
from scipy.spatial import KDTree
import math
from skimage.draw import line_nd

class RRT:

    def __init__(self):
        pass

    def get_trajectory(self, start_pos, end_pos, pgm_map):
        metadata, pixel_data = self._parse_pgm(pgm_map)

        filtered_data = [sublist for sublist in pixel_data if sublist]

        expanded_data = self._expand_obstacles(filtered_data, radius=5)

        self.write_pgm(expanded_data, 'map.pgm')

        tree, last_point = self.rrt(expanded_data, start_pos, end_pos)
        path = self.find_path(tree, start_pos, last_point)
        # reverse_path = path.reverse()
        simple_path = self.simplify_path(path, start_pos, last_point, expanded_data)

        return simple_path

    def simplify_path(self, path, start_pos, last_point, map):
        map = np.array(map)
        new_path = [start_pos]

        step = 0;
        index_of_start_point = 0
        while new_path[-1] != last_point:
            current_point = new_path[-1]

            for step in range(index_of_start_point, len(path)):
                next_point = path[step]
                if self.is_obstacle_in_path(np.array(current_point), np.array(next_point), map):
                    # Go one step backwards
                    next_point = path[step-1]
                    new_path.append(next_point)
                    index_of_start_point = step-1
                    break
            if step == len(path) - 1:
                next_point = path[step]
                new_path.append(next_point)
                break
        
        return new_path



        

    def find_path(self, tree, start_pos, last_point):
        path = [last_point]


        current_point = last_point
        while current_point != start_pos:
            if current_point in tree:
                next_point = tree[current_point]
                path.insert(0,next_point)
                current_point = next_point
        return path
                
        # path = [last_point]
        # tree = [tuple(row) for row in tree]
        # while path[-1] != start_pos:
        #     point = path[-1]
        #     # nearest_point = tuple(np.asarray(self.closest_point(point, tree)))
        #     nearest_point = min(tree, key=lambda x: np.linalg.norm(np.array(x) - np.array(point)))
        #     path.append(nearest_point)
        #     plt.plot([point[1], nearest_point[1]], [point[0], nearest_point[0]], 'r-', linewidth=2)


    def write_pgm(self, pixel_data, filename, max_value=255):
        # Ensure max_value is valid
        max_value = min(max(max_value, 0), 255)

        # Determine the dimensions of the image
        height = len(pixel_data)
        width = len(pixel_data[0]) if height > 0 else 0

        # Write header and pixel data to file
        with open(filename, 'w') as f:
            f.write(f"P2\n{width} {height}\n{max_value}\n")
            for row in pixel_data:
                f.write(' '.join(map(str, row)) + '\n')

    def rrt(self, map, start_pos, end_pos, num_nodes=1000, delta=10):
        tree = [start_pos]
        dictionary_tree = {}
        map = np.array(map)
        rows, cols =  np.shape(map)
        x = 0
        while True:
            x += 1
            random_point = np.asarray((np.random.randint(0, cols), np.random.randint(0, rows)))
            nearest_point = np.asarray(self.closest_point(random_point, tree))

            # https://stackoverflow.com/questions/17332759/finding-vectors-with-2-points
            distance = random_point - nearest_point
            norm = math.sqrt(distance[0] ** 2 + distance[1] ** 2)
            if norm == 0:
                direction = np.array([0, 0])
            else:
                direction = np.array([distance[0] / norm, distance[1] / norm])
            new_point = nearest_point + direction * delta
            new_point = np.round(new_point).astype(int)

            if not self.is_obstacle_in_path(nearest_point,new_point, map):
                tree.append(new_point)

                if not tuple(new_point) in dictionary_tree:
                    dictionary_tree[tuple(new_point)] = tuple(nearest_point)

                if self.is_point_in_end_point(new_point, end_pos, delta):

                    dictionary_tree[tuple(end_pos)] = tuple(new_point)
                    return dictionary_tree, end_pos



    def is_point_in_end_point(self, new_point, end_pos, threshold):
        current_x, current_y = new_point[0], new_point[1]
        target_x, target_y = end_pos[0], end_pos[1]

        distance = self._euclidean_distance(current_x, current_y, target_x, target_y)

        return distance <= threshold

    def _euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def is_obstacle_in_path(self,start_point, end_point, map):
        # rr, cc = line_nd(start_point, end_point, endpoint=False)
        # obstacle_on_line = np.any(map[rr, cc] == 0)
        # return obstacle_on_line
        rr, cc = line_nd(start_point, end_point, endpoint=False)
        for step in range(len(rr)):
            if map[rr[step], cc[step]] == 0:
                return True
            
        return False


        
    def closest_point(self, point, tree):
        kd_tree = KDTree(tree)
        distances, indexes = kd_tree.query(point, k=2)
        closest_index = indexes[0] if distances[0] != 0 else indexes[1]
        return tree[closest_index]
      

    def _expand_obstacles(self, data, radius=1):

        # Create a deep copy of the data so we don't modify the original
        expanded_data = [row.copy() for row in data]
        
        height = len(data)
        width = len(data[0]) if height > 0 else 0
        
        # Iterate over each cell in the grid
        for i in range(height):
            for j in range(width):
                if data[i][j] == 0:
                    # If the cell is an obstacle, mark its neighbors as obstacles in the expanded_data
                    for x in range(-radius, radius+1):
                        for y in range(-radius, radius+1):
                            # Make sure we don't go out of bounds
                            if 0 <= i + x < height and 0 <= j + y < width:
                                expanded_data[i + x][j + y] = 0
                                
        return expanded_data

    def _parse_pgm(self, data):
        lines = data.split("\n")
        metadata = {}
        pixel_data = []

        # Loop through lines and parse data
        for line in lines:
            # Skip comments
            if line.startswith("#"):
                continue
            # Check for magic number P2
            elif line == "P2":
                metadata["type"] = "P2"
            # Check for width and height
            elif "width" not in metadata:
                metadata["width"], metadata["height"] = map(int, line.split())
            # Check for max gray value
            elif "max_gray" not in metadata:
                metadata["max_gray"] = int(line)
            # Parse pixel data
            else:
                pixel_data.append(list(map(int, line.split())))
        return metadata, pixel_data