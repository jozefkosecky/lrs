# Simple example of BFS algorithm adapted from https://github.com/shkolovy/path-finder-algorithms          
#!/usr/bin/env python
from pprint import pprint
from collections import deque
import copy
import numpy as np
from PIL import Image
import numpy as np


START_COL = "S"
END_COL = "E"
VISITED_COL = "x"
OBSTACLE_COL = "#"
PATH_COL = "@"

class BFS:
    def __init__(self):
        pass

    def get_trajectory(self, start_pos, end_pos, pgm_map):
        metadata, pixel_data = self._parse_pgm(pgm_map)

        pixel_data = self._replace_values_in_array(pixel_data)
        filtered_data = [sublist for sublist in pixel_data if sublist]

        expanded_data = self._expand_obstacles(filtered_data, radius=10) 

        directions = self._scan_grid(expanded_data, start_pos)
        path = self._find_path(start_pos, end_pos, directions)

        simplified_path = self._simplify_path(path)

        return simplified_path

    def _scan_grid(self, grid, start=(0, 0)):
        """Scan all grid, so we can find a path from 'start' to any point"""

        q = deque()
        q.append(start)
        came_from = {start: None}
        while len(q) > 0:
            current_pos = q.popleft()
            neighbors = self._get_neighbors(grid, current_pos[0], current_pos[1])
            for neighbor in neighbors:
                if neighbor not in came_from:
                    q.append(neighbor)
                    came_from[neighbor] = current_pos

        return came_from

    def _get_neighbors(self, grid, row, col):
        height = len(grid)
        width = len(grid[0])

        neighbors = [(row + 1, col), (row, col - 1), (row - 1, col), (row, col + 1)]

        # make path nicer
        if (row + col) % 2 == 0:
            neighbors.reverse()

        # check borders
        neighbors = filter(lambda t: (0 <= t[0] < height and 0 <= t[1] < width), neighbors)
        # check obstacles
        neighbors = filter(lambda t: (grid[t[0]][t[1]] != OBSTACLE_COL), neighbors)

        return neighbors

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

    def _replace_values_in_array(self, pixel_data):
        for i in range(len(pixel_data)):
            for j in range(len(pixel_data[i])):
                if pixel_data[i][j] == 255:
                    pixel_data[i][j] = '.'
                elif pixel_data[i][j] == 0:
                    pixel_data[i][j] = '#'
        return pixel_data

    def _write_2d_array_to_file(self, pixel_data, filename):
        max_width = max(len(str(item)) for row in pixel_data for item in row)  # Find the maximum width of the items
        with open(filename, 'w') as file:
            for row in pixel_data:
                # Create a formatted string with even spacing, write it to the file
                line = ''.join(f'{item:>{max_width+1}}' for item in row)
                file.write(line + '\n')

    def _write_pgm(self, pixel_data, filename, max_value=255):
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

    def _convert_to_numeric(self, pixel_data):
        """
        Convert a 2D array of '.' and '#' symbols to a 2D array of 0 and 255 values, respectively.

        :param pixel_data: 2D array containing '.' and '#' symbols.
        :return: A new 2D array with numerical values.
        """
        return [[255 if pixel == '.' else 0 for pixel in row] for row in pixel_data]

    def _find_path(self, start, end, came_from):
        """Find the shortest path from start to end point"""

        # check is kye exist
        if end in came_from:
            path = [end]
            current = end
        else:
            last_key = list(came_from.keys())[-1]
            last_value = came_from[last_key]

            path = [last_value]
            current = last_value

        while current != start:
            current = came_from[current]
            path.append(current)

        # reverse to have Start -> Target
        # just looks nicer
        path.reverse()

        return path

    def _draw_path(self, path, grid):
        for row, col in path:
            grid[row][col] = PATH_COL

        # draw start and end
        start_pos = path[0]
        end_pos = path[-1]
        grid[start_pos[0]][start_pos[1]] = START_COL
        grid[end_pos[0]][end_pos[1]] = END_COL

        return grid

    def _expand_obstacles(self, data, radius=1):
        """
        Expands obstacles in a given 2D grid by a specified radius.

        :param data: A 2D list containing '.' for free spaces and '#' for obstacles.
        :param radius: Number of cells by which to expand the obstacles.
        :return: A new 2D list with expanded obstacles.
        """
        # Create a deep copy of the data so we don't modify the original
        expanded_data = [row.copy() for row in data]
        
        height = len(data)
        width = len(data[0]) if height > 0 else 0
        
        # Iterate over each cell in the grid
        for i in range(height):
            for j in range(width):
                if data[i][j] == '#':
                    # If the cell is an obstacle, mark its neighbors as obstacles in the expanded_data
                    for x in range(-radius, radius+1):
                        for y in range(-radius, radius+1):
                            # Make sure we don't go out of bounds
                            if 0 <= i + x < height and 0 <= j + y < width:
                                expanded_data[i + x][j + y] = '#'
                                
        return expanded_data

    def _simplify_path(self, path):
        if not path or len(path) < 2:
            return path

        def direction(p1, p2):
            """Get direction from point p1 to p2."""
            return (p2[0] - p1[0], p2[1] - p1[1])

        simplified_path = [path[0]]
        prev_direction = direction(path[0], path[1])
        was_diagonal = False

        for i in range(1, len(path) - 2):
            current_direction = direction(path[i], path[i+1])
            future_direction = direction(path[i+1], path[i+2])

            is_diagonal = prev_direction == future_direction

            if not was_diagonal and is_diagonal:
                was_diagonal = True

            if was_diagonal and not is_diagonal:
                simplified_path.append(path[i])
                was_diagonal = False
            elif current_direction != prev_direction and not is_diagonal:
                simplified_path.append(path[i])

            prev_direction = current_direction

        simplified_path.append(path[-1])  # Always include the end point

        return simplified_path


