

from enum import Enum

class Task(Enum):
    TAKEOFF = 1
    LAND = 2
    YAW180 = 3
    YAW90 = 4
    YAW270 = 5
    YAW0 = 6
    


from typing import List

class Mission:
    def __init__(self, x: float, y: float, z: float, is_precision_hard: bool, tasks: List[Task]):
        self.x = x
        self.y = y
        self.z = z
        self.is_precision_hard = is_precision_hard
        self.tasks = tasks  # Note the change to 'tasks' to indicate multiple tasks



