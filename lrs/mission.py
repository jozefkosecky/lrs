

from enum import Enum

class Task(Enum):
    LANDTAKEOFF = 1


class Mission:
    def __init__(self, x: float, y: float, z: float, is_precision_hard: bool, task: Task):
        self.x = x
        self.y = y
        self.z = z
        self.is_precision_hard = is_precision_hard
        self.task = task


