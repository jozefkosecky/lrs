
class Mission:
    def __init__(self, x: float, y: float, z: float, precision_hard: bool, task: str):
        self.x = x
        self.y = y
        self.z = z
        self.precision_hard = precision_hard
        self.task = task