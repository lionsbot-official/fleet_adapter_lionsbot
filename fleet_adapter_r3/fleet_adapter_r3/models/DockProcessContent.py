class DockProcessContent:
    def __init__(self,
                 x: int,
                 y: int,
                 orientation_radians: float,
                 dock_name: str):
        self.x = x
        self.y = y
        self.heading = orientation_radians
        self.name = dock_name
