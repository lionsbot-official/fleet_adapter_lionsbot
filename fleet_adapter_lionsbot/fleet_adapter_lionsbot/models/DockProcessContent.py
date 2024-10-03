class DockProcessContent:
    def __init__(self,
                 x: int,
                 y: int,
                 theta: float,
                 dock_name: str):
        self.x = x
        self.y = y
        self.heading = theta
        self.name = dock_name