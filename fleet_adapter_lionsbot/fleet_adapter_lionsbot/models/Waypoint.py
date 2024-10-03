class Waypoint:
    def __init__(self,
                 theta: float,
                 x: int,
                 y: int,
                 waypoint_name: str):
        self.heading = theta
        self.x = x
        self.y = y
        self.name = waypoint_name
