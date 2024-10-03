class NavigateContent:
    def __init__(self,
                 heading: float,
                 x: int,
                 y: int,
                 waypoint: str,
                 waypoint_id: str):
        self.heading = heading
        self.x = x
        self.y = y
        self.name = waypoint
        self.id = waypoint_id
