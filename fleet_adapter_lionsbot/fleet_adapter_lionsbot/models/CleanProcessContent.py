from typing import List
from .Zone import Zone
from .Waypoint import Waypoint


class CleanProcessContent:
    def __init__(self,
                 working_type: str,
                 section_id: str,
                 section_name: str,
                 robot_type: str,
                 map_id: str,
                 map_name: str,
                 map_level: str,
                 zones: List[dict],
                 operator: str):
        self.working_type = working_type
        self.section_id = section_id
        self.section_name = section_name
        self.robot_type = robot_type
        self.map_id = map_id
        self.map_name = map_name
        self.map_level = map_level
        self.zones = zones
        self.operator = operator
