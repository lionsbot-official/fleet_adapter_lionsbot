from typing import List

from .EqualizerConfig import EqualizerConfig


class Zone:
    def __init__(self,
                 area: float,
                 configs: List[dict],
                 selected_mode_name: str,
                 zone_name: str,
                 zone_id: str):
        self.area = area
        self.configs = configs
        self.selectedModeName = selected_mode_name
        self.name = zone_name
        self.id = zone_id
