from collections.abc import Mapping
from typing import Any, NamedTuple

import yaml

from lbrmf import errors
from lbrmf.frame import LevelTransform, TransformValues


class _Key:
    RMF_FLEET = "rmf_fleet"
    NAME = "name"
    MAP_TRANSFORM = "map_transform"
    TRANSFORM_VALUES = "transform_values"
    LEVEL_TRANSFORM = "level_transform"
    ROBOTS = "robots"
    RMF_CONFIG = "rmf_config"
    ROBOT_CONFIG = "robot_config"
    START = "start"
    RMF_LEVEL = "rmf_level"
    ROBOT_MAP_NAME = "robot_map_name"
    ROBOT_MAP_LEVEL = "robot_map_level"
    UUID = "uuid"
    ROBOT_MAPS = "robot_maps"
    ROBOT_MAP_NAMES = "robot_map_names"
    ROBOT_MAP_LEVELS = "robot_map_levels"
    MAP_NAME = "name"
    MAP_LEVEL = "level"


class RobotMap(NamedTuple):
    robot: str
    robot_id: str
    map_name: str
    map_level: str


class FleetConfig:
    def __init__(self, data: Mapping[str, Any]):
        self._data = data

    @property
    def fleet_name(self) -> str:
        return self._data[_Key.RMF_FLEET][_Key.NAME]

    def level_transform(self, level: str) -> tuple[TransformValues, LevelTransform]:
        transforms = self._data.get(_Key.MAP_TRANSFORM) or {}
        level_config = transforms.get(level) or {}
        if _Key.TRANSFORM_VALUES not in level_config:
            raise errors.LevelNotInConfigError(
                f"the config has no map_transform.{level}.transform_values"
            )
        return (
            TransformValues.from_config(level_config[_Key.TRANSFORM_VALUES]),
            LevelTransform.from_config(level_config.get(_Key.LEVEL_TRANSFORM)),
        )

    def robot_map(self, level: str) -> RobotMap | None:
        for name, robot in (self._data.get(_Key.ROBOTS) or {}).items():
            robot_id = ((robot or {}).get(_Key.ROBOT_CONFIG) or {}).get(_Key.UUID)
            if not robot_id:
                continue
            names, levels = _robot_maps((robot or {}).get(_Key.RMF_CONFIG) or {})
            if level in names:
                return RobotMap(
                    name, str(robot_id), str(names[level]), str(levels.get(level, level))
                )
        return None


def robot_map_names(robot: Mapping[str, Any]) -> dict[str, Any]:
    """The robot map name of each RMF level for one robot entry of the config."""
    names, _ = _robot_maps((robot or {}).get(_Key.RMF_CONFIG) or {})
    return names


# The same merge order as the adapter: legacy name and level maps, then robot_maps, then start.
def _robot_maps(rmf_config: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    names = dict(rmf_config.get(_Key.ROBOT_MAP_NAMES) or {})
    levels = dict(rmf_config.get(_Key.ROBOT_MAP_LEVELS) or {})
    for level_name, robot_map in (rmf_config.get(_Key.ROBOT_MAPS) or {}).items():
        if isinstance(robot_map, Mapping):
            if _Key.MAP_NAME in robot_map:
                names[level_name] = robot_map[_Key.MAP_NAME]
            if _Key.MAP_LEVEL in robot_map:
                levels[level_name] = robot_map[_Key.MAP_LEVEL]
        else:
            names[level_name] = robot_map
    start = rmf_config.get(_Key.START) or {}
    start_level = start.get(_Key.RMF_LEVEL)
    if start_level is not None:
        names.setdefault(start_level, start.get(_Key.ROBOT_MAP_NAME, start_level))
        levels.setdefault(start_level, start.get(_Key.ROBOT_MAP_LEVEL, start_level))
    return names, levels


def parse_fleet_config(text: str) -> FleetConfig:
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as err:
        raise errors.MalformedConfigError(f"not valid YAML: {err}") from err
    if not isinstance(data, dict):
        raise errors.MalformedConfigError("the fleet config is not a YAML mapping")
    fleet = data.get(_Key.RMF_FLEET)
    if not isinstance(fleet, dict) or not isinstance(fleet.get(_Key.NAME), str):
        raise errors.MalformedConfigError("the fleet config has no rmf_fleet.name")
    if not isinstance(data.get(_Key.MAP_TRANSFORM) or {}, dict):
        raise errors.MalformedConfigError("map_transform is not a mapping of levels")
    return FleetConfig(data)
