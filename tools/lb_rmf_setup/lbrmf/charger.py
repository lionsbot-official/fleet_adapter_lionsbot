"""Fill in the charger waypoint of each robot in the fleet config.

The config template has `charger: waypoint: <charger_waypoint>`. The export knows the docks
of the robot map, so it writes the dock name there. It changes only that value, so the
comments and the layout of the file stay the same.
"""

import re
from collections.abc import Mapping
from typing import Any, NamedTuple

import yaml

from lbrmf import yamledit
from lbrmf.fleetconfig import robot_map_names
from lbrmf.geometry import MapGeometry

PLACEHOLDER = re.compile(r"<[^<>]*>")
WAYPOINT_PATH = ("rmf_config", "charger", "waypoint")


class ChargerResult(NamedTuple):
    text: str
    filled: dict[str, str]
    notes: list[str]


def _needs_charger(robot: Mapping[str, Any]) -> bool:
    charger = (robot.get("rmf_config") or {}).get("charger")
    value = charger.get("waypoint") if isinstance(charger, Mapping) else None
    return not isinstance(value, str) or not value or PLACEHOLDER.fullmatch(value) is not None


def _dock_for(geometry: MapGeometry) -> str | None:
    docks = [marker.name for marker in geometry.markers if marker.dock]
    if geometry.home_dock in docks:
        return geometry.home_dock
    return docks[0] if len(docks) == 1 else None


def fill_chargers(
    text: str, level: str, geometry: MapGeometry, *, any_map: bool = False
) -> ChargerResult:
    """Write the dock name for each robot that starts on this level on this map and has none.

    any_map: the map came from a local file for this level, so its name does not matter.
    """
    data = yaml.safe_load(text) or {}
    root = yaml.compose(text)
    robots_node = yamledit.child(root, "robots")
    docks = [marker.name for marker in geometry.markers if marker.dock]
    edits = []
    filled = {}
    notes = []
    for name, robot in (data.get("robots") or {}).items():
        robot = robot or {}
        start = (robot.get("rmf_config") or {}).get("start") or {}
        if start.get("rmf_level") != level:
            continue
        if not any_map and robot_map_names(robot).get(level) != geometry.map_name:
            continue
        if not _needs_charger(robot):
            continue
        setting = f"robots.{name}.rmf_config.charger.waypoint"
        dock = _dock_for(geometry)
        if dock is None:
            if docks:
                notes.append(f"set {setting} to one of: {', '.join(docks)}")
            else:
                notes.append(f"the robot map {geometry.map_name} has no dock for {setting}")
            continue
        node = yamledit.find(robots_node, name, *WAYPOINT_PATH)
        if not isinstance(node, yaml.ScalarNode):
            notes.append(f"add {setting}: {dock} to the config")
            continue
        edits.append((node, dock))
        filled[name] = dock
    return ChargerResult(yamledit.replace(text, edits), filled, notes)
