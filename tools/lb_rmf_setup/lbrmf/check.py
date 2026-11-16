"""Find mistakes in a building file before the nav graph export.

Traffic editor makes a new vertex when a lane click lands next to a vertex instead of on
it. The lane then ends on the new vertex, and the named point stays out of the nav graph.
The robot cannot reach it, and nothing says why. These checks name each such problem.
"""

import math
from collections import Counter
from typing import Any

import yaml

from lbrmf import errors
from lbrmf.building import BuildingFile, Vertex
from lbrmf.charger import PLACEHOLDER
from lbrmf.constants import VertexParam
from lbrmf.frame import floorplan_scale

# An unnamed vertex this close to a named one is almost always a missed click.
NEAR_METERS = 0.5
# Used when the level has no measurement to give the scale.
NEAR_PIXELS = 10.0


def _label(vertex: Vertex) -> str:
    return vertex.name or f"an unnamed vertex at ({vertex.x:.0f}, {vertex.y:.0f})"


def _near_pixels(building: BuildingFile, level: str) -> tuple[float, float | None]:
    try:
        scale = floorplan_scale(building.measurements(level))
    except errors.LocalInputError:
        return NEAR_PIXELS, None
    return NEAR_METERS / scale, scale


def check_level(building: BuildingFile, level: str) -> list[str]:
    vertices = building.vertices(level)
    lanes: list[Any] = building.lanes(level)
    degree = Counter(index for lane in lanes for index in lane[:2])
    drawn = building.drawn_vertices(level)
    near_pixels, scale = _near_pixels(building, level)
    named = [vertex for vertex in vertices if vertex.name]
    issues = []
    for index, vertex in enumerate(vertices):
        if vertex.name:
            if degree[index] == 0 and index not in drawn:
                issues.append(
                    f"{level}: {vertex.name} has no lane, so it is not in the nav graph. "
                    "Connect it with a lane."
                )
            continue
        if degree[index] == 0 and index in drawn:
            continue
        closest = min(
            named,
            key=lambda other: math.dist((vertex.x, vertex.y), (other.x, other.y)),
            default=None,
        )
        pixels = (
            None if closest is None else math.dist((vertex.x, vertex.y), (closest.x, closest.y))
        )
        if pixels is not None and pixels <= near_pixels:
            distance = f"{pixels * scale:.2f} m" if scale else f"{pixels:.0f} px"
            issues.append(
                f"{level}: {_label(vertex)} is {distance} from {closest.name}. "
                f"A lane that ends on it does not reach {closest.name}. "
                f"Delete it and draw the lane to {closest.name}."
            )
        elif degree[index] == 0:
            issues.append(f"{level}: {_label(vertex)} has no lane. Delete it.")
    return issues


def check_names(building: BuildingFile) -> list[str]:
    levels: dict[str, list[str]] = {}
    for level in building.level_names:
        for vertex in building.vertices(level):
            if vertex.name:
                levels.setdefault(vertex.name, []).append(level)
    return [
        f"{name} is on more than one level ({', '.join(on)}). RMF refuses a nav graph that "
        "has the same waypoint name twice. Rename it on the robot map of one level."
        for name, on in levels.items()
        if len(on) > 1
    ]


def _chargers(building: BuildingFile, levels: list[str]) -> set[str]:
    return {
        vertex.name
        for level in levels
        for vertex in building.vertices(level)
        if vertex.name and VertexParam.IS_CHARGER.value in vertex.params
    }


def check_config(building: BuildingFile, config_text: str) -> list[str]:
    data = yaml.safe_load(config_text) or {}
    chargers = _chargers(building, building.level_names)
    issues = []
    for name, robot in (data.get("robots") or {}).items():
        charger = ((robot or {}).get("rmf_config") or {}).get("charger") or {}
        waypoint = charger.get("waypoint") if isinstance(charger, dict) else None
        setting = f"robots.{name}.rmf_config.charger.waypoint"
        if not isinstance(waypoint, str) or not waypoint or PLACEHOLDER.fullmatch(waypoint):
            issues.append(f"config: {setting} is not set. Run export to fill it in.")
        elif waypoint not in chargers:
            issues.append(
                f"config: {setting} is {waypoint}, which is not a dock in the building file."
            )
    return issues
