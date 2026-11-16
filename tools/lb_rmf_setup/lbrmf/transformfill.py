"""Fill in the map transform of a level in the fleet config.

The map transform lines up the robot map with the RMF building map. Traffic
editor already holds it: when the robot map image is a layer of the level, the
layer transform (scale, translation and yaw) is the map transform. When there is
no layer, the robot map is the floor plan itself, and only the scale is needed.
The export fills it in while the config still has the template value, so nobody
copies the numbers by hand.
"""

import itertools
import math
from pathlib import PurePosixPath
from typing import Any, NamedTuple

import yaml

from lbrmf import errors, yamledit
from lbrmf.building import BuildingFile
from lbrmf.charger import PLACEHOLDER
from lbrmf.frame import TRANSFORM_VALUE_KEYS, floorplan_scale

# The adapter moves a level but cannot turn it. RMF turns it about the middle of the
# fiducials, so 0.5 degrees is 0.09 m at 10 m from there, under the 0.1 m of the lift check.
# The scale written is the one of RMF. 1 % off the measurements moves a vertex 0.1 m on the
# floor plan at 10 m from the image corner. A fiducial clicked 1 pixel off, 200 pixels from
# the next one, stays inside both limits.
MAX_LEVEL_ROTATION_DEGREES = 0.5
MAX_LEVEL_SCALE_ERROR = 0.01


class TransformResult(NamedTuple):
    text: str
    source: str | None
    notes: list[str]


def _number(value: float) -> str:
    return f"{round(value, 6):g}"


def _shows(filename: Any, map_name: str, robot_type: str | None) -> bool:
    """True when the image file is the robot map, in no folder or the folder of the robot type."""
    path = PurePosixPath(str(filename or ""))
    folder = path.parent.name
    return path.stem == map_name and (not folder or folder == robot_type)


def _measured(building: BuildingFile, level: str) -> tuple[dict[str, float] | None, str | None]:
    try:
        scale = floorplan_scale(building.measurements(level))
    except errors.LocalInputError as err:
        return None, f"{level} has no measurement for the scale: {err}"
    return {"rotation_degrees": 0.0, "scale": scale, "tx_meters": 0.0, "ty_meters": 0.0}, None


def _values(
    building: BuildingFile, level: str, map_name: str, robot_type: str | None
) -> tuple[dict[str, float] | None, str | None, str | None]:
    """The transform values, where they came from, or a note on why there are none."""
    if _shows(building.drawing(level), map_name, robot_type):
        values, note = _measured(building, level)
        return values, f"the measurements of {level}, whose floor plan is the robot map", note
    layers = building.layers(level)
    matches = [
        name
        for name, layer in layers.items()
        if isinstance(layer, dict) and _shows(layer.get("filename"), map_name, robot_type)
    ]
    if len(matches) > 1:
        return None, None, f"{level} has more than one layer of the robot map {map_name}"
    if not matches:
        return (
            None,
            None,
            f"no image of {level} is the robot map {map_name}. The floor plan drawing of "
            f"{level} must be the file {map_name}.png (file name only, next to the building "
            f"file), or add {map_name}.png as a layer in traffic editor and line it up. "
            "Then run export again",
        )
    transform: dict[str, Any] = layers[matches[0]].get("transform") or {}
    try:
        values = {
            "rotation_degrees": math.degrees(float(transform["yaw"])),
            "scale": float(transform["scale"]),
            "tx_meters": float(transform["translation_x"]),
            "ty_meters": float(transform["translation_y"]),
        }
    except (KeyError, TypeError, ValueError):
        return None, None, f"the layer {matches[0]} of {level} has no complete transform"
    return values, f"the traffic editor layer {matches[0]}", None


def fill_transform(
    text: str,
    building: BuildingFile,
    level: str,
    map_name: str | None,
    robot_type: str | None = None,
) -> TransformResult:
    """Write the transform values of the level when the config still has the template value."""
    root = yaml.compose(text)
    values_node = yamledit.find(root, "map_transform", level, "transform_values")
    scale_node = yamledit.child(values_node, "scale")
    if not isinstance(scale_node, yaml.ScalarNode) or not PLACEHOLDER.fullmatch(scale_node.value):
        return TransformResult(text, None, [])
    if map_name is None:
        return TransformResult(text, None, [f"no robot in the config has a map on {level}"])
    values, source, note = _values(building, level, map_name, robot_type)
    if values is None:
        return TransformResult(text, None, [f"map_transform.{level} is not set: {note}"])
    edits = []
    for key in TRANSFORM_VALUE_KEYS:
        node = yamledit.child(values_node, key)
        if not isinstance(node, yaml.ScalarNode):
            return TransformResult(
                text, None, [f"map_transform.{level}.transform_values has no {key}"]
            )
        edits.append((node, _number(values[key])))
    return TransformResult(yamledit.replace(text, edits), source, [])


def fill_level_transform(text: str, building: BuildingFile, level: str) -> tuple[str, str | None]:
    """Write the level transform of a level that is not the reference level, when it has none.

    The numbers are the ones RMF takes from the fiducials (building_map/transform.py), less
    the rotation. Returns the new text and where the numbers came from.
    """
    reference = building.reference_level
    level_node = yamledit.find(yaml.compose(text), "map_transform", level)
    keys = {key.value: key for key, _ in getattr(level_node, "value", [])}
    if level == reference or "transform_values" not in keys or "level_transform" in keys:
        return text, None
    pairs = [
        (ours, theirs)
        for name, ours in building.fiducials(reference)
        for other, theirs in building.fiducials(level)
        if name == other
    ]
    measurements = building.measurements(reference)
    if len(pairs) < 2 or not measurements:
        raise errors.LevelFiducialError(
            f"RMF cannot place {level} against the reference level {reference}: they share "
            f"{len(pairs)} fiducial names and {reference} has {len(measurements)} measurements. "
            "In traffic editor add 2 fiducials with the same names on both levels, each name "
            f"on the same real place, and a measurement on {reference}. Then run export again"
        )
    spans = list(itertools.combinations(pairs, 2))
    turn_sin = turn_cos = ratio = 0.0
    for (ref_a, a), (ref_b, b) in spans:
        if ref_a == ref_b or a == b:
            raise errors.LevelFiducialError(
                f"two fiducials of {reference} or {level} are at the same place. Move them apart"
            )
        turn = math.atan2(b.y - a.y, b.x - a.x) - math.atan2(ref_b.y - ref_a.y, ref_b.x - ref_a.x)
        turn_sin += math.sin(turn)
        turn_cos += math.cos(turn)
        ratio += math.dist(a, b) / math.dist(ref_a, ref_b)
    ratio /= len(spans)
    rotation = math.degrees(math.atan2(turn_sin, turn_cos))
    if abs(rotation) > MAX_LEVEL_ROTATION_DEGREES:
        raise errors.LevelFiducialError(
            f"the fiducials turn the floor plan of {level} by {rotation:.2f} degrees against "
            f"{reference}. A rotated floor plan is not supported, the limit is "
            f"{MAX_LEVEL_ROTATION_DEGREES} degrees. Put each fiducial on the same real place "
            f"on both levels, or turn the image of {level} to match {reference}"
        )
    scale = floorplan_scale(measurements) / ratio
    own = building.measurements(level)
    if own and abs(floorplan_scale(own) / scale - 1) > MAX_LEVEL_SCALE_ERROR:
        raise errors.LevelFiducialError(
            f"the fiducials give {level} {scale:.5f} metres per pixel but its measurements "
            f"give {floorplan_scale(own):.5f}. They must agree within "
            f"{MAX_LEVEL_SCALE_ERROR:.0%}. Fix the fiducials or the measurements of {level} "
            f"and {reference} in traffic editor"
        )
    tx, ty = (
        sum(theirs[axis] - ours[axis] * ratio for ours, theirs in pairs) / len(pairs)
        for axis in (0, 1)
    )
    key = keys["transform_values"].start_mark
    line = f"level_transform: {{scale: {scale:.10g}, tx_pixels: {tx:.10g}, ty_pixels: {ty:.10g}}}"
    line += ", " if level_node.flow_style else "\n" + " " * key.column
    source = f"{len(pairs)} fiducials shared with the reference level {reference}"
    return text[: key.index] + line + text[key.index :], source
