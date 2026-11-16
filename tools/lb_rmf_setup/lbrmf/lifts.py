import math
from collections.abc import Mapping, Sequence
from typing import Any, NamedTuple

from lbrmf import errors
from lbrmf.building import BuildingFile
from lbrmf.constants import DOOR_POINT_SUFFIX
from lbrmf.frame import MapTransform, Point, wrap_angle
from lbrmf.geometry import Lift, MapGeometry, Marker
from lbrmf.plan import Plan, make_plan

# The adapter refuses to start when one lift's car waypoints differ more than this across levels.
CABIN_ALIGNMENT_METERS = 0.1
DOOR_NAME = "door_1"
DOOR_TYPE_DOUBLE_SLIDING = 2


class ExportKinds(NamedTuple):
    cabins: bool = False
    approach: bool = False
    lobbies: bool = False


NO_EXTRA_KINDS = ExportKinds()


class LevelInput(NamedTuple):
    level: str
    geometry: MapGeometry
    transform: MapTransform


class LiftsResult(NamedTuple):
    plans: dict[str, Plan]
    lifts: dict[str, dict[str, Any]] | None
    messages: list[str]


def approach_vertex(lift: str, level: str) -> str:
    return f"{lift}_{level}_approach"


def cabin_vertex(lift: str, level: str) -> str:
    return f"{lift}_{level}_cabin"


class _Placed(NamedTuple):
    lift: Lift
    cabin: Point
    door: Point | None
    source: LevelInput


def _place(inputs: Sequence[LevelInput]) -> dict[str, dict[str, _Placed]]:
    placed: dict[str, dict[str, _Placed]] = {}
    for item in inputs:
        markers = {marker.name: marker for marker in item.geometry.markers}
        for lift in item.geometry.lifts:
            cabin = markers.get(lift.name)
            if cabin is None:
                raise errors.MissingCabinPointError(
                    f"lift {lift.name} has no cabin point on {item.level}: on that robot map, "
                    f"add a cabin point named {lift.name} inside the car"
                )
            if lift.car_width <= 0:
                raise errors.MissingLiftSizeError(
                    f"lift {lift.name} has no car width on {item.level}"
                )
            door = markers.get(lift.name + DOOR_POINT_SUFFIX)
            placed.setdefault(lift.name, {})[item.level] = _Placed(
                lift, cabin.point, door.point if door else None, item
            )
    return placed


def _reference(levels: Mapping[str, _Placed], reference_level: str) -> _Placed:
    return levels.get(reference_level) or next(iter(levels.values()))


def _check_alignment(placed: Mapping[str, Mapping[str, _Placed]], reference_level: str) -> None:
    problems = []
    for name, levels in placed.items():
        reference = _reference(levels, reference_level)
        anchor = reference.source.transform.robot_to_rmf_meters(reference.cabin)
        for level, item in levels.items():
            gap = math.dist(anchor, item.source.transform.robot_to_rmf_meters(item.cabin))
            if gap > CABIN_ALIGNMENT_METERS:
                problems.append(f"{name}: {level} is {gap:.2f} m from {reference.source.level}")
    if problems:
        raise errors.MisalignedCabinError(
            f"lift car points do not line up within {CABIN_ALIGNMENT_METERS} m: "
            + "; ".join(problems)
        )


def _yaw(item: _Placed) -> float:
    transform = item.source.transform
    approach = transform.robot_to_rmf_meters(item.lift.approach)
    cabin = transform.robot_to_rmf_meters(item.cabin)
    return wrap_angle(math.atan2(cabin.y - approach.y, cabin.x - approach.x) - math.pi / 2)


def _depth(item: _Placed, depths: Mapping[str, float], messages: list[str]) -> float:
    name = item.lift.name
    if name in depths:
        return depths[name]
    if item.door is not None:
        transform = item.source.transform
        depth = 2 * math.dist(
            transform.robot_to_rmf_meters(item.cabin), transform.robot_to_rmf_meters(item.door)
        )
        messages.append(f"{name}: depth {depth:.2f} m from the door point")
        return depth
    messages.append(
        f"{name}: no door point {name}{DOOR_POINT_SUFFIX}; depth set to the car width "
        f"{item.lift.car_width:.2f} m"
    )
    return item.lift.car_width


def _entry(
    building: BuildingFile,
    levels: Mapping[str, _Placed],
    reference: _Placed,
    depth: float,
) -> dict[str, Any]:
    position = reference.source.transform.robot_to_floorplan_pixels(reference.cabin)
    by_elevation = sorted(levels, key=building.elevation)
    return {
        "depth": depth,
        "doors": {
            DOOR_NAME: {
                "door_type": DOOR_TYPE_DOUBLE_SLIDING,
                "motion_axis_orientation": 0.0,
                "width": reference.lift.door_width,
                "x": 0.0,
                "y": -depth / 2,
            }
        },
        "highest_floor": by_elevation[-1],
        "initial_floor_name": reference.source.level,
        "level_doors": {level: [DOOR_NAME] for level in levels},
        "lowest_floor": by_elevation[0],
        "plugins": False,
        "reference_floor_name": reference.source.level,
        "width": reference.lift.car_width,
        "x": position.x,
        "y": position.y,
        "yaw": _yaw(reference),
    }


def _check_cars_are_empty(
    building: BuildingFile, placed: Mapping[str, Mapping[str, _Placed]], entries: Mapping[str, Any]
) -> None:
    for name, levels in placed.items():
        entry = entries[name]
        yaw = entry["yaw"]
        for level, item in levels.items():
            scale = item.source.transform.floorplan_scale
            centre = item.source.transform.robot_to_floorplan_pixels(item.cabin)
            own = {cabin_vertex(name, level), approach_vertex(name, level)}
            for vertex in building.vertices(level):
                if vertex.name in own:
                    continue
                dx = (vertex.x - centre.x) * scale
                dy = -(vertex.y - centre.y) * scale
                across = dx * math.cos(yaw) + dy * math.sin(yaw)
                along = -dx * math.sin(yaw) + dy * math.cos(yaw)
                if abs(across) <= entry["width"] / 2 and abs(along) <= entry["depth"] / 2:
                    if vertex.name in {name, name + DOOR_POINT_SUFFIX}:
                        raise errors.ForeignVertexInCabinError(
                            f"vertex {vertex.name} on {level} is the lift's own cabin point from "
                            "an older export; delete it from the building file and run again"
                        )
                    label = vertex.name or f"at ({vertex.x}, {vertex.y})"
                    raise errors.ForeignVertexInCabinError(
                        f"vertex {label} on {level} is inside the car of {name}; "
                        "RMF would treat it as a lift waypoint"
                    )


def lobby_vertex(lobby: str, level: str) -> str:
    return f"{lobby}_{level}"


def _level_markers(
    item: LevelInput, placed: Mapping[str, Mapping[str, _Placed]], kinds: ExportKinds
) -> list[Marker]:
    markers = []
    for lift in item.geometry.lifts:
        if kinds.approach:
            markers.append(
                Marker(approach_vertex(lift.name, item.level), lift.approach, 0.0, False)
            )
        if kinds.cabins:
            cabin = placed[lift.name][item.level].cabin
            markers.append(Marker(cabin_vertex(lift.name, item.level), cabin, 0.0, False))
    if kinds.lobbies:
        markers += [
            Marker(lobby_vertex(lobby.name, item.level), lobby.point, 0.0, False)
            for lobby in item.geometry.lobbies
        ]
    return markers


def _links(
    building: BuildingFile, item: LevelInput, plan: Plan, kinds: ExportKinds, graph_idx: int
) -> list[list[Any]]:
    vertices = building.vertices(item.level)
    index = {vertex.name: i for i, vertex in enumerate(vertices) if vertex.name}
    index.update({vertex.name: len(vertices) + i for i, vertex in enumerate(plan.added)})
    existing_pairs = {frozenset(lane[:2]) for lane in building.lanes(item.level)}
    lanes = []
    if kinds.approach and kinds.cabins:
        for lift in item.geometry.lifts:
            pair = [
                index[approach_vertex(lift.name, item.level)],
                index[cabin_vertex(lift.name, item.level)],
            ]
            if frozenset(pair) not in existing_pairs:
                lanes.append([*pair, {"bidirectional": [4, True], "graph_idx": [2, graph_idx]}])
    return lanes


def plan_lifts(
    building: BuildingFile,
    inputs: Sequence[LevelInput],
    *,
    reference_level: str,
    depths: Mapping[str, float],
    update_positions: bool,
    kinds: ExportKinds = NO_EXTRA_KINDS,
    graph_idx: int = 0,
) -> LiftsResult:
    placed = _place(inputs)
    _check_alignment(placed, reference_level)
    messages: list[str] = []
    entries = {}
    for name, levels in placed.items():
        reference = _reference(levels, reference_level)
        entries[name] = _entry(building, levels, reference, _depth(reference, depths, messages))
    _check_cars_are_empty(building, placed, entries)

    plans = {}
    for item in inputs:
        markers = _level_markers(item, placed, kinds)
        pseudo = MapGeometry(item.geometry.map_name, item.geometry.frame, tuple(markers), ())
        plan = make_plan(
            building, item.level, pseudo, item.transform, update_positions=update_positions
        )
        plan.lanes = _links(building, item, plan, kinds, graph_idx)
        # The plan sees only the lift points, so every other point would look missing.
        plan.not_on_map = []
        plans[item.level] = plan

    existing = building.lifts()
    merged = dict(existing)
    changed = False
    for name, entry in entries.items():
        if existing.get(name) == entry:
            continue
        if name in existing and not update_positions:
            messages.append(f"{name}: kept the lifts entry; --update-positions replaces it")
            continue
        merged[name] = entry
        changed = True
    messages += [
        f"{name} is in the building file but on no robot map (kept)"
        for name in existing
        if name not in entries
    ]
    return LiftsResult(plans, merged if changed else None, messages)
