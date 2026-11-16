import difflib
import math
from collections import Counter
from dataclasses import dataclass, field
from typing import Any, NamedTuple

from lbrmf import errors
from lbrmf.building import BuildingFile, LevelEdit, Param, ParamType, Vertex
from lbrmf.constants import APPROACH_POINT_SUFFIX, VertexParam
from lbrmf.frame import MapTransform, Point, wrap_angle
from lbrmf.geometry import MapGeometry, Marker

# Positions closer than this are the same vertex; it hides float noise, not real moves.
SAME_POSITION_PIXELS = 1e-6
# Smaller moves than this are rounding, not a hand-placed change, so the summary leaves them out.
REPORT_MOVE_PIXELS = 0.5

TOOL_PARAMS = frozenset(param.value for param in VertexParam)

# A robot map keeps a named point in front of each dock, facing the dock. The robot drives
# from it onto the dock. The dock marker's angle points out of the dock, toward that point.
APPROACH_MIN_METERS = 0.3
APPROACH_MAX_METERS = 1.5
APPROACH_TOLERANCE_RADIANS = math.radians(15.0)
# Where the script places an approach point for a dock that has none.
NEW_APPROACH_METERS = 0.7


class VertexChange(NamedTuple):
    index: int
    before: Vertex
    after: Vertex


class KeptPosition(NamedTuple):
    name: str
    pixels: float
    meters: float


class DockApproach(NamedTuple):
    dock: str
    approach: str
    created: bool
    # The heading in RMF radians at which the robot arrives at the dock, facing into it.
    arrival_heading: float


class CleaningPath(NamedTuple):
    name: str
    level_name: str
    finish_waypoint: str
    robot_zone_name: str
    path: list[tuple[float, float, float]]


@dataclass
class Plan:
    level: str
    added: list[Vertex] = field(default_factory=list)
    changed: list[VertexChange] = field(default_factory=list)
    unchanged: list[str] = field(default_factory=list)
    kept_positions: list[KeptPosition] = field(default_factory=list)
    not_on_map: list[str] = field(default_factory=list)
    dock_headings: dict[str, float] = field(default_factory=dict)
    dock_approaches: list[DockApproach] = field(default_factory=list)
    cleaning_paths: list[CleaningPath] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    lanes: list[list[Any]] = field(default_factory=list)
    skipped_lift_points: list[str] = field(default_factory=list)

    @property
    def has_changes(self) -> bool:
        return bool(self.added or self.changed or self.lanes)

    def level_edit(self) -> LevelEdit:
        return LevelEdit(
            replaced={change.index: change.after for change in self.changed},
            appended=list(self.added),
            lanes=list(self.lanes),
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "new": [vertex.name for vertex in self.added],
            "changed": [change.after.name for change in self.changed],
            "unchanged": list(self.unchanged),
            "kept_positions": [kept._asdict() for kept in self.kept_positions],
            "dock_headings_degrees": {
                name: math.degrees(heading) for name, heading in self.dock_headings.items()
            },
            "dock_approaches": [
                {
                    "dock": approach.dock,
                    "approach": approach.approach,
                    "created": approach.created,
                    "arrival_heading_degrees": math.degrees(approach.arrival_heading),
                }
                for approach in self.dock_approaches
            ],
            "not_on_map": list(self.not_on_map),
            "new_lanes": len(self.lanes),
            "skipped_lift_points": list(self.skipped_lift_points),
            "warnings": list(self.warnings),
        }

    def summary(self) -> list[str]:
        lines = [
            f"level {self.level}: {len(self.added)} new, {len(self.changed)} changed, "
            f"{len(self.unchanged)} unchanged"
        ]
        lines += [f"  new: {vertex.name}" for vertex in self.added]
        lines += [f"  changed: {change.after.name}" for change in self.changed]
        lines += [
            f"  kept position: {kept.name} would move {kept.pixels:.1f} px ({kept.meters:.2f} m)"
            " with --update-positions"
            for kept in self.kept_positions
        ]
        lines += [
            f"  dock: {a.dock} is reached from {a.approach}"
            + (f", a new point {NEW_APPROACH_METERS:.1f} m in front of it" if a.created else "")
            + f". The robot arrives at {math.degrees(a.arrival_heading):.1f} degrees in RMF."
            f" Connect {a.approach} to your lanes"
            for a in self.dock_approaches
        ]
        if self.skipped_lift_points:
            lines.append(
                "  skipped lift points (lifts export places them): "
                + ", ".join(self.skipped_lift_points)
            )
        if self.not_on_map:
            lines.append(f"  not on this robot map (kept): {', '.join(self.not_on_map)}")
        lines += [f"  warning: {warning}" for warning in self.warnings]
        return lines


def _dock_params(name: str) -> dict[str, Param]:
    return {
        VertexParam.DOCK_NAME.value: Param(ParamType.STRING, name),
        VertexParam.IS_CHARGER.value: Param(ParamType.BOOL, True),
        VertexParam.ROBOT_DOCK_NAME.value: Param(ParamType.STRING, name),
    }


def _zone_params(name: str) -> dict[str, Param]:
    return {
        VertexParam.IS_CLEANING_ZONE.value: Param(ParamType.BOOL, True),
        VertexParam.ROBOT_ZONE_NAME.value: Param(ParamType.STRING, name),
    }


class _Target(NamedTuple):
    name: str
    position: Point
    params: dict[str, Param]


def _targets(geometry: MapGeometry, transform: MapTransform) -> list[_Target]:
    targets = [
        _Target(
            marker.name,
            transform.robot_to_floorplan_pixels(marker.point),
            _dock_params(marker.name) if marker.dock else {},
        )
        for marker in geometry.markers
    ]
    targets += [
        _Target(area.name, transform.robot_to_floorplan_pixels(area.start), _zone_params(area.name))
        for area in geometry.areas
    ]
    repeated = sorted(name for name, count in Counter(t.name for t in targets).items() if count > 1)
    if repeated:
        raise errors.NameCollisionError(
            f"the robot map {geometry.map_name} uses these names more than once: "
            f"{', '.join(repeated)}"
        )
    return targets


def _bearing(origin: Point, target: Point) -> float:
    """The robot map angle from origin to target. Map image rows count down."""
    return math.atan2(origin.y - target.y, target.x - origin.x)


def _faces(angle: float, expected: float) -> bool:
    return abs(wrap_angle(angle - expected)) <= APPROACH_TOLERANCE_RADIANS


def find_approach(dock: Marker, markers: tuple[Marker, ...], resolution: float) -> Marker | None:
    """The named point in front of the dock that faces it, the nearest one if several do."""
    best = None
    for marker in markers:
        if marker.dock or marker.name == dock.name:
            continue
        meters = math.dist(marker.point, dock.point) * resolution
        if not APPROACH_MIN_METERS <= meters <= APPROACH_MAX_METERS:
            continue
        if not _faces(_bearing(dock.point, marker.point), dock.angle):
            continue
        if not _faces(marker.angle, dock.angle + math.pi):
            continue
        if best is None or meters < best[0]:
            best = (meters, marker)
    return None if best is None else best[1]


def new_approach(dock: Marker, resolution: float) -> Marker:
    pixels = NEW_APPROACH_METERS / resolution
    point = Point(
        dock.point.x + pixels * math.cos(dock.angle),
        dock.point.y - pixels * math.sin(dock.angle),
    )
    return Marker(
        dock.name + APPROACH_POINT_SUFFIX, point, wrap_angle(dock.angle + math.pi), dock=False
    )


def _dock_approaches(geometry: MapGeometry) -> tuple[list[tuple[Marker, Marker, bool]], list[str]]:
    docks = [marker for marker in geometry.markers if marker.dock]
    if not docks:
        return [], []
    if geometry.frame is None:
        return [], [
            "the robot map has no frame, so the script adds no dock approach point or dock lane"
        ]
    pairs = []
    for dock in docks:
        approach = find_approach(dock, geometry.markers, geometry.frame.resolution)
        if approach is None:
            pairs.append((dock, new_approach(dock, geometry.frame.resolution), True))
        else:
            pairs.append((dock, approach, False))
    return pairs, []


def _new_lanes(
    vertices: list[Vertex], added: list[Vertex], existing_lanes: list[Any], pairs, graph_idx: int
) -> list[list[Any]]:
    index = {vertex.name: i for i, vertex in enumerate(vertices) if vertex.name}
    index.update({vertex.name: len(vertices) + i for i, vertex in enumerate(added)})
    existing = {frozenset(lane[:2]) for lane in existing_lanes}
    lanes = []
    for first, second in pairs:
        pair = [index[first], index[second]]
        if frozenset(pair) not in existing:
            existing.add(frozenset(pair))
            lanes.append([*pair, {"bidirectional": [4, True], "graph_idx": [2, graph_idx]}])
    return lanes


def _existing_by_name(vertices: list[Vertex], names: set[str]) -> dict[str, int]:
    counts = Counter(vertex.name for vertex in vertices if vertex.name in names)
    repeated = sorted(name for name, count in counts.items() if count > 1)
    if repeated:
        raise errors.DuplicateVertexNameError(
            f"the building file has more than one vertex named {', '.join(repeated)}"
        )
    return {vertex.name: index for index, vertex in enumerate(vertices) if vertex.name in names}


def make_plan(
    building: BuildingFile,
    level: str,
    geometry: MapGeometry,
    transform: MapTransform,
    *,
    update_positions: bool,
    graph_idx: int = 0,
) -> Plan:
    approaches, approach_warnings = _dock_approaches(geometry)
    created = tuple(approach for _, approach, made in approaches if made)
    targets = _targets(geometry._replace(markers=geometry.markers + created), transform)
    vertices = building.vertices(level)
    existing = _existing_by_name(vertices, {target.name for target in targets})
    plan = Plan(level=level)

    for target in targets:
        if target.name not in existing:
            plan.added.append(
                Vertex(target.position.x, target.position.y, 0, target.name, target.params)
            )
            continue
        index = existing[target.name]
        before = vertices[index]
        params = {**before.params, **target.params}
        moved = math.dist((before.x, before.y), target.position)
        x, y = before.x, before.y
        if moved > SAME_POSITION_PIXELS:
            if update_positions:
                x, y = target.position
            elif moved >= REPORT_MOVE_PIXELS:
                plan.kept_positions.append(
                    KeptPosition(target.name, moved, moved * transform.floorplan_scale)
                )
        after = Vertex(x, y, before.z, before.name, params)
        if after == before:
            plan.unchanged.append(target.name)
        else:
            plan.changed.append(VertexChange(index, before, after))

    target_names = {target.name for target in targets}
    plan.not_on_map = [
        vertex.name
        for vertex in vertices
        if vertex.name and vertex.name not in target_names and TOOL_PARAMS & set(vertex.params)
    ]
    plan.dock_headings = {
        marker.name: transform.heading(marker.angle) for marker in geometry.markers if marker.dock
    }
    plan.dock_approaches = [
        DockApproach(
            dock.name,
            approach.name,
            made,
            wrap_angle(transform.heading(dock.angle) + math.pi),
        )
        for dock, approach, made in approaches
    ]
    plan.warnings += approach_warnings
    plan.lanes = _new_lanes(
        vertices,
        plan.added,
        building.lanes(level),
        [(approach.name, dock.name) for dock, approach, _ in approaches],
        graph_idx,
    )
    for area in geometry.areas:
        if not area.path:
            plan.warnings.append(
                f"{area.name} has no cleaning path; its dock summary path is empty"
            )
        plan.cleaning_paths.append(
            CleaningPath(
                name=area.name,
                level_name=level,
                finish_waypoint=area.name,
                robot_zone_name=area.name,
                path=[(*transform.robot_to_rmf_meters(point), 0.0) for point in area.path],
            )
        )
    return plan


def unified_diff(path: str, before: str, after: str) -> list[str]:
    return list(
        difflib.unified_diff(
            before.splitlines(),
            after.splitlines(),
            fromfile=path,
            tofile=f"{path} (after)",
            lineterm="",
        )
    )
