from collections.abc import Collection
from typing import NamedTuple

from lbrmf import docksummary
from lbrmf.building import BuildingFile
from lbrmf.charger import fill_chargers
from lbrmf.constants import DOOR_POINT_SUFFIX
from lbrmf.fleetconfig import FleetConfig
from lbrmf.frame import MapTransform, floorplan_scale
from lbrmf.geometry import MapGeometry
from lbrmf.plan import Plan, make_plan


class ExportResult(NamedTuple):
    plan: Plan
    building_after: str
    dock_summary_after: str | None
    config_after: str | None = None
    charger_notes: tuple[str, ...] = ()


def export_level(
    building_text: str,
    config: FleetConfig,
    level: str,
    geometry: MapGeometry,
    dock_summary_text: str | None,
    *,
    update_positions: bool,
    lift_names: Collection[str] | None = None,
    graph_idx: int = 0,
    config_text: str | None = None,
    local_map: bool = False,
) -> ExportResult:
    building = BuildingFile.parse(building_text)
    values, level_transform = config.level_transform(level)
    measured = None
    if level_transform.scale is None:
        measured = floorplan_scale(building.measurements(level))
    transform = MapTransform(values, level_transform, measured)
    names = {lift.name for lift in geometry.lifts} if lift_names is None else set(lift_names)
    lift_points = names | {name + DOOR_POINT_SUFFIX for name in names}
    skipped = [marker.name for marker in geometry.markers if marker.name in lift_points]
    geometry = geometry._replace(
        markers=tuple(m for m in geometry.markers if m.name not in lift_points)
    )
    plan = make_plan(
        building, level, geometry, transform, update_positions=update_positions, graph_idx=graph_idx
    )
    plan.skipped_lift_points = skipped
    building_after = building.render({level: plan.level_edit()})

    summary = docksummary.parse(dock_summary_text)
    merged = docksummary.merge(summary, config.fleet_name, plan.cleaning_paths)
    dock_summary_after = None if merged == summary else docksummary.render(merged)
    config_after = None
    charger_notes: tuple[str, ...] = ()
    if config_text is not None:
        chargers = fill_chargers(config_text, level, geometry, any_map=local_map)
        if chargers.text != config_text:
            config_after = chargers.text
        charger_notes = tuple(
            f"charger: {robot} goes home to {dock}" for robot, dock in chargers.filled.items()
        ) + tuple(chargers.notes)
    return ExportResult(plan, building_after, dock_summary_after, config_after, charger_notes)
