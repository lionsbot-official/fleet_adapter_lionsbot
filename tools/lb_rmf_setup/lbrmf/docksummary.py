from collections.abc import Mapping, Sequence
from enum import Enum
from typing import Any

import yaml

from lbrmf import errors
from lbrmf.plan import CleaningPath

DockSummary = dict[str, dict[str, dict[str, Any]]]


class _Key(str, Enum):
    FINISH_WAYPOINT = "finish_waypoint"
    LEVEL_NAME = "level_name"
    PATH = "path"
    ROBOT_ZONE_NAME = "robot_zone_name"


def parse(text: str | None) -> DockSummary:
    try:
        data = yaml.safe_load(text) if text else None
    except yaml.YAMLError as err:
        raise errors.MalformedDockSummaryError(f"not valid YAML: {err}") from err
    if data is None:
        return {}
    if not isinstance(data, dict) or not all(
        isinstance(entries, dict) and all(isinstance(entry, dict) for entry in entries.values())
        for entries in data.values()
    ):
        raise errors.MalformedDockSummaryError(
            "the dock summary must map each fleet to a mapping of named entries"
        )
    return data


def _entry(path: CleaningPath) -> dict[str, Any]:
    return {
        _Key.FINISH_WAYPOINT.value: path.finish_waypoint,
        _Key.LEVEL_NAME.value: path.level_name,
        _Key.PATH.value: [list(point) for point in path.path],
        _Key.ROBOT_ZONE_NAME.value: path.robot_zone_name,
    }


def merge(summary: Mapping[str, Any], fleet: str, paths: Sequence[CleaningPath]) -> DockSummary:
    merged = {name: dict(entries) for name, entries in summary.items()}
    entries = merged.setdefault(fleet, {})
    for path in paths:
        entries[path.name] = _entry(path)
    return merged


def render(summary: DockSummary) -> str:
    return yaml.safe_dump(summary, sort_keys=True, default_flow_style=False)
