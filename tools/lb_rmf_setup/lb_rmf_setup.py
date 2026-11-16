#!/usr/bin/env python3
"""Make the Open-RMF map files for LionsBot robots."""

import argparse
import dataclasses
import json
import logging
import os
import subprocess
import sys
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

from lbrmf import __version__, check, fileio, init
from lbrmf.api import ApiClient, credentials_from_env
from lbrmf.building import BuildingFile
from lbrmf.constants import PROGRAM_NAME, EnvVar, ExitCode
from lbrmf.errors import (
    DriftError,
    IssuesFoundError,
    MapNotFoundError,
    OutputExistsError,
    SetupError,
    UsageError,
)
from lbrmf.export import export_level
from lbrmf.fleetconfig import FleetConfig, parse_fleet_config
from lbrmf.frame import MapTransform, floorplan_scale
from lbrmf.geometry import MapGeometry
from lbrmf.inforeader import read_geometry, read_info
from lbrmf.lifts import ExportKinds, LevelInput, plan_lifts
from lbrmf.lionsbot import LionsBotMaps, MapSummary, sanitize_file_name
from lbrmf.mapfiles import geometry_json, ros_map_yaml
from lbrmf.plan import unified_diff
from lbrmf.transformfill import fill_level_transform, fill_transform

TOOL_DIR = Path(__file__).resolve().parent


@dataclass(frozen=True)
class Command:
    name: str
    help: str
    add_arguments: Callable[[argparse.ArgumentParser], None]
    run: Callable[[argparse.Namespace], ExitCode]


DOCK_SUMMARY_FILE = "dock_summary.yaml"
ADAPTER_DIR = TOOL_DIR.parents[1] / "fleet_adapter"
CONFIG_TEMPLATE = Path("configs") / "config_template.yaml"


class Output:
    def __init__(self, as_json: bool):
        self.as_json = as_json
        self.data: dict[str, object] = {}

    def line(self, text: str) -> None:
        if not self.as_json:
            print(text)

    def set(self, key: str, value: object) -> None:
        self.data[key] = value

    def warn(self, text: str) -> None:
        if self.as_json:
            self.data.setdefault("warnings", []).append(text)
        else:
            print(f"warning: {text}")

    def rows(self, header: Sequence[str], rows: Sequence[Sequence[object]]) -> None:
        table = [list(header)] + [
            ["-" if value is None else str(value) for value in row] for row in rows
        ]
        widths = [max(len(row[i]) for row in table) for i in range(len(header))]
        for row in table:
            self.line("  ".join(v.ljust(w) for v, w in zip(row, widths, strict=True)).rstrip())

    def succeed(self) -> None:
        if self.as_json:
            print(json.dumps({"ok": True, **self.data}, indent=1, default=str))

    def fail(self, err: SetupError) -> None:
        if self.as_json:
            body = {"ok": False, "exit_code": int(err.exit_code), "error": str(err)}
            print(json.dumps(body, indent=1))
        else:
            print(f"error: {err}", file=sys.stderr)


def _environment(args: argparse.Namespace) -> dict[str, str]:
    env = dict(os.environ)
    if args.env_file is not None:
        for key, value in fileio.parse_dotenv(fileio.read_text(args.env_file)).items():
            env.setdefault(key, value)
    return env


def _client(args: argparse.Namespace) -> ApiClient:
    return ApiClient(credentials_from_env(_environment(args)))


def _add_robot(parser: argparse.ArgumentParser, required: bool) -> None:
    parser.add_argument(
        "--robot", required=required, help="the robot UUID or serial, for example R3SCP-B02357"
    )


def _add_map_choice(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--map-id", help="the robot map UUID")
    parser.add_argument("--map-name", help="the robot map name; give --map-level too")
    parser.add_argument("--map-level", help="the robot map level")


def _run_auth_check(args: argparse.Namespace) -> ExitCode:
    out = args.output
    client = _client(args)
    info = client.token_info()
    expires = info.expires_at.isoformat() if info.expires_at else None
    details = {
        "method": info.method.value,
        "login": info.principal,
        "account_id": info.account_id,
        "expires": expires,
        "base_url": client.credentials.base_url,
    }
    out.set("auth", details)
    out.line(f"method:     {details['method']}")
    out.line(f"login:      {details['login']}")
    out.line(f"account id: {details['account_id'] or 'unknown'}")
    out.line(f"expires:    {expires or 'unknown'}")
    out.line(f"base URL:   {details['base_url']}")
    return ExitCode.OK


def _run_robots_list(args: argparse.Namespace) -> ExitCode:
    out = args.output
    robots = LionsBotMaps(_client(args)).robots()
    out.set("robots", [robot._asdict() for robot in robots])
    out.rows(
        ["serial", "id", "family", "type", "autonomy", "worksite"],
        [(r.serial, r.id, r.family, r.type, r.autonomy_version, r.worksite) for r in robots],
    )
    return ExitCode.OK


def _run_maps_list(args: argparse.Namespace) -> ExitCode:
    out = args.output
    source = LionsBotMaps(_client(args))
    maps = source.maps(source.resolve_robot_id(args.robot))
    rows = [{k: v for k, v in m._asdict().items() if k != "image_link"} for m in maps]
    for row in rows:
        row["frame"] = None if row["frame"] is None else dataclasses.asdict(row["frame"])
    out.set("maps", rows)
    out.rows(
        ["id", "name", "level", "building", "last change", "frame"],
        [
            (m.id, m.name, m.level, m.building, m.last_modified, "yes" if m.frame else "no")
            for m in maps
        ],
    )
    return ExitCode.OK


def _select_map(maps: Sequence[MapSummary], map_id, map_name, map_level) -> MapSummary:
    for candidate in maps:
        if map_id is not None and candidate.id == map_id:
            return candidate
        if map_id is None and (candidate.name, candidate.level) == (map_name, map_level):
            return candidate
    wanted = map_id or f"{map_name} (level {map_level})"
    names = ", ".join(f"{m.name} (level {m.level})" for m in maps) or "none"
    raise MapNotFoundError(f"the robot has no map {wanted}; its maps: {names}")


def _map_choice(args: argparse.Namespace) -> tuple[str | None, str | None, str | None]:
    if (args.map_name is None) != (args.map_level is None):
        raise UsageError("give --map-name and --map-level together")
    if args.map_id is not None and args.map_name is not None:
        raise UsageError("give --map-id or --map-name with --map-level, not both")
    return args.map_id, args.map_name, args.map_level


def _run_maps_fetch(args: argparse.Namespace) -> ExitCode:
    map_id, map_name, map_level = _map_choice(args)
    if map_id is None and map_name is None:
        raise UsageError("give --map-id, or --map-name with --map-level")
    source = LionsBotMaps(_client(args))
    chosen = _select_map(
        source.maps(source.resolve_robot_id(args.robot)), map_id, map_name, map_level
    )
    out = args.output
    geometry, warnings = source.geometry(chosen)
    for warning in warnings:
        out.warn(warning)
    lifts, lobbies = source.lifts(chosen)
    geometry = geometry._replace(lifts=lifts, lobbies=lobbies)
    image = source.image(chosen)
    base = sanitize_file_name(chosen.name)
    files = {
        f"{base}.png": image,
        f"{base}.yaml": ros_map_yaml(
            f"{base}.png", chosen.frame, chosen.occupied_thresh, chosen.free_thresh
        ).encode(),
        f"{base}.geometry.json": geometry_json(geometry).encode(),
    }
    if not args.overwrite:
        _refuse_existing([args.out / name for name in files])
    args.out.mkdir(parents=True, exist_ok=True)
    for name, data in files.items():
        fileio.write_atomic_bytes(args.out / name, data)
        out.line(f"wrote {args.out / name}")
    out.set("written", [str(args.out / name) for name in files])
    return ExitCode.OK


def _refuse_existing(paths: Sequence[Path]) -> None:
    existing = [str(path) for path in paths if path.exists()]
    if existing:
        raise OutputExistsError(
            f"{', '.join(existing)} already exist(s); give --overwrite to replace them"
        )


def _add_overwrite(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--overwrite", action="store_true", help="replace output files that already exist"
    )


def _add_maps_fetch_arguments(parser: argparse.ArgumentParser) -> None:
    _add_robot(parser, required=True)
    _add_map_choice(parser)
    parser.add_argument("--out", type=Path, required=True, help="the folder for the map files")
    _add_overwrite(parser)


def _add_maps_list_arguments(parser: argparse.ArgumentParser) -> None:
    _add_robot(parser, required=True)


def _add_export_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--building", type=Path, required=True, help="the <rmf>.building.yaml")
    parser.add_argument("--config", type=Path, required=True, help="the fleet config_<type>.yaml")
    parser.add_argument("--level", required=True, help="the RMF level name, for example L8")
    parser.add_argument(
        "--dock-summary",
        type=Path,
        help=f"default: {DOCK_SUMMARY_FILE} beside the building file",
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--from-info", type=Path, help="a robot map .info file")
    source.add_argument("--from-geometry", type=Path, help="a .geometry.json file")
    _add_robot(parser, required=False)
    _add_map_choice(parser)
    parser.add_argument(
        "--layer-png", type=Path, help="also write the plain map image to this path"
    )
    _add_overwrite(parser)
    parser.add_argument(
        "--update-positions",
        action="store_true",
        help="move existing vertices to the robot map position",
    )
    parser.add_argument(
        "--graph-idx",
        type=int,
        default=0,
        help="the nav graph of the dock lanes the script adds (default 0)",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="print the diff, write nothing")
    mode.add_argument(
        "--check", action="store_true", help="print the diff; exit 10 when files would change"
    )


def _api_map(args: argparse.Namespace, config: FleetConfig) -> tuple[LionsBotMaps, MapSummary]:
    map_id, map_name, map_level = _map_choice(args)
    default = config.robot_map(args.level)
    robot = args.robot or (default.robot_id if default else None)
    if robot is None:
        raise UsageError(
            f"no robot in the config is on {args.level}. Add the robot map of "
            f"{args.level} to rmf_config.robot_maps of the robot, or give --robot"
        )
    if map_id is None and map_name is None:
        if default is None:
            raise UsageError("give --map-id, or --map-name with --map-level")
        map_name, map_level = default.map_name, default.map_level
    source = LionsBotMaps(_client(args))
    maps = source.maps(source.resolve_robot_id(robot))
    return source, _select_map(maps, map_id, map_name, map_level)


def _run_export(args: argparse.Namespace) -> ExitCode:
    dock_summary_path = args.dock_summary or args.building.parent / DOCK_SUMMARY_FILE
    building_text = fileio.read_text(args.building)
    original_config_text = fileio.read_text(args.config)
    default_map = parse_fleet_config(original_config_text).robot_map(args.level)
    building = BuildingFile.parse(building_text)
    transform = fill_transform(
        original_config_text,
        building,
        args.level,
        args.map_name or (default_map.map_name if default_map else None),
        args.config.stem.removeprefix("config_"),
    )
    for note in transform.notes:
        args.output.warn(note)
    if transform.source is not None:
        args.output.line(f"map transform: {args.level} from {transform.source}")
    config_text, level_source = fill_level_transform(transform.text, building, args.level)
    if level_source is not None:
        args.output.line(f"level transform: {args.level} from {level_source}")
    config = parse_fleet_config(config_text)
    layer_png = None
    lift_names = None
    if args.from_info is not None:
        geometry = read_info(fileio.read_text(args.from_info), args.from_info.stem)
    elif args.from_geometry is not None:
        geometry = read_geometry(fileio.read_text(args.from_geometry))
    else:
        source, chosen = _api_map(args, config)
        geometry, warnings = source.geometry(chosen)
        for warning in warnings:
            args.output.warn(warning)
        lift_names = source.lift_names(chosen)
        if args.layer_png is not None:
            layer_png = source.image(chosen)
    if args.layer_png is not None and layer_png is None:
        raise UsageError("--layer-png needs the map from the API, not a local file")
    dock_summary_text = fileio.read_text(dock_summary_path) if dock_summary_path.exists() else None

    result = export_level(
        building_text,
        config,
        args.level,
        geometry,
        dock_summary_text,
        update_positions=args.update_positions,
        lift_names=lift_names,
        graph_idx=args.graph_idx,
        config_text=config_text,
        local_map=args.from_info is not None or args.from_geometry is not None,
    )

    changes = []
    if result.building_after != building_text:
        changes.append((args.building, building_text, result.building_after))
    if result.dock_summary_after is not None:
        changes.append((dock_summary_path, dock_summary_text or "", result.dock_summary_after))
    config_after = result.config_after or config_text
    if config_after != original_config_text:
        changes.append((args.config, original_config_text, config_after))
    if layer_png is not None and not args.overwrite and not (args.dry_run or args.check):
        _refuse_existing([args.layer_png])
    _report(args.output, {args.level: result.plan}, list(result.charger_notes), changes, args)
    if layer_png is not None and not (args.dry_run or args.check):
        fileio.write_atomic_bytes(args.layer_png, layer_png)
        args.output.line(f"wrote {args.layer_png}")
    return ExitCode.OK


def _report(out: Output, plans, messages, changes, args: argparse.Namespace) -> None:
    for plan in plans.values():
        out.line("\n".join(plan.summary()))
    for message in messages:
        out.line(message)
    out.set("levels", {level: plan.as_dict() for level, plan in plans.items()})
    if messages:
        out.set("messages", list(messages))
    out.set("files_to_change", [str(path) for path, _, _ in changes])
    preview = args.dry_run or args.check
    if preview:
        diffs = {
            str(path): unified_diff(str(path), before, after) for path, before, after in changes
        }
        for lines in diffs.values():
            out.line("\n".join(lines))
        out.set("diffs", diffs)
    out.set("written", bool(changes) and not preview)
    if args.check and changes:
        raise DriftError(f"{len(changes)} file(s) would change; run without --check to write them")
    if preview:
        return
    for path, _, after in changes:
        fileio.write_atomic(path, after)
        out.line(f"wrote {path}")


def _level_value(text: str) -> tuple[str, str]:
    level, sep, value = text.partition("=")
    if not sep or not level or not value:
        raise UsageError(f"{text!r} is not LEVEL=VALUE")
    return level, value


def _add_lifts_export_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--building", type=Path, required=True, help="the <rmf>.building.yaml")
    parser.add_argument("--config", type=Path, required=True, help="the fleet config_<type>.yaml")
    parser.add_argument(
        "--level", action="append", default=[], help="a level read from the API (repeat)"
    )
    parser.add_argument(
        "--from-info", action="append", default=[], help="LEVEL=path of a .info file (repeat)"
    )
    parser.add_argument(
        "--from-geometry",
        action="append",
        default=[],
        help="LEVEL=path of a .geometry.json file (repeat)",
    )
    parser.add_argument("--reference-level", help="default: the first level given")
    kinds = parser.add_argument_group("what to add as vertices (nothing unless asked)")
    kinds.add_argument("--lift-cabins", action="store_true", help="a cabin vertex in each car")
    kinds.add_argument(
        "--lift-approach",
        action="store_true",
        help="an approach vertex outside each lift door; with --lift-cabins, a lane to the cabin",
    )
    kinds.add_argument("--lobbies", action="store_true", help="a vertex at each lift lobby")
    kinds.add_argument("--all", action="store_true", help="all three kinds above")
    parser.add_argument(
        "--graph-idx", type=int, default=0, help="the nav graph of new lanes (default 0)"
    )
    parser.add_argument(
        "--lift-depth", action="append", default=[], help="LIFT=metres, the car depth (repeat)"
    )
    parser.add_argument(
        "--update-positions",
        action="store_true",
        help="move existing lift vertices and replace existing lift entries",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="print the diff, write nothing")
    mode.add_argument(
        "--check", action="store_true", help="print the diff; exit 10 when files would change"
    )


def _level_transform(building: BuildingFile, config: FleetConfig, level: str) -> MapTransform:
    values, level_transform = config.level_transform(level)
    measured = None
    if level_transform.scale is None:
        measured = floorplan_scale(building.measurements(level))
    return MapTransform(values, level_transform, measured)


def _lift_geometries(
    args: argparse.Namespace, config: FleetConfig
) -> list[tuple[str, MapGeometry]]:
    geometries = []
    for text in args.from_info:
        level, path = _level_value(text)
        geometries.append((level, read_info(fileio.read_text(Path(path)), Path(path).stem)))
    for text in args.from_geometry:
        level, path = _level_value(text)
        geometries.append((level, read_geometry(fileio.read_text(Path(path)))))
    for level in args.level:
        default = config.robot_map(level)
        if default is None:
            raise UsageError(f"no robot in the config is on {level}; give --from-info for it")
        source = LionsBotMaps(_client(args))
        maps = source.maps(source.resolve_robot_id(default.robot_id))
        chosen = _select_map(maps, None, default.map_name, default.map_level)
        geometry, warnings = source.geometry(chosen)
        for warning in warnings:
            args.output.warn(warning)
        lifts, lobbies = source.lifts(chosen)
        geometries.append((level, geometry._replace(lifts=lifts, lobbies=lobbies)))
    if not geometries:
        raise UsageError("give at least one level: --level, --from-info or --from-geometry")
    return geometries


def _run_lifts_export(args: argparse.Namespace) -> ExitCode:
    depths = {}
    for text in args.lift_depth:
        name, value = _level_value(text)
        try:
            depths[name] = float(value)
        except ValueError as err:
            raise UsageError(f"--lift-depth {text!r}: {value!r} is not metres") from err
    building_text = fileio.read_text(args.building)
    building = BuildingFile.parse(building_text)
    config = parse_fleet_config(fileio.read_text(args.config))
    inputs = [
        LevelInput(level, geometry, _level_transform(building, config, level))
        for level, geometry in _lift_geometries(args, config)
    ]
    result = plan_lifts(
        building,
        inputs,
        reference_level=args.reference_level or inputs[0].level,
        depths=depths,
        update_positions=args.update_positions,
        kinds=ExportKinds(
            cabins=args.lift_cabins or args.all,
            approach=args.lift_approach or args.all,
            lobbies=args.lobbies or args.all,
        ),
        graph_idx=args.graph_idx,
    )
    after = building.render(
        {level: plan.level_edit() for level, plan in result.plans.items()}, result.lifts
    )
    changes = [] if after == building_text else [(args.building, building_text, after)]
    _report(args.output, result.plans, result.messages, changes, args)
    return ExitCode.OK


def _no_arguments(parser: argparse.ArgumentParser) -> None:
    pass


def _add_init_arguments(parser: argparse.ArgumentParser) -> None:
    _add_robot(parser, required=True)
    parser.add_argument(
        "--building", required=True, help="the site folder name, for example pdd_office"
    )
    _add_map_choice(parser)
    parser.add_argument(
        "--level", help="the RMF level name (default: the level of the robot map, or L1)"
    )
    parser.add_argument(
        "--robot-type", help="the fleet type in the config name (default: from the robot)"
    )
    parser.add_argument("--fleet-name", help="the RMF fleet name (default: <robot type>_fleet)")
    parser.add_argument(
        "--fleet-number",
        type=int,
        default=1,
        help="the FLEET_<n>_ settings to write in --env-file (default 1)",
    )
    parser.add_argument(
        "--adapter-dir",
        type=Path,
        default=ADAPTER_DIR,
        help="the fleet_adapter folder (default: the one in this repository)",
    )
    _add_overwrite(parser)
    parser.add_argument("--dry-run", action="store_true", help="print the changes, write nothing")


def _init_map(args: argparse.Namespace, source: LionsBotMaps, robot_id: str) -> MapSummary:
    maps = source.maps(robot_id)
    map_id, map_name, map_level = _map_choice(args)
    if map_id is not None or map_name is not None:
        return _select_map(maps, map_id, map_name, map_level)
    if len(maps) == 1:
        return maps[0]
    current = source.current_map_ids.get(robot_id)
    for candidate in maps:
        if candidate.id == current:
            return candidate
    names = ", ".join(f"{m.name} (level {m.level})" for m in maps) or "none"
    raise UsageError(f"the robot has several maps, so give --map-name and --map-level: {names}")


def _run_init(args: argparse.Namespace) -> ExitCode:
    env = _environment(args)
    source = LionsBotMaps(_client(args))
    robot = next(
        (r for r in source.robots() if args.robot in (r.serial, r.id)),
        None,
    )
    if robot is None:
        raise UsageError(f"the account cannot see a robot {args.robot}")
    chosen = _init_map(args, source, robot.id)
    level = args.level or chosen.level or "L1"
    kind = args.robot_type or init.robot_type(robot.family, robot.type)
    values = init.InitValues(
        fleet_name=args.fleet_name or f"{kind}_fleet",
        robot=robot.serial,
        robot_id=robot.id,
        rmf_level=level,
        map_name=chosen.name,
        map_level=chosen.level or level,
        api_host=init.api_host(env.get(EnvVar.BASE_URL.value, "")),
    )
    config_path = args.adapter_dir / "configs" / args.building / f"config_{kind}.yaml"
    maps_dir = args.adapter_dir / "maps" / args.building
    config_text = init.render_config(fileio.read_text(args.adapter_dir / CONFIG_TEMPLATE), values)
    changes = [(config_path, "", config_text)]
    if not args.overwrite and not args.dry_run:
        _refuse_existing([config_path])
    if args.env_file is not None and args.env_file.exists():
        env_text = fileio.read_text(args.env_file)
        number = args.fleet_number
        env_after = init.update_env(
            env_text,
            {
                "FOLDER_NAME": args.building,
                f"FLEET_{number}_CONFIG": config_path.name,
                f"FLEET_{number}_NAV_GRAPH": "0.yaml",
                "RMF_FLOOR_NAME": level,
            },
        )
        if env_after != env_text:
            changes.append((args.env_file, env_text, env_after))

    out = args.output
    out.line(
        f"robot {robot.serial}: map {chosen.name} (level {values.map_level}) on RMF level {level}"
    )
    out.set("config", str(config_path))
    out.set("values", values._asdict())
    if args.dry_run:
        for path, before, after in changes:
            out.line("\n".join(unified_diff(str(path), before, after)))
    else:
        maps_dir.mkdir(parents=True, exist_ok=True)
        config_path.parent.mkdir(parents=True, exist_ok=True)
        for path, _, after in changes:
            fileio.write_atomic(path, after)
            out.line(f"wrote {path}")
    building = maps_dir / f"rmf_{args.building}.building.yaml"
    out.line(
        f"next: download the robot map image with maps fetch --robot {robot.serial} "
        f"--map-name {chosen.name} --map-level {values.map_level} --out {maps_dir}/. "
        "Then in traffic editor draw the level, a measurement and a wall, and save "
        f"{building}. Then run export with --building {building} --config {config_path} "
        f"--level {level}"
    )
    return ExitCode.OK


def _add_check_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--building", type=Path, required=True, help="the <rmf>.building.yaml")
    parser.add_argument(
        "--level",
        action="append",
        help="an RMF level to check, more than once for several (default: every level)",
    )
    parser.add_argument(
        "--config",
        type=Path,
        action="append",
        help="a fleet config whose charger waypoints to check, more than once for several",
    )


def _run_check(args: argparse.Namespace) -> ExitCode:
    building = BuildingFile.parse(fileio.read_text(args.building))
    levels = args.level or building.level_names
    issues = [issue for level in levels for issue in check.check_level(building, level)]
    issues += check.check_names(building)
    for config in args.config or []:
        issues += check.check_config(building, fileio.read_text(config))
    args.output.set("issues", issues)
    for issue in issues:
        args.output.line(issue)
    if issues:
        raise IssuesFoundError(
            f"found {len(issues)} problem(s) to fix in traffic editor or the config"
        )
    args.output.line("no problems found")
    return ExitCode.OK


COMMANDS: tuple[Command, ...] = (
    Command("auth check", "log in and show the account", _no_arguments, _run_auth_check),
    Command(
        "robots list",
        "list the robots the account can see",
        _no_arguments,
        _run_robots_list,
    ),
    Command("maps list", "list a robot's maps", _add_maps_list_arguments, _run_maps_list),
    Command(
        "maps fetch",
        "download a map image, ROS map yaml and geometry file",
        _add_maps_fetch_arguments,
        _run_maps_fetch,
    ),
    Command(
        "lifts export",
        "add lift vertices on each level and write the RMF lifts block",
        _add_lifts_export_arguments,
        _run_lifts_export,
    ),
    Command(
        "export",
        "add the robot map's named points to the building file and dock summary",
        _add_export_arguments,
        _run_export,
    ),
    Command(
        "init",
        "write a new fleet config for a robot and set the site in the .env file",
        _add_init_arguments,
        _run_init,
    ),
    Command(
        "check",
        "find stray vertices, points with no lane and unset chargers",
        _add_check_arguments,
        _run_check,
    ),
)


def git_commit() -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(TOOL_DIR), "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError:
        return None
    if result.returncode != 0:
        return None
    return result.stdout.strip()


def version_text() -> str:
    commit = git_commit()
    if commit is None:
        return f"{PROGRAM_NAME} {__version__}"
    return f"{PROGRAM_NAME} {__version__} (commit {commit})"


# A custom action so that only --version runs git, not every command.
class _VersionAction(argparse.Action):
    def __init__(self, option_strings, dest, **kwargs):
        super().__init__(option_strings, dest, nargs=0, **kwargs)

    def __call__(self, parser, namespace, values, option_string=None):
        print(version_text())
        parser.exit(ExitCode.OK)


def build_parser(commands: Sequence[Command]) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog=PROGRAM_NAME, description=__doc__.splitlines()[0])
    parser.add_argument("--version", action=_VersionAction, help="print the version and exit")
    parser.add_argument("--env-file", type=Path, help="read unset variables from a dotenv file")
    parser.add_argument(
        "--json", action="store_true", help="print one JSON result object, also on failure"
    )
    parser.add_argument(
        "--verbose", action="store_true", help="print each API call: method, host, path, status"
    )
    parser.set_defaults(run=None)
    subparsers = parser.add_subparsers(title="commands", metavar="<command>")
    groups: dict[str, argparse._SubParsersAction] = {}
    for command in commands:
        group, _, action = command.name.partition(" ")
        target = subparsers
        name = group
        if action:
            if group not in groups:
                group_parser = subparsers.add_parser(group, help=f"{group} commands")
                groups[group] = group_parser.add_subparsers(metavar="<action>")
            target, name = groups[group], action
        sub = target.add_parser(name, help=command.help)
        command.add_arguments(sub)
        sub.set_defaults(run=command.run)
    return parser


def _log_to_stderr() -> None:
    logger = logging.getLogger("lbrmf")
    for handler in list(logger.handlers):
        logger.removeHandler(handler)
    handler = logging.StreamHandler(sys.stderr)
    handler.setFormatter(logging.Formatter("%(message)s"))
    logger.addHandler(handler)
    logger.setLevel(logging.DEBUG)
    logger.propagate = False


def main(argv: Sequence[str] | None = None, commands: Sequence[Command] = COMMANDS) -> int:
    parser = build_parser(commands)
    args = parser.parse_args(argv)
    if args.run is None:
        parser.print_usage(sys.stderr)
        return ExitCode.USAGE
    if args.verbose:
        _log_to_stderr()
    args.output = Output(args.json)
    try:
        code = args.run(args)
    except SetupError as err:
        args.output.fail(err)
        return err.exit_code
    args.output.succeed()
    return code


def console_main() -> None:
    sys.exit(main())


if __name__ == "__main__":
    console_main()
