import json
import math
import shutil

import pytest
import yaml
from apiserver import Reply, ScriptedServer
from fixtures import TESTDATA

import lb_rmf_setup
from lbrmf import errors
from lbrmf.constants import ExitCode
from lbrmf.frame import Point
from lbrmf.geometry import Lift, Lobby
from lbrmf.inforeader import read_geometry, read_info
from lbrmf.mapfiles import geometry_json

TWOFLOOR = TESTDATA / "twofloor"
LIFT_A = {
    "depth": 2.0,
    "doors": {
        "door_1": {
            "door_type": 2,
            "motion_axis_orientation": 0.0,
            "width": 0.9,
            "x": 0.0,
            "y": -1.0,
        }
    },
    "highest_floor": "L3",
    "initial_floor_name": "L1",
    "level_doors": {"L1": ["door_1"], "L3": ["door_1"]},
    "lowest_floor": "L1",
    "plugins": False,
    "reference_floor_name": "L1",
    "width": 1.6,
    "x": 200.0,
    "y": 300.0,
    "yaw": pytest.approx(-math.pi),
}


@pytest.fixture
def site(tmp_path):
    for path in TWOFLOOR.iterdir():
        shutil.copy(path, tmp_path / path.name)
    return tmp_path


def _lifts_export(site, *extra):
    return lb_rmf_setup.main(
        [
            "lifts",
            "export",
            "--building",
            str(site / "twofloor.building.yaml"),
            "--config",
            str(site / "config_r3.yaml"),
            "--from-info",
            f"L1={site / 'L1.info'}",
            "--from-info",
            f"L3={site / 'L3.info'}",
            *extra,
        ]
    )


def _building(site):
    return yaml.safe_load((site / "twofloor.building.yaml").read_text())


def _named(level):
    return {vertex[3]: vertex for vertex in level["vertices"] if vertex[3]}


def test_reads_lifts_from_the_info_file():
    geometry = read_info((TWOFLOOR / "L1.info").read_text(), "L1")

    assert geometry.lifts == (Lift("lift_a", Point(200, 260), 0.9, 1.6),)


def test_reads_lobbies_from_the_info_file():
    geometry = read_info((TWOFLOOR / "L1.info").read_text(), "L1")

    assert geometry.lobbies == (Lobby("lobby_1", Point(200, 240)),)


def test_geometry_file_keeps_lifts():
    geometry = read_info((TWOFLOOR / "L1.info").read_text(), "L1")

    assert read_geometry(geometry_json(geometry)) == geometry


def test_without_flags_only_the_lifts_block_is_written(site, capsys):
    before = _building(site)["levels"]

    assert _lifts_export(site) == ExitCode.OK

    building = _building(site)
    assert building["levels"] == before
    assert building["lifts"] == {"lift_a": LIFT_A}
    assert "lift_a: depth 2.00 m from the door point" in capsys.readouterr().out


def test_the_rmf_lift_is_the_car_point_not_the_lift_marker(site):
    _edit_info(
        site, "L1", lambda info: info["lift_lobbies"][0]["lifts"][0].update(coords="150 260")
    )
    _edit_info(
        site, "L3", lambda info: info["lift_lobbies"][0]["lifts"][0].update(coords="150 260")
    )

    assert _lifts_export(site) == ExitCode.OK

    lift = _building(site)["lifts"]["lift_a"]
    assert (lift["x"], lift["y"]) == (200, 300)


@pytest.mark.parametrize(
    ("flags", "expected"),
    [
        (["--lift-cabins"], {"lift_a_L1_cabin"}),
        (["--lift-approach"], {"lift_a_L1_approach"}),
        (["--lobbies"], {"lobby_1_L1"}),
        (["--all"], {"lift_a_L1_cabin", "lift_a_L1_approach", "lobby_1_L1"}),
    ],
    ids=["cabins", "approach", "lobbies", "all"],
)
def test_each_flag_exports_only_its_kind(site, flags, expected):
    assert _lifts_export(site, *flags) == ExitCode.OK

    assert set(_named(_building(site)["levels"]["L1"])) == expected


def test_all_kinds_with_their_positions_and_lane(site):
    assert _lifts_export(site, "--all", "--graph-idx", "1") == ExitCode.OK

    l1 = _building(site)["levels"]["L1"]
    named = _named(l1)
    assert named["lift_a_L1_approach"][:3] == [200, 260, 0]
    assert named["lift_a_L1_cabin"][:3] == [200, 300, 0]
    assert named["lobby_1_L1"][:3] == [200, 240, 0]
    index = {vertex[3]: i for i, vertex in enumerate(l1["vertices"])}
    assert l1["lanes"] == [
        [
            index["lift_a_L1_approach"],
            index["lift_a_L1_cabin"],
            {"bidirectional": [4, True], "graph_idx": [2, 1]},
        ]
    ]
    l3 = _named(_building(site)["levels"]["L3"])
    assert l3["lift_a_L3_cabin"][:3] == [200.5, 300, 0]


def test_a_second_run_changes_nothing(site):
    assert _lifts_export(site, "--all") == ExitCode.OK
    before = (site / "twofloor.building.yaml").read_text()

    assert _lifts_export(site, "--all", "--check") == ExitCode.OK
    assert (site / "twofloor.building.yaml").read_text() == before


def test_existing_lift_entry_is_kept_unless_asked(site, capsys):
    assert _lifts_export(site) == ExitCode.OK
    path = site / "twofloor.building.yaml"
    path.write_text(path.read_text().replace("width: 1.6", "width: 1.8"))

    assert _lifts_export(site) == ExitCode.OK
    assert _building(site)["lifts"]["lift_a"]["width"] == 1.8
    assert "lift_a: kept the lifts entry" in capsys.readouterr().out

    assert _lifts_export(site, "--update-positions") == ExitCode.OK
    assert _building(site)["lifts"]["lift_a"]["width"] == 1.6


def test_lift_depth_flag_and_fallback(site, capsys):
    for level in ["L1", "L3"]:
        info = json.loads((site / f"{level}.info").read_text())
        info["markers"]["loc_point"] = [
            m for m in info["markers"]["loc_point"] if m["name"] == "lift_a"
        ]
        (site / f"{level}.info").write_text(json.dumps(info))

    assert _lifts_export(site, "--dry-run") == ExitCode.OK
    assert "lift_a: no door point lift_a_door; depth set to the car width 1.60 m" in (
        capsys.readouterr().out
    )

    assert _lifts_export(site, "--lift-depth", "lift_a=1.4") == ExitCode.OK
    assert _building(site)["lifts"]["lift_a"]["depth"] == 1.4


def test_stale_lift_is_reported_not_deleted(site, capsys):
    path = site / "twofloor.building.yaml"
    path.write_text(
        path.read_text().replace("lifts: {}", "lifts:\n  old_lift: {depth: 1, width: 1}")
    )

    assert _lifts_export(site) == ExitCode.OK

    assert set(_building(site)["lifts"]) == {"lift_a", "old_lift"}
    assert "old_lift is in the building file but on no robot map (kept)" in capsys.readouterr().out


def _edit_info(site, level, change):
    path = site / f"{level}.info"
    info = json.loads(path.read_text())
    change(info)
    path.write_text(json.dumps(info))


def _drop_cabin(info):
    info["markers"]["loc_point"] = [
        m for m in info["markers"]["loc_point"] if m["name"] != "lift_a"
    ]


def _move_cabin(info):
    info["markers"]["loc_point"][0]["coords"] = "210 300"


def _foreign_vertex(site):
    path = site / "twofloor.building.yaml"
    path.write_text(
        path.read_text().replace(
            '[200, 100, 0, ""]', '[200, 100, 0, ""]\n      - [205, 305, 0, x]', 1
        )
    )


@pytest.mark.parametrize(
    ("change", "error_text"),
    [
        (lambda site: _edit_info(site, "L3", _drop_cabin), "has no cabin point"),
        (lambda site: _edit_info(site, "L3", _move_cabin), "do not line up"),
        (_foreign_vertex, "inside the car of lift_a"),
        (
            lambda site: _edit_info(
                site,
                "L1",
                lambda info: info["lift_lobbies"][0]["lifts"][0].update(carriage_width=0),
            ),
            "has no car width",
        ),
    ],
    ids=["missing car point", "misaligned cars", "foreign vertex in car", "no car width"],
)
def test_lift_data_errors_stop_before_any_write(site, capsys, change, error_text):
    change(site)
    before = (site / "twofloor.building.yaml").read_text()

    assert _lifts_export(site, "--all") == ExitCode.LIFT_DATA

    assert error_text in capsys.readouterr().err
    assert (site / "twofloor.building.yaml").read_text() == before


@pytest.mark.parametrize(
    "extra",
    [["--from-info", "L1"], ["--lift-depth", "lift_a"], ["--lift-depth", "lift_a=deep"]],
    ids=["source without level", "depth without value", "depth not a number"],
)
def test_usage_errors(site, extra):
    assert _lifts_export(site, *extra) == ExitCode.USAGE


def test_level_not_in_the_building(site):
    assert _lifts_export(site, "--from-info", f"L9={site / 'L1.info'}") == ExitCode.LOCAL_INPUT


def test_lift_errors_are_lift_data_errors():
    for error in [
        errors.MissingCabinPointError,
        errors.MisalignedCabinError,
        errors.ForeignVertexInCabinError,
        errors.MissingLiftSizeError,
    ]:
        assert issubclass(error, errors.LiftDataError)


ROBOT_ID = "37defd58-0f13-4e68-99be-f7987b3a6900"
ONE_ENTRY_PER_LEVEL = "robots:\n" + "".join(
    f"  r{level}:\n    rmf_config:\n      start:\n        rmf_level: {level}\n"
    f"        robot_map_name: map_{level}\n    robot_config:\n      uuid: {ROBOT_ID}\n"
    for level in ["L1", "L3"]
)
ONE_ROBOT_WITH_ROBOT_MAPS = (
    "robots:\n  R3SCP-B01284:\n    rmf_config:\n      start:\n        rmf_level: L1\n"
    "        robot_map_name: map_L1\n      robot_maps:\n        L3: {name: map_L3, level: L3}\n"
    f"    robot_config:\n      uuid: {ROBOT_ID}\n"
)


@pytest.mark.parametrize(
    "robots",
    [ONE_ENTRY_PER_LEVEL, ONE_ROBOT_WITH_ROBOT_MAPS],
    ids=["entry per level", "robot_maps"],
)
def test_lifts_export_from_the_api(site, monkeypatch, robots):
    robot_id = ROBOT_ID
    config = site / "config_r3.yaml"
    config.write_text(config.read_text() + robots)
    frame = {"resolution": 0.05, "originX": 0, "originY": 0, "width": 400, "height": 400}

    def point(x, y):
        return {"x": x * 0.05, "y": (400 - y) * 0.05}

    with ScriptedServer() as server:
        monkeypatch.setenv("LIONSBOT_API_BASE_URL", server.url)
        monkeypatch.setenv("LIONSBOT_USER", "u")
        monkeypatch.setenv("LIONSBOT_PASSWORD", "p")
        server.script(
            "/openapi/v2/edge/auth/login",
            *[Reply(200, json.loads((TESTDATA / "api" / "login_response.json").read_text()))] * 2,
        )
        for index, level in enumerate(["L1", "L3"]):
            map_id = f"00000000-0000-4000-8000-00000000000{index}"
            server.script(
                f"/openapi/v2/robot/{robot_id}/map",
                Reply(
                    200,
                    {
                        "currentMapId": map_id,
                        "workSiteMaps": [
                            {"id": map_id, "name": f"map_{level}", "level": level, "frame": frame}
                        ],
                    },
                ),
            )
            server.script(
                f"/openapi/v2/robot/map/{map_id}/markers",
                Reply(
                    200,
                    [
                        {
                            "type": "loc_point",
                            "name": "lift_a",
                            "angle": 0,
                            "point": point(200, 300),
                        },
                        {
                            "type": "loc_point",
                            "name": "lift_a_door",
                            "angle": 0,
                            "point": point(200, 280),
                        },
                    ],
                ),
            )
            server.script(f"/openapi/v2/robot/map/{map_id}/zones", Reply(200, []))
            server.script(
                f"/openapi/v2/robot/lift/lift-lobbies/{map_id}",
                Reply(
                    200,
                    [
                        {
                            "lobbyName": "lobby_1",
                            "point": point(200, 240),
                            "lifts": [
                                {
                                    "liftName": "lift_a",
                                    "point": point(200, 260),
                                    "doorWidth": 0.9,
                                    "carriageWidth": 1.6,
                                }
                            ],
                        }
                    ],
                ),
            )

        result = lb_rmf_setup.main(
            [
                "lifts",
                "export",
                "--building",
                str(site / "twofloor.building.yaml"),
                "--config",
                str(config),
                "--level",
                "L1",
                "--level",
                "L3",
                "--all",
            ]
        )

    assert result == ExitCode.OK
    named = _named(_building(site)["levels"]["L1"])
    assert named["lobby_1_L1"][:3] == pytest.approx([200, 240, 0])
    lift = _building(site)["lifts"]["lift_a"]
    assert (lift["x"], lift["y"]) == pytest.approx((200, 300))
    assert lift["depth"] == pytest.approx(2.0)


def test_lifts_export_needs_a_level(site):
    assert (
        lb_rmf_setup.main(
            [
                "lifts",
                "export",
                "--building",
                str(site / "twofloor.building.yaml"),
                "--config",
                str(site / "config_r3.yaml"),
            ]
        )
        == ExitCode.USAGE
    )


def test_points_of_the_robot_map_are_not_listed_as_missing(site, capsys):
    path = site / "twofloor.building.yaml"
    data = yaml.safe_load(path.read_text())
    data["levels"]["L1"]["vertices"].append([10, 10, 0, "dock_1", {"is_charger": [4, True]}])
    path.write_text(yaml.safe_dump(data))

    assert _lifts_export(site) == ExitCode.OK

    assert "not on this robot map" not in capsys.readouterr().out
