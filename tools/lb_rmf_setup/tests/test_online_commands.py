import json
import shutil

import pytest
import yaml
from apiserver import Reply, ScriptedServer
from fixtures import OFFICE_NEW, TESTDATA, make_png

import lb_rmf_setup
from lbrmf.constants import EnvVar, ExitCode
from lbrmf.inforeader import read_geometry

API_DATA = TESTDATA / "api"
ROBOT_ID = "37defd58-0f13-4e68-99be-f7987b3a6900"
MAP_ID = "5d1f6c1a-8e37-4f0e-9b8e-1f2a3b4c5d6e"
LOGIN = "/openapi/v2/edge/auth/login"
ROBOTS = "/openapi/v2/robot"
MAPS = f"/openapi/v2/robot/{ROBOT_ID}/map"
MARKERS = f"/openapi/v2/robot/map/{MAP_ID}/markers"
ZONES = f"/openapi/v2/robot/map/{MAP_ID}/zones"
HOME_POI = f"/openapi/v2/robot/map/{MAP_ID}/markers/homepoi"
LIFT_LOBBIES = f"/openapi/v2/robot/lift/lift-lobbies/{MAP_ID}"
IMAGE = "/signed/rmf_office_new.png"
NO_WAIT = {"Retry-After": "0"}


def _fixture(name, server):
    return json.loads((API_DATA / name).read_text().replace("{server}", server.url))


@pytest.fixture
def server(monkeypatch):
    with ScriptedServer() as running:
        monkeypatch.setenv(EnvVar.BASE_URL.value, running.url)
        monkeypatch.setenv(EnvVar.USER.value, "ops@example.com")
        monkeypatch.setenv(EnvVar.PASSWORD.value, "pw-value")
        running.script(LOGIN, Reply(200, _fixture("login_response.json", running)))
        yield running


def _script_map(server, image=None):
    server.script(MAPS, Reply(200, _fixture("r3scp_maps.json", server)))
    server.script(MARKERS, Reply(200, _fixture("r3scp_markers.json", server)))
    server.script(ZONES, Reply(200, _fixture("r3scp_zones.json", server)))
    server.script(HOME_POI, Reply(200, {"type": "dock", "name": "refuel_891", "angle": 0.083}))
    server.script(LIFT_LOBBIES, Reply(200, []))
    if image is not None:
        server.script(IMAGE, Reply(200, image))


@pytest.fixture
def site(tmp_path):
    for name in ["rmf_office_new.building.yaml", "dock_summary.yaml", "config_r3scp.yaml"]:
        shutil.copy(OFFICE_NEW / name, tmp_path / name)
    return tmp_path


def _export(site, *extra):
    return lb_rmf_setup.main(
        [
            "export",
            "--building",
            str(site / "rmf_office_new.building.yaml"),
            "--config",
            str(site / "config_r3scp.yaml"),
            "--level",
            "L8",
            *extra,
        ]
    )


def test_auth_check_prints_no_secret(server, capsys):
    assert lb_rmf_setup.main(["auth", "check"]) == ExitCode.OK

    out = capsys.readouterr().out
    assert "method:     password" in out
    assert "login:      ops@example.com" in out
    assert "account id: 0b6d2c1e-5f0a-4a59-9a33-3c1d1b0e7a10" in out
    assert "password-session-1" not in out
    assert "pw-value" not in out


@pytest.mark.parametrize("output", ["table", "json"])
def test_robots_list(server, capsys, output):
    server.script(ROBOTS, Reply(200, _fixture("robots_page.json", server)))

    argv = ["--json", "robots", "list"] if output == "json" else ["robots", "list"]

    assert lb_rmf_setup.main(argv) == ExitCode.OK

    out = capsys.readouterr().out
    assert "R3SCP-B02357" in out
    if output == "json":
        assert json.loads(out)["robots"][0]["id"] == ROBOT_ID


@pytest.mark.parametrize("output", ["table", "json"])
def test_maps_list_has_no_signed_links(server, capsys, output):
    server.script(ROBOTS, Reply(200, _fixture("robots_page.json", server)))
    server.script(MAPS, Reply(200, _fixture("r3scp_maps.json", server)))

    argv = ["maps", "list", "--robot", "R3SCP-B02357"]
    result = lb_rmf_setup.main(["--json", *argv] if output == "json" else argv)

    assert result == ExitCode.OK
    out = capsys.readouterr().out
    assert "rmf_office_new" in out
    assert "Signature" not in out


def test_maps_fetch_writes_three_files(server, tmp_path):
    _script_map(server, make_png(647, 1193))
    out = tmp_path / "maps"

    result = lb_rmf_setup.main(
        ["maps", "fetch", "--robot", ROBOT_ID, "--map-id", MAP_ID, "--out", str(out)]
    )

    assert result == ExitCode.OK
    assert sorted(path.name for path in out.iterdir()) == [
        "rmf_office_new.geometry.json",
        "rmf_office_new.png",
        "rmf_office_new.yaml",
    ]
    assert yaml.safe_load((out / "rmf_office_new.yaml").read_text()) == {
        "image": "rmf_office_new.png",
        "mode": "trinary",
        "resolution": 0.05,
        "origin": [-17.56, -20.93, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.195,
    }
    geometry = read_geometry((out / "rmf_office_new.geometry.json").read_text())
    assert [marker.name for marker in geometry.markers] == ["refuel_891", "location_394"]


def test_maps_fetch_keeps_the_lifts_of_the_map(server, tmp_path):
    server.script(MAPS, Reply(200, _fixture("r3scp_maps.json", server)))
    server.script(MARKERS, Reply(200, _fixture("r3scp_markers.json", server)))
    server.script(ZONES, Reply(200, _fixture("r3scp_zones.json", server)))
    server.script(HOME_POI, Reply(404, {}))
    server.script(IMAGE, Reply(200, make_png(647, 1193)))
    lift = {
        "liftName": "lift_a",
        "point": {"x": 1.0, "y": 2.0},
        "doorWidth": 0.9,
        "carriageWidth": 1.6,
    }
    server.script(
        LIFT_LOBBIES,
        Reply(200, [{"lobbyName": "lobby_1", "point": {"x": 1.0, "y": 1.0}, "lifts": [lift]}]),
    )

    assert (
        lb_rmf_setup.main(
            ["maps", "fetch", "--robot", ROBOT_ID, "--map-id", MAP_ID, "--out", str(tmp_path)]
        )
        == ExitCode.OK
    )

    geometry = read_geometry((tmp_path / "rmf_office_new.geometry.json").read_text())
    assert [(lift.name, lift.car_width) for lift in geometry.lifts] == [("lift_a", 1.6)]
    assert [lobby.name for lobby in geometry.lobbies] == ["lobby_1"]


def test_export_from_the_api_uses_the_config_robot_map(server, site, capsys):
    maps = _fixture("r3scp_maps.json", server)
    maps["workSiteMaps"][0].update({"name": "rmf_test", "level": "L8"})
    server.script(MAPS, Reply(200, maps))
    server.script(MARKERS, Reply(200, _fixture("r3scp_markers.json", server)))
    server.script(ZONES, Reply(200, _fixture("r3scp_zones.json", server)))
    server.script(LIFT_LOBBIES, Reply(200, []))
    server.script(IMAGE, Reply(200, make_png(647, 1193)))
    layer = site / "layer.png"

    assert _export(site, "--layer-png", str(layer)) == ExitCode.OK

    assert "level L8: 0 new, 0 changed, 3 unchanged" in capsys.readouterr().out
    assert layer.read_bytes() == make_png(647, 1193)


def test_export_with_map_name_and_level(server, site):
    _script_map(server)

    result = _export(
        site, "--robot", ROBOT_ID, "--map-name", "rmf_office_new", "--map-level", "8", "--dry-run"
    )

    assert result == ExitCode.OK


def test_env_file_fills_unset_variables(server, tmp_path, monkeypatch, capsys):
    monkeypatch.delenv(EnvVar.PASSWORD.value)
    env_file = tmp_path / ".env"
    env_file.write_text(
        "# adapter settings\nexport LIONSBOT_PASSWORD='pw-value'\nLIONSBOT_USER=other@example.com\n"
    )

    assert lb_rmf_setup.main(["--env-file", str(env_file), "auth", "check"]) == ExitCode.OK
    assert "login:      ops@example.com" in capsys.readouterr().out


def test_verbose_prints_calls_without_secrets(server, capsys):
    lb_rmf_setup.main(["--verbose", "auth", "check"])

    err = capsys.readouterr().err
    assert f"POST 127.0.0.1 {LOGIN} -> 200" in err
    assert "pw-value" not in err


@pytest.mark.parametrize(
    ("setup", "argv", "exit_code"),
    [
        (lambda server, monkeypatch: None, ["auth", "check"], ExitCode.OK),
        (
            lambda server, monkeypatch: None,
            ["maps", "fetch", "--robot", ROBOT_ID, "--out", "x"],
            ExitCode.USAGE,
        ),
        (
            lambda server, monkeypatch: monkeypatch.delenv(EnvVar.BASE_URL.value),
            ["auth", "check"],
            ExitCode.LOCAL_INPUT,
        ),
        (
            lambda server, monkeypatch: server.replies[LOGIN].appendleft(Reply(401)),
            ["auth", "check"],
            ExitCode.AUTH,
        ),
        (
            lambda server, monkeypatch: server.script(MAPS, Reply(403)),
            ["maps", "list", "--robot", ROBOT_ID],
            ExitCode.ACCESS,
        ),
        (
            lambda server, monkeypatch: server.script(MAPS, *[Reply(429, headers=NO_WAIT)] * 4),
            ["maps", "list", "--robot", ROBOT_ID],
            ExitCode.API,
        ),
        (
            lambda server, monkeypatch: _script_map(server),
            [
                "maps",
                "fetch",
                "--robot",
                ROBOT_ID,
                "--map-name",
                "old_map",
                "--map-level",
                "1",
                "--out",
                "x",
            ],
            ExitCode.MAP_DATA,
        ),
    ],
    ids=["ok", "usage", "local input", "auth", "access", "api", "map data"],
)
def test_exit_codes(server, monkeypatch, tmp_path, setup, argv, exit_code):
    monkeypatch.chdir(tmp_path)
    setup(server, monkeypatch)

    assert lb_rmf_setup.main(argv) == exit_code


@pytest.mark.parametrize(
    "extra",
    [
        ["--robot", ROBOT_ID, "--map-name", "x"],
        ["--robot", ROBOT_ID, "--map-id", MAP_ID, "--map-name", "x", "--map-level", "1"],
        ["--from-info", "x.info", "--layer-png", "x.png"],
    ],
    ids=["name without level", "id and name", "layer png from a local file"],
)
def test_export_usage_errors(server, site, extra):
    (site / "x.info").write_text("{}")
    extra = [str(site / v) if v.endswith((".info", ".png")) else v for v in extra]

    assert _export(site, *extra) == ExitCode.USAGE


def test_export_without_a_robot_on_the_level(server, site):
    config = site / "config_r3scp.yaml"
    config.write_text(config.read_text().replace("rmf_level: L8", "rmf_level: L9"))

    assert _export(site) == ExitCode.USAGE


def test_map_not_found_lists_the_robot_maps(server, site, capsys):
    _script_map(server)

    assert _export(site, "--robot", ROBOT_ID, "--map-id", "nope") == ExitCode.ACCESS
    assert "rmf_office_new (level 8)" in capsys.readouterr().err


def test_export_prints_skipped_items(server, site, capsys):
    server.script(MAPS, Reply(200, _fixture("r3scp_maps.json", server)))
    markers = [*_fixture("r3scp_markers.json", server), {"type": "poi", "name": "lost_poi"}]
    server.script(MARKERS, Reply(200, markers))
    server.script(ZONES, Reply(200, _fixture("r3scp_zones.json", server)))
    server.script(LIFT_LOBBIES, Reply(200, []))

    result = _export(site, "--robot", ROBOT_ID, "--map-id", MAP_ID, "--dry-run")

    assert result == ExitCode.OK
    assert "warning: marker lost_poi has no point in metres; skipped" in capsys.readouterr().out


def test_export_from_the_api_skips_lift_points(server, site, capsys):
    server.script(MAPS, Reply(200, _fixture("r3scp_maps.json", server)))
    server.script(MARKERS, Reply(200, _fixture("r3scp_markers.json", server)))
    server.script(ZONES, Reply(200, _fixture("r3scp_zones.json", server)))
    server.script(
        LIFT_LOBBIES, Reply(200, [{"lobbyName": "lobby", "lifts": [{"liftName": "location_394"}]}])
    )

    assert _export(site, "--robot", ROBOT_ID, "--map-id", MAP_ID, "--dry-run") == ExitCode.OK

    assert "skipped lift points (lifts export places them): location_394" in (
        capsys.readouterr().out
    )


def test_export_of_a_map_that_is_not_synced_names_the_fix(server, site, capsys):
    server.script(MAPS, Reply(200, _fixture("r3scp_maps.json", server)))
    server.script(MARKERS, Reply(200, _fixture("r3scp_markers.json", server)))
    server.script(ZONES, Reply(200, _fixture("r3scp_zones.json", server)))
    server.script(LIFT_LOBBIES, Reply(400, {"errorCode": "RSE30153"}))

    result = _export(site, "--robot", ROBOT_ID, "--map-id", MAP_ID, "--dry-run")

    assert result == ExitCode.MAP_DATA
    assert (
        "map rmf_office_new is not synced to the robot yet: sync it in the app, then run again"
        in capsys.readouterr().err
    )


def _init(tmp_path, *extra):
    adapter = tmp_path / "fleet_adapter"
    (adapter / "configs").mkdir(parents=True, exist_ok=True)
    template = TESTDATA.parents[2] / "fleet_adapter" / "configs" / "config_template.yaml"
    shutil.copy(template, adapter / "configs" / "config_template.yaml")
    env = tmp_path / ".env"
    env.write_text("FOLDER_NAME=office_new\nFLEET_1_CONFIG=config_r5.yaml\n")
    code = lb_rmf_setup.main(
        [
            "--env-file",
            str(env),
            "init",
            "--robot",
            "R3SCP-B02357",
            "--building",
            "site",
            "--adapter-dir",
            str(adapter),
            *extra,
        ]
    )
    return code, adapter, env


def test_init_writes_the_config_and_the_env(server, tmp_path, capsys):
    server.script(ROBOTS, Reply(200, _fixture("robots_page.json", server)))
    server.script(MAPS, Reply(200, _fixture("r3scp_maps.json", server)))

    code, adapter, env = _init(tmp_path)

    assert code == ExitCode.OK
    config = yaml.safe_load((adapter / "configs" / "site" / "config_r3scrubpro.yaml").read_text())
    robot = config["robots"]["R3SCP-B02357"]
    assert robot["robot_config"]["uuid"] == ROBOT_ID
    # The robot has two maps, so init takes the one it uses now.
    assert robot["rmf_config"]["start"]["robot_map_name"] == "rmf_office_new"
    assert robot["rmf_config"]["start"]["rmf_level"] == "8"
    assert (adapter / "maps" / "site").is_dir()
    assert env.read_text() == (
        "FOLDER_NAME=site\nFLEET_1_CONFIG=config_r3scrubpro.yaml\n"
        "FLEET_1_NAV_GRAPH=0.yaml\nRMF_FLOOR_NAME=8\n"
    )
    assert "next: download the robot map image with maps fetch" in capsys.readouterr().out


def test_init_never_replaces_a_config_without_the_flag(server, tmp_path):
    server.script(ROBOTS, Reply(200, _fixture("robots_page.json", server)))
    server.script(MAPS, Reply(200, _fixture("r3scp_maps.json", server)))
    target = tmp_path / "fleet_adapter" / "configs" / "site" / "config_r3scrubpro.yaml"
    target.parent.mkdir(parents=True)
    target.write_text("keep me\n")

    code, _, _ = _init(tmp_path, "--level", "L8")

    assert code == ExitCode.LOCAL_INPUT
    assert target.read_text() == "keep me\n"


def test_init_with_an_unknown_robot(server, tmp_path):
    server.script(ROBOTS, Reply(200, _fixture("robots_page.json", server)))

    code, _, _ = _init(tmp_path, "--robot", "NOPE")

    assert code == ExitCode.USAGE
