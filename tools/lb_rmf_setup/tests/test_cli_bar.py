import json
import shutil

import pytest
from apiserver import Reply, ScriptedServer
from fixtures import OFFICE_NEW, TESTDATA, make_png

import lb_rmf_setup
from lbrmf.constants import EnvVar, ExitCode

API_DATA = TESTDATA / "api"
ROBOT_ID = "37defd58-0f13-4e68-99be-f7987b3a6900"
MAP_ID = "5d1f6c1a-8e37-4f0e-9b8e-1f2a3b4c5d6e"
MAPS = f"/openapi/v2/robot/{ROBOT_ID}/map"


def _fixture(name, server):
    return json.loads((API_DATA / name).read_text().replace("{server}", server.url))


@pytest.fixture
def server(monkeypatch):
    with ScriptedServer() as running:
        monkeypatch.setenv(EnvVar.BASE_URL.value, running.url)
        monkeypatch.setenv(EnvVar.USER.value, "ops@example.com")
        monkeypatch.setenv(EnvVar.PASSWORD.value, "pw-value")
        running.script(
            "/openapi/v2/edge/auth/login", Reply(200, _fixture("login_response.json", running))
        )
        yield running


def _script_map(server):
    server.script(MAPS, Reply(200, _fixture("r3scp_maps.json", server)))
    server.script(
        f"/openapi/v2/robot/map/{MAP_ID}/markers",
        Reply(200, _fixture("r3scp_markers.json", server)),
    )
    server.script(
        f"/openapi/v2/robot/map/{MAP_ID}/zones", Reply(200, _fixture("r3scp_zones.json", server))
    )
    server.script(
        f"/openapi/v2/robot/map/{MAP_ID}/markers/homepoi",
        Reply(200, {"type": "dock", "name": "refuel_891", "angle": 0.083}),
    )
    server.script(f"/openapi/v2/robot/lift/lift-lobbies/{MAP_ID}", Reply(200, []))
    server.script("/signed/rmf_office_new.png", Reply(200, make_png(647, 1193)))


def _json_out(capsys):
    return json.loads(capsys.readouterr().out)


def test_json_robots_list_uses_public_words(server, capsys):
    server.script("/openapi/v2/robot", Reply(200, _fixture("robots_page.json", server)))

    assert lb_rmf_setup.main(["--json", "robots", "list"]) == ExitCode.OK

    out = _json_out(capsys)
    assert out["ok"] is True
    assert out["robots"][0]["serial"] == "R3SCP-B02357"
    assert "encoding" not in json.dumps(out)


def test_json_failure_is_one_object_with_the_exit_code(server, monkeypatch, capsys):
    monkeypatch.delenv(EnvVar.BASE_URL.value)

    assert lb_rmf_setup.main(["--json", "auth", "check"]) == ExitCode.LOCAL_INPUT

    out = _json_out(capsys)
    assert out == {
        "ok": False,
        "exit_code": int(ExitCode.LOCAL_INPUT),
        "error": "missing environment: LIONSBOT_API_BASE_URL",
    }


def test_json_export_reports_the_plan_and_writes_nothing_on_dry_run(server, tmp_path, capsys):
    for name in ["rmf_office_new.building.yaml", "dock_summary.yaml", "config_r3scp.yaml"]:
        shutil.copy(OFFICE_NEW / name, tmp_path / name)
    _script_map(server)
    before = (tmp_path / "dock_summary.yaml").read_text()

    result = lb_rmf_setup.main(
        [
            "--json",
            "export",
            "--building",
            str(tmp_path / "rmf_office_new.building.yaml"),
            "--config",
            str(tmp_path / "config_r3scp.yaml"),
            "--level",
            "L8",
            "--robot",
            ROBOT_ID,
            "--map-id",
            MAP_ID,
            "--dry-run",
        ]
    )

    assert result == ExitCode.OK
    out = _json_out(capsys)
    assert out["ok"] is True
    assert out["written"] is False
    assert out["levels"]["L8"]["unchanged"] == ["refuel_891", "location_394", "cZone_356"]
    assert [kept["name"] for kept in out["levels"]["L8"]["kept_positions"]] == [
        "refuel_891",
        "location_394",
        "cZone_356",
    ]
    assert out["files_to_change"] == [str(tmp_path / "dock_summary.yaml")]
    assert (tmp_path / "dock_summary.yaml").read_text() == before


@pytest.mark.parametrize("overwrite", [False, True])
def test_maps_fetch_never_overwrites_without_the_flag(server, tmp_path, capsys, overwrite):
    _script_map(server)
    (tmp_path / "rmf_office_new.png").write_bytes(b"mine")
    argv = ["maps", "fetch", "--robot", ROBOT_ID, "--map-id", MAP_ID, "--out", str(tmp_path)]

    result = lb_rmf_setup.main([*argv, "--overwrite"] if overwrite else argv)

    if overwrite:
        assert result == ExitCode.OK
        assert (tmp_path / "rmf_office_new.png").read_bytes() == make_png(647, 1193)
    else:
        assert result == ExitCode.LOCAL_INPUT
        assert (tmp_path / "rmf_office_new.png").read_bytes() == b"mine"
        assert "--overwrite" in capsys.readouterr().err
        assert not (tmp_path / "rmf_office_new.yaml").exists()


def test_api_format_errors_tell_the_user_what_to_do(server, capsys):
    server.script(MAPS, Reply(200, {"unexpected": []}))

    assert lb_rmf_setup.main(["maps", "list", "--robot", ROBOT_ID]) == ExitCode.API

    err = capsys.readouterr().err
    assert "update the setup script" in err
    assert "workSiteMaps" not in err


def test_help_uses_public_words(capsys):
    with pytest.raises(SystemExit):
        lb_rmf_setup.main(["maps", "list", "--help"])

    out = capsys.readouterr().out
    assert "serial" in out
    assert "encoding" not in out
