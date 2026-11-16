import json
import shutil

import pytest
import yaml
from fixtures import OFFICE_NEW

import lb_rmf_setup
from lbrmf.constants import ExitCode

ROBOT_FLEETS = [("r3scp", "r3"), ("r5", "r5")]


@pytest.fixture
def site(tmp_path):
    for name in [
        "rmf_office_new.building.yaml",
        "dock_summary.yaml",
        "config_r3scp.yaml",
        "config_r5.yaml",
        "r3scp.info",
        "r5.info",
    ]:
        shutil.copy(OFFICE_NEW / name, tmp_path / name)
    return tmp_path


def _export(site, robot, *extra):
    return lb_rmf_setup.main(
        [
            "export",
            "--building",
            str(site / "rmf_office_new.building.yaml"),
            "--config",
            str(site / f"config_{robot}.yaml"),
            "--level",
            "L8",
            "--from-info",
            str(site / f"{robot}.info"),
            *extra,
        ]
    )


def _snapshot(site):
    return {path.name: path.read_bytes() for path in sorted(site.iterdir())}


@pytest.mark.parametrize(("robot", "fleet"), ROBOT_FLEETS)
def test_matches_the_old_script_except_the_intended_differences(site, robot, fleet):
    old = json.loads((OFFICE_NEW / f"old_script_{robot}.json").read_text())
    config_before = (site / f"config_{robot}.yaml").read_bytes()

    assert _export(site, robot, "--update-positions") == ExitCode.OK

    building = yaml.safe_load((site / "rmf_office_new.building.yaml").read_text())
    new_vertices = building["levels"]["L8"]["vertices"]
    assert len(new_vertices) == len(old["vertices"])
    for new, before in zip(new_vertices, old["vertices"], strict=True):
        assert new[3:] == before[3:]
        assert new[:2] == pytest.approx(before[:2], abs=1e-9)
        is_dock = len(new) > 4 and "is_charger" in new[4]
        if not is_dock:
            assert new[2] == before[2]
    docks = [vertex for vertex in new_vertices if len(vertex) > 4 and "is_charger" in vertex[4]]
    assert all(vertex[2] == 0 for vertex in docks)
    summary = yaml.safe_load((site / "dock_summary.yaml").read_text())[fleet]
    for name, entry in old["dock_summary"].items():
        assert summary[name]["path"] == [pytest.approx(point, abs=1e-9) for point in entry["path"]]
        assert {k: v for k, v in summary[name].items() if k != "path"} == {
            k: v for k, v in entry.items() if k != "path"
        }
    assert (site / f"config_{robot}.yaml").read_bytes() == config_before


def test_default_run_keeps_positions_and_changes_nothing_on_the_example(site, capsys):
    before = _snapshot(site)

    assert _export(site, "r3scp") == ExitCode.OK

    assert _snapshot(site) == before
    out = capsys.readouterr().out
    assert "level L8: 0 new, 0 changed, 3 unchanged" in out
    assert "kept position: cZone_356 would move" in out


@pytest.mark.parametrize("mode", ["--dry-run", "--check"])
def test_dry_run_and_check_write_nothing(site, capsys, mode):
    before = _snapshot(site)

    result = _export(site, "r3scp", "--update-positions", mode)

    assert _snapshot(site) == before
    assert result == (ExitCode.DRIFT if mode == "--check" else ExitCode.OK)
    out = capsys.readouterr().out
    assert f"--- {site / 'rmf_office_new.building.yaml'}" in out
    assert "+      - [" in out


def test_check_passes_after_a_write(site):
    assert _export(site, "r5", "--update-positions") == ExitCode.OK

    assert _export(site, "r5", "--update-positions", "--check") == ExitCode.OK


def test_writes_a_new_dock_summary_next_to_the_building_file(site):
    (site / "dock_summary.yaml").unlink()

    assert _export(site, "r5") == ExitCode.OK

    assert set(yaml.safe_load((site / "dock_summary.yaml").read_text())) == {"r5"}


def test_dock_summary_flag(site):
    target = site / "other_summary.yaml"

    assert _export(site, "r5", "--dock-summary", str(target)) == ExitCode.OK

    assert set(yaml.safe_load(target.read_text())) == {"r5"}


def test_from_geometry(site):
    geometry = {
        "schema": 1,
        "map_name": "rmf_office_new",
        "frame": None,
        "markers": [{"name": "dock_new", "x": 337, "y": 796, "angle": 0.0, "dock": True}],
        "areas": [],
    }
    (site / "map.geometry.json").write_text(json.dumps(geometry))

    result = lb_rmf_setup.main(
        [
            "export",
            "--building",
            str(site / "rmf_office_new.building.yaml"),
            "--config",
            str(site / "config_r3scp.yaml"),
            "--level",
            "L8",
            "--from-geometry",
            str(site / "map.geometry.json"),
        ]
    )

    assert result == ExitCode.OK
    assert "dock_new" in (site / "rmf_office_new.building.yaml").read_text()


@pytest.mark.parametrize(
    ("change", "exit_code"),
    [
        (lambda site: (site / "r3scp.info").unlink(), ExitCode.LOCAL_INPUT),
        (lambda site: (site / "r3scp.info").write_text("{"), ExitCode.LOCAL_INPUT),
        (
            lambda site: (site / "config_r3scp.yaml").write_text("rmf_fleet: {name: r3}\n"),
            ExitCode.LOCAL_INPUT,
        ),
        (
            lambda site: (site / "rmf_office_new.building.yaml").write_text(
                "name: x\nlevels: {}\n"
            ),
            ExitCode.LOCAL_INPUT,
        ),
        (
            lambda site: (site / "rmf_office_new.building.yaml").write_text(
                (site / "rmf_office_new.building.yaml")
                .read_text()
                .replace("measurements:\n      - [0, 1, {distance: [3, 3.7599999999999998]}]\n", "")
            ),
            ExitCode.LOCAL_INPUT,
        ),
        (
            lambda site: (site / "r3scp.info").write_text(
                json.dumps(
                    {
                        "markers": {
                            "dock": [{"name": "a", "coords": "1 2"}],
                            "poi": [{"name": "a", "coords": "3 4"}],
                        }
                    }
                )
            ),
            ExitCode.MAP_DATA,
        ),
    ],
    ids=[
        "missing map file",
        "malformed map file",
        "level not in config",
        "level not in building",
        "no floorplan scale",
        "name collision",
    ],
)
def test_error_exit_codes(site, change, exit_code):
    change(site)
    before = _snapshot(site)

    assert _export(site, "r3scp") == exit_code
    assert _snapshot(site) == before


@pytest.mark.parametrize(
    "extra",
    [["--dry-run", "--check"], ["--from-geometry", "x.json"]],
    ids=["dry run and check", "two sources"],
)
def test_usage_errors(site, extra):
    with pytest.raises(SystemExit) as exited:
        _export(site, "r3scp", *extra)

    assert exited.value.code == ExitCode.USAGE


def test_fills_the_charger_placeholder_in_the_config(site, capsys):
    config = site / "config_r3scp.yaml"
    config.write_text(
        config.read_text().replace("waypoint: refuel_891", "waypoint: <charger_waypoint>")
    )

    assert _export(site, "r3scp") == ExitCode.OK

    robot = yaml.safe_load(config.read_text())["robots"]["R3SCP-B02357"]
    assert robot["rmf_config"]["charger"]["waypoint"] == "refuel_891"
    assert "charger: R3SCP-B02357 goes home to refuel_891" in capsys.readouterr().out
