import shutil

import pytest
import yaml
from fixtures import TESTDATA

import lb_rmf_setup
from lbrmf.constants import ExitCode

TWOFLOOR = TESTDATA / "twofloor"


@pytest.fixture
def site(tmp_path):
    for path in TWOFLOOR.iterdir():
        shutil.copy(path, tmp_path / path.name)
    return tmp_path


def _export(site, level, *extra):
    return lb_rmf_setup.main(
        [
            "export",
            "--building",
            str(site / "twofloor.building.yaml"),
            "--config",
            str(site / "config_r3.yaml"),
            "--level",
            level,
            "--from-info",
            str(site / f"{level}.info"),
            *extra,
        ]
    )


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
            "--all",
            *extra,
        ]
    )


def _names(site, level):
    building = yaml.safe_load((site / "twofloor.building.yaml").read_text())
    return {vertex[3] for vertex in building["levels"][level]["vertices"] if vertex[3]}


def test_export_skips_the_cabin_and_door_points_of_each_lift(site, capsys):
    assert _export(site, "L1") == ExitCode.OK

    assert {"lift_a", "lift_a_door"}.isdisjoint(_names(site, "L1"))
    assert "skipped lift points (lifts export places them): lift_a, lift_a_door" in (
        capsys.readouterr().out
    )


def test_the_documented_order_works_for_two_levels(site):
    assert _export(site, "L1") == ExitCode.OK
    assert _export(site, "L3") == ExitCode.OK

    assert _lifts_export(site) == ExitCode.OK

    assert _export(site, "L1", "--check") == ExitCode.OK
    assert _export(site, "L3", "--check") == ExitCode.OK
    assert _lifts_export(site, "--check") == ExitCode.OK
    assert {"lift_a_L1_cabin", "lift_a_L1_approach"} <= _names(site, "L1")


def test_a_cabin_vertex_from_an_older_export_names_the_fix(site, capsys):
    path = site / "twofloor.building.yaml"
    path.write_text(
        path.read_text().replace(
            '[200, 100, 0, ""]', '[200, 100, 0, ""]\n      - [200, 300, 0, lift_a]', 1
        )
    )

    assert _lifts_export(site) == ExitCode.LIFT_DATA

    err = capsys.readouterr().err
    assert "vertex lift_a on L1 is the lift's own cabin point from an older export" in err
    assert "delete it" in err
