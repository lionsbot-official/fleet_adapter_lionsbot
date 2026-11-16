import pytest
from fixtures import OFFICE_NEW, OFFICE_NEW_BUILDING

import lb_rmf_setup
from lbrmf.building import BuildingFile
from lbrmf.check import check_config, check_level, check_names
from lbrmf.constants import ExitCode

# 100 px is 5 m, so 0.05 m per pixel.
BUILDING = """\
coordinate_system: reference_image
levels:
  L1:
    elevation: 0
    measurements:
      - [0, 1, {{distance: [3, 5]}}]
    walls:
      - [0, 1, {{}}]
    lanes:
{lanes}
    vertices:
      - [0, 0, 0, ""]
      - [100, 0, 0, ""]
      - [50, 50, 0, charging_1, {{is_charger: [4, true], dock_name: [1, charging_1]}}]
      - [50, 70, 0, location_1]
      - [10, 90, 0, ""]
      - [52, 52, 0, ""]
      - [90, 90, 0, ""]
lifts: {{}}
name: test
"""
GOOD_LANES = "      - [3, 2, {}]\n      - [3, 4, {}]"


def _building(lanes=GOOD_LANES):
    return BuildingFile.parse(BUILDING.format(lanes=lanes))


def test_stray_vertices_are_named():
    issues = check_level(_building(), "L1")

    assert issues == [
        "L1: an unnamed vertex at (52, 52) is 0.14 m from charging_1. "
        "A lane that ends on it does not reach charging_1. "
        "Delete it and draw the lane to charging_1.",
        "L1: an unnamed vertex at (90, 90) has no lane. Delete it.",
    ]


def test_a_lane_that_misses_the_dock_is_named():
    lanes = "      - [3, 5, {}]\n      - [3, 4, {}]\n      - [6, 4, {}]"

    issues = check_level(_building(lanes), "L1")

    assert issues == [
        "L1: charging_1 has no lane, so it is not in the nav graph. Connect it with a lane.",
        "L1: an unnamed vertex at (52, 52) is 0.14 m from charging_1. "
        "A lane that ends on it does not reach charging_1. "
        "Delete it and draw the lane to charging_1.",
    ]


def test_wall_and_measurement_vertices_need_no_lane():
    issues = check_level(_building(), "L1")

    assert not any("(0, 0)" in issue or "(100, 0)" in issue for issue in issues)


@pytest.mark.parametrize(
    ("waypoint", "issue"),
    [
        ("charging_1", None),
        ("<charger_waypoint>", "is not set. Run export to fill it in."),
        ("location_1", "is location_1, which is not a dock in the building file."),
    ],
)
def test_config_charger(waypoint, issue):
    config = f"robots:\n  R3:\n    rmf_config:\n      charger:\n        waypoint: {waypoint}\n"

    issues = check_config(_building(), config)

    if issue is None:
        assert issues == []
    else:
        assert issues == [f"config: robots.R3.rmf_config.charger.waypoint {issue}"]


def test_the_example_site_passes(capsys):
    code = lb_rmf_setup.main(
        [
            "check",
            "--building",
            str(OFFICE_NEW_BUILDING),
            "--config",
            str(OFFICE_NEW / "config_r3scp.yaml"),
            "--config",
            str(OFFICE_NEW / "config_r5.yaml"),
        ]
    )

    assert code == ExitCode.OK
    assert "no problems found" in capsys.readouterr().out


def test_problems_exit_with_code_9(tmp_path, capsys):
    path = tmp_path / "test.building.yaml"
    path.write_text(BUILDING.format(lanes=GOOD_LANES))

    code = lb_rmf_setup.main(["check", "--building", str(path), "--level", "L1"])

    assert code == ExitCode.ISSUES
    assert "has no lane. Delete it." in capsys.readouterr().out


def test_a_name_on_two_levels_is_named():
    text = BUILDING.format(lanes=GOOD_LANES)
    level = text[text.index("  L1:\n") : text.index("lifts:")]
    building = BuildingFile.parse(text.replace("lifts:", level.replace("L1:", "L2:") + "lifts:"))

    assert check_names(building) == [
        f"{name} is on more than one level ({levels}). RMF refuses a nav graph that has "
        "the same waypoint name twice. Rename it on the robot map of one level."
        for name, levels in [("charging_1", "L1, L2"), ("location_1", "L1, L2")]
    ]
    assert check_names(_building()) == []
