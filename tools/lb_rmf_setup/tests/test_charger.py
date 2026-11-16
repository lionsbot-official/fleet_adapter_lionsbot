import pytest
import yaml

from lbrmf.charger import fill_chargers
from lbrmf.frame import Point
from lbrmf.geometry import MapGeometry, Marker

CONFIG = """\
rmf_fleet:
  name: lb_fleet
robots:
  R3VAC-B00218:
    rmf_config:
      # This must name a charger waypoint in the selected RMF navigation graph.
      charger:
        waypoint: <charger_waypoint>
      start:
        rmf_level: L8
        robot_map_name: rmf8
        robot_map_level: L8
"""

DOCK = Marker("charging_793", Point(388, 474), 2.17, dock=True)
OTHER_DOCK = Marker("charging_800", Point(100, 100), 0.0, dock=True)
POINT = Marker("location_303", Point(380, 462), -0.97, dock=False)


def _map(*markers, home_dock=None, name="rmf8"):
    return MapGeometry(name, None, tuple(markers), (), home_dock=home_dock)


def _waypoint(text):
    return yaml.safe_load(text)["robots"]["R3VAC-B00218"]["rmf_config"]["charger"]["waypoint"]


def test_fills_the_placeholder_with_the_only_dock():
    result = fill_chargers(CONFIG, "L8", _map(DOCK, POINT))

    assert _waypoint(result.text) == "charging_793"
    assert result.filled == {"R3VAC-B00218": "charging_793"}
    assert result.notes == []
    assert result.text == CONFIG.replace("<charger_waypoint>", "charging_793")


def test_the_home_dock_wins_when_the_map_has_several():
    result = fill_chargers(CONFIG, "L8", _map(OTHER_DOCK, DOCK, home_dock="charging_793"))

    assert _waypoint(result.text) == "charging_793"


def test_several_docks_and_no_home_dock_leaves_a_note():
    result = fill_chargers(CONFIG, "L8", _map(OTHER_DOCK, DOCK))

    assert result.text == CONFIG
    assert result.notes == [
        "set robots.R3VAC-B00218.rmf_config.charger.waypoint to one of: charging_800, charging_793"
    ]


def test_a_map_without_a_dock_leaves_a_note():
    result = fill_chargers(CONFIG, "L8", _map(POINT))

    assert result.text == CONFIG
    assert result.notes == [
        "the robot map rmf8 has no dock for robots.R3VAC-B00218.rmf_config.charger.waypoint"
    ]


@pytest.mark.parametrize(
    ("level", "geometry"),
    [
        ("L9", _map(DOCK)),
        ("L8", _map(DOCK, name="another_map")),
    ],
)
def test_other_levels_and_other_maps_change_nothing(level, geometry):
    result = fill_chargers(CONFIG, level, geometry)

    assert result.text == CONFIG
    assert result.filled == {}


def test_a_charger_that_is_set_is_kept():
    text = CONFIG.replace("<charger_waypoint>", "my_charger")

    result = fill_chargers(text, "L8", _map(DOCK))

    assert result.text == text
    assert result.filled == {}


def test_a_config_without_the_charger_key_gets_a_note():
    text = CONFIG.replace("      charger:\n        waypoint: <charger_waypoint>\n", "")

    result = fill_chargers(text, "L8", _map(DOCK))

    assert result.text == text
    assert result.notes == [
        "add robots.R3VAC-B00218.rmf_config.charger.waypoint: charging_793 to the config"
    ]


def test_a_local_map_file_is_the_map_of_its_level():
    result = fill_chargers(CONFIG, "L8", _map(DOCK, name="r3vac"), any_map=True)

    assert _waypoint(result.text) == "charging_793"
