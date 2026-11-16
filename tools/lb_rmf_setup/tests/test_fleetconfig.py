import pytest
from fixtures import OFFICE_NEW, TESTDATA

from lbrmf import errors
from lbrmf.fleetconfig import parse_fleet_config
from lbrmf.frame import LevelTransform, TransformValues


def test_office_new_config():
    config = parse_fleet_config((OFFICE_NEW / "config_r3scp.yaml").read_text())

    assert config.fleet_name == "r3"
    assert config.level_transform("L8") == (
        TransformValues(178.947, 0.0514713, 46.278, 75.834),
        LevelTransform(),
    )


def test_level_transform_with_level_shift():
    text = (TESTDATA / "transform_config.yaml").read_text() + "rmf_fleet:\n  name: r3\n"

    _, level = parse_fleet_config(text).level_transform("L5")

    assert level == LevelTransform(scale=0.0604, tx_pixels=1002.4, ty_pixels=26.2)


@pytest.mark.parametrize(
    ("text", "error"),
    [
        ("- a\n", errors.MalformedConfigError),
        ("rmf_fleet: {}\n", errors.MalformedConfigError),
        ("rmf_fleet: {name: r3}\nmap_transform: [L8]\n", errors.MalformedConfigError),
        ("rmf_fleet: {name: [\n", errors.MalformedConfigError),
    ],
    ids=["not a mapping", "no fleet name", "map_transform not a mapping", "not yaml"],
)
def test_malformed_config(text, error):
    with pytest.raises(error):
        parse_fleet_config(text)


@pytest.mark.parametrize(
    "text",
    [
        "rmf_fleet: {name: r3}\n",
        "rmf_fleet: {name: r3}\nmap_transform: {L9: {}}\n",
        "rmf_fleet: {name: r3}\nmap_transform: {L8: {}}\n",
    ],
    ids=["no map_transform", "other level only", "no transform_values"],
)
def test_level_not_in_config(text):
    with pytest.raises(errors.LevelNotInConfigError):
        parse_fleet_config(text).level_transform("L8")


ROBOT_ID = "37defd58-0f13-4e68-99be-f7987b3a6900"
START = (
    "    rmf_config:\n      start: {rmf_level: L1, robot_map_name: map_1, robot_map_level: '1'}\n"
)


def _config(rmf_extra=""):
    return (
        "rmf_fleet: {name: r3}\nrobots:\n  R3SCP-B01284:\n"
        f"    robot_config: {{uuid: {ROBOT_ID}}}\n{START}{rmf_extra}"
    )


@pytest.mark.parametrize(
    ("extra", "level", "expected"),
    [
        ("", "L1", ("map_1", "1")),
        ("      robot_maps: {L3: {name: map_3, level: '3'}}\n", "L3", ("map_3", "3")),
        ("      robot_maps: {L3: map_3}\n", "L3", ("map_3", "L3")),
        (
            "      robot_map_names: {L3: map_3}\n      robot_map_levels: {L3: '3'}\n",
            "L3",
            ("map_3", "3"),
        ),
        ("      robot_maps: {L3: {name: map_3, level: '3'}}\n", "L1", ("map_1", "1")),
    ],
    ids=["start level", "robot_maps mapping", "robot_maps name only", "legacy keys", "start kept"],
)
def test_robot_map_per_level_like_the_adapter(extra, level, expected):
    robot_map = parse_fleet_config(_config(extra)).robot_map(level)

    assert (robot_map.robot, robot_map.robot_id) == ("R3SCP-B01284", ROBOT_ID)
    assert (robot_map.map_name, robot_map.map_level) == expected


def test_no_robot_on_the_level():
    assert parse_fleet_config(_config()).robot_map("L3") is None
