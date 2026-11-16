import pytest
import yaml
from fixtures import TESTDATA

from lbrmf import errors
from lbrmf.init import InitValues, api_host, render_config, robot_type, update_env

TEMPLATE = (TESTDATA.parents[2] / "fleet_adapter" / "configs" / "config_template.yaml").read_text()
VALUES = InitValues(
    fleet_name="r3vac_fleet",
    robot="R3VAC-B00218",
    robot_id="f01e99e4-d676-4941-a374-afc2f0be6b7d",
    rmf_level="L8",
    map_name="rmf8",
    map_level="L8",
    api_host="au-api.lionsbot.io",
)


def test_fills_every_value_of_a_new_site():
    config = yaml.safe_load(render_config(TEMPLATE, VALUES))

    robot = config["robots"]["R3VAC-B00218"]
    assert config["rmf_fleet"]["name"] == "r3vac_fleet"
    assert config["rmf_fleet"]["fleet_manager"]["prefix"] == "au-api.lionsbot.io"
    assert robot["robot_config"]["uuid"] == VALUES.robot_id
    assert robot["rmf_config"]["start"] == {
        "localization_starting_point": {"x": 0.0, "y": 0.0, "heading": 0.0},
        "rmf_level": "L8",
        "robot_map_name": "rmf8",
        "robot_map_level": "L8",
    }
    assert list(config["map_transform"]) == ["L8"]


def test_leaves_the_transform_and_the_charger_for_export():
    text = render_config(TEMPLATE, VALUES)

    assert "scale: <metres_per_pixel>" in text
    assert "waypoint: <charger_waypoint>" in text
    assert "  <rmf_level>:" not in text


def test_a_changed_template_stops_the_init():
    with pytest.raises(errors.MalformedConfigError):
        render_config(TEMPLATE.replace("uuid: <robot_uuid>", "id: <robot_uuid>"), VALUES)


@pytest.mark.parametrize(
    ("family", "kind", "expected"),
    [
        ("R3", "Vac", "r3vac"),
        ("R3", "Scrub Pro", "r3scrubpro"),
        ("R5", None, "r5"),
        (None, None, "robot"),
    ],
)
def test_robot_type(family, kind, expected):
    assert robot_type(family, kind) == expected


def test_api_host():
    assert api_host("https://eu-api.lionsbot.io/") == "eu-api.lionsbot.io"
    with pytest.raises(errors.UsageError):
        api_host("")


def test_update_env_sets_keys_and_keeps_the_rest():
    text = "# comment\nLIONSBOT_USER=me\nFOLDER_NAME=office_new\n# FLEET_1_CONFIG=old\n"

    after = update_env(text, {"FOLDER_NAME": "site", "FLEET_1_CONFIG": "config_r3vac.yaml"})

    assert after == (
        "# comment\nLIONSBOT_USER=me\nFOLDER_NAME=site\n# FLEET_1_CONFIG=old\n"
        "FLEET_1_CONFIG=config_r3vac.yaml\n"
    )
