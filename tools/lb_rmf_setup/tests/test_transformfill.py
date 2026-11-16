import math

import pytest
import yaml
from fixtures import OFFICE_NEW, OFFICE_NEW_BUILDING

from lbrmf import errors
from lbrmf.building import BuildingFile
from lbrmf.transformfill import fill_level_transform, fill_transform

TEMPLATE_VALUES = """\
    transform_values:
      rotation_degrees: 0.0
      scale: <metres_per_pixel>
      tx_meters: 0.0
      ty_meters: 0.0
"""


def _config_with_template_values(robot_type):
    text = (OFFICE_NEW / f"config_{robot_type}.yaml").read_text()
    start = text.index("    transform_values:\n")
    end = text.index("ty_meters", start)
    end = text.index("\n", end) + 1
    return text[:start] + TEMPLATE_VALUES + text[end:]


def _values(text):
    return yaml.safe_load(text)["map_transform"]["L8"]["transform_values"]


def _building(drawing=None):
    text = OFFICE_NEW_BUILDING.read_text()
    if drawing is not None:
        text = text.replace("filename: rmf_office_new.png", f"filename: {drawing}", 1)
    return BuildingFile.parse(text)


def test_a_floor_plan_that_is_the_robot_map_needs_only_the_scale():
    expected = _values((OFFICE_NEW / "config_r5.yaml").read_text())

    result = fill_transform(
        _config_with_template_values("r5"), _building(), "L8", "rmf_office_new", "r5"
    )

    values = _values(result.text)
    assert values["rotation_degrees"] == 0
    assert values["tx_meters"] == 0
    assert values["ty_meters"] == 0
    assert values["scale"] == pytest.approx(expected["scale"], rel=1e-3)
    assert "floor plan is the robot map" in result.source


def test_the_layer_of_the_robot_map_gives_the_transform():
    expected = _values((OFFICE_NEW / "config_r3scp.yaml").read_text())

    result = fill_transform(
        _config_with_template_values("r3scp"),
        _building(drawing="floor.png"),
        "L8",
        "rmf_office_new",
        "r3scp",
    )

    values = _values(result.text)
    for key, value in expected.items():
        assert values[key] == pytest.approx(value, abs=1e-3)
    assert result.source == "the traffic editor layer r3scp"


def test_a_layer_in_the_folder_of_another_robot_type_is_not_used():
    result = fill_transform(
        _config_with_template_values("r5"),
        _building(drawing="floor.png"),
        "L8",
        "rmf_office_new",
        "r5",
    )

    assert result.text == _config_with_template_values("r5")
    (note,) = result.notes
    assert note.startswith("map_transform.L8 is not set: no image of L8 is the robot map")
    assert "drawing of L8 must be the file rmf_office_new.png (file name only" in note
    assert "README" not in note


def test_values_that_are_set_are_kept():
    text = (OFFICE_NEW / "config_r3scp.yaml").read_text()

    result = fill_transform(text, _building(), "L8", "rmf_office_new", "r3scp")

    assert result == (text, None, [])


def test_only_the_four_values_change():
    before = _config_with_template_values("r5")

    after = fill_transform(before, _building(), "L8", "rmf_office_new", "r5").text

    changed = [
        (old, new)
        for old, new in zip(before.splitlines(), after.splitlines(), strict=True)
        if old != new
    ]
    assert [old.split(":")[0].strip() for old, _ in changed] == [
        "rotation_degrees",
        "scale",
        "tx_meters",
        "ty_meters",
    ]


LEVEL_CONFIG = """\
map_transform:
  L2:
    # lined up by hand
    transform_values: {rotation_degrees: 0.0, scale: 0.05, tx_meters: 0.0, ty_meters: 0.0}
"""
REFERENCE_FIDUCIALS = {"a": (300.0, 400.0), "b": (700.0, 900.0), "c": (100.0, 800.0)}


def _two_levels(zoom=1.0, turn_degrees=0.0, offset=(0.0, 0.0), names="abc", l2_scale=None, **top):
    """L2 shows the same place as L1 at 0.05 m per pixel, zoomed, turned and moved."""
    turn = math.radians(turn_degrees)
    cos, sin = zoom * math.cos(turn), zoom * math.sin(turn)

    def level(fiducials, scale):
        return {
            "fiducials": fiducials,
            "measurements": [[0, 1, {"distance": [3, 5.0]}]] if scale else [],
            "vertices": [[0.0, 0.0, 0, ""], [5.0 / (scale or 1), 0.0, 0, ""]],
        }

    l1 = [[x, y, name] for name, (x, y) in REFERENCE_FIDUCIALS.items()]
    l2 = [
        [cos * x - sin * y + offset[0], sin * x + cos * y + offset[1], name]
        for x, y, name in l1
        if name in names
    ]
    levels = {"L1": level(l1, 0.05), "L2": level(l2, 0.05 / zoom if l2_scale is None else l2_scale)}
    return BuildingFile.parse(yaml.safe_dump({"name": "site", "levels": levels, **top}))


def _level_transform(text):
    return yaml.safe_load(text)["map_transform"]["L2"]["level_transform"]


def test_fiducials_give_the_level_transform_and_the_rest_of_the_file_stays():
    text, source = fill_level_transform(LEVEL_CONFIG, _two_levels(1.25, 0, (250, -120)), "L2")

    assert _level_transform(text) == pytest.approx(
        {"scale": 0.04, "tx_pixels": 250, "ty_pixels": -120}
    )
    assert source == "3 fiducials shared with the reference level L1"
    added = [line for line in text.splitlines() if line not in LEVEL_CONFIG.splitlines()]
    assert added == ["    level_transform: {scale: 0.04, tx_pixels: 250, ty_pixels: -120}"]
    assert [line for line in text.splitlines() if line not in added] == LEVEL_CONFIG.splitlines()


def test_level_transform_goes_into_a_config_in_flow_style():
    config = "map_transform: {L2: {transform_values: {scale: 0.05}}}\n"

    text, _ = fill_level_transform(config, _two_levels(offset=(10, 20)), "L2")

    assert _level_transform(text) == {"scale": 0.05, "tx_pixels": 10, "ty_pixels": 20}
    assert yaml.safe_load(text)["map_transform"]["L2"]["transform_values"] == {"scale": 0.05}


def test_a_small_click_error_on_a_fiducial_is_accepted():
    text, _ = fill_level_transform(LEVEL_CONFIG, _two_levels(1.004, 0.3, (250, -120)), "L2")

    assert _level_transform(text)["scale"] == pytest.approx(0.05 / 1.004)


@pytest.mark.parametrize(
    ("config", "building", "level"),
    [
        (LEVEL_CONFIG, _two_levels(offset=(250, -120)), "L1"),
        (LEVEL_CONFIG, _two_levels(offset=(250, -120), reference_level_name="L2"), "L2"),
        (LEVEL_CONFIG + "    level_transform: {scale: 0.05}\n", _two_levels(names=""), "L2"),
        ("map_transform: {L2: {}}\n", _two_levels(names=""), "L2"),
        ("rmf_fleet: {name: r3}\n", _two_levels(names=""), "L2"),
    ],
    ids=["reference level", "named reference level", "set by hand", "no values", "no level"],
)
def test_level_transform_is_left_alone(config, building, level):
    assert fill_level_transform(config, building, level) == (config, None)


@pytest.mark.parametrize(
    ("building", "message"),
    [
        (_two_levels(names="a"), "share 1 fiducial names and L1 has 1 measurements"),
        (_two_levels(reference_level_name="L2", l2_scale=0), "L2 has 0 measurements"),
        (_two_levels(turn_degrees=1), "by 1.00 degrees against L1. A rotated floor plan is not"),
        (_two_levels(1.25, l2_scale=0.05), "0.04000 metres per pixel but its measurements give"),
        (_two_levels(zoom=0, l2_scale=0.05), "at the same place"),
    ],
    ids=["one fiducial", "no measurement", "turned", "scale differs", "same place"],
)
def test_level_transform_errors(building, message):
    level = "L1" if building.reference_level == "L2" else "L2"
    config = LEVEL_CONFIG.replace("L2", level)

    with pytest.raises(errors.LevelFiducialError, match=message):
        fill_level_transform(config, building, level)


def test_a_fiducial_that_is_not_x_y_name_is_refused():
    building = BuildingFile.parse("name: site\nlevels: {L1: {vertices: [], fiducials: [[1, 2]]}}")

    with pytest.raises(errors.MalformedBuildingError, match="is not \\[x, y, name\\]"):
        building.fiducials("L1")
