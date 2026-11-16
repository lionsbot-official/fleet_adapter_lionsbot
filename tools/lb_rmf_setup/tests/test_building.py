import difflib
import math
import re
from pathlib import Path

import pytest
import yaml

from lbrmf import errors
from lbrmf.building import (
    BuildingFile,
    LevelEdit,
    Param,
    ParamType,
    Vertex,
    emit_scalar,
    emit_vertex,
)
from lbrmf.frame import Measurement

TESTDATA = Path(__file__).resolve().parents[1] / "testdata"
OFFICE_NEW = TESTDATA / "office_new" / "rmf_office_new.building.yaml"
EMPTY_LEVEL = TESTDATA / "building" / "empty_level.building.yaml"
BLOCK_STYLE = TESTDATA / "building" / "block_style.building.yaml"

NEW_DOCK = Vertex(
    x=600.5,
    y=700.25,
    z=0,
    name="dock_9",
    params={
        "robot_dock_name": Param(ParamType.STRING, "dock_9"),
        "is_charger": Param(ParamType.BOOL, True),
        "dock_name": Param(ParamType.STRING, "dock_9"),
    },
)
NEW_DOCK_LINE = (
    "[600.5, 700.25, 0, dock_9, {dock_name: [1, dock_9], is_charger: [4, true], "
    "robot_dock_name: [1, dock_9]}]"
)


def _changed_lines(before: str, after: str) -> list[str]:
    return [
        line
        for line in difflib.unified_diff(before.splitlines(), after.splitlines(), lineterm="", n=0)
        if line[:1] in "+-" and not line.startswith(("+++", "---"))
    ]


@pytest.mark.parametrize("path", [OFFICE_NEW, EMPTY_LEVEL, BLOCK_STYLE])
def test_unchanged_file_round_trips_byte_for_byte(path):
    text = path.read_text()

    assert BuildingFile.parse(text).render({}) == text
    assert BuildingFile.parse(text).render({"L8": LevelEdit()} if "L8" in text else {}) == text


def test_emitter_reproduces_every_traffic_editor_vertex_line():
    text = OFFICE_NEW.read_text()
    building = BuildingFile.parse(text)
    original_lines = re.findall(r"^      - (\[.*\])$", text.split("    vertices:\n")[1], re.M)

    emitted = [emit_vertex(vertex) for vertex in building.vertices("L8")]

    assert emitted == original_lines[: len(emitted)]
    assert len(emitted) == 40


def test_reads_vertices_and_params():
    building = BuildingFile.parse(OFFICE_NEW.read_text())
    vertices = building.vertices("L8")

    assert building.name == "rmf_office_new"
    assert building.level_names == ["L8"]
    assert vertices[4] == Vertex(
        565.399,
        694.101,
        0,
        "charging_1",
        {
            "dock_name": Param(ParamType.STRING, "charging_1"),
            "is_charger": Param(ParamType.BOOL, True),
            "robot_dock_name": Param(ParamType.STRING, "charging_1"),
        },
    )
    assert vertices[0].params == {}


def test_measurements_use_vertex_pixel_lengths():
    building = BuildingFile.parse(OFFICE_NEW.read_text())

    (measurement,) = building.measurements("L8")

    assert measurement == Measurement(
        pixel_length=math.dist((501.701, 700.644), (501.914, 773.752)), meters=3.76
    )


def test_replace_and_append_touch_only_their_lines():
    text = OFFICE_NEW.read_text()
    building = BuildingFile.parse(text)
    moved = building.vertices("L8")[5]._replace(x=557.0)

    after = building.render({"L8": LevelEdit(replaced={5: moved}, appended=[NEW_DOCK])})

    assert _changed_lines(text, after) == [
        "-      - [556, 694, 0, location_001]",
        "+      - [557, 694, 0, location_001]",
        f"+      - {NEW_DOCK_LINE}",
    ]
    reread = BuildingFile.parse(after)
    assert reread.vertices("L8")[5] == moved
    assert reread.vertices("L8")[-1] == NEW_DOCK


@pytest.mark.parametrize("path", [EMPTY_LEVEL, BLOCK_STYLE])
def test_append_to_other_list_styles(path):
    building = BuildingFile.parse(path.read_text())
    before = building.vertices("L1")

    after = BuildingFile.parse(building.render({"L1": LevelEdit(appended=[NEW_DOCK])}))

    assert after.vertices("L1") == [*before, NEW_DOCK]
    assert yaml.safe_load(after.render({}))["lifts"] == {}


def test_replace_in_block_style():
    building = BuildingFile.parse(BLOCK_STYLE.read_text())
    moved = building.vertices("L1")[1]._replace(x=250.0)

    after = BuildingFile.parse(building.render({"L1": LevelEdit(replaced={1: moved})}))

    assert after.vertices("L1") == [building.vertices("L1")[0], moved]
    assert after.render({}).count("dock_name") == 1


@pytest.mark.parametrize(
    ("value", "text"),
    [
        ("charging_1", "charging_1"),
        ("", '""'),
        ("{3304deb2-740c}", '"{3304deb2-740c}"'),
        ("true", '"true"'),
        ("No", '"No"'),
        ("123", '"123"'),
        ("1.5", '"1.5"'),
        ("lift a", '"lift a"'),
        ("café", '"café"'),
        (True, "true"),
        (False, "false"),
        (0, "0"),
        (7, "7"),
        (556.0, "556"),
        (0.1, "0.10000000000000001"),
        (501.701, "501.70100000000002"),
        (1e22, "1.0e+22"),
        (-3.5e-07, "-3.4999999999999998e-07"),
    ],
)
def test_emit_scalar(value, text):
    assert emit_scalar(value) == text
    assert yaml.safe_load(text) == value


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_emit_non_finite_number(value):
    with pytest.raises(errors.NonFiniteNumberError):
        emit_scalar(value)


@pytest.mark.parametrize(
    ("text", "error"),
    [
        ("- not a mapping\n", errors.MalformedBuildingError),
        ("levels: {L1: {vertices: []}}\n", errors.MalformedBuildingError),
        ("name: x\nlevels: {L1: {vertices: [[1, 2]]}}\n", errors.MalformedBuildingError),
        ("name: x\nlevels: {L1: {vertices: [[1, 2, 0, 3]]}}\n", errors.MalformedBuildingError),
        ("name: x\nlevels: {L1: {vertices: [[a, 2, 0, v]]}}\n", errors.MalformedBuildingError),
        (
            "name: x\nlevels: {L1: {vertices: [[1, 2, 0, v, {p: 3}]]}}\n",
            errors.MalformedBuildingError,
        ),
        ("name: x\nlevels: {L1: {vertices: [[1, 2, 0, v, [1]]]}}\n", errors.MalformedBuildingError),
        ("name: x\nlevels: {L1: {lanes: []}}\n", errors.MalformedBuildingError),
        ("name: x\nlevels: [L1]\n", errors.MalformedBuildingError),
        ("name: [x\n", errors.MalformedBuildingError),
    ],
    ids=[
        "not a mapping",
        "no name",
        "short vertex",
        "name not a string",
        "x not a number",
        "param not a pair",
        "params not a mapping",
        "no vertices",
        "levels not a mapping",
        "not yaml",
    ],
)
def test_malformed_building_file(text, error):
    with pytest.raises(error):
        BuildingFile.parse(text).vertices("L1")


def test_unknown_level():
    with pytest.raises(errors.LevelNotFoundError):
        BuildingFile.parse(OFFICE_NEW.read_text()).vertices("L9")


@pytest.mark.parametrize(
    "measurements",
    ["[[0, 99, {distance: [3, 1]}]]", "[[0, 1, {}]]", "[[0]]"],
    ids=["vertex index out of range", "no distance", "short measurement"],
)
def test_malformed_measurement(measurements):
    text = (
        f"name: x\nlevels:\n  L1:\n    measurements: {measurements}\n"
        "    vertices: [[0, 0, 0, a], [1, 0, 0, b]]\n"
    )

    with pytest.raises(errors.MalformedBuildingError):
        BuildingFile.parse(text).measurements("L1")


def test_level_without_measurements():
    text = "name: x\nlevels:\n  L1:\n    vertices: []\n"

    assert BuildingFile.parse(text).measurements("L1") == []


def test_replace_last_vertex_and_append():
    text = OFFICE_NEW.read_text()
    building = BuildingFile.parse(text)
    last = building.vertices("L8")[-1]._replace(name="moved_last")

    after = BuildingFile.parse(
        building.render({"L8": LevelEdit(replaced={39: last}, appended=[NEW_DOCK])})
    )

    assert after.vertices("L8")[39:] == [last, NEW_DOCK]


def test_flow_style_level_cannot_be_edited():
    building = BuildingFile.parse("name: x\nlevels: {L1: {vertices: [[1, 2, 0, v]]}}\n")

    with pytest.raises(errors.MalformedBuildingError):
        building.render({"L1": LevelEdit(appended=[NEW_DOCK])})


def test_flow_style_vertex_list_in_block_level():
    building = BuildingFile.parse("name: x\nlevels:\n  L1:\n    vertices:  [[1, 2, 0, v]]\n")

    moved = building.vertices("L1")[0]._replace(x=3)

    after = building.render({"L1": LevelEdit(replaced={0: moved}, appended=[NEW_DOCK])})

    assert after == (
        f"name: x\nlevels:\n  L1:\n    vertices:\n      - [3, 2, 0, v]\n      - {NEW_DOCK_LINE}\n"
    )
