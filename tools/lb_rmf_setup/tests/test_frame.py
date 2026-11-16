import math
import sys
from pathlib import Path

import pytest
import yaml

from lbrmf import errors
from lbrmf.frame import (
    LevelTransform,
    MapFrame,
    MapTransform,
    Measurement,
    Point,
    TransformValues,
    floorplan_scale,
    wrap_angle,
)

TESTDATA = Path(__file__).resolve().parents[1] / "testdata"
ADAPTER_PACKAGE_ROOT = Path(__file__).resolve().parents[3] / "fleet_adapter"

ROBOT_POINTS = [
    Point(0, 0),
    Point(163, 356),
    Point(390, 184),
    Point(647, 1193),
    Point(-12.5, 40.25),
]


def _transform(config_path, level, floorplan_scale_default):
    level_config = yaml.safe_load(config_path.read_text())["map_transform"][level]
    return MapTransform(
        TransformValues.from_config(level_config["transform_values"]),
        LevelTransform.from_config(level_config.get("level_transform")),
        floorplan_scale_default,
    )


def _adapter_transform(config_path, level, floorplan_scale_default):
    sys.path.insert(0, str(ADAPTER_PACKAGE_ROOT))
    try:
        from fleet_adapter.utils.Coordinate import LionsbotCoord
        from fleet_adapter.utils.MapTransform import MapTransform as AdapterMapTransform
    finally:
        sys.path.remove(str(ADAPTER_PACKAGE_ROOT))
    level_config = yaml.safe_load(config_path.read_text())["map_transform"][level]
    values = level_config["transform_values"]
    level_transform = level_config.get("level_transform", {})
    return LionsbotCoord, AdapterMapTransform.compute(
        tx_meters=values["tx_meters"],
        ty_meters=values["ty_meters"],
        rotation_in_radians=math.radians(values["rotation_degrees"]),
        lbmap_translation_scale_factor=values["scale"],
        level_tx_pixels=level_transform.get("tx_pixels", 0),
        level_ty_pixels=level_transform.get("ty_pixels", 0),
        floorplan_scale_meters_per_pixel=level_transform.get("scale", floorplan_scale_default),
    )


LEVELS = [
    (TESTDATA / "transform_config.yaml", "L3", 0.0544),
    (TESTDATA / "transform_config.yaml", "L5", 0.0544),
    (TESTDATA / "office_new" / "config_r3scp.yaml", "L8", 0.0514305),
    (TESTDATA / "office_new" / "config_r5.yaml", "L8", 0.0514305),
]


@pytest.mark.parametrize(("config_path", "level", "default_scale"), LEVELS)
@pytest.mark.parametrize("robot_point", ROBOT_POINTS)
def test_matches_the_adapter_transform(config_path, level, default_scale, robot_point):
    pytest.importorskip("nudged")
    LionsbotCoord, adapter = _adapter_transform(config_path, level, default_scale)
    ours = _transform(config_path, level, default_scale)

    adapter_meters = adapter.robot_to_rmf_meters(LionsbotCoord(robot_point.x, robot_point.y))
    adapter_pixels = adapter.robot_to_rmf_pixels(LionsbotCoord(robot_point.x, robot_point.y))
    adapter_heading = adapter.robot_to_rmf_meters(
        LionsbotCoord(robot_point.x, robot_point.y, orientation_radians=2.7)
    ).orientation_radians

    assert ours.robot_to_rmf_meters(robot_point) == pytest.approx(
        (adapter_meters.x, adapter_meters.y), abs=1e-9
    )
    assert ours.robot_to_floorplan_pixels(robot_point) == pytest.approx(
        (adapter_pixels.x, adapter_pixels.y), abs=1e-9
    )
    assert ours.heading(2.7) == pytest.approx(adapter_heading, abs=1e-12)


def test_reproduces_the_committed_dock_summary():
    dock_summary = yaml.safe_load((TESTDATA / "office_new" / "dock_summary.yaml").read_text())
    expected_x, expected_y, _ = dock_summary["r3"]["cZone_356"]["path"][0]
    ours = _transform(TESTDATA / "office_new" / "config_r3scp.yaml", "L8", 0.0514305)

    assert ours.robot_to_rmf_meters(Point(364, 652)) == pytest.approx(
        (expected_x, expected_y), abs=1e-12
    )


@pytest.mark.parametrize(
    ("level_transform", "default_scale", "expected"),
    [
        (LevelTransform(), 0.05, 0.05),
        (LevelTransform(scale=0.0604), 0.05, 0.0604),
        (LevelTransform(scale=0.0604), None, 0.0604),
    ],
)
def test_floorplan_scale_choice(level_transform, default_scale, expected):
    values = TransformValues(rotation_degrees=0, scale=0.05, tx_meters=0, ty_meters=0)

    assert MapTransform(values, level_transform, default_scale).floorplan_scale == expected


def test_no_floorplan_scale_is_an_error():
    values = TransformValues(rotation_degrees=0, scale=0.05, tx_meters=0, ty_meters=0)

    with pytest.raises(errors.NoFloorplanScaleError):
        MapTransform(values, LevelTransform(), None)


FRAME = MapFrame(resolution=0.05, origin_x=-10.0, origin_y=-20.0, width=647, height=1193)


@pytest.mark.parametrize(
    ("meters", "pixels"),
    [
        (Point(-10.0, -20.0), Point(0.0, 1193.0)),
        (Point(-10.0, 39.65), Point(0.0, 0.0)),
        (Point(6.35, 12.5), Point(327.0, 543.0)),
    ],
)
def test_map_frame_round_trip(meters, pixels):
    assert FRAME.to_pixels(meters) == pytest.approx(pixels, abs=1e-9)
    assert FRAME.to_meters(pixels) == pytest.approx(meters, abs=1e-9)


@pytest.mark.parametrize(
    ("angle", "expected"),
    [(0.0, 0.0), (math.pi, -math.pi), (3 * math.pi / 2, -math.pi / 2), (-7.0, -7.0 + 2 * math.pi)],
)
def test_wrap_angle(angle, expected):
    assert wrap_angle(angle) == pytest.approx(expected)


@pytest.mark.parametrize(
    ("measurements", "expected"),
    [
        ([Measurement(pixel_length=73.108, meters=3.76)], 3.76 / 73.108),
        ([Measurement(100.0, 5.0), Measurement(50.0, 3.0)], (0.05 + 0.06) / 2),
    ],
)
def test_floorplan_scale_from_measurements(measurements, expected):
    assert floorplan_scale(measurements) == pytest.approx(expected)


@pytest.mark.parametrize(
    ("measurements", "error"),
    [
        ([], errors.NoFloorplanScaleError),
        ([Measurement(pixel_length=0.0, meters=3.0)], errors.ZeroLengthMeasurementError),
    ],
)
def test_floorplan_scale_errors(measurements, error):
    with pytest.raises(error):
        floorplan_scale(measurements)


@pytest.mark.parametrize(
    "config",
    [
        {"rotation_degrees": 0, "scale": 0.05, "tx_meters": 0},
        {"rotation_degrees": 0, "scale": "a", "tx_meters": 0, "ty_meters": 0},
        None,
    ],
)
def test_malformed_transform_values(config):
    with pytest.raises(errors.MalformedTransformError):
        TransformValues.from_config(config)


@pytest.mark.parametrize(
    ("config", "expected"),
    [
        (None, LevelTransform()),
        ({"scale": 0.06}, LevelTransform(scale=0.06)),
        (
            {"scale": 0.06, "tx_pixels": 1, "ty_pixels": 2},
            LevelTransform(scale=0.06, tx_pixels=1.0, ty_pixels=2.0),
        ),
    ],
)
def test_level_transform_from_config(config, expected):
    assert LevelTransform.from_config(config) == expected


@pytest.mark.parametrize("config", [{"scale": "big"}, {"tx_pixels": 1}, {"ty_pixels": 2}])
def test_malformed_level_transform(config):
    with pytest.raises(errors.MalformedTransformError):
        LevelTransform.from_config(config)


@pytest.mark.parametrize(
    ("level_transform", "measured"),
    [(LevelTransform(scale=1.0), None), (LevelTransform(), 0.0001), (LevelTransform(), 2.0)],
    ids=["template default 1.0", "too small", "too large"],
)
def test_impossible_floorplan_scale_is_an_error(level_transform, measured):
    values = TransformValues(rotation_degrees=0, scale=0.05, tx_meters=0, ty_meters=0)

    with pytest.raises(errors.ImplausibleFloorplanScaleError) as raised:
        MapTransform(values, level_transform, measured)

    assert "metres per pixel" in str(raised.value)
