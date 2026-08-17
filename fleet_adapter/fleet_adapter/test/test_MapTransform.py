"""Regression and round-trip tests for map-coordinate transformations."""

import math
from pathlib import Path

import pytest
import yaml

from ..utils.Coordinate import LionsbotCoord
from ..utils.Coordinate import RmfCoord
from ..utils.MapTransform import MapTransform


CONFIG_PATH = Path(__file__).parent / 'fixtures' / 'test_config.yaml'


def _transform_from_config(config, level):
    transform_values = config['map_transform'][level]['transform_values']
    level_transform = config['map_transform'][level].get('level_transform', {})
    return MapTransform.compute(
        tx_meters=transform_values['tx_meters'],
        ty_meters=transform_values['ty_meters'],
        rotation_in_radians=math.radians(
            transform_values['rotation_degrees']),
        lbmap_translation_scale_factor=transform_values['scale'],
        level_tx_pixels=level_transform.get('tx_pixels', 0),
        level_ty_pixels=level_transform.get('ty_pixels', 0),
        floorplan_scale_meters_per_pixel=level_transform.get('scale', 0.0544),
    )


@pytest.fixture
def map_transforms():
    with CONFIG_PATH.open() as config_file:
        config = yaml.safe_load(config_file)
    return {
        'L3': _transform_from_config(config, 'L3'),
        'L5': _transform_from_config(config, 'L5'),
    }


@pytest.mark.parametrize(
    ('level', 'robot_coord', 'expected'),
    [
        ('L3', LionsbotCoord(163, 356), RmfCoord(20.997367, -44.158144)),
        ('L5', LionsbotCoord(390, 184), RmfCoord(-35.291870, -14.606162)),
    ],
)
def test_robot_to_rmf_meters(map_transforms, level, robot_coord, expected):
    actual = map_transforms[level].robot_to_rmf_meters(robot_coord)

    assert actual.x == pytest.approx(expected.x, abs=1e-5)
    assert actual.y == pytest.approx(expected.y, abs=1e-5)


@pytest.mark.parametrize(
    ('level', 'rmf_coord', 'expected'),
    [
        ('L3', RmfCoord(20.9447, -44.1199), LionsbotCoord(162.172783, 356.881082)),
        ('L5', RmfCoord(-35.2868, -14.5922), LionsbotCoord(389.924880, 184.284608)),
    ],
)
def test_rmf_meters_to_robot(map_transforms, level, rmf_coord, expected):
    actual = map_transforms[level].rmf_meters_to_robot(rmf_coord)

    assert actual.x == pytest.approx(expected.x, abs=1e-5)
    assert actual.y == pytest.approx(expected.y, abs=1e-5)


@pytest.mark.parametrize('level', ['L3', 'L5'])
def test_coordinate_and_orientation_round_trip(map_transforms, level):
    transform = map_transforms[level]
    original = LionsbotCoord(123.456, 789.012, orientation_radians=2.7)

    restored = transform.rmf_meters_to_robot(
        transform.robot_to_rmf_meters(original))

    assert restored.x == pytest.approx(original.x, abs=1e-9)
    assert restored.y == pytest.approx(original.y, abs=1e-9)
    assert restored.orientation_radians == pytest.approx(
        original.orientation_radians, abs=1e-9)
