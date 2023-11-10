import pytest
import math
import yaml
import os
from typing import Dict

from ..utils.MapTransform import MapTransform
from ..utils.Coordinate import LionsbotCoord
from ..utils.Coordinate import RmfCoord

# Run pytest in fleet_adapter_r3/fleet_adapter_r3/ directory
CONFIG_FILE_FILE_PATH = os.path.dirname(__file__) + '/../../../scripts/utils/test/fixtures/test_config.yaml' 
   
@pytest.fixture
def map_transforms() -> Dict[str, MapTransform]:
    config_file = _read_config_file()
    return {
        'reference_coordinates_transform': _map_transform_reference_coordinates(config_file), 
        'transform_values_transform': _map_transform_transform_values(config_file), 
        'reference_coordinates_fiducials_transform': _map_transform_reference_coordinates_fiducials(config_file), 
        'transform_values_fiducials_transform': _map_transform_transform_values_fiducials(config_file)
    }

def test_robot_to_rmf_meters(map_transforms: Dict[str, MapTransform]):
    TOLERABLE_DIFFERENCE_METERS = 1
    test_cases = [
        {
            'name': 'reference_coordinate_dustbin', 
            'input': LionsbotCoord(163, 356), 
            'transform': map_transforms['reference_coordinates_transform'],
            'expected': RmfCoord(21, -44)
        }, 
        {
            'name': 'transform_values_dustbin', 
            'input': LionsbotCoord(163, 356), 
            'transform': map_transforms['transform_values_transform'],
            'expected': RmfCoord(21, -44)
        }, 
        {
            'name': 'fiducial_reference_coordinate_location_258', 
            'input': LionsbotCoord(390, 184), 
            'transform': map_transforms['reference_coordinates_fiducials_transform'],
            'expected': RmfCoord(-35, -14)
        }, 
        {
            'name': 'fiducial_transform_values_location_258', 
            'input': LionsbotCoord(390, 184), 
            'transform': map_transforms['transform_values_fiducials_transform'],
            'expected': RmfCoord(-35, -14)
        }, 
    ]

    for test_case in test_cases:
        actual = test_case['transform'].robot_to_rmf_meters(test_case['input'])
        assert _approximately_equal(actual, 
                                    test_case['expected'], 
                                    TOLERABLE_DIFFERENCE_METERS), f'{test_case["name"]}: Actual {actual} Expected {test_case["expected"]}' 
        
def test_rmf_meters_to_robot(map_transforms: Dict[str, MapTransform]):
    TOLERABLE_DIFFERENCE_PIXELS = 3
    test_cases = [
        {
            'name': 'reference_coordinate_dustbin', 
            'input': RmfCoord(20.9447, -44.1199), 
            'transform': map_transforms['reference_coordinates_transform'],
            'expected': LionsbotCoord(163, 356)
        }, 
        {
            'name': 'transform_values_dustbin', 
            'input': RmfCoord(20.9447, -44.1199), 
            'transform': map_transforms['transform_values_transform'],
            'expected': LionsbotCoord(163, 356)
        }, 
        {
            'name': 'fiducial_reference_coordinate_location_258', 
            'input': RmfCoord(-35.2868, -14.5922), 
            'transform': map_transforms['reference_coordinates_fiducials_transform'],
            'expected': LionsbotCoord(390, 184)
        }, 
        {
            'name': 'fiducial_transform_values_location_258', 
            'input': RmfCoord(-35.2868, -14.5922), 
            'transform': map_transforms['transform_values_fiducials_transform'],
            'expected': LionsbotCoord(390, 184)
        }, 
    ]

    for test_case in test_cases:
        actual = test_case['transform'].rmf_meters_to_robot(test_case['input'])
        assert _approximately_equal(actual, 
                                    test_case['expected'], 
                                    TOLERABLE_DIFFERENCE_PIXELS), f'{test_case["name"]}: Actual {actual} Expected {test_case["expected"]}' 


def _read_config_file():
    with open(CONFIG_FILE_FILE_PATH, 'r') as f:
        return yaml.safe_load(f)

def _map_transform_reference_coordinates(config_file) -> MapTransform:
    reference_coordinates = config_file['map_transform']['L3']['reference_coordinates']
    return MapTransform.estimate(
        rmf_coords=reference_coordinates['rmf'], 
        robot_coords=reference_coordinates['robot'], 
        floorplan_scale_meters_per_pixel=0.0544, 
        level_tx_pixels=0, 
        level_ty_pixels=0, 
    )

def _map_transform_reference_coordinates_fiducials(config_file) -> MapTransform:
    reference_coordinates = config_file['map_transform']['L5']['reference_coordinates']
    level_transform = config_file['map_transform']['L5']['level_transform']
    return MapTransform.estimate(
        rmf_coords=reference_coordinates['rmf'], 
        robot_coords=reference_coordinates['robot'], 
        level_tx_pixels=level_transform['tx_pixels'], 
        level_ty_pixels=level_transform['ty_pixels'], 
        floorplan_scale_meters_per_pixel=level_transform['scale']
    )

def _map_transform_transform_values(config_file) -> MapTransform:
    transform_values = config_file['map_transform']['L3']['transform_values']
    return MapTransform.compute(
        tx_meters=transform_values['tx_meters'],
        ty_meters=transform_values['ty_meters'],
        rotation_in_radians=math.radians(transform_values['rotation_degrees']), 
        lbmap_translation_scale_factor=transform_values['scale'],
        level_tx_pixels=0, 
        level_ty_pixels=0, 
        floorplan_scale_meters_per_pixel=0.0544
    )

def _map_transform_transform_values_fiducials(config_file) -> MapTransform:
    transform_values = config_file['map_transform']['L5']['transform_values']
    level_transform = config_file['map_transform']['L5']['level_transform']
    return MapTransform.compute(
        tx_meters=transform_values['tx_meters'],
        ty_meters=transform_values['ty_meters'],
        rotation_in_radians=math.radians(transform_values['rotation_degrees']), 
        lbmap_translation_scale_factor=transform_values['scale'],
        level_tx_pixels=level_transform['tx_pixels'], 
        level_ty_pixels=level_transform['ty_pixels'], 
        floorplan_scale_meters_per_pixel=level_transform['scale']
    )

def _approximately_equal(robot_coord: LionsbotCoord, rmf_coord: RmfCoord, diff):
    return abs(robot_coord.x - rmf_coord.x) < diff and abs(robot_coord.y - rmf_coord.y) < diff