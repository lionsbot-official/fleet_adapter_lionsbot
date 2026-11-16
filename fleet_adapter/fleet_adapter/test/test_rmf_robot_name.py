# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the robot name that the adapter gives to RMF."""

from pathlib import Path
import re

import pytest
import yaml

from ..utils.robot_config import DuplicateRmfRobotNameError
from ..utils.robot_config import InvalidRmfRobotNameError
from ..utils.robot_config import rmf_robot_name
from ..utils.robot_config import rmf_robot_names_from_config

try:
    from rclpy.exceptions import InvalidTopicNameException
    from rclpy.validate_topic_name import validate_topic_name
except ImportError:
    validate_topic_name = None


CONFIG_PATH = Path(__file__).parent / 'fixtures' / 'rmf_robot_names.yaml'

# RMF publishes one topic for each robot under this base name.
DYNAMIC_EVENT_TOPIC_BASE = 'rmf/dynamic_event/begin'

# The rcl rule for one token of a topic name, for a machine without rclpy.
ROS_TOPIC_TOKEN = re.compile(r'[A-Za-z_][A-Za-z0-9_]*')


def ros_accepts_topic(topic: str) -> bool:
    if validate_topic_name is None:
        return all(
            ROS_TOPIC_TOKEN.fullmatch(token) for token in topic.split('/'))
    try:
        validate_topic_name(topic)
    except InvalidTopicNameException:
        return False
    return True


def dynamic_event_topic(fleet_name: str, robot_name: str) -> str:
    return f'{DYNAMIC_EVENT_TOPIC_BASE}/{fleet_name}/{robot_name}'


@pytest.fixture(scope='module')
def config():
    with CONFIG_PATH.open() as config_file:
        return yaml.safe_load(config_file)


def robot(rmf_name=None, **robot_config):
    if rmf_name is not None:
        robot_config['rmf_name'] = rmf_name
    return {'robot_config': robot_config}


@pytest.mark.parametrize('vendor_name', [
    'R3SCP-B02357',
    'R5SCR-DV30019',
    'R3-2200888-SCR',
    'R3SCP-TEST01',
])
def test_ros_rejects_the_topic_of_an_encoding_id(config, vendor_name):
    fleet_name = config['rmf_fleet']['name']
    robot_config = config['robots'][vendor_name]

    assert not ros_accepts_topic(dynamic_event_topic(fleet_name, vendor_name))
    assert ros_accepts_topic(dynamic_event_topic(
        fleet_name, rmf_robot_name(vendor_name, robot_config)))


@pytest.mark.parametrize('vendor_name, robot_config, expected', [
    ('R3SCP-B02357', robot(), 'R3SCP_B02357'),
    ('R3-2200888-SCR', robot(), 'R3_2200888_SCR'),
    ('already_valid_1', robot(), 'already_valid_1'),
    ('_leading_underscore', robot(), '_leading_underscore'),
    ('7 seas.bot', robot(), 'r_7_seas_bot'),
    ('', robot(), 'r_'),
    ('R3SCP-B02357', {}, 'R3SCP_B02357'),
    ('R3SCP-B02357', robot(rmf_name='lobby_scrubber'), 'lobby_scrubber'),
    ('R3SCP-B02357', robot(rmf_name='_scrubber2'), '_scrubber2'),
])
def test_rmf_robot_name(vendor_name, robot_config, expected):
    assert rmf_robot_name(vendor_name, robot_config) == expected


@pytest.mark.parametrize('rmf_name', [
    'lobby-scrubber',
    '7scrubber',
    'lobby scrubber',
    'lobby/scrubber',
    'lobby.scrubber',
    ' lobby_scrubber',
    'lobby_scrubber\n',
    '~scrubber',
    '{scrubber}',
    '',
    7,
    True,
])
def test_rmf_robot_name_rejects_an_invalid_rmf_name(rmf_name):
    with pytest.raises(InvalidRmfRobotNameError) as error:
        rmf_robot_name('R3SCP-B02357', robot(rmf_name=rmf_name))

    assert error.value.vendor_name == 'R3SCP-B02357'
    assert error.value.rmf_name == rmf_name


def test_rmf_robot_names_from_config(config):
    assert rmf_robot_names_from_config(config) == {
        'R3SCP-B02357': 'R3SCP_B02357',
        'R5SCR-DV30019': 'R5SCR_DV30019',
        'R3-2200888-SCR': 'R3_2200888_SCR',
        'R3SCP-TEST01': 'lobby_scrubber',
    }


@pytest.mark.parametrize('robots, duplicates', [
    (
        {'R3-1': robot(), 'R3_1': robot()},
        {'R3_1': ['R3-1', 'R3_1']},
    ),
    (
        {'R3-1': robot(), 'R3-2': robot(rmf_name='R3_1')},
        {'R3_1': ['R3-1', 'R3-2']},
    ),
    (
        {
            'R3-1': robot(rmf_name='scrubber'),
            'R3-2': robot(rmf_name='scrubber'),
            'R3-3': robot(),
        },
        {'scrubber': ['R3-1', 'R3-2']},
    ),
])
def test_rmf_robot_names_from_config_rejects_a_shared_name(robots, duplicates):
    with pytest.raises(DuplicateRmfRobotNameError) as error:
        rmf_robot_names_from_config({'robots': robots})

    assert error.value.duplicates == duplicates


def test_rmf_robot_names_from_config_rejects_an_invalid_rmf_name():
    with pytest.raises(InvalidRmfRobotNameError):
        rmf_robot_names_from_config(
            {'robots': {'R3-1': robot(rmf_name='lobby-scrubber')}})
