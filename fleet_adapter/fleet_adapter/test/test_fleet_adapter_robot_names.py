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

"""Tests for the robot name at each point where the adapter meets RMF."""

from pathlib import Path
import threading
import time
from unittest.mock import MagicMock
from unittest.mock import patch

import pytest
from rmf_charger_msgs.msg import ChargerRequest
from rmf_fleet_msgs.msg import ModeRequest
from rmf_fleet_msgs.msg import RobotMode
import yaml

from .. import fleet_adapter as adapter
from ..utils.Coordinate import LionsbotCoord


PACKAGE_PATH = Path(__file__).parents[2]
CONFIG_PATH = PACKAGE_PATH / 'configs' / 'office_new' / 'config_r3scp.yaml'
MAP_PATH = PACKAGE_PATH / 'maps' / 'office_new'
NAV_GRAPH_PATH = MAP_PATH / '1.yaml'
DOCK_SUMMARY_PATH = MAP_PATH / 'dock_summary.yaml'

VENDOR_NAME = 'R3SCP-B02357'
RMF_NAME = 'R3SCP_B02357'
CHARGER_NAME = 'refuel_891'
ADD_ROBOT_TIMEOUT_SEC = 10.0


@pytest.fixture(scope='module')
def config():
    with CONFIG_PATH.open() as config_file:
        return yaml.safe_load(config_file)


@pytest.fixture(scope='module')
def dock_summary():
    with DOCK_SUMMARY_PATH.open() as dock_summary_file:
        return yaml.safe_load(dock_summary_file)


def make_robot(**kwargs):
    robot = MagicMock()
    robot.name = kwargs['name']
    robot.rmf_name = kwargs['rmf_name']
    return robot


def add_robots(config):
    """Run the add-robot thread to its end and return what it gave RMF."""
    api = MagicMock()
    api.is_subscribed.return_value = True
    api.position.return_value = LionsbotCoord(
        x=357.0, y=773.0, orientation_radians=0.0)
    api.get_robot_status.return_value = {'localized': True}
    api.change_map.return_value = True
    fleet_config = MagicMock()
    fleet_handle = MagicMock()
    robots = {}

    with patch.object(adapter.rclpy, 'ok', return_value=True), \
            patch.object(adapter, 'LionsbotRobot', side_effect=make_robot):
        adapter.start_add_fleet_robots_thread(
            config_yaml=config,
            fleet_name=config['rmf_fleet']['name'],
            node=MagicMock(),
            nav_graph=MagicMock(),
            fleet_config=fleet_config,
            fleet_handle=fleet_handle,
            transforms={'L8': MagicMock()},
            dock_paths={},
            api=api,
            robots=robots,
            robots_lock=threading.Lock(),
        )
        deadline = time.monotonic() + ADD_ROBOT_TIMEOUT_SEC
        while not robots and time.monotonic() < deadline:
            time.sleep(0.05)

    return robots, fleet_config, fleet_handle, api


def subscription_callback(node, topic):
    for call in node.create_subscription.call_args_list:
        if call.args[1] == topic:
            return call.args[2]
    raise AssertionError(f'No subscription to [{topic}]')


def test_fleet_configuration_knows_the_robot_by_its_rmf_name(
        config, dock_summary):
    fleet_config, _, _, _ = adapter.make_fleet_configuration(
        config, str(NAV_GRAPH_PATH), '', dock_summary)

    assert fleet_config.known_robots == [RMF_NAME]
    assert fleet_config.get_known_robot_configuration(RMF_NAME) is not None


def test_robot_is_added_to_rmf_by_its_rmf_name(config):
    robots, fleet_config, fleet_handle, api = add_robots(config)

    fleet_config.get_known_robot_configuration.assert_called_once_with(
        RMF_NAME)
    assert fleet_handle.add_robot.call_args.args[0] == RMF_NAME
    assert list(robots) == [RMF_NAME]
    assert robots[RMF_NAME].name == VENDOR_NAME
    api.change_map.assert_called_once_with(
        robot_encoding_id=VENDOR_NAME, map_name='rmf_test', map_level='L8')


def mode_request(fleet_name, robot_name):
    request = ModeRequest()
    request.fleet_name = fleet_name
    request.robot_name = robot_name
    request.mode.mode = RobotMode.MODE_IDLE
    return request


def charger_request(fleet_name, robot_name):
    request = ChargerRequest()
    request.fleet_name = fleet_name
    request.robot_name = robot_name
    request.charger_name = CHARGER_NAME
    return request


@pytest.mark.parametrize('topic, make_request, robot_call', [
    (
        'action_execution_notice',
        mode_request,
        lambda robot: robot.finish_action,
    ),
    (
        'charger_requests',
        charger_request,
        lambda robot: robot.graph.find_waypoint,
    ),
])
def test_ros_request_finds_the_robot_by_its_rmf_name(
        config, topic, make_request, robot_call):
    fleet_name = config['rmf_fleet']['name']
    robots, _, fleet_handle, _ = add_robots(config)
    fleet_handle.more.return_value.fleet_name = fleet_name
    node = MagicMock()
    adapter.ros_connections(node, robots, threading.Lock(), fleet_handle)
    callback = subscription_callback(node, topic)

    callback(make_request(fleet_name, VENDOR_NAME))
    robot_call(robots[RMF_NAME]).assert_not_called()

    callback(make_request(fleet_name, RMF_NAME))
    robot_call(robots[RMF_NAME]).assert_called_once()
