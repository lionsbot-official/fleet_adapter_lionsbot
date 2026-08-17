import math
import threading
from unittest.mock import MagicMock
from unittest.mock import patch

import pytest

from ..enums.enums import CommandResult
from ..enums.enums import CommandState
from ..enums.enums import RobotRuntimeStatusText
from ..LionsbotRobot import LionsbotRobot
from ..RobotClientAPI import RobotAPI
from ..utils.Coordinate import LionsbotCoord
from ..utils.Coordinate import RmfCoord
from ..utils.MapTransform import MapTransform


class FakeTransform:
    def __init__(self, rmf_pose=None):
        self.rmf_pose = rmf_pose
        self.return_pose_rmf = None

    def robot_to_rmf_meters(self, robot_coord):
        return self.rmf_pose

    def rmf_meters_to_robot(self, rmf_coord):
        self.return_pose_rmf = rmf_coord
        return LionsbotCoord(
            x=rmf_coord.x * 10.0,
            y=rmf_coord.y * 10.0,
            orientation_radians=rmf_coord.orientation_radians,
        )


def make_robot(transform, max_retries=1):
    robot = LionsbotRobot.__new__(LionsbotRobot)
    robot.name = 'test_robot'
    robot.map_name = 'L8'
    robot.robot_map_names = {'L8': 'robot_map'}
    robot.robot_map_levels = {'L8': '8'}
    robot.transforms = {'L8': transform}
    robot.dock_paths = {}
    robot.navigation_max_retries = max_retries
    robot.node = MagicMock()
    robot.api = MagicMock()
    robot.command_handle = MagicMock()
    robot._execution_is_active = MagicMock(return_value=True)
    robot._prepare_map = MagicMock(return_value=True)
    robot._ensure_undocked = MagicMock(return_value=True)
    robot._wait_until_ready = MagicMock(return_value=True)
    robot._sleep = MagicMock(return_value=True)
    robot._finish_execution = MagicMock()
    robot._fail_execution = MagicMock()
    return robot


def test_clean_returns_to_captured_pose_with_reversed_heading():
    transform = FakeTransform(RmfCoord(
        x=4.0,
        y=5.0,
        orientation_radians=0.25,
    ))
    robot = make_robot(transform)
    execution = MagicMock()
    robot.api.position.return_value = LionsbotCoord(40.0, 50.0, 0.25)
    robot.api.send_cleaning_command.return_value = True
    robot.api.process_completed.return_value = CommandResult.SUCCESS
    robot.api.send_navigation_command.return_value = True
    robot.api.navigate_robot.return_value = CommandResult.SUCCESS

    robot._clean_worker('clean_zone', 'test', execution)

    robot.api.send_cleaning_command.assert_called_once_with(
        'test_robot', 'clean_zone', 'robot_map', '8')
    assert transform.return_pose_rmf.x == pytest.approx(4.0)
    assert transform.return_pose_rmf.y == pytest.approx(5.0)
    assert transform.return_pose_rmf.orientation_radians == pytest.approx(
        MapTransform.wrap_orientation(0.25 + math.pi))

    return_command = robot.api.send_navigation_command.call_args.args
    assert return_command[0] == 'test_robot'
    assert return_command[1].x == pytest.approx(40.0)
    assert return_command[1].y == pytest.approx(50.0)
    assert return_command[1].orientation_radians == pytest.approx(
        MapTransform.wrap_orientation(0.25 + math.pi))
    assert return_command[2] == 'robot_map'
    robot.command_handle.set_state.assert_called_with(
        execution, CommandState.P2P_MOVE)
    robot._finish_execution.assert_called_once_with(execution)
    robot._fail_execution.assert_not_called()


def test_clean_return_retries_navigation_failure():
    transform = FakeTransform()
    robot = make_robot(transform, max_retries=1)
    execution = MagicMock()
    return_pose_rmf = RmfCoord(4.0, 5.0, -2.0)
    robot.api.send_navigation_command.return_value = True
    robot.api.navigate_robot.side_effect = [
        CommandResult.ERROR,
        CommandResult.SUCCESS,
    ]

    robot._return_to_clean_start_waypoint('L8', return_pose_rmf, execution)

    assert robot.api.send_navigation_command.call_count == 2
    assert robot.api.navigate_robot.call_count == 2
    robot._finish_execution.assert_called_once_with(execution)
    robot._fail_execution.assert_not_called()


def test_clean_return_timeout_fails_execution_when_retries_exhausted():
    transform = FakeTransform()
    robot = make_robot(transform, max_retries=0)
    execution = MagicMock()
    robot.api.send_navigation_command.return_value = True
    robot.api.navigate_robot.return_value = CommandResult.TIMEOUT

    robot._return_to_clean_start_waypoint(
        'L8',
        RmfCoord(4.0, 5.0, -2.0),
        execution)

    robot._finish_execution.assert_not_called()
    robot._fail_execution.assert_called_once()
    assert 'timed out returning after cleaning' in \
        robot._fail_execution.call_args.args[1]


def test_charging_status_detects_boolean_or_text_status():
    assert RobotAPI._is_charging_status({
        'charging': True,
        'status': RobotRuntimeStatusText.RESTING.value,
    })
    assert RobotAPI._is_charging_status({
        'charging': False,
        'status': RobotRuntimeStatusText.CHARGING.value,
    })
    assert not RobotAPI._is_charging_status({
        'charging': False,
        'status': RobotRuntimeStatusText.RESTING.value,
    })


def test_docked_not_charging_sends_one_alert_per_incident():
    robot = LionsbotRobot.__new__(LionsbotRobot)
    robot.name = 'test_robot'
    robot.node = MagicMock()
    robot.api = MagicMock()
    robot._alert_lock = threading.Lock()
    robot.docked_not_charging_grace_sec = 60.0
    robot._docked_not_charging_since = None
    robot._docked_not_charging_alert_sent = False
    robot._send_docked_not_charging_alert = MagicMock()
    robot.api.get_robot_status.return_value = {
        'status': RobotRuntimeStatusText.DOCKED.value,
        'charging': False,
        'timestamp': 123,
    }

    with patch(
            'fleet_adapter.fleet_adapter.LionsbotRobot.time.monotonic',
            side_effect=[100.0, 159.0, 160.0]):
        robot._monitor_docked_not_charging()
        robot._monitor_docked_not_charging()
        robot._monitor_docked_not_charging()

    robot._send_docked_not_charging_alert.assert_called_once()
    alert_args = robot._send_docked_not_charging_alert.call_args.args
    assert alert_args[1]['status'] == RobotRuntimeStatusText.DOCKED.value
    assert alert_args[1]['charging'] is False

    robot.api.get_robot_status.return_value = {
        'status': RobotRuntimeStatusText.DOCKED.value,
        'charging': True,
    }
    with patch(
            'fleet_adapter.fleet_adapter.LionsbotRobot.time.monotonic',
            return_value=161.0):
        robot._monitor_docked_not_charging()

    assert not robot._docked_not_charging_alert_sent
