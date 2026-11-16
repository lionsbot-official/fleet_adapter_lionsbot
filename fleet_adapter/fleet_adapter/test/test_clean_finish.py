"""Tests for how a clean ends and when it times out.

Each test replays robot statuses on a fake clock through the real clean
worker, the real command timeouts and the real end status handling.
"""

import threading
from unittest.mock import MagicMock
from unittest.mock import patch

import pytest

from ..CommandHandle import CommandHandle
from ..enums.enums import CommandKind
from ..enums.enums import OperationEndStatus
from ..enums.enums import RobotRuntimeStatusText
from ..LionsbotRobot import LionsbotRobot
from ..RobotClientAPI import RobotAPI
from ..utils.Coordinate import LionsbotCoord
from ..utils.Coordinate import RmfCoord

ROBOT = 'R3VAC-B00218'
ZONE = 'l7_path'
RESTING = RobotRuntimeStatusText.RESTING.value
CLEANING = RobotRuntimeStatusText.CLEANING.value
CLEANING_PAUSED = RobotRuntimeStatusText.CLEANING_PAUSED.value
DOCKING_IN_PROGRESS = RobotRuntimeStatusText.DOCKING_IN_PROGRESS.value
DOCKED = RobotRuntimeStatusText.DOCKED.value
# The values of configs/config_template.yaml, which the real run used.
GRACE_SECONDS = 60.0
MAX_TIMEOUT_SECONDS = 120.0
REPLAY_LIMIT_SECONDS = 3600.0
# The timeout counts from the last poll that saw the robot cleaning.
POLL_SECONDS = 0.5

# Seconds after the clean_start of 15:21:31 on the real robot. The log has no
# clean_end_status. The times of the first Cleaning and of Resting and Docked
# are not in the log and are set here.
REAL_RUN = [
    (0.0, RESTING),
    (2.0, CLEANING),
    (105.0, CLEANING_PAUSED),       # 15:23:16
    (115.0, CLEANING),              # 15:23:26
    (168.0, DOCKING_IN_PROGRESS),   # 15:24:19
    (280.0, RESTING),
    (290.0, DOCKED),
]


class FakeTransform:
    def robot_to_rmf_meters(self, robot_coord):
        return RmfCoord(4.0, 5.0, 0.25)


class Replay:
    """A robot that reports the statuses of a run, and the adapter on it."""

    def __init__(self, statuses, end_status=None, max_retries=5):
        self.now = 1000.0
        self.started_at = self.now
        self.statuses = statuses
        self.end_status = end_status
        self.clean_starts = []

        api = RobotAPI.__new__(RobotAPI)
        api._lock = threading.RLock()
        api.token_expiry = float('inf')
        api.robot_status = {}
        api.robot_operation_end_status = {}
        api.robot_pose = {ROBOT: LionsbotCoord(40.0, 50.0, 0.25)}
        api.send_cleaning_command = MagicMock(side_effect=self._clean_start)
        api.send_navigation_command = MagicMock()
        self.api = api

        self.handle = CommandHandle(
            name=ROBOT,
            logger=MagicMock(),
            run_command=MagicMock(),
            stop_robot=MagicMock(),
            wait_for_robot_terminal=MagicMock(return_value=True),
            timeout_grace_sec=GRACE_SECONDS,
            max_timeout_sec=MAX_TIMEOUT_SECONDS)
        self.handle._running = False
        self.execution = MagicMock()
        self.handle._activate(self.handle.make_event(
            CommandKind.CLEAN, execution=self.execution))

        robot = LionsbotRobot.__new__(LionsbotRobot)
        robot.name = ROBOT
        robot.map_name = 'L7'
        robot.robot_map_names = {'L7': 'robot_map'}
        robot.robot_map_levels = {'L7': '7'}
        robot.transforms = {'L7': FakeTransform()}
        robot.dock_paths = {}
        robot.navigation_max_retries = max_retries
        robot.node = MagicMock()
        robot.api = api
        robot.command_handle = self.handle
        robot._ensure_undocked = MagicMock(return_value=True)
        robot._wait_until_ready = MagicMock(return_value=True)
        robot._prepare_map = MagicMock(return_value=True)
        robot._return_to_clean_start_waypoint = MagicMock()
        robot._sleep = self._sleep
        self.robot = robot
        self._report()

    def elapsed(self):
        return self.now - self.started_at

    def status(self):
        return [status for at, status in self.statuses
                if at <= self.elapsed()][-1]

    def _report(self):
        self.api.robot_status[ROBOT] = {'status': self.status()}
        if self.end_status is not None and \
                self.elapsed() >= self.end_status[0]:
            self.api.robot_operation_end_status[ROBOT] = {
                'operation_fb': OperationEndStatus.CLEAN_END_STATUS,
                'content': {'status': self.end_status[1]},
            }
            self.end_status = None

    def _clean_start(self, *args):
        self.clean_starts.append((self.elapsed(), self.status()))
        return True

    def _sleep(self, seconds, execution):
        self.now += seconds
        assert self.elapsed() < REPLAY_LIMIT_SECONDS, 'the clean never ended'
        self._report()
        return self.handle.is_active(execution)

    def run(self):
        with patch(
                'fleet_adapter.fleet_adapter.CommandHandle.time.monotonic',
                side_effect=lambda: self.now):
            self.robot._clean_worker(ZONE, 'test', self.execution)

    def logged(self, text):
        logger = self.robot.node.get_logger.return_value
        return any(
            text in str(call) for call in
            logger.info.call_args_list + logger.error.call_args_list)

    def assert_finished_with_no_return(self, at_seconds):
        """Check that RMF was told the action finished, with no command."""
        self.execution.finished.assert_called_once_with()
        self.execution.error.assert_not_called()
        assert self.elapsed() == pytest.approx(at_seconds)
        self.robot._return_to_clean_start_waypoint.assert_not_called()
        self.api.send_navigation_command.assert_not_called()

    def assert_returns_to_start(self, at_seconds):
        """Check that the clean ended as with a good clean_end_status."""
        self.robot._return_to_clean_start_waypoint.assert_called_once()
        assert self.elapsed() == pytest.approx(at_seconds, abs=POLL_SECONDS)
        self.execution.error.assert_not_called()


def test_real_run_gets_one_clean_start_and_finishes_when_docking_starts():
    replay = Replay(REAL_RUN)

    replay.run()

    assert replay.clean_starts == [(0.0, RESTING)]
    assert not replay.logged('timed out cleaning')
    replay.assert_finished_with_no_return(at_seconds=168.0)


def test_clean_is_not_timed_out_while_cleaning_or_paused():
    replay = Replay(
        [(0.0, CLEANING), (1000.0, CLEANING_PAUSED), (2000.0, CLEANING)],
        end_status=(3000.0, True))

    replay.run()

    assert replay.clean_starts == [(0.0, CLEANING)]
    assert not replay.logged('timed out cleaning')
    replay.assert_returns_to_start(at_seconds=3000.0)


@pytest.mark.parametrize('status', [
    RobotRuntimeStatusText.DOCKING.value,
    RobotRuntimeStatusText.DOCKING_MOVE.value,
    DOCKING_IN_PROGRESS,
    RobotRuntimeStatusText.DOCKING_PAUSED.value,
    DOCKED,
    RobotRuntimeStatusText.CHARGING.value,
])
def test_docking_status_after_cleaning_finishes_the_clean(status):
    replay = Replay([(0.0, CLEANING), (30.0, status)])

    replay.run()

    assert len(replay.clean_starts) == 1
    replay.assert_finished_with_no_return(at_seconds=30.0)


def test_good_clean_end_status_still_returns_to_start():
    replay = Replay(
        [(0.0, CLEANING), (30.0, DOCKING_IN_PROGRESS)],
        end_status=(30.0, True))

    replay.run()

    assert len(replay.clean_starts) == 1
    replay.assert_returns_to_start(at_seconds=30.0)
    replay.execution.finished.assert_not_called()


def test_failed_clean_end_status_still_retries():
    replay = Replay(
        [(0.0, CLEANING), (30.0, RESTING), (31.0, CLEANING)],
        end_status=(30.0, False), max_retries=1)
    replay.statuses.append((40.0, DOCKING_IN_PROGRESS))

    replay.run()

    assert replay.clean_starts == [(0.0, CLEANING), (30.0, RESTING)]
    assert replay.logged('failed to clean')
    replay.assert_finished_with_no_return(at_seconds=40.0)


@pytest.mark.parametrize('status', [RESTING, DOCKED])
def test_clean_that_never_starts_still_times_out_and_retries(status):
    replay = Replay([(0.0, status)], max_retries=1)

    replay.run()

    assert replay.clean_starts == [(0.0, status), (GRACE_SECONDS, status)]
    assert replay.elapsed() == pytest.approx(2 * GRACE_SECONDS)
    replay.execution.error.assert_called_once()
    assert 'timed out cleaning' in replay.execution.error.call_args.args[0]
    replay.execution.finished.assert_called_once_with()


def test_resting_after_cleaning_finishes_the_clean_after_the_grace_time():
    replay = Replay([(0.0, CLEANING), (30.0, RESTING)])

    replay.run()

    assert len(replay.clean_starts) == 1
    replay.assert_returns_to_start(at_seconds=30.0 + GRACE_SECONDS)


def test_short_resting_between_cleaning_does_not_end_the_clean():
    replay = Replay([
        (0.0, CLEANING), (30.0, RESTING), (80.0, CLEANING),
        (200.0, DOCKING_IN_PROGRESS)])

    replay.run()

    assert len(replay.clean_starts) == 1
    replay.assert_finished_with_no_return(at_seconds=200.0)


@pytest.mark.parametrize('status', [
    RobotRuntimeStatusText.ERROR.value,
    RobotRuntimeStatusText.ESTOP.value,
    RobotRuntimeStatusText.OFFLINE.value,
])
def test_fault_after_cleaning_keeps_the_timeout_and_retry_rules(status):
    replay = Replay([(0.0, CLEANING), (30.0, status)], max_retries=0)

    replay.run()

    assert len(replay.clean_starts) == 1
    assert replay.elapsed() == pytest.approx(
        30.0 + GRACE_SECONDS, abs=POLL_SECONDS)
    replay.robot._return_to_clean_start_waypoint.assert_not_called()
    assert 'timed out cleaning' in replay.execution.error.call_args.args[0]
