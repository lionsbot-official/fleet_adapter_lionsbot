"""Tests for how Estop, Error and Offline are reported to RMF.

The robot must get no command because of these states, and the task must not
be ended because of them.
"""

from unittest.mock import MagicMock
from unittest.mock import patch

import pytest
import rmf_adapter

from ..enums.enums import RobotRuntimeStatusText
from ..LionsbotRobot import LionsbotRobot

ESTOP = RobotRuntimeStatusText.ESTOP.value
ERROR = RobotRuntimeStatusText.ERROR.value
OFFLINE = RobotRuntimeStatusText.OFFLINE.value
RESTING = RobotRuntimeStatusText.RESTING.value
CLEANING = RobotRuntimeStatusText.CLEANING.value
GRACE_SECONDS = 60.0

# The members of the rmf_adapter 2.7.2 execution types. A navigate or localize
# command gets a CommandExecution, which has no delayed, blocked, error or
# underway. A clean or dock action gets an ActionExecution.
COMMAND_EXECUTION = ['finished', 'identifier', 'okay', 'override_schedule']
ACTION_EXECUTION = COMMAND_EXECUTION + [
    'blocked', 'delayed', 'error', 'underway', 'update_remaining_time']


class FakeClock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


def make_robot(execution=None):
    robot = LionsbotRobot.__new__(LionsbotRobot)
    robot.name = 'test_robot'
    robot.node = MagicMock()
    robot.api = MagicMock()
    robot.command_handle = MagicMock()
    robot.command_handle.active_execution.return_value = execution
    robot.update_handle = MagicMock()
    robot.timeout_grace_sec = GRACE_SECONDS
    robot._clock = FakeClock()
    robot._fault_seen = None
    robot._fault_seen_since = 0.0
    robot._fault_reported = None
    robot._fault_ticket = None
    robot._fault_execution = None
    robot._send_rmf_web_alert = MagicMock()
    return robot


def poll(robot, status, after_seconds=0.0):
    robot._clock.now += after_seconds
    robot.api.get_robot_status.return_value = {'status': status}
    robot._monitor_fault_status()


def rmf_handle(robot):
    return robot.update_handle.more.return_value


def ticket(robot):
    return rmf_handle(robot).create_issue.return_value


def assert_only_reported(robot, execution=None):
    """The robot got no command and RMF was not told the command ended."""
    assert {call[0] for call in robot.api.method_calls} == \
        {'get_robot_status'}
    assert {call[0] for call in robot.command_handle.method_calls} == \
        {'active_execution'}
    assert {call[0] for call in rmf_handle(robot).method_calls} <= \
        {'create_issue', 'create_issue().resolve'}
    if execution is not None:
        execution.finished.assert_not_called()
        if hasattr(execution, 'error'):
            execution.error.assert_not_called()


@pytest.mark.parametrize('status', [ESTOP, ERROR])
def test_idle_robot_fault_is_reported_once_as_an_error_issue(status):
    robot = make_robot()

    poll(robot, status)
    poll(robot, status, after_seconds=1.0)

    tier = rmf_adapter.robot_update_handle.Tier
    rmf_handle(robot).create_issue.assert_called_once_with(
        tier.Error, status.lower(), {'status': status})
    robot._send_rmf_web_alert.assert_called_once()
    assert robot._send_rmf_web_alert.call_args.args[0] == status.lower()
    ticket(robot).resolve.assert_not_called()
    assert_only_reported(robot)


@pytest.mark.parametrize('status', [ESTOP, ERROR])
def test_fault_during_an_action_marks_the_action_blocked(status):
    execution = MagicMock(spec=ACTION_EXECUTION)
    robot = make_robot(execution)

    poll(robot, status)
    poll(robot, status, after_seconds=1.0)

    execution.blocked.assert_called_once_with(
        f'Robot [test_robot] reports {status}')
    execution.delayed.assert_not_called()
    rmf_handle(robot).create_issue.assert_called_once()
    assert_only_reported(robot, execution)


def test_fault_during_navigation_is_reported_as_an_issue_only():
    execution = MagicMock(spec=COMMAND_EXECUTION)
    robot = make_robot(execution)

    poll(robot, ESTOP)
    poll(robot, RESTING, after_seconds=1.0)

    rmf_handle(robot).create_issue.assert_called_once()
    ticket(robot).resolve.assert_called_once_with({})
    assert_only_reported(robot, execution)


def test_estop_is_not_cleared_by_the_adapter():
    execution = MagicMock(spec=ACTION_EXECUTION)
    robot = make_robot(execution)

    for _ in range(10):
        poll(robot, ESTOP, after_seconds=600.0)

    ticket(robot).resolve.assert_not_called()
    execution.underway.assert_not_called()
    assert_only_reported(robot, execution)


def test_fault_that_ends_resolves_the_issue_and_marks_the_action_underway():
    execution = MagicMock(spec=ACTION_EXECUTION)
    robot = make_robot(execution)

    poll(robot, ESTOP)
    poll(robot, CLEANING, after_seconds=5.0)
    poll(robot, CLEANING, after_seconds=1.0)

    ticket(robot).resolve.assert_called_once_with({})
    execution.underway.assert_called_once_with(
        'Robot [test_robot] no longer reports Estop')
    assert_only_reported(robot, execution)


def test_brief_offline_is_not_reported():
    execution = MagicMock(spec=ACTION_EXECUTION)
    robot = make_robot(execution)

    poll(robot, OFFLINE)
    poll(robot, OFFLINE, after_seconds=GRACE_SECONDS - 1.0)
    poll(robot, CLEANING, after_seconds=1.0)
    poll(robot, OFFLINE, after_seconds=1.0)
    poll(robot, OFFLINE, after_seconds=GRACE_SECONDS - 1.0)

    rmf_handle(robot).create_issue.assert_not_called()
    robot._send_rmf_web_alert.assert_not_called()
    assert execution.method_calls == []
    assert_only_reported(robot, execution)


def test_offline_that_lasts_is_a_warning_issue_and_delays_the_action():
    execution = MagicMock(spec=ACTION_EXECUTION)
    robot = make_robot(execution)

    poll(robot, OFFLINE)
    poll(robot, OFFLINE, after_seconds=GRACE_SECONDS)
    poll(robot, OFFLINE, after_seconds=1.0)

    tier = rmf_adapter.robot_update_handle.Tier
    rmf_handle(robot).create_issue.assert_called_once_with(
        tier.Warning, 'offline', {'status': OFFLINE})
    execution.delayed.assert_called_once_with(
        'Robot [test_robot] reports Offline')
    execution.blocked.assert_not_called()
    robot._send_rmf_web_alert.assert_called_once()
    assert_only_reported(robot, execution)


def test_estop_stays_reported_through_a_brief_offline():
    robot = make_robot()

    poll(robot, ESTOP)
    poll(robot, OFFLINE, after_seconds=1.0)
    poll(robot, ESTOP, after_seconds=1.0)

    rmf_handle(robot).create_issue.assert_called_once()
    ticket(robot).resolve.assert_not_called()


def test_change_of_fault_replaces_the_issue():
    execution = MagicMock(spec=ACTION_EXECUTION)
    robot = make_robot(execution)

    poll(robot, ESTOP)
    poll(robot, ERROR, after_seconds=1.0)

    assert [call.args[1] for call in
            rmf_handle(robot).create_issue.call_args_list] == \
        ['estop', 'error']
    ticket(robot).resolve.assert_called_once_with({})
    assert execution.blocked.call_count == 2
    execution.underway.assert_not_called()


def test_action_that_starts_during_a_fault_is_marked_blocked():
    robot = make_robot()
    poll(robot, ESTOP)

    execution = MagicMock(spec=ACTION_EXECUTION)
    robot.command_handle.active_execution.return_value = execution
    poll(robot, ESTOP, after_seconds=1.0)

    execution.blocked.assert_called_once()
    rmf_handle(robot).create_issue.assert_called_once()


def test_action_that_rmf_ended_is_not_marked():
    execution = MagicMock(spec=ACTION_EXECUTION)
    execution.okay.return_value = False
    robot = make_robot(execution)

    poll(robot, ESTOP)
    poll(robot, RESTING, after_seconds=1.0)

    execution.blocked.assert_not_called()
    execution.underway.assert_not_called()
    ticket(robot).resolve.assert_called_once_with({})


def test_no_status_yet_is_not_a_fault():
    robot = make_robot()
    robot.api.get_robot_status.return_value = None

    robot._monitor_fault_status()

    rmf_handle(robot).create_issue.assert_not_called()


def test_fault_alert_is_posted_to_rmf_web():
    robot = make_robot()
    del robot._send_rmf_web_alert
    robot.rmf_name = 'test_robot'
    robot.fleet_name = 'r3'
    robot.rmf_web_alerts_url = 'http://rmf-web.test/alerts/request'
    robot.rmf_web_api_token = 'test'

    with patch(
            'fleet_adapter.fleet_adapter.LionsbotRobot.requests.post') as post:
        poll(robot, ESTOP)

    payload = post.call_args.kwargs['json']
    assert payload['id'].startswith('estop-test_robot-')
    assert payload['title'] == 'Robot reports Estop'
    assert {'name': 'vendor_status', 'value': 'Estop'} in \
        payload['alert_parameters']


def test_update_runs_the_fault_monitor_before_the_rmf_update():
    robot = make_robot()
    calls = []
    robot._monitor_docked_not_charging = MagicMock()
    robot._monitor_fault_status = MagicMock(
        side_effect=lambda: calls.append('fault'))
    robot.make_state = MagicMock(side_effect=lambda: calls.append('state'))

    robot.update()

    assert calls == ['fault', 'state']
    robot.update_handle.update.assert_called_once()
