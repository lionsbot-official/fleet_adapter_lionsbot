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

"""Unit tests for the lift commands on the robot status WebSocket."""

import json
from pathlib import Path
import threading
from unittest.mock import MagicMock

import pytest

from ..enums.enums import LiftCommand
from ..enums.enums import LiftMoveState
from ..LiftMoveTracker import LiftCommandError
from ..LiftMoveTracker import LiftCommandSendError
from ..LiftMoveTracker import LiftMapUnavailableError
from ..LiftMoveTracker import LiftMoveInProgressError
from ..LiftMoveTracker import LiftMoveTracker
from ..LiftMoveTracker import new_lift_request_id
from ..RobotClientAPI import RobotAPI

FIXTURE = json.loads(
    (Path(__file__).parent / 'fixtures' / 'lift_commands.json').read_text())
ROBOT = FIXTURE['robot_id']
OTHER_ROBOT = FIXTURE['other_robot_id']
REQUEST_ID = FIXTURE['request_id']


def case_ids(cases):
    return [case['name'] for case in cases]


class FakeClock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeWebSocket:
    def __init__(self, fail=False):
        self.fail = fail
        self.frames = []

    def send(self, message):
        if self.fail:
            raise RuntimeError('simulated send failure')
        self.frames.append(json.loads(message))


def make_tracker(clock=None, request_ids=None):
    ids = iter(request_ids or [REQUEST_ID])
    return LiftMoveTracker(
        clock=clock or FakeClock(),
        new_request_id=lambda robot_encoding_id: next(ids))


def make_api(clock=None, request_ids=None, websocket=None):
    """Build a RobotAPI instance without starting its network threads."""
    api = object.__new__(RobotAPI)
    api._lock = threading.Lock()
    api.token = 'test'
    api.token_expiry = float('inf')
    api.robot_names_by_uuid = {}
    api.robot_status = {}
    api.robot_operation_end_status = {}
    api._last_status_ws_message_at = None
    api.robot_status_ws_connection = websocket or FakeWebSocket()
    api.lift_moves = make_tracker(clock, request_ids)
    api.get_map = lambda map_name, robot_encoding_id, map_level: \
        FIXTURE['map']
    return api


def send(api, command=LiftCommand.ENTER, lift='lift_a'):
    robot_map = api.lift_command_map(
        ROBOT, FIXTURE['map']['name'], FIXTURE['map']['level'])
    return api.send_lift_command(ROBOT, command, lift, robot_map)


def receive(api, frame):
    api._on_robot_status_message(api.robot_status_ws_connection,
                                 json.dumps(frame))


@pytest.mark.parametrize(
    'case', FIXTURE['commands'], ids=case_ids(FIXTURE['commands']))
def test_command_frame_is_the_documented_message(case):
    api = make_api()

    request_id = send(api, LiftCommand(case['command']), case['lift'])

    assert request_id == REQUEST_ID
    assert api.robot_status_ws_connection.frames == [case['frame']]
    status = api.lift_move_status(ROBOT)
    assert status.state == LiftMoveState.AWAITING_ANSWER
    assert status.request_id == REQUEST_ID


@pytest.mark.parametrize(
    'case', FIXTURE['end_statuses'], ids=case_ids(FIXTURE['end_statuses']))
def test_end_status_with_the_request_id_ends_the_move(case):
    api = make_api()
    send(api, LiftCommand(case['command']))

    receive(api, case['frame'])

    status = api.lift_move_status(ROBOT)
    assert status.state == LiftMoveState(case['state'])
    assert status.request_id == REQUEST_ID
    assert status.error_code == case.get('error_code')
    assert status.aborted is case.get('aborted', False)


@pytest.mark.parametrize(
    'case',
    FIXTURE['ignored_end_statuses'],
    ids=case_ids(FIXTURE['ignored_end_statuses']))
def test_message_that_does_not_match_the_move_is_ignored(case):
    api = make_api()
    send(api, LiftCommand(case['command']))

    receive(api, case['frame'])

    assert api.lift_move_status(ROBOT).state == LiftMoveState.AWAITING_ANSWER


@pytest.mark.parametrize(
    'case', FIXTURE['end_statuses'], ids=case_ids(FIXTURE['end_statuses']))
def test_end_status_for_another_robot_is_ignored(case):
    api = make_api()
    send(api, LiftCommand(case['command']))

    receive(api, {**case['frame'], 'robot_id': OTHER_ROBOT})

    assert api.lift_move_status(ROBOT).state == LiftMoveState.AWAITING_ANSWER


@pytest.mark.parametrize(
    'case', FIXTURE['errors'], ids=case_ids(FIXTURE['errors']))
def test_command_error_refuses_the_move_with_its_reason_code(case):
    api = make_api()
    send(api, LiftCommand(case['command']))

    receive(api, case['frame'])

    status = api.lift_move_status(ROBOT)
    assert status.state == LiftMoveState.REFUSED
    assert status.error_code == case['error_code']
    assert status.reason_code == case['reason_code']
    assert status.reason == case['reason']
    assert api.robot_status_ws_connection.frames[1:] == []


def test_lift_messages_do_not_change_the_other_end_statuses():
    api = make_api()
    send(api)

    receive(api, FIXTURE['end_statuses'][0]['frame'])

    assert api.robot_operation_end_status == {}


@pytest.mark.parametrize('seconds, state', [
    (0.0, LiftMoveState.AWAITING_ANSWER),
    (19.9, LiftMoveState.AWAITING_ANSWER),
    (20.0, LiftMoveState.RUNNING),
    (359.9, LiftMoveState.RUNNING),
    (360.0, LiftMoveState.TIMED_OUT),
    (4000.0, LiftMoveState.TIMED_OUT),
])
def test_no_error_in_20_s_means_accepted_and_6_min_is_the_watchdog(
        seconds, state):
    clock = FakeClock()
    api = make_api(clock)
    send(api)

    clock.advance(seconds)

    assert api.lift_move_status(ROBOT).state == state


@pytest.mark.parametrize(
    'case', FIXTURE['errors'], ids=case_ids(FIXTURE['errors']))
def test_command_error_after_20_s_is_not_for_this_command(case):
    clock = FakeClock()
    api = make_api(clock)
    send(api, LiftCommand(case['command']))
    clock.advance(LiftMoveTracker.NO_ERROR_SECONDS)

    receive(api, case['frame'])

    assert api.lift_move_status(ROBOT).state == LiftMoveState.RUNNING


def test_watchdog_sends_p2p_stop_one_time():
    clock = FakeClock()
    api = make_api(clock)
    send(api)
    clock.advance(LiftMoveTracker.WATCHDOG_SECONDS)

    for _ in range(3):
        assert api.lift_move_status(ROBOT).state == LiftMoveState.TIMED_OUT

    stop_frames = [
        {key: frame[key] for key in FIXTURE['stop_frame']}
        for frame in api.robot_status_ws_connection.frames[1:]]
    assert stop_frames == [FIXTURE['stop_frame']]


@pytest.mark.parametrize('first_frame', [
    FIXTURE['end_statuses'][0]['frame'],
    FIXTURE['end_statuses'][2]['frame'],
    FIXTURE['errors'][0]['frame'],
], ids=['succeeded', 'failed', 'refused'])
def test_ended_move_does_not_change_or_time_out(first_frame):
    clock = FakeClock()
    api = make_api(clock)
    send(api)
    receive(api, first_frame)
    ended = api.lift_move_status(ROBOT)

    receive(api, FIXTURE['end_statuses'][4]['frame'])
    receive(api, FIXTURE['errors'][1]['frame'])
    clock.advance(LiftMoveTracker.WATCHDOG_SECONDS)

    assert api.lift_move_status(ROBOT) == ended
    assert len(api.robot_status_ws_connection.frames) == 1


def test_end_status_after_the_watchdog_is_ignored():
    clock = FakeClock()
    api = make_api(clock)
    send(api)
    clock.advance(LiftMoveTracker.WATCHDOG_SECONDS)
    assert api.lift_move_status(ROBOT).state == LiftMoveState.TIMED_OUT

    receive(api, FIXTURE['end_statuses'][0]['frame'])

    assert api.lift_move_status(ROBOT).state == LiftMoveState.TIMED_OUT


def test_second_command_is_refused_while_a_move_runs():
    api = make_api(request_ids=[REQUEST_ID, 'unused'])
    send(api)

    with pytest.raises(LiftMoveInProgressError) as raised:
        send(api, LiftCommand.EXIT)

    assert raised.value.robot_encoding_id == ROBOT
    assert raised.value.request_id == REQUEST_ID
    assert len(api.robot_status_ws_connection.frames) == 1


@pytest.mark.parametrize('first_frame', [
    FIXTURE['end_statuses'][0]['frame'],
    FIXTURE['end_statuses'][2]['frame'],
    FIXTURE['errors'][0]['frame'],
], ids=['succeeded', 'failed', 'refused'])
def test_new_attempt_gets_a_new_request_id(first_frame):
    second_id = 'rmf-R3VAC-B00000-00000000000000000000000000000002'
    api = make_api(request_ids=[REQUEST_ID, second_id])
    send(api)
    receive(api, first_frame)

    assert send(api) == second_id

    sent_ids = [
        frame['content']['request_id']
        for frame in api.robot_status_ws_connection.frames]
    assert sent_ids == [REQUEST_ID, second_id]
    assert api.lift_move_status(ROBOT).state == LiftMoveState.AWAITING_ANSWER


def test_old_end_status_does_not_end_the_new_attempt():
    second_id = 'rmf-R3VAC-B00000-00000000000000000000000000000002'
    api = make_api(request_ids=[REQUEST_ID, second_id])
    send(api)
    receive(api, FIXTURE['end_statuses'][2]['frame'])
    send(api)

    receive(api, FIXTURE['end_statuses'][0]['frame'])

    status = api.lift_move_status(ROBOT)
    assert status.state == LiftMoveState.AWAITING_ANSWER
    assert status.request_id == second_id


def test_generated_request_ids_are_never_the_same():
    request_ids = {new_lift_request_id(ROBOT) for _ in range(10000)}

    assert len(request_ids) == 10000
    assert all(request_id.startswith(f'rmf-{ROBOT}-')
               for request_id in request_ids)


def test_send_failure_leaves_no_move_and_the_next_id_is_new():
    second_id = 'rmf-R3VAC-B00000-00000000000000000000000000000002'
    websocket = FakeWebSocket(fail=True)
    api = make_api(request_ids=[REQUEST_ID, second_id], websocket=websocket)

    with pytest.raises(LiftCommandSendError) as raised:
        send(api)

    assert raised.value.robot_encoding_id == ROBOT
    assert api.lift_move_status(ROBOT) is None
    websocket.fail = False
    assert send(api) == second_id


def test_lift_command_map_is_the_robot_map_of_the_level():
    api = make_api()
    api.get_map = MagicMock(return_value=FIXTURE['map'])

    robot_map = api.lift_command_map(
        ROBOT, FIXTURE['map']['name'], FIXTURE['map']['level'])

    assert robot_map == FIXTURE['map']
    api.get_map.assert_called_once_with(
        map_name=FIXTURE['map']['name'],
        robot_encoding_id=ROBOT,
        map_level=FIXTURE['map']['level'])


def test_send_makes_no_other_request():
    api = make_api()
    robot_map = api.lift_command_map(
        ROBOT, FIXTURE['map']['name'], FIXTURE['map']['level'])
    api.get_map = MagicMock()

    api.send_lift_command(ROBOT, LiftCommand.ENTER, 'lift_a', robot_map)

    api.get_map.assert_not_called()


def test_command_without_the_robot_map_is_not_sent():
    api = make_api()
    api.get_map = lambda map_name, robot_encoding_id, map_level: None

    with pytest.raises(LiftMapUnavailableError) as raised:
        send(api)

    assert raised.value.map_name == FIXTURE['map']['name']
    assert raised.value.map_level == FIXTURE['map']['level']
    assert api.robot_status_ws_connection.frames == []
    assert api.lift_move_status(ROBOT) is None


@pytest.mark.parametrize('error_type', [
    LiftCommandSendError,
    LiftMapUnavailableError,
    LiftMoveInProgressError,
])
def test_each_send_refusal_is_a_lift_command_error(error_type):
    assert issubclass(error_type, LiftCommandError)


def test_abort_sends_p2p_stop():
    api = make_api()
    send(api)

    assert api.abort_lift_move(ROBOT) is True

    stop_frame = api.robot_status_ws_connection.frames[-1]
    assert {key: stop_frame[key] for key in FIXTURE['stop_frame']} == \
        FIXTURE['stop_frame']


def test_clear_forgets_an_ended_move():
    api = make_api()
    send(api)
    receive(api, FIXTURE['end_statuses'][0]['frame'])

    api.clear_lift_move(ROBOT)

    assert api.lift_move_status(ROBOT) is None
