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

"""Unit tests for the robot moves at a lift."""

from pathlib import Path
import threading
from types import SimpleNamespace
from unittest.mock import create_autospec
from unittest.mock import MagicMock

import pytest
import yaml

from ..CommandHandle import CommandHandle
from ..enums.enums import CommandEvent
from ..enums.enums import CommandKind
from ..enums.enums import CommandResult
from ..enums.enums import LiftCommand
from ..enums.enums import LiftMoveState
from ..enums.enums import LiftMoveStatus
from ..enums.enums import RobotRuntimeStatusText
from ..LiftGuard import LIFT_READY_WAIT_SECONDS
from ..LiftGuard import LiftStateCache
from ..LiftMoveTracker import LiftCommandSendError
from ..LiftMoveTracker import LiftMapUnavailableError
from ..LionsbotRobot import LionsbotRobot
from ..utils.Coordinate import LionsbotCoord
from ..utils.Coordinate import RmfCoord

FIXTURE = yaml.safe_load(
    (Path(__file__).parent / 'fixtures' / 'lift_states.yaml').read_text())
LIFT = FIXTURE['lift']
START_LEVEL = FIXTURE['floor']
END_LEVEL = 'L3'
ROBOT = 'R3VAC-B00000'
REQUEST_ID = 'rmf-R3VAC-B00000-00000000000000000000000000000001'
ROBOT_MAPS = {START_LEVEL: ('E1_L1', '1'), END_LEVEL: ('E1_L3', '3')}
ROBOT_MAP = {'building': 'E1', 'level': '1', 'name': 'E1_L1'}
CABIN_WAYPOINT = 7
CABIN_XY = (19.7, -18.9)
TASK_ID = 'patrol.dispatch-7'
NO_ANSWER = dict(
    error_code='WSSE00019',
    reason_code='RCSE00006',
    reason='RCSE00006: command timed out waiting for robot feedback')


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class TenTimesTransform:
    def rmf_meters_to_robot(self, rmf_coord):
        return LionsbotCoord(
            x=rmf_coord.x * 10.0,
            y=rmf_coord.y * 10.0,
            orientation_radians=rmf_coord.orientation_radians)

    def robot_to_rmf_meters(self, robot_coord):
        return RmfCoord(
            x=robot_coord.x / 10.0,
            y=robot_coord.y / 10.0,
            orientation_radians=robot_coord.orientation_radians)


def lift_state(**changes):
    return SimpleNamespace(**{**FIXTURE['ready_state'], **changes})


def destination(level=START_LEVEL, lift=None, position=(1.0, 2.0, 0.5)):
    return SimpleNamespace(
        map=level,
        position=list(position),
        graph_index=CABIN_WAYPOINT if lift else 3,
        name='',
        dock=None,
        inside_lift=SimpleNamespace(name=lift) if lift else None)


def move_status(state, command=LiftCommand.ENTER, **fields):
    return LiftMoveStatus(
        state=state, command=command, request_id=REQUEST_ID, **fields)


def make_robot(lift_commands=True, max_retries=1):
    clock = FakeClock()
    robot = LionsbotRobot.__new__(LionsbotRobot)
    robot.name = ROBOT
    robot.rmf_name = 'R3VAC_B00000'
    robot.fleet_name = 'r3'
    robot.map_name = START_LEVEL
    robot.robot_map_names = {
        level: names[0] for level, names in ROBOT_MAPS.items()}
    robot.robot_map_levels = {
        level: names[1] for level, names in ROBOT_MAPS.items()}
    robot.transforms = {level: TenTimesTransform() for level in ROBOT_MAPS}
    robot.position = RmfCoord(1.0, 2.0, 0.5)
    robot.navigation_max_retries = max_retries
    robot.dock_paths = {}
    robot.node = MagicMock()
    robot.api = MagicMock()
    robot.api.send_lift_command.return_value = REQUEST_ID
    robot.api.lift_command_map.return_value = ROBOT_MAP
    robot.api.abort_lift_move.return_value = True
    robot.api.position.return_value = LionsbotCoord(10.0, 20.0, 0.5)
    robot.command_handle = MagicMock()
    robot.command_handle.make_event.side_effect = \
        lambda kind, **fields: CommandEvent(command_id=1, kind=kind, **fields)
    robot.update_handle = MagicMock()
    robot.graph = MagicMock()
    robot.graph.all_known_lifts = []
    robot.graph.get_waypoint.return_value = SimpleNamespace(
        location=list(CABIN_XY))
    robot.lift_commands = lift_commands
    robot.lift_states = LiftStateCache(clock=clock)
    robot.lift_states.update(lift_state())
    robot._clock = clock
    robot._lock = threading.Lock()
    robot._pending_dock_destinations = {}
    robot._lift_in_use = None
    robot._lift_move_active = False
    robot._lift_leg_failures = 0
    robot._lift_commands_refusal = None
    robot.update_handle.more().current_task_id.return_value = TASK_ID

    def sleep(seconds, execution=None):
        clock.now += seconds
        return True

    robot._sleep = MagicMock(side_effect=sleep)
    robot.command_handle.wait.side_effect = sleep
    robot._execution_is_active = MagicMock(return_value=True)
    robot._wait_until_ready = MagicMock(return_value=True)
    robot._prepare_map = MagicMock(return_value=True)
    robot._finish_execution = MagicMock()
    robot._fail_execution = MagicMock()
    return robot


def enqueued_event(robot):
    robot.command_handle.enqueue.assert_called_once()
    return robot.command_handle.enqueue.call_args.args[0]


def assert_not_reported_to_rmf_as_arrived(robot, execution):
    robot._finish_execution.assert_not_called()
    robot._fail_execution.assert_not_called()
    robot.command_handle.abandon.assert_called_with(execution)


def put_robot_in_car(robot, lift=LIFT):
    car = MagicMock()
    car.name = lift
    # The RMF binding has no default value for the envelope.
    car.is_in_lift = create_autospec(
        lambda position, envelope: True, return_value=True)
    robot.graph.all_known_lifts = [car]
    return car


def assert_no_point_to_point_move(robot):
    robot.api.send_navigation_command.assert_not_called()
    robot._prepare_map.assert_not_called()


@pytest.mark.parametrize(
    'lift_commands, target_lift, lift_in_use, lift_at_position, leg', [
        (False, None, None, None, None),
        (False, LIFT, None, None, (LiftCommand.ENTER, LIFT)),
        (False, None, None, LIFT, (LiftCommand.EXIT, LIFT)),
        (True, None, None, None, None),
        (True, LIFT, None, None, (LiftCommand.ENTER, LIFT)),
        (True, LIFT, None, LIFT, (LiftCommand.ENTER, LIFT)),
        (True, LIFT, LIFT, LIFT, (LiftCommand.ENTER, LIFT)),
        (True, None, LIFT, LIFT, (LiftCommand.EXIT, LIFT)),
        (True, None, None, LIFT, (LiftCommand.EXIT, LIFT)),
        (True, None, LIFT, None, None),
    ], ids=[
        'flag off, normal move',
        'flag off, into a car',
        'flag off, out of a car',
        'flag on, normal move',
        'flag on, into a car',
        'flag on, into a car again after a failed entry',
        'flag on, to the car the robot is in',
        'flag on, out of the car',
        'flag on, out of a car after an adapter restart',
        'flag on, robot left the car with no completed move out',
    ])
def test_lift_leg_selects_the_command_for_the_move(
        lift_commands, target_lift, lift_in_use, lift_at_position, leg):
    robot = make_robot(lift_commands=lift_commands)
    robot._lift_in_use = lift_in_use
    if lift_at_position:
        put_robot_in_car(robot, lift_at_position)

    assert robot._lift_leg(destination(lift=target_lift)) == leg


def test_robot_that_left_the_car_is_no_longer_in_the_lift():
    robot = make_robot()
    robot._lift_in_use = LIFT

    robot._lift_leg(destination())

    assert robot._lift_in_use is None


def test_lift_leg_uses_the_position_of_the_robot_now():
    robot = make_robot()
    robot.api.position.return_value = LionsbotCoord(197.0, -189.0, 0.5)
    car = put_robot_in_car(robot)
    car.is_in_lift.return_value = False

    assert robot._lift_leg(destination()) is None

    assert list(car.is_in_lift.call_args.args[0]) == pytest.approx(
        [19.7, -18.9])


def test_lift_at_position_with_the_rmf_binding(tmp_path):
    graph = pytest.importorskip('rmf_adapter.graph')
    geometry = pytest.importorskip('rmf_adapter.geometry')
    traits = pytest.importorskip('rmf_adapter.vehicletraits')
    nav_graph = tmp_path / '0.yaml'
    nav_graph.write_text(yaml.safe_dump({
        'building_name': 'test',
        'levels': {START_LEVEL: {
            'lanes': [[0, 1, {}]],
            'vertices': [
                [10.0, -13.0, {'name': 'approach'}],
                [10.0, -15.0, {'name': 'cabin', 'lift': LIFT}]]}},
        'lifts': {LIFT: {'dims': [1.6, 2.0], 'position': [10.0, -15.0, 0.0]}},
    }))
    robot = make_robot()
    robot.graph = graph.parse_graph(str(nav_graph), traits.VehicleTraits(
        linear=traits.Limits(0.5, 0.3),
        angular=traits.Limits(0.6, 0.6),
        profile=traits.Profile(geometry.make_final_convex_circle(0.3))))

    robot.position = RmfCoord(10.0, -15.0, 0.0)
    assert robot._lift_at_position() == LIFT
    robot.position = RmfCoord(10.0, -13.0, 0.0)
    assert robot._lift_at_position() is None


def test_navigate_enqueues_a_move_and_selects_the_command_when_it_runs():
    robot = make_robot()
    put_robot_in_car(robot)
    target = destination()
    execution = MagicMock()
    robot._lift_move_worker = MagicMock()
    robot._navigate_worker = MagicMock()

    robot.navigate(target, execution)
    event = enqueued_event(robot)
    assert event.kind == CommandKind.P2P
    robot._lift_move_worker.assert_not_called()

    robot._run_command_event(event)

    robot._lift_move_worker.assert_called_once_with(
        LiftCommand.EXIT, LIFT, target, execution)
    robot._navigate_worker.assert_not_called()


def test_move_queued_during_an_entry_does_not_leave_the_car_with_p2p():
    robot = make_robot()
    robot._lift_move_worker = MagicMock()
    robot._navigate_worker = MagicMock()
    robot.navigate(destination(), MagicMock())
    event = enqueued_event(robot)

    # The entry that was in progress ends before the queued move starts.
    robot._lift_in_use = LIFT
    put_robot_in_car(robot)
    robot._run_command_event(event)

    robot._navigate_worker.assert_not_called()
    assert robot._lift_move_worker.call_args.args[0] == LiftCommand.EXIT


def test_move_with_an_error_in_the_lift_check_is_not_reported_as_arrived():
    robot = make_robot()
    put_robot_in_car(robot).is_in_lift.side_effect = TypeError(
        'wrong arguments')
    robot._lift_move_worker = MagicMock()
    robot._navigate_worker = MagicMock()
    execution = MagicMock()

    robot._run_command_event(CommandEvent(
        command_id=1, kind=CommandKind.P2P, execution=execution,
        destination=destination()))

    assert_not_reported_to_rmf_as_arrived(robot, execution)
    robot._lift_move_worker.assert_not_called()
    robot._navigate_worker.assert_not_called()
    assert 'wrong arguments' in \
        robot.update_handle.more().log_error.call_args.args[0]


@pytest.mark.parametrize('command, level, robot_map', [
    (LiftCommand.ENTER, START_LEVEL, ROBOT_MAPS[START_LEVEL]),
    (LiftCommand.EXIT, START_LEVEL, ROBOT_MAPS[START_LEVEL]),
], ids=['enter', 'exit'])
def test_lift_move_sends_the_lift_command_and_ends_on_its_end_status(
        command, level, robot_map):
    robot = make_robot()
    robot._lift_in_use = LIFT if command == LiftCommand.EXIT else None
    execution = MagicMock()
    robot.api.lift_move_status.side_effect = [
        move_status(LiftMoveState.AWAITING_ANSWER, command),
        move_status(LiftMoveState.RUNNING, command),
        move_status(LiftMoveState.SUCCEEDED, command),
    ]

    robot._lift_move_worker(
        command, LIFT, destination(lift=LIFT), execution)

    robot.api.lift_command_map.assert_called_once_with(
        ROBOT, robot_map[0], robot_map[1])
    robot.api.send_lift_command.assert_called_once_with(
        ROBOT, command, LIFT, ROBOT_MAP)
    robot._finish_execution.assert_called_once_with(execution)
    robot.api.abort_lift_move.assert_not_called()
    robot.api.clear_lift_move.assert_called_once_with(ROBOT)
    assert robot._lift_in_use == (LIFT if command == LiftCommand.ENTER
                                  else None)
    assert robot._lift_move_active is False
    assert_no_point_to_point_move(robot)


def test_move_to_the_car_the_robot_is_in_ends_with_no_command():
    robot = make_robot()
    robot._lift_in_use = LIFT
    execution = MagicMock()

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), execution)

    robot.api.send_lift_command.assert_not_called()
    robot._finish_execution.assert_called_once_with(execution)
    assert_no_point_to_point_move(robot)


@pytest.mark.parametrize(
    'case',
    [case for case in FIXTURE['ready'] if case['problem']],
    ids=[case['name'] for case in FIXTURE['ready'] if case['problem']])
def test_lift_command_is_not_sent_when_the_car_is_not_ready(case):
    robot = make_robot()
    robot.lift_states = LiftStateCache(clock=robot._clock)
    if case['state'] is not None:
        robot.lift_states.update(SimpleNamespace(**case['state']))
        robot._clock.now += case['age_seconds']
    execution = MagicMock()
    started_at = robot._clock.now

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), execution)

    robot.api.send_lift_command.assert_not_called()
    assert robot._clock.now - started_at >= LIFT_READY_WAIT_SECONDS
    assert_not_reported_to_rmf_as_arrived(robot, execution)
    robot.update_handle.more().replan.assert_called_once_with()
    assert_no_point_to_point_move(robot)


def test_lift_command_waits_for_the_car_to_become_ready():
    robot = make_robot()
    robot.lift_states.update(lift_state(door_state=1))
    execution = MagicMock()
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.SUCCEEDED)

    def doors_open_after_two_seconds(seconds, execution=None):
        robot._clock.now += seconds
        if robot._clock.now >= 102.0:
            robot.lift_states.update(lift_state())
        return True

    robot._sleep.side_effect = doors_open_after_two_seconds

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), execution)

    robot.api.send_lift_command.assert_called_once()
    robot._finish_execution.assert_called_once_with(execution)


def test_lift_command_goes_out_at_once_after_the_lift_check():
    robot = make_robot()
    order = MagicMock()
    order.attach_mock(robot.api.lift_command_map, 'lift_command_map')
    order.attach_mock(robot.api.send_lift_command, 'send_lift_command')
    robot._wait_for_lift_car = MagicMock(return_value=None)
    order.attach_mock(robot._wait_for_lift_car, 'wait_for_lift_car')
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.SUCCEEDED)

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), MagicMock())

    assert [call[0] for call in order.mock_calls] == [
        'lift_command_map', 'wait_for_lift_car', 'send_lift_command']


def test_lift_command_is_not_sent_after_rmf_cancels_the_move():
    robot = make_robot()
    execution = MagicMock()
    robot._execution_is_active.return_value = False

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), execution)

    robot.api.send_lift_command.assert_not_called()
    robot.command_handle.abandon.assert_not_called()
    robot.update_handle.more().replan.assert_not_called()


@pytest.mark.parametrize('command, lift_in_use, status, in_use_after', [
    (LiftCommand.ENTER, None,
     move_status(LiftMoveState.REFUSED, error_code='WSSE00019',
                 reason_code='RCSE00013',
                 reason='RCSE00013: robot is offline'),
     None),
    (LiftCommand.ENTER, None,
     move_status(LiftMoveState.FAILED, error_code='CIM00009'), None),
    (LiftCommand.EXIT, LIFT,
     move_status(LiftMoveState.FAILED, LiftCommand.EXIT,
                 error_code='CIM00445'),
     LIFT),
    (LiftCommand.EXIT, LIFT,
     move_status(LiftMoveState.REFUSED, LiftCommand.EXIT,
                 error_code='WSSE00019', reason_code='RCSE00008',
                 reason='RCSE00008: duplicate command pending'),
     LIFT),
], ids=['enter refused', 'enter failed', 'exit failed', 'exit refused'])
def test_failed_lift_move_asks_rmf_for_a_new_plan(
        command, lift_in_use, status, in_use_after):
    robot = make_robot()
    robot._lift_in_use = lift_in_use
    execution = MagicMock()
    robot.api.lift_move_status.return_value = status

    robot._lift_move_worker(
        command, LIFT, destination(lift=LIFT), execution)

    assert_not_reported_to_rmf_as_arrived(robot, execution)
    robot.update_handle.more().replan.assert_called_once_with()
    assert robot._lift_in_use == in_use_after
    assert robot._lift_move_active is False
    code = status.reason_code or status.error_code
    assert code in robot.node.get_logger().error.call_args.args[0]
    assert_no_point_to_point_move(robot)


@pytest.mark.parametrize('method, error', [
    ('send_lift_command', LiftCommandSendError(ROBOT, LiftCommand.ENTER)),
    ('lift_command_map', LiftMapUnavailableError(ROBOT, 'E1_L1', '1')),
], ids=['send failure', 'robot map not found'])
def test_lift_command_that_is_not_sent_asks_rmf_for_a_new_plan(
        method, error):
    robot = make_robot()
    execution = MagicMock()
    getattr(robot.api, method).side_effect = error

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), execution)

    assert_not_reported_to_rmf_as_arrived(robot, execution)
    robot.update_handle.more().replan.assert_called_once_with()
    robot.api.lift_move_status.assert_not_called()


def test_lift_move_stops_asking_for_a_new_plan_after_the_retry_limit():
    robot = make_robot(max_retries=2)
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.FAILED, error_code='CIM00005')
    rmf_handle = robot.update_handle.more()

    for _ in range(3):
        robot._lift_move_worker(
            LiftCommand.ENTER, LIFT, destination(lift=LIFT), MagicMock())

    assert rmf_handle.replan.call_count == 2
    assert rmf_handle.log_error.call_count == 1
    assert robot.command_handle.abandon.call_count == 3
    robot._finish_execution.assert_not_called()


def test_retry_count_starts_again_after_the_robot_was_held():
    robot = make_robot(max_retries=0)
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.FAILED, error_code='CIM00005')
    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), MagicMock())

    assert robot._lift_leg_failures == 0
    robot.update_handle.more().replan.assert_not_called()


def test_successful_lift_move_resets_the_retry_count():
    robot = make_robot(max_retries=1)
    robot.api.lift_move_status.side_effect = [
        move_status(LiftMoveState.FAILED, error_code='CIM00009'),
        move_status(LiftMoveState.SUCCEEDED),
    ]

    for _ in range(2):
        robot._lift_move_worker(
            LiftCommand.ENTER, LIFT, destination(lift=LIFT), MagicMock())

    assert robot._lift_leg_failures == 0


@pytest.mark.parametrize('change', [
    {'motion_state': 1},
    {'current_floor': END_LEVEL},
    {'session_id': 'r3/R3VAC_B00001'},
], ids=['car moves', 'car at a different floor', 'session changes'])
def test_lift_move_stops_the_robot_when_the_car_is_not_safe(change):
    robot = make_robot()
    execution = MagicMock()
    statuses = iter([
        move_status(LiftMoveState.RUNNING),
        move_status(LiftMoveState.RUNNING),
        move_status(LiftMoveState.RUNNING),
        move_status(LiftMoveState.FAILED, aborted=True),
    ])

    def next_status(robot_encoding_id):
        status = next(statuses)
        robot.lift_states.update(lift_state(**change))
        return status

    robot.api.lift_move_status.side_effect = next_status

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), execution)

    robot.api.abort_lift_move.assert_called_once_with(ROBOT)
    assert_not_reported_to_rmf_as_arrived(robot, execution)
    robot.update_handle.more().replan.assert_not_called()
    robot.update_handle.more().log_error.assert_called_once()
    assert robot._lift_in_use is None


@pytest.mark.parametrize('command, lift_in_use', [
    (LiftCommand.ENTER, None),
    (LiftCommand.EXIT, LIFT),
], ids=['enter', 'exit'])
def test_failed_lift_move_after_the_robot_moved_gets_no_new_plan(
        command, lift_in_use):
    robot = make_robot()
    robot._lift_in_use = lift_in_use
    execution = MagicMock()
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.FAILED, command, error_code='CIM00445')
    robot.api.position.side_effect = [
        LionsbotCoord(10.0, 20.0, 0.5),
        LionsbotCoord(10.0, 23.0, 0.5),
    ]

    robot._lift_move_worker(
        command, LIFT, destination(lift=LIFT), execution)

    assert_not_reported_to_rmf_as_arrived(robot, execution)
    robot.update_handle.more().replan.assert_not_called()
    robot.update_handle.more().log_error.assert_called_once()
    assert 'moved' in robot.update_handle.more().log_error.call_args.args[0]
    assert robot._lift_in_use == lift_in_use


def test_stop_is_sent_again_when_the_websocket_did_not_take_it():
    robot = make_robot()
    robot.api.abort_lift_move.side_effect = [False, False, True]
    statuses = iter([
        move_status(LiftMoveState.RUNNING),
        move_status(LiftMoveState.RUNNING),
        move_status(LiftMoveState.RUNNING),
        move_status(LiftMoveState.RUNNING),
        move_status(LiftMoveState.FAILED, aborted=True),
    ])

    def next_status(robot_encoding_id):
        robot.lift_states.update(lift_state(motion_state=1))
        return next(statuses)

    robot.api.lift_move_status.side_effect = next_status

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), MagicMock())

    assert robot.api.abort_lift_move.call_count == 3


def test_watchdog_stop_is_sent_again_when_the_websocket_did_not_take_it():
    robot = make_robot()
    robot.api.abort_lift_move.side_effect = [False, True]
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.TIMED_OUT)

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), MagicMock())

    assert robot.api.abort_lift_move.call_count == 2


@pytest.mark.parametrize('worker, arguments', [
    ('_lift_move_worker', (LiftCommand.ENTER, LIFT)),
    ('_localize_worker', ()),
], ids=['lift move', 'localize'])
def test_adapter_error_at_a_lift_is_not_reported_to_rmf_as_arrived(
        worker, arguments):
    robot = make_robot()
    robot._lift_move_active = True
    execution = MagicMock()
    failing_worker = MagicMock(side_effect=ValueError('not JSON'))

    robot._run_lift_step(
        execution, failing_worker, *arguments, destination(), execution)

    assert_not_reported_to_rmf_as_arrived(robot, execution)
    robot.update_handle.more().replan.assert_not_called()
    assert 'not JSON' in robot.update_handle.more().log_error.call_args.args[0]
    assert robot._lift_move_active is False


def test_lift_move_that_a_stop_command_ended_gets_no_new_plan():
    robot = make_robot()
    execution = MagicMock()
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.FAILED, aborted=True)

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), execution)

    robot.api.abort_lift_move.assert_not_called()
    assert_not_reported_to_rmf_as_arrived(robot, execution)
    robot.update_handle.more().replan.assert_not_called()


def test_watchdog_timeout_holds_the_robot_with_no_new_plan():
    robot = make_robot()
    execution = MagicMock()
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.TIMED_OUT)

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), execution)

    assert_not_reported_to_rmf_as_arrived(robot, execution)
    robot.update_handle.more().replan.assert_not_called()
    robot.update_handle.more().log_error.assert_called_once()


def test_lift_move_on_a_different_level_is_not_sent():
    robot = make_robot()
    robot._lift_in_use = LIFT
    execution = MagicMock()

    robot._lift_move_worker(
        LiftCommand.EXIT, LIFT, destination(level=END_LEVEL), execution)

    robot.api.send_lift_command.assert_not_called()
    assert_not_reported_to_rmf_as_arrived(robot, execution)
    robot.update_handle.more().replan.assert_not_called()
    assert robot._lift_in_use == LIFT
    assert_no_point_to_point_move(robot)


def test_rmf_stop_does_not_stop_a_lift_move():
    robot = make_robot()
    robot._lift_move_active = True

    assert robot._stop_robot_until_success(robot._clock() + 5.0) is True

    robot.api.stop.assert_not_called()


def test_rmf_stop_stops_a_move_that_is_not_a_lift_move():
    robot = make_robot()
    robot.max_timeout_sec = 5.0
    robot.api.stop.return_value = True

    import time
    assert robot._stop_robot_until_success(time.monotonic() + 5.0) is True

    robot.api.stop.assert_called_once()


def test_localize_changes_the_map_and_sets_the_pose_at_the_cabin_point():
    robot = make_robot()
    robot._lift_in_use = LIFT
    execution = MagicMock()
    robot.api.change_map.return_value = True
    robot.api.localize.return_value = True
    estimate = destination(
        level=END_LEVEL, lift=None, position=(19.6, -18.8, 1.5))
    estimate.graph_index = CABIN_WAYPOINT

    robot._localize_worker(estimate, execution)

    robot.api.change_map.assert_called_once_with(
        robot_encoding_id=ROBOT,
        map_name=ROBOT_MAPS[END_LEVEL][0],
        map_level=ROBOT_MAPS[END_LEVEL][1])
    robot.graph.get_waypoint.assert_called_once_with(CABIN_WAYPOINT)
    seed = robot.api.localize.call_args.args[0]
    assert (seed.x, seed.y) == pytest.approx(
        (CABIN_XY[0] * 10.0, CABIN_XY[1] * 10.0))
    assert seed.orientation_radians == pytest.approx(1.5)
    assert robot.api.localize.call_args.kwargs == {'robot_encoding_id': ROBOT}
    assert robot.map_name == END_LEVEL
    assert robot._lift_in_use == LIFT
    robot._finish_execution.assert_called_once_with(execution)


def test_localize_without_a_waypoint_uses_the_estimate_position():
    robot = make_robot()
    execution = MagicMock()
    robot.api.change_map.return_value = True
    robot.api.localize.return_value = True
    estimate = destination(level=END_LEVEL, position=(19.6, -18.8, 1.5))
    estimate.graph_index = None

    robot._localize_worker(estimate, execution)

    seed = robot.api.localize.call_args.args[0]
    assert (seed.x, seed.y) == pytest.approx((196.0, -188.0))
    robot._finish_execution.assert_called_once_with(execution)


def test_localize_on_the_current_level_ends_with_no_map_change():
    robot = make_robot()
    execution = MagicMock()

    robot._localize_worker(destination(level=START_LEVEL), execution)

    robot.api.change_map.assert_not_called()
    robot.api.localize.assert_not_called()
    robot._finish_execution.assert_called_once_with(execution)


@pytest.mark.parametrize('change_map, localize, change_calls', [
    ([False, False], [], 2),
    ([True, True], [False, False], 2),
], ids=['map change fails', 'set pose fails'])
def test_localize_that_fails_is_not_reported_to_rmf_as_done(
        change_map, localize, change_calls):
    robot = make_robot(max_retries=1)
    execution = MagicMock()
    robot.api.change_map.side_effect = change_map
    robot.api.localize.side_effect = localize

    robot._localize_worker(destination(level=END_LEVEL), execution)

    assert robot.api.change_map.call_count == change_calls
    assert robot.map_name == START_LEVEL
    assert_not_reported_to_rmf_as_arrived(robot, execution)
    robot.update_handle.more().log_error.assert_called_once()


def test_localize_retries_until_the_pose_is_set():
    robot = make_robot(max_retries=1)
    execution = MagicMock()
    robot.api.change_map.return_value = True
    robot.api.localize.side_effect = [False, True]

    robot._localize_worker(destination(level=END_LEVEL), execution)

    assert robot.map_name == END_LEVEL
    robot._finish_execution.assert_called_once_with(execution)


def test_localize_enqueues_a_localize_command():
    robot = make_robot()
    estimate = destination(level=END_LEVEL)
    execution = MagicMock()

    robot.localize(estimate, execution)

    event = enqueued_event(robot)
    assert event.kind == CommandKind.LOCALIZE
    assert event.destination is estimate
    assert event.execution is execution


def test_queued_localize_command_runs_the_localize_worker():
    robot = make_robot()
    robot._localize_worker = MagicMock()
    estimate = destination(level=END_LEVEL)
    execution = MagicMock()

    robot._run_command_event(CommandEvent(
        command_id=1, kind=CommandKind.LOCALIZE, execution=execution,
        destination=estimate))

    robot._localize_worker.assert_called_once_with(estimate, execution)


def test_abandoned_command_is_not_finished_or_failed_towards_rmf():
    handle = CommandHandle(
        name=ROBOT,
        logger=MagicMock(),
        run_command=MagicMock(),
        stop_robot=MagicMock(),
        wait_for_robot_terminal=MagicMock(return_value=True),
        timeout_grace_sec=1.0,
        max_timeout_sec=1.0)
    handle._running = False
    execution = MagicMock()
    event = handle.make_event(CommandKind.P2P, execution=execution)
    handle._activate(event)

    assert handle.abandon(MagicMock()) is False
    assert handle.active_execution() is execution
    assert handle.abandon(execution) is True

    assert handle.active_execution() is None
    assert handle.fail(execution, 'late error') is False
    assert handle.finish(execution) is False
    assert handle.request_stop(execution.identifier) is False
    handle._finalize_unfinished(event)
    execution.finished.assert_not_called()
    handle._wait_for_robot_terminal.assert_not_called()


def make_trip_robot():
    """Return a robot whose queue runs each command at once."""
    robot = make_robot()
    for name in ('_wait_until_ready', '_prepare_map', '_finish_execution',
                 '_fail_execution', '_execution_is_active'):
        delattr(robot, name)
    robot.max_timeout_sec = 5.0
    handle = robot.command_handle
    handle.enqueue.side_effect = robot._run_command_event
    handle.is_active.return_value = True
    handle.operation_timed_out.return_value = False
    handle.finish.side_effect = lambda execution: execution.finished()
    api = robot.api
    api.get_robot_status.return_value = {
        'status': RobotRuntimeStatusText.RESTING.value, 'localized': True}
    api.send_navigation_command.return_value = True
    api.navigate_robot.return_value = CommandResult.SUCCESS
    api.change_map.return_value = True
    api.localize.return_value = True
    api.lift_move_status.side_effect = lambda robot_encoding_id: move_status(
        LiftMoveState.SUCCEEDED)
    return robot


def test_two_floor_trip_uses_lift_commands_at_the_car_and_p2p_elsewhere():
    robot = make_trip_robot()
    api = robot.api
    executions = [MagicMock() for _ in range(5)]

    # RMF: drive to the point in front of the lift on the start level.
    robot.navigate(destination(START_LEVEL), executions[0])
    # RMF and the lift adapter bring the car; then RMF moves the robot in.
    robot.lift_states.update(lift_state(current_floor=START_LEVEL))
    robot.navigate(destination(START_LEVEL, lift=LIFT), executions[1])
    car = put_robot_in_car(robot)
    # The car rides to the end level and RMF asks for the new position.
    robot.lift_states.update(lift_state(current_floor=END_LEVEL))
    robot.localize(destination(END_LEVEL), executions[2])
    # RMF moves the robot out, then on to its goal.
    robot.navigate(destination(END_LEVEL), executions[3])
    car.is_in_lift.return_value = False
    robot.navigate(destination(END_LEVEL), executions[4])

    commands = [
        (call[0], call.args[1] if call[0] == 'send_lift_command' else None)
        for call in api.mock_calls
        if call[0] in ('send_navigation_command', 'send_lift_command',
                       'change_map', 'localize')]
    assert commands == [
        ('send_navigation_command', None),
        ('send_lift_command', LiftCommand.ENTER),
        ('change_map', None),
        ('localize', None),
        ('send_lift_command', LiftCommand.EXIT),
        ('send_navigation_command', None),
    ]
    lift_maps = [
        call.args[1] for call in api.lift_command_map.call_args_list]
    assert lift_maps == [ROBOT_MAPS[START_LEVEL][0], ROBOT_MAPS[END_LEVEL][0]]
    for execution in executions:
        execution.finished.assert_called_once_with()
    assert robot.map_name == END_LEVEL
    assert robot._lift_in_use is None


def test_with_the_flag_off_the_robot_is_not_moved_into_a_car():
    robot = make_trip_robot()
    robot.lift_commands = False
    into_car = MagicMock()

    robot.navigate(destination(START_LEVEL), MagicMock())
    robot.navigate(destination(START_LEVEL, lift=LIFT), into_car)

    robot.api.send_lift_command.assert_not_called()
    assert robot.api.send_navigation_command.call_count == 1
    into_car.finished.assert_not_called()
    robot.command_handle.abandon.assert_called_with(into_car)
    cancel = robot.update_handle.more().cancel_task
    cancel.assert_called_once()
    assert 'lift_commands' in cancel.call_args.args[1][0]


def test_with_the_flag_off_the_robot_is_not_moved_out_of_a_car():
    robot = make_trip_robot()
    robot.lift_commands = False
    put_robot_in_car(robot)
    out_of_car = MagicMock()

    robot.navigate(destination(START_LEVEL), out_of_car)

    robot.api.send_navigation_command.assert_not_called()
    robot.api.send_lift_command.assert_not_called()
    out_of_car.finished.assert_not_called()
    robot.update_handle.more().cancel_task.assert_not_called()


@pytest.mark.parametrize('command, reason, words', [
    (LiftCommand.ENTER, NO_ANSWER, 'did not answer'),
    (LiftCommand.EXIT, NO_ANSWER, 'did not answer'),
    (LiftCommand.ENTER,
     dict(error_code='WSSE00019', reason_code='RCSE00005',
          reason='RCSE00005: robot does not satisfy the minimum autonomy '
                 'version'),
     'does not support'),
], ids=['no answer to enter', 'no answer to exit', 'model not supported'])
def test_robot_that_cannot_use_lift_commands_fails_with_a_plain_message(
        command, reason, words):
    robot = make_robot(max_retries=5)
    execution = MagicMock()
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.REFUSED, command, **reason)

    robot._lift_move_worker(
        command, LIFT, destination(lift=LIFT), execution)

    assert_not_reported_to_rmf_as_arrived(robot, execution)
    rmf_handle = robot.update_handle.more()
    rmf_handle.replan.assert_not_called()
    message = rmf_handle.log_error.call_args.args[0]
    assert words in message
    assert reason['reason_code'] in message
    assert 'did not move' in message
    assert 'Contact LionsBot' in message
    assert message in robot.node.get_logger().error.call_args.args[0]
    assert robot._lift_commands_refusal == message


def test_task_of_a_robot_that_did_not_answer_at_the_lift_is_cancelled():
    robot = make_robot()
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.REFUSED, **NO_ANSWER)

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), MagicMock())

    cancel = robot.update_handle.more().cancel_task
    cancel.assert_called_once()
    assert cancel.call_args.args[0] == TASK_ID
    assert 'did not answer' in cancel.call_args.args[1][0]
    assert callable(cancel.call_args.args[2])


@pytest.mark.parametrize('lift_in_use', [LIFT, None], ids=[
    'after a completed move into the car',
    'in the car after an adapter restart',
])
def test_task_is_not_cancelled_when_the_robot_is_in_the_car(lift_in_use):
    robot = make_robot()
    robot._lift_in_use = lift_in_use
    put_robot_in_car(robot)
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.REFUSED, LiftCommand.EXIT, **NO_ANSWER)

    robot._lift_move_worker(
        LiftCommand.EXIT, LIFT, destination(), MagicMock())

    robot.update_handle.more().cancel_task.assert_not_called()
    robot.update_handle.more().log_error.assert_called_once()
    assert robot._lift_in_use == lift_in_use


def test_no_task_to_cancel_is_not_an_error():
    robot = make_robot()
    robot.update_handle.more().current_task_id.return_value = ''
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.REFUSED, **NO_ANSWER)

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), MagicMock())

    robot.update_handle.more().cancel_task.assert_not_called()


def test_robot_that_did_not_answer_gets_no_more_lift_commands():
    robot = make_robot()
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.REFUSED, **NO_ANSWER)
    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), MagicMock())
    first_message = robot.update_handle.more().log_error.call_args.args[0]
    execution = MagicMock()

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), execution)

    robot.api.send_lift_command.assert_called_once()
    assert_not_reported_to_rmf_as_arrived(robot, execution)
    rmf_handle = robot.update_handle.more()
    assert rmf_handle.log_error.call_count == 2
    assert rmf_handle.log_error.call_args.args[0] == first_message
    assert rmf_handle.cancel_task.call_count == 2
    rmf_handle.replan.assert_not_called()
    assert_no_point_to_point_move(robot)


@pytest.mark.parametrize('reason_code', ['RCSE00013', 'RCSE00014', 'RCSE00008'])
def test_refusal_that_can_clear_keeps_the_robot_able_to_use_lifts(
        reason_code):
    robot = make_robot()
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.REFUSED, error_code='WSSE00019',
        reason_code=reason_code, reason=f'{reason_code}: not now')

    robot._lift_move_worker(
        LiftCommand.ENTER, LIFT, destination(lift=LIFT), MagicMock())

    assert robot._lift_commands_refusal is None
    robot.update_handle.more().replan.assert_called_once_with()
    robot.update_handle.more().cancel_task.assert_not_called()


def test_single_level_work_continues_after_the_robot_did_not_answer():
    robot = make_trip_robot()
    robot.api.lift_move_status.side_effect = None
    robot.api.lift_move_status.return_value = move_status(
        LiftMoveState.REFUSED, **NO_ANSWER)
    lift_execution = MagicMock()
    robot.navigate(destination(START_LEVEL, lift=LIFT), lift_execution)
    assert robot._lift_commands_refusal is not None
    lift_execution.finished.assert_not_called()
    execution = MagicMock()

    robot.navigate(destination(START_LEVEL), execution)

    robot.api.send_navigation_command.assert_called_once()
    execution.finished.assert_called_once_with()
