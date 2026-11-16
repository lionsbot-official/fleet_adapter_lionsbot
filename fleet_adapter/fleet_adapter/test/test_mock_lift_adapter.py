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

"""Unit tests for the state of the mock lift."""

from types import SimpleNamespace

import pytest

from .. import mock_lift_adapter as mock
from ..LiftGuard import car_not_ready_problem
from ..LiftGuard import LiftStateCache

ROBOT = 'lb_fleet/R3VAC-B00000'
OTHER_ROBOT = 'lb_fleet/R3VAC-B00001'
TRAVEL_SECONDS = 3.0


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def lift(clock):
    return mock.MockLift('lift_a', ['L7', 'L8'], TRAVEL_SECONDS, clock)


def state(lift):
    lift.update()
    return (
        lift.current_floor,
        lift.destination_floor,
        lift.door_state,
        lift.motion_state,
        lift.session_id)


def call(lift, session_id, floor):
    lift.request(session_id, mock.REQUEST_AGV_MODE, floor, mock.DOOR_OPEN)


def end(lift, session_id):
    lift.request(session_id, mock.REQUEST_END_SESSION, '', mock.DOOR_CLOSED)


def test_starts_at_the_first_floor_with_closed_doors_and_no_session(lift):
    assert state(lift) == (
        'L7', 'L7', mock.DOOR_CLOSED, mock.MOTION_STOPPED, '')
    assert lift.current_mode == mock.MODE_HUMAN


def test_a_call_to_the_car_floor_opens_the_doors(lift):
    call(lift, ROBOT, 'L7')

    assert state(lift) == (
        'L7', 'L7', mock.DOOR_OPEN, mock.MOTION_STOPPED, ROBOT)
    assert lift.current_mode == mock.MODE_AGV


def test_the_car_moves_with_closed_doors_then_opens_them(lift, clock):
    call(lift, ROBOT, 'L8')
    assert state(lift) == ('L7', 'L8', mock.DOOR_CLOSED, mock.MOTION_UP, ROBOT)

    clock.now += TRAVEL_SECONDS - 0.1
    assert state(lift) == ('L7', 'L8', mock.DOOR_CLOSED, mock.MOTION_UP, ROBOT)

    clock.now += 0.1
    assert state(lift) == (
        'L8', 'L8', mock.DOOR_OPEN, mock.MOTION_STOPPED, ROBOT)

    call(lift, ROBOT, 'L7')
    assert state(lift) == (
        'L8', 'L7', mock.DOOR_CLOSED, mock.MOTION_DOWN, ROBOT)


def test_a_car_that_moves_takes_a_new_destination_after_it_stops(lift, clock):
    call(lift, ROBOT, 'L8')
    call(lift, ROBOT, 'L7')
    assert state(lift)[:2] == ('L7', 'L8')

    clock.now += TRAVEL_SECONDS
    call(lift, ROBOT, 'L7')
    assert state(lift)[:2] == ('L8', 'L7')


def test_only_the_session_that_holds_the_car_counts(lift):
    call(lift, ROBOT, 'L7')
    held = state(lift)

    call(lift, OTHER_ROBOT, 'L8')
    end(lift, OTHER_ROBOT)
    # The lift supervisor sends this for a lift that it thinks is free.
    end(lift, '')

    assert state(lift) == held


def test_end_session_frees_the_car_and_closes_the_doors(lift):
    call(lift, ROBOT, 'L7')
    end(lift, ROBOT)
    assert state(lift) == (
        'L7', 'L7', mock.DOOR_CLOSED, mock.MOTION_STOPPED, '')
    assert lift.current_mode == mock.MODE_HUMAN

    call(lift, OTHER_ROBOT, 'L7')
    assert lift.session_id == OTHER_ROBOT


def test_a_request_with_closed_doors_keeps_them_closed(lift):
    lift.request(ROBOT, mock.REQUEST_AGV_MODE, 'L7', mock.DOOR_CLOSED)

    assert state(lift) == (
        'L7', 'L7', mock.DOOR_CLOSED, mock.MOTION_STOPPED, ROBOT)


def test_a_floor_that_the_lift_does_not_serve_is_ignored(lift):
    call(lift, ROBOT, 'L9')

    assert state(lift) == (
        'L7', 'L7', mock.DOOR_CLOSED, mock.MOTION_STOPPED, '')


def test_the_adapter_accepts_the_car_only_when_it_arrived(lift, clock):
    cache = LiftStateCache(clock)

    def problem():
        lift.update()
        cache.update(SimpleNamespace(lift_name=lift.name, **vars(lift)))
        return car_not_ready_problem(
            cache.get('lift_a'), 'L8', ROBOT, clock.now)

    call(lift, ROBOT, 'L8')
    assert problem() is not None

    clock.now += TRAVEL_SECONDS
    assert problem() is None


def test_values_are_those_of_the_rmf_lift_messages():
    messages = pytest.importorskip('rmf_lift_msgs.msg')
    request, lift_state = messages.LiftRequest, messages.LiftState

    assert mock.REQUEST_END_SESSION == request.REQUEST_END_SESSION
    assert mock.REQUEST_AGV_MODE == request.REQUEST_AGV_MODE
    assert mock.DOOR_CLOSED == lift_state.DOOR_CLOSED == request.DOOR_CLOSED
    assert mock.DOOR_OPEN == lift_state.DOOR_OPEN == request.DOOR_OPEN
    assert mock.MOTION_STOPPED == lift_state.MOTION_STOPPED
    assert mock.MOTION_UP == lift_state.MOTION_UP
    assert mock.MOTION_DOWN == lift_state.MOTION_DOWN
    assert mock.MODE_HUMAN == lift_state.MODE_HUMAN
    assert mock.MODE_AGV == lift_state.MODE_AGV
