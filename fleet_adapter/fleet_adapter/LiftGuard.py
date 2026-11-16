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

"""Checks of the lift car before and during a robot move at a lift.

RMF and the site's lift adapter request the lift and hold its doors. The fleet
adapter only reads the lift state that the lift adapter publishes.
"""

from collections.abc import Callable
from dataclasses import dataclass
import threading
import time

from .enums.enums import LiftCarProblem
from .enums.enums import LiftDoorState
from .enums.enums import LiftMotionState

# A lift adapter publishes its state about one time each second. An older
# state does not show where the car is now.
LIFT_STATE_MAX_AGE_SECONDS = 10.0
LIFT_READY_WAIT_SECONDS = 10.0
# A robot that ends a failed lift move farther than this from its start
# position has moved.
LIFT_MOVE_MOTION_TOLERANCE_METERS = 0.2
LIFT_STOP_ATTEMPTS = 5


@dataclass(frozen=True)
class LiftCarState:
    lift_name: str
    current_floor: str
    door_state: int
    motion_state: int
    session_id: str
    received_at: float


class LiftStateCache:
    """The latest state of each lift, shared by all robots of the fleet."""

    def __init__(self, clock: Callable[[], float] = time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._states: dict[str, LiftCarState] = {}

    def update(self, lift_state):
        """Store one rmf_lift_msgs/LiftState message."""
        state = LiftCarState(
            lift_name=lift_state.lift_name,
            current_floor=lift_state.current_floor,
            door_state=lift_state.door_state,
            motion_state=lift_state.motion_state,
            session_id=lift_state.session_id,
            received_at=self._clock())
        with self._lock:
            self._states[state.lift_name] = state

    def get(self, lift_name: str) -> LiftCarState | None:
        with self._lock:
            return self._states.get(lift_name)


def car_not_ready_problem(
        state: LiftCarState | None,
        floor: str,
        session_id: str,
        now: float) -> LiftCarProblem | None:
    """Return why the robot must not start a lift move, or None."""
    if state is None:
        return LiftCarProblem.NO_STATE
    if now - state.received_at > LIFT_STATE_MAX_AGE_SECONDS:
        return LiftCarProblem.STALE_STATE
    if state.session_id != session_id:
        return LiftCarProblem.OTHER_SESSION
    if state.current_floor != floor:
        return LiftCarProblem.OTHER_FLOOR
    if state.motion_state != LiftMotionState.STOPPED:
        return LiftCarProblem.NOT_STOPPED
    if state.door_state != LiftDoorState.OPEN:
        return LiftCarProblem.DOORS_NOT_OPEN
    return None


def car_unsafe_problem(
        state: LiftCarState | None,
        floor: str,
        session_id: str) -> LiftCarProblem | None:
    """Return why a lift move in progress must stop, or None."""
    if state is None:
        return None
    if state.session_id != session_id:
        return LiftCarProblem.OTHER_SESSION
    if state.current_floor != floor or state.motion_state in (
            LiftMotionState.UP, LiftMotionState.DOWN):
        return LiftCarProblem.CAR_MOVED
    return None
