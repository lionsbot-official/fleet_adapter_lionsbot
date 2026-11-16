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

"""State of the lift command that each robot runs."""

from collections.abc import Callable
from dataclasses import dataclass
from dataclasses import replace
import re
import threading
import time
import uuid

from .enums.enums import LiftCommand
from .enums.enums import LiftMoveState
from .enums.enums import LiftMoveStatus

REQUEST_ID_PREFIX = 'rmf'

MESSAGE_OPERATION_KEY = 'operation_fb'
MESSAGE_CONTENT_KEY = 'content'
CONTENT_STATUS_KEY = 'status'
CONTENT_REQUEST_ID_KEY = 'request_id'
CONTENT_ERROR_CODE_KEY = 'error_code'
CONTENT_ABORTED_KEY = 'aborted'
CONTENT_ERROR_KEY = 'error'
ERROR_CODE_KEY = 'errorCode'
ERROR_DATA_KEY = 'errorData'
ERROR_REASON_KEY = 'reason'

_REASON_CODE = re.compile(r'[A-Z]+[0-9]+')


class LiftCommandError(Exception):
    """A lift command was not sent to the robot."""


class LiftMoveInProgressError(LiftCommandError):
    """The robot has a lift command that has not ended."""

    def __init__(self, robot_encoding_id: str, request_id: str):
        super().__init__(
            f'Robot [{robot_encoding_id}] has a lift command that has not '
            f'ended (request_id {request_id})')
        self.robot_encoding_id = robot_encoding_id
        self.request_id = request_id


class LiftMapUnavailableError(LiftCommandError):
    """The robot map that the lift command must name was not found."""

    def __init__(self, robot_encoding_id: str, map_name: str, map_level):
        super().__init__(
            f'Robot map [{map_name}] level [{map_level}] of robot '
            f'[{robot_encoding_id}] is not available for a lift command')
        self.robot_encoding_id = robot_encoding_id
        self.map_name = map_name
        self.map_level = map_level


class LiftCommandSendError(LiftCommandError):
    """The WebSocket did not take the lift command."""

    def __init__(self, robot_encoding_id: str, command: LiftCommand):
        super().__init__(
            f'Lift command [{command.value}] was not sent to robot '
            f'[{robot_encoding_id}]')
        self.robot_encoding_id = robot_encoding_id
        self.command = command


def new_lift_request_id(robot_encoding_id: str) -> str:
    """Return a request ID that no other lift command has.

    The robot does not remember an ended request ID: the same ID sent again
    starts the move again. A random ID also stays new after a restart.
    """
    return f'{REQUEST_ID_PREFIX}-{robot_encoding_id}-{uuid.uuid4().hex}'


@dataclass
class _LiftMove:
    status: LiftMoveStatus
    sent_at: float
    stop_claimed: bool = False


class LiftMoveTracker:
    """Follows one lift command for each robot, from send to end."""

    # The server reports a command that did not reach the robot in this time.
    NO_ERROR_SECONDS = 20.0
    # The robot ends a lift move in less than this time.
    WATCHDOG_SECONDS = 360.0

    def __init__(
            self,
            clock: Callable[[], float] = time.monotonic,
            new_request_id: Callable[[str], str] = new_lift_request_id):
        self._clock = clock
        self._new_request_id = new_request_id
        self._lock = threading.Lock()
        self._moves: dict[str, _LiftMove] = {}

    def begin(self, robot_encoding_id: str, command: LiftCommand) -> str:
        """Record a new lift command and return its request ID."""
        with self._lock:
            current = self._current(robot_encoding_id)
            if current is not None and not current.status.state.ended:
                raise LiftMoveInProgressError(
                    robot_encoding_id, current.status.request_id)
            request_id = self._new_request_id(robot_encoding_id)
            self._moves[robot_encoding_id] = _LiftMove(
                status=LiftMoveStatus(
                    state=LiftMoveState.AWAITING_ANSWER,
                    command=command,
                    request_id=request_id),
                sent_at=self._clock())
            return request_id

    def discard(self, robot_encoding_id: str, request_id: str):
        """Forget a command that the WebSocket did not take."""
        with self._lock:
            move = self._moves.get(robot_encoding_id)
            if move is not None and move.status.request_id == request_id:
                del self._moves[robot_encoding_id]

    def clear(self, robot_encoding_id: str):
        """Forget the robot's lift command when it has ended."""
        with self._lock:
            current = self._current(robot_encoding_id)
            if current is not None and current.status.state.ended:
                del self._moves[robot_encoding_id]

    def status(self, robot_encoding_id: str) -> LiftMoveStatus | None:
        with self._lock:
            current = self._current(robot_encoding_id)
            return current.status if current is not None else None

    def claim_stop(self, robot_encoding_id: str) -> bool:
        """Return True one time for a move that the watchdog ended."""
        with self._lock:
            current = self._current(robot_encoding_id)
            if current is None or current.stop_claimed or \
                    current.status.state != LiftMoveState.TIMED_OUT:
                return False
            current.stop_claimed = True
            return True

    def handle_message(self, robot_encoding_id: str, message: dict) -> bool:
        """Apply a robot status message; return True when it ended the move."""
        content = message.get(MESSAGE_CONTENT_KEY)
        if not isinstance(content, dict):
            return False
        operation_fb = message.get(MESSAGE_OPERATION_KEY)

        with self._lock:
            current = self._current(robot_encoding_id)
            if current is None or current.status.state.ended:
                return False
            command = current.status.command

            if operation_fb == command.end_status:
                ended = self._end_status(current.status, content)
            elif operation_fb == command.value and \
                    current.status.state == LiftMoveState.AWAITING_ANSWER:
                # After this time the robot has the command. A later error
                # with this command name is not for this command.
                ended = self._command_error(current.status, content)
            else:
                return False

            if ended is None:
                return False
            current.status = ended
            return True

    def _current(self, robot_encoding_id: str) -> _LiftMove | None:
        move = self._moves.get(robot_encoding_id)
        if move is None or move.status.state.ended:
            return move

        elapsed = self._clock() - move.sent_at
        if elapsed >= self.WATCHDOG_SECONDS:
            move.status = replace(move.status, state=LiftMoveState.TIMED_OUT)
        elif elapsed >= self.NO_ERROR_SECONDS:
            move.status = replace(move.status, state=LiftMoveState.RUNNING)
        return move

    @staticmethod
    def _end_status(
            status: LiftMoveStatus, content: dict) -> LiftMoveStatus | None:
        # The robot also sends these end statuses for a lift ride that it
        # runs itself. Those have no request_id and are not for this command.
        request_id = content.get(CONTENT_REQUEST_ID_KEY)
        if not request_id or request_id != status.request_id:
            return None
        succeeded = content.get(CONTENT_STATUS_KEY)
        if not isinstance(succeeded, bool):
            return None
        if succeeded:
            return replace(status, state=LiftMoveState.SUCCEEDED)
        return replace(
            status,
            state=LiftMoveState.FAILED,
            error_code=content.get(CONTENT_ERROR_CODE_KEY),
            aborted=content.get(CONTENT_ABORTED_KEY) is True)

    @staticmethod
    def _command_error(
            status: LiftMoveStatus, content: dict) -> LiftMoveStatus | None:
        error = content.get(CONTENT_ERROR_KEY)
        if content.get(CONTENT_STATUS_KEY) is not False or \
                not isinstance(error, dict):
            return None
        error_data = error.get(ERROR_DATA_KEY)
        reason = None
        if isinstance(error_data, dict):
            reason = error_data.get(ERROR_REASON_KEY)
        reason_code = None
        if isinstance(reason, str):
            code = reason.split(':', 1)[0].strip()
            if _REASON_CODE.fullmatch(code):
                reason_code = code
        return replace(
            status,
            state=LiftMoveState.REFUSED,
            error_code=error.get(ERROR_CODE_KEY),
            reason_code=reason_code,
            reason=reason if isinstance(reason, str) else None)
