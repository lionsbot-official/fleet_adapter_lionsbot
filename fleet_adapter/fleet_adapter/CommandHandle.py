import itertools
import queue
import threading
import time
import traceback
from collections.abc import Callable

from .enums.enums import ActiveCommand
from .enums.enums import CommandEvent
from .enums.enums import CommandKind
from .enums.enums import CommandState


class CommandHandle:
    """Serializes RMF commands for one physical robot."""

    def __init__(
            self,
            name: str,
            logger,
            run_command: Callable[[CommandEvent], None],
            stop_robot: Callable[[float], None],
            wait_for_robot_terminal: Callable[[], bool],
            timeout_grace_sec: float,
            max_timeout_sec: float):
        self._name = name
        self._logger = logger
        self._run_command = run_command
        self._stop_robot = stop_robot
        self._wait_for_robot_terminal = wait_for_robot_terminal
        self._timeout_grace_sec = timeout_grace_sec
        self._max_timeout_sec = max_timeout_sec

        self._command_queue: queue.Queue[CommandEvent] = queue.Queue()
        self._control_queue: queue.Queue[CommandEvent] = queue.Queue()
        self._worker_wakeup = threading.Event()
        self._command_state_lock = threading.Lock()
        self._active_command: ActiveCommand | None = None
        self._command_ids = itertools.count(1)
        self._running = True
        self._worker_thread = threading.Thread(
            target=self._worker_loop,
            name=f'{name}-command-worker',
            daemon=True)
        self._worker_thread.start()

    def make_event(self, kind: CommandKind, **kwargs) -> CommandEvent:
        return CommandEvent(
            command_id=next(self._command_ids),
            kind=kind,
            **kwargs)

    def enqueue(self, event: CommandEvent):
        self._command_queue.put(event)
        self._worker_wakeup.set()

    def request_stop(self, activity_id) -> bool:
        with self._command_state_lock:
            active = self._active_command
            if active is None or active.activity_id is None:
                return False
            if not active.activity_id.is_same(activity_id):
                return False

        self._control_queue.put(CommandEvent(
            command_id=active.command_id,
            kind=CommandKind.STOP,
            activity_id=activity_id))
        self._worker_wakeup.set()
        return True

    def active_execution(self):
        with self._command_state_lock:
            active = self._active_command
            return active.execution if active is not None else None

    def is_active(self, execution) -> bool:
        with self._command_state_lock:
            active = self._active_command
            if active is None or active.execution is not execution:
                return False
            if active.cancel_requested:
                return False

        if execution is not None and not execution.okay():
            with self._command_state_lock:
                if self._active_command is active:
                    active.state = CommandState.RMF_INVALIDATED
            return False

        return True

    def set_state(self, execution, state: CommandState) -> bool:
        with self._command_state_lock:
            active = self._active_command
            if active is None or active.execution is not execution:
                return False
            active.state = state
            return True

    def start_operation_attempt(self, execution) -> bool:
        now = time.monotonic()
        with self._command_state_lock:
            active = self._active_command
            if active is None or active.execution is not execution:
                return False
            active.operation_started_at = now
            active.operation_deadline = now + self._timeout_grace_sec
            active.operation_attempt += 1
            return True

    def update_operation_eta(self, execution, eta_seconds: float | None):
        if eta_seconds is None or eta_seconds <= 0:
            return
        now = time.monotonic()
        with self._command_state_lock:
            active = self._active_command
            if active is None or active.execution is not execution or \
                    active.operation_started_at is None:
                return
            active.operation_deadline = min(
                now + max(eta_seconds, self._timeout_grace_sec),
                active.operation_started_at + self._max_timeout_sec)

    def operation_timed_out(self, execution) -> bool:
        with self._command_state_lock:
            active = self._active_command
            if active is None or active.execution is not execution or \
                    active.operation_deadline is None:
                return False
            return time.monotonic() >= active.operation_deadline

    def clear_operation_attempt(self, execution):
        with self._command_state_lock:
            active = self._active_command
            if active is None or active.execution is not execution:
                return
            active.operation_started_at = None
            active.operation_deadline = None

    def finish(self, execution) -> bool:
        with self._command_state_lock:
            active = self._active_command
            if active is None or active.execution is not execution:
                return False
            if active.cancel_requested or not execution.okay():
                return False
            active.state = CommandState.FINISHING
            self._active_command = None

        execution.finished()
        return True

    def fail(self, execution, message: str) -> bool:
        with self._command_state_lock:
            active = self._active_command
            if active is None or active.execution is not execution:
                return False
            if active.cancel_requested or not execution.okay():
                return False
            active.state = CommandState.FAILING
            self._active_command = None

        if hasattr(execution, 'error'):
            execution.error(message)
        execution.finished()
        return True

    def wait(self, seconds: float, execution=None) -> bool:
        if self._worker_wakeup.wait(seconds):
            self._worker_wakeup.clear()
        self._process_control_events()
        return execution is None or self.is_active(execution)

    def _worker_loop(self):
        while self._running:
            self._process_control_events()
            try:
                event = self._command_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            self._activate(event)
            try:
                self._run_command(event)
            except Exception as error:
                self._logger.error(
                    f'Unhandled exception in command worker for robot '
                    f'[{self._name}]: {error!r}\n'
                    f'{traceback.format_exc()}')
                self.fail(event.execution, 'Unhandled adapter command error')
            finally:
                self._finalize_unfinished(event)

    def _activate(self, event: CommandEvent):
        target = event.destination
        if target is None:
            target = event.description
        with self._command_state_lock:
            self._active_command = ActiveCommand(
                command_id=event.command_id,
                kind=event.kind,
                state=CommandState.PREPARING,
                execution=event.execution,
                activity_id=(
                    event.execution.identifier
                    if event.execution is not None else event.activity_id),
                rmf_task_id=None,
                target=target,
                started_at=event.created_at)

    def _finalize_unfinished(self, event: CommandEvent):
        with self._command_state_lock:
            active = self._active_command
            if active is None or active.command_id != event.command_id:
                return
            if active.cancel_requested:
                active.state = CommandState.CANCELING

        # A callback may invalidate its RMF execution while the robot is still
        # operating. Do not accept the next queued robot command until the
        # physical robot has reached a terminal/idle state.
        if not self._wait_for_robot_terminal():
            self._logger.error(
                f'Robot [{self._name}] did not reach a terminal state after '
                'command cancellation; subsequent commands may fail until it '
                'reports ready')
        with self._command_state_lock:
            if self._active_command is active:
                self._active_command = None

    def _process_control_events(self):
        while True:
            try:
                event = self._control_queue.get_nowait()
            except queue.Empty:
                return

            if event.kind not in (CommandKind.CANCEL, CommandKind.STOP):
                continue

            with self._command_state_lock:
                active = self._active_command
                if active is None or active.command_id != event.command_id:
                    continue
                active.cancel_requested = True
                active.state = CommandState.STOPPING
                now = time.monotonic()
                active.operation_started_at = now
                active.operation_deadline = now + self._max_timeout_sec
                active.operation_attempt += 1
                stop_deadline = active.operation_deadline

            self._stop_robot(stop_deadline)
