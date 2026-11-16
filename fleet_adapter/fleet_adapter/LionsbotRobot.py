import math
import os
import threading
import time
import traceback
import uuid

import numpy as np
import rclpy
import rmf_adapter
import rmf_adapter.easy_full_control as rmf_easy
import requests

from .enums.enums import CommandResult
from .enums.enums import CommandKind
from .enums.enums import CommandState
from .enums.enums import LiftCommand
from .enums.enums import LiftMoveState
from .enums.enums import LiftMoveStatus
from .enums.enums import LiftRefusalReason
from .enums.enums import OperationStatus
from .enums.enums import RobotRuntimeStatusText
from .CommandHandle import CommandHandle
from .LiftGuard import car_not_ready_problem
from .LiftGuard import car_unsafe_problem
from .LiftGuard import LIFT_MOVE_MOTION_TOLERANCE_METERS
from .LiftGuard import LIFT_READY_WAIT_SECONDS
from .LiftGuard import LIFT_STOP_ATTEMPTS
from .LiftGuard import LiftStateCache
from .LiftMoveTracker import LiftCommandError
from .models.DockProcessContent import DockProcessContent
from .RobotClientAPI import RobotAPI
from .utils.Coordinate import RmfCoord
from .utils.MapTransform import MapTransform

# Refusals that do not clear until LionsBot changes the robot.
_LIFT_COMMANDS_NOT_AVAILABLE = {
    LiftRefusalReason.NO_ROBOT_ANSWER:
        'did not answer the lift command. Lift commands are not enabled on '
        'this robot',
    LiftRefusalReason.ROBOT_MODEL_NOT_SUPPORTED:
        'does not support lift commands',
}

# Robot states that need a person. The adapter reports them to RMF and sends
# the robot no command because of them.
_FAULT_STATUSES = (
    RobotRuntimeStatusText.ESTOP.value,
    RobotRuntimeStatusText.ERROR.value,
    RobotRuntimeStatusText.OFFLINE.value,
)

_CLEANING_STATUSES = (
    RobotRuntimeStatusText.CLEANING.value,
    RobotRuntimeStatusText.CLEANING_PAUSED.value,
)

# The robot docks by itself only after it ends a clean.
_DOCKING_STATUSES = (
    RobotRuntimeStatusText.DOCKING.value,
    RobotRuntimeStatusText.DOCKING_MOVE.value,
    RobotRuntimeStatusText.DOCKING_IN_PROGRESS.value,
    RobotRuntimeStatusText.DOCKING_PAUSED.value,
    RobotRuntimeStatusText.DOCKED.value,
    RobotRuntimeStatusText.CHARGING.value,
)


class LionsbotRobot:
    def __init__(
        self,
        name: str,
        fleet_name: str,
        config: dict,
        rmf_config: dict,
        node,
        nav_graph,
        transforms: dict[str, MapTransform],
        map_name: str,
        robot_map_names: dict[str, str],
        position: RmfCoord,
        configuration,
        dock_paths: dict,
        api: RobotAPI,
        robot_map_levels: dict[str, str] | None = None,
        rmf_name: str | None = None,
        lift_commands: bool = False,
        lift_states: LiftStateCache | None = None,
    ):
        # `name` is the LionsBot encoding ID used for every cloud API call.
        # `rmf_name` is what RMF is told, which must be a valid ROS topic
        # segment; they differ whenever the vendor ID contains a hyphen.
        self.name = name
        self.rmf_name = rmf_name or name
        self.fleet_name = fleet_name
        self.config = config
        self.rmf_config = rmf_config
        self.node = node
        self.graph = nav_graph
        self.transforms = transforms
        self.map_name = map_name
        self.robot_map_names = robot_map_names
        self.robot_map_levels = robot_map_levels or {}
        self.position = position
        self.configuration = configuration
        self.dock_paths = dock_paths
        self.api = api
        self.lift_commands = lift_commands
        self.lift_states = lift_states or LiftStateCache()
        # The lift whose car the robot is in. RMF names the lift only for the
        # move into the car, not for the localize call or the move out.
        self._lift_in_use = None
        self._lift_move_active = False
        self._lift_leg_failures = 0
        # The message for a robot that cannot use lift commands. The adapter
        # sends such a robot no more lift commands until it starts again.
        self._lift_commands_refusal = None
        self._clock = time.monotonic

        self.update_handle = None
        self.battery_soc = 0.0
        self._lock = threading.Lock()
        self._alert_lock = threading.Lock()
        self._pending_dock_destinations = {}
        navigation_config = (
            self.config.get('robot_config', {}).get('navigation')
            or self.config.get('navigation', {})
        )
        if 'xy_goal_tolerance' not in navigation_config:
            raise KeyError(
                f'Missing navigation.xy_goal_tolerance for robot '
                f'[{self.name}]')
        self.navigation_max_retries = int(
            navigation_config.get('max_retries', 3))
        self.timeout_grace_sec = float(navigation_config.get(
            'timeout_grace_sec', 60))
        self.max_timeout_sec = float(navigation_config.get(
            'max_timeout_sec', 300))
        self.docked_not_charging_grace_sec = 5
        # float(navigation_config.get(
        #     'timeout_grace_sec', 60))
        self._docked_not_charging_since = None
        self._docked_not_charging_alert_sent = False
        self._fault_seen = None
        self._fault_seen_since = 0.0
        self._fault_reported = None
        self._fault_ticket = None
        self._fault_execution = None
        self.rmf_web_alerts_url = os.environ.get(
            'RMF_WEB_ALERTS_URL',
            'http://127.0.0.1:8000/alerts/request')
        self.rmf_web_api_token = os.environ.get('RMF_WEB_API_TOKEN')
        self.api.set_xy_goal_tolerance(
            self.name,
            float(navigation_config.get(
                'xy_goal_tolerance')))

        self.charger_waypoint_index = None
        charger_waypoint = self.rmf_config.get('charger', {}).get('waypoint')
        if charger_waypoint is not None:
            waypoint = self.graph.find_waypoint(charger_waypoint)
            if waypoint is not None:
                self.charger_waypoint_index = waypoint.index
                self.rmf_config.setdefault(
                    'charger', {})['waypoint'] = charger_waypoint

        self.command_handle = CommandHandle(
            name=self.name,
            logger=self.node.get_logger(),
            run_command=self._run_command_event,
            stop_robot=self._stop_robot_until_success,
            wait_for_robot_terminal=self._wait_for_robot_terminal,
            timeout_grace_sec=self.timeout_grace_sec,
            max_timeout_sec=self.max_timeout_sec,
        )

    def _robot_map_name(self, rmf_level: str) -> str:
        return self.robot_map_names.get(rmf_level, rmf_level)

    def _robot_map_level(self, rmf_level: str):
        if rmf_level in self.robot_map_levels:
            return self.robot_map_levels[rmf_level]
        return rmf_level

    def make_callbacks(self):
        callbacks = rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(
                destination, execution
            ),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(
                category, description, execution
            ),
        )
        if self.lift_commands:
            callbacks.localize = lambda estimate, execution: self.localize(
                estimate, execution)
        return callbacks

    def get_position(self) -> RmfCoord:
        position = self.api.position(self.name)
        if position is None:
            self.node.get_logger().error(
                f'Unable to retrieve live position from robot [{self.name}]')
            return self.position

        transform = self.transforms.get(self.map_name)
        if transform is None:
            self.node.get_logger().error(
                f'Missing map transform for [{self.map_name}]')
            return self.position

        return transform.robot_to_rmf_meters(robot_coord=position)

    def get_battery_soc(self) -> float:
        battery_soc = self.api.battery_soc(self.name)
        if battery_soc is None:
            self.node.get_logger().error(
                f'Unable to retrieve battery data from robot [{self.name}]')
            return self.battery_soc
        return battery_soc

    def make_state(self):
        self.position = self.get_position()
        self.battery_soc = self.get_battery_soc()
        yaw = self.position.orientation_radians
        if yaw is None:
            yaw = 0.0

        return rmf_easy.RobotState(
            self.map_name,
            np.array([self.position.x, self.position.y, yaw],
                     dtype=np.float64),
            self.battery_soc,
        )

    def update(self):
        if self.update_handle is None:
            return

        self._monitor_docked_not_charging()
        self._monitor_fault_status()

        execution = self.command_handle.active_execution()
        activity_identifier = (
            execution.identifier if execution is not None else None)

        self.update_handle.update(self.make_state(), activity_identifier)

    def _monitor_docked_not_charging(self):
        """Post one RMF Web alert for each docked/non-charging incident."""
        robot_status = self.api.get_robot_status(self.name)
        if not isinstance(robot_status, dict):
            return

        is_docked = (
            robot_status.get('status') == RobotRuntimeStatusText.DOCKED.value)
        is_charging = robot_status.get('charging') is True
        problem_present = is_docked and not is_charging
        now = time.monotonic()

        with self._alert_lock:
            if not problem_present:
                self._docked_not_charging_since = None
                self._docked_not_charging_alert_sent = False
                return

            if self._docked_not_charging_since is None:
                self._docked_not_charging_since = now
                return

            elapsed_sec = now - self._docked_not_charging_since
            if elapsed_sec < self.docked_not_charging_grace_sec or \
                    self._docked_not_charging_alert_sent:
                return

            self._docked_not_charging_alert_sent = True
            self.node.get_logger().warning(
                f'Robot [{self.name}] has been docked for '
                f'{elapsed_sec:.0f}s but is not charging')
            self._send_docked_not_charging_alert(elapsed_sec, robot_status)

    def _monitor_fault_status(self):
        """Report Estop, Error and Offline to RMF, and their end.

        Estop and Error are reported at once. Offline is reported only after
        it lasts timeout_grace_sec, the time the adapter already gives a robot
        to answer a command. The task is left to the command timeouts.
        """
        robot_status = self.api.get_robot_status(self.name)
        status = (
            robot_status.get('status')
            if isinstance(robot_status, dict) else None)
        if status not in _FAULT_STATUSES:
            status = None
        offline = status == RobotRuntimeStatusText.OFFLINE.value

        now = self._clock()
        if status != self._fault_seen:
            self._fault_seen = status
            self._fault_seen_since = now
        if offline and now - self._fault_seen_since < self.timeout_grace_sec:
            return

        execution = self.command_handle.active_execution()
        if status != self._fault_reported:
            self._report_fault_change(status, robot_status, execution)
        if status is None or execution is None or \
                execution is self._fault_execution:
            return

        # Only the execution of a clean or dock action has these calls. The
        # execution of a navigation command does not.
        report = getattr(
            execution, 'delayed' if offline else 'blocked', None)
        if report is not None and execution.okay():
            report(f'Robot [{self.name}] reports {status}')
        self._fault_execution = execution

    def _report_fault_change(self, status, robot_status, execution):
        previous = self._fault_reported
        self._fault_reported = status
        if self._fault_ticket is not None:
            self._fault_ticket.resolve({})
            self._fault_ticket = None
        if execution is not None and execution is self._fault_execution and \
                status is None and hasattr(execution, 'underway') and \
                execution.okay():
            execution.underway(
                f'Robot [{self.name}] no longer reports {previous}')
        self._fault_execution = None
        if status is None:
            self.node.get_logger().info(
                f'Robot [{self.name}] no longer reports {previous}')
            return

        self.node.get_logger().error(
            f'Robot [{self.name}] reports {status}')
        tier = rmf_adapter.robot_update_handle.Tier
        self._fault_ticket = self._rmf_robot_handle().create_issue(
            tier.Warning
            if status == RobotRuntimeStatusText.OFFLINE.value
            else tier.Error,
            status.lower(),
            {'status': status})
        self._send_rmf_web_alert(
            status.lower(),
            f'Robot reports {status}',
            f'Robot [{self.name}] reports {status}. '
            'A person must attend to the robot.',
            robot_status)

    def _send_docked_not_charging_alert(self, elapsed_sec, robot_status):
        self._send_rmf_web_alert(
            'docked_not_charging',
            'Robot docked but not charging',
            f'Robot [{self.name}] has reported Docked for '
            f'{elapsed_sec:.0f}s but is not charging.',
            robot_status)

    def _send_rmf_web_alert(self, key, title, message, robot_status):
        if not self.rmf_web_api_token:
            self.node.get_logger().error(
                f'Cannot create RMF Web alert [{key}] for '
                f'[{self.name}]: RMF_WEB_API_TOKEN is not configured')
            return

        payload = {
            'id': f'{key.replace("_", "-")}-{self.name}-{uuid.uuid4()}',
            'unix_millis_alert_time': int(time.time() * 1000),
            'title': title,
            'subtitle': f'{self.fleet_name}/{self.rmf_name}',
            'message': message,
            'display': True,
            'tier': 'warning',
            'responses_available': ['Acknowledge'],
            'alert_parameters': [
                {'name': 'robot', 'value': self.rmf_name},
                {'name': 'fleet', 'value': self.fleet_name},
                {'name': 'vendor_status', 'value': str(robot_status.get('status'))},
                {'name': 'charging', 'value': str(robot_status.get('charging'))},
            ],
            'task_id': None,
        }
        headers = {'Authorization': f'Bearer {self.rmf_web_api_token}'}

        try:
            response = requests.post(
                self.rmf_web_alerts_url,
                headers=headers,
                json=payload,
                timeout=5.0)
            response.raise_for_status()
        except requests.RequestException as error:
            self.node.get_logger().error(
                f'Failed to create RMF Web alert [{key}] for '
                f'[{self.name}]: {error!r}')
            return

        self.node.get_logger().warning(
            f'Created RMF Web alert [{key}] for robot [{self.name}]')

    def navigate(self, destination, execution):
        self.node.get_logger().info(
            f'RMF navigate(): '
            f'Navigating [{self.name}] to '
            f'point [{destination.name}] '
            f'graph_index={destination.graph_index}, '
            f'position={destination.position}')

        dock_name = destination.dock
        if dock_name is not None:
            dock_name = str(dock_name)
            with self._lock:
                self._pending_dock_destinations[dock_name] = destination
            self.node.get_logger().info(
                f'RMF navigate() for [{self.name}] has dock=[{dock_name}] '
                f'at point {destination.position}; docking will be performed '
                'after reaching the destination')

        self.node.get_logger().info(
            f'RMF navigate() for [{self.name}] at point '
            f'{destination.position}; enqueuing P2P command')
        self.command_handle.enqueue(self.command_handle.make_event(
            CommandKind.P2P,
            execution=execution,
            destination=destination,
        ))

    def localize(self, estimate, execution):
        self.node.get_logger().info(
            f'RMF localize(): [{self.name}] on level [{estimate.map}] '
            f'graph_index={estimate.graph_index}, '
            f'position={estimate.position}')
        self.command_handle.enqueue(self.command_handle.make_event(
            CommandKind.LOCALIZE,
            execution=execution,
            destination=estimate,
        ))

    def _lift_leg(self, destination) -> tuple[LiftCommand, str] | None:
        '''Return the lift command for one RMF move, or None for a P2P move.

        This runs when the move starts, not when RMF sends it: a lift move
        that is still in progress changes the answer. A move into or out of
        a lift car is never a P2P move, also when lift commands are off.
        '''
        self.position = self.get_position()
        lift_at_position = self._lift_at_position()
        if self._lift_in_use is not None and \
                self._lift_in_use != lift_at_position:
            # The robot left the car without a completed move out, for
            # example a person moved it. Its position is the truth.
            self.node.get_logger().warning(
                f'Robot [{self.name}] is no longer in the car of lift '
                f'[{self._lift_in_use}]')
            self._lift_in_use = None

        target_lift = destination.inside_lift
        if target_lift is not None:
            return LiftCommand.ENTER, str(target_lift.name)
        if lift_at_position is not None:
            return LiftCommand.EXIT, lift_at_position
        return None

    def _lift_at_position(self) -> str | None:
        '''Return the lift whose car holds the robot's last known position.

        RMF uses the same test to decide that a robot is in a lift car.
        '''
        xy = np.array([self.position.x, self.position.y], dtype=np.float64)
        for lift in self.graph.all_known_lifts:
            if lift.is_in_lift(xy, 0.0):
                return str(lift.name)
        return None

    def stop(self, activity):
        self.node.get_logger().debug(f'RMF stop()')
        self.command_handle.request_stop(activity)

    def execute_action(self, category: str, description: dict, execution):
        self.node.get_logger().debug(f'RMF execute_action({category})')
        if category == 'docking':
            dock_name = self._dock_name_from_action_description(description)
            if dock_name is None:
                message = (
                    'Docking action request is missing [dock_name] and no '
                    'charger waypoint is configured')
                self.node.get_logger().error(message)
                if hasattr(execution, 'error'):
                    execution.error(message)
                execution.finished()
                return

            self.node.get_logger().info(
                f'RMF execute_action() for [{self.name}] '
                f'category=[{category}] dock=[{dock_name}] '
                f'description={description}; enqueuing dock command')
            with self._lock:
                dock_destination = self._pending_dock_destinations.get(
                    dock_name)
            self.command_handle.enqueue(self.command_handle.make_event(
                CommandKind.DOCK,
                execution=execution,
                destination=dock_destination,
                description={
                    'dock_name': dock_name,
                    'source': 'execute_action(docking)',
                },
            ))
            return

        if category == 'clean':
            zone = None
            if isinstance(description, dict):
                zone = description.get('zone')

            if not zone:
                message = 'Clean action request is missing [zone]'
                self.node.get_logger().error(message)
                if hasattr(execution, 'error'):
                    execution.error(message)
                execution.finished()
                return

            self.node.get_logger().info(
                f'RMF execute_action() for [{self.name}] category=[{category}] '
                f'zone=[{zone}] description={description}; '
                'enqueuing clean command')
            self.command_handle.enqueue(self.command_handle.make_event(
                CommandKind.CLEAN,
                execution=execution,
                description={'zone': zone, 'source': 'execute_action(clean)'},
            ))
            return 
        
        message = f'Unsupported action category [{category}]'
        self.node.get_logger().error(message)
        if hasattr(execution, 'error'):
            execution.error(message)
        execution.finished()
        return

    def finish_action(self):
        self.node.get_logger().debug(
            'Ignoring legacy MODE_IDLE notice for robot [%s]; operation end '
            'status remains the command completion authority', self.name)

    def _run_command_event(self, event):
        if event.kind == CommandKind.P2P:
            try:
                lift_leg = self._lift_leg(event.destination)
            except Exception as error:
                # This can be a move at a lift. See _run_lift_step().
                self.node.get_logger().error(traceback.format_exc())
                self._hold_lift_leg(
                    event.execution,
                    f'Robot [{self.name}] had an adapter error before a '
                    f'move: {error!r}.')
                return
            if lift_leg is None:
                self._navigate_worker(event.destination, event.execution)
                return
            self._run_lift_step(
                event.execution,
                self._lift_move_worker,
                *lift_leg,
                event.destination,
                event.execution)
            return

        if event.kind == CommandKind.LOCALIZE:
            self._run_lift_step(
                event.execution,
                self._localize_worker,
                event.destination,
                event.execution)
            return

        if event.kind == CommandKind.DOCK:
            description = event.description
            self._dock_worker(
                description['dock_name'],
                description['source'],
                event.destination,
                event.execution)
            return

        if event.kind == CommandKind.CLEAN:
            description = event.description
            self._clean_worker(
                description['zone'],
                description['source'],
                event.execution)
            return

        self._fail_execution(
            event.execution,
            f'Unsupported queued command [{event.kind.value}]')

    def _execution_is_active(self, execution) -> bool:
        return self.command_handle.is_active(execution)

    def _finish_execution(self, execution):
        self.command_handle.finish(execution)

    def _fail_execution(self, execution, message: str):
        self.node.get_logger().error(message)
        self.command_handle.fail(execution, message)

    def _operation_result(self, execution, status) -> CommandResult:
        # Accept CommandResult temporarily for existing mock-based tests.
        if isinstance(status, OperationStatus):
            result = status.result
            eta_seconds = status.eta_seconds
        else:
            result = status
            eta_seconds = None

        if result == CommandResult.RUNNING:
            self.command_handle.update_operation_eta(execution, eta_seconds)
            if self.command_handle.operation_timed_out(execution):
                self.command_handle.clear_operation_attempt(execution)
                return CommandResult.TIMEOUT
            return result

        self.command_handle.clear_operation_attempt(execution)
        return result

    def _sleep(self, seconds: float, execution=None) -> bool:
        return self.command_handle.wait(seconds, execution)

    def _wait_for_robot_terminal(self) -> bool:
        deadline = time.monotonic() + self.max_timeout_sec
        if self._wait_for_robot_available(deadline):
            return True

        if not rclpy.ok():
            return False

        self.node.get_logger().error(
            f'Robot [{self.name}] did not reach a terminal state within '
            f'{self.max_timeout_sec:.1f}s; requesting stop')
        stop_deadline = time.monotonic() + self.max_timeout_sec
        if not self._stop_robot_until_success(stop_deadline):
            return False

        deadline = time.monotonic() + self.max_timeout_sec
        if self._wait_for_robot_available(deadline):
            return True

        self.node.get_logger().error(
            f'Robot [{self.name}] did not reach a terminal state after stop '
            f'within {self.max_timeout_sec:.1f}s')
        return False

    def _stop_robot_until_success(self, stop_deadline: float):
        if self._lift_move_active:
            # A stop here can leave the robot in the lift doorway. The lift
            # move ends by itself, and its watchdog bounds it.
            self.node.get_logger().info(
                f'Robot [{self.name}] is in a lift move; the stop request '
                'waits for the move to end')
            return True

        while rclpy.ok() and time.monotonic() < stop_deadline:
            self.node.get_logger().info(
                f'Requesting robot [{self.name}] to stop...')
            remaining_seconds = stop_deadline - time.monotonic()
            if self.api.stop(
                    robot_encoding_id=self.name,
                    timeout_seconds=remaining_seconds):
                self.node.get_logger().info(
                    f'Robot [{self.name}] accepted stop request')
                return True
            time.sleep(min(0.2, max(0.0, remaining_seconds)))

        self.node.get_logger().error(
            f'Robot [{self.name}] did not stop within '
            f'{self.max_timeout_sec:.1f}s')
        return False
    
    def _idle_wait_summary(self, robot_status: dict | None) -> str:
        runtime_status = None
        if robot_status is not None:
            runtime_status = robot_status.get('status')

        return f'runtime_status={runtime_status}'

    def _wait_for_robot_available(self, deadline: float, execution=None) -> bool:
        last_log_at = 0.0

        while rclpy.ok() and time.monotonic() < deadline:
            if execution is not None and not self._execution_is_active(execution):
                return False

            robot_status = self.api.get_robot_status(
                robot_encoding_id=self.name)
            if RobotAPI.robot_is_available(robot_status):
                return True

            now = time.monotonic()
            if execution is not None and now - last_log_at >= 5.0:
                wait_summary = self._idle_wait_summary(robot_status)
                self.node.get_logger().info(
                    f'Waiting for robot [{self.name}] to become idle before '
                    f'next command ({wait_summary})')
                last_log_at = now

            remaining_seconds = max(0, deadline - now)
            if execution is None:
                time.sleep(min(0.5, remaining_seconds))
            elif not self._sleep(min(0.5, remaining_seconds), execution):
                return False

        return False

    def _wait_until_ready(self, execution) -> bool:
        deadline = time.monotonic() + self.max_timeout_sec
        return self._wait_for_robot_available(deadline, execution)

    def _prepare_map(self, target_map: str, execution) -> bool:
        if target_map == self.map_name:
            return True

        current_transform = self.transforms.get(self.map_name)
        target_transform = self.transforms.get(target_map)
        if current_transform is None or target_transform is None:
            self.node.get_logger().error(
                f'Missing map transform while changing [{self.name}] from '
                f'[{self.map_name}] to [{target_map}]')
            return False

        current_pose_robot = self.api.position(self.name)
        if current_pose_robot is None:
            return False

        current_pose_rmf = current_transform.robot_to_rmf_meters(
            robot_coord=current_pose_robot)
        next_pose_robot = target_transform.rmf_meters_to_robot(
            rmf_coord=current_pose_rmf)

        if not self.api.change_map(
                robot_encoding_id=self.name,
                map_name=self._robot_map_name(target_map),
                map_level=self._robot_map_level(target_map)):
            return False

        last_status_wait_log = 0.0
        while self._execution_is_active(execution):
            robot_status = self.api.get_robot_status(
                robot_encoding_id=self.name)
            if robot_status is None:
                now = time.monotonic()
                if now - last_status_wait_log >= 5.0:
                    self.node.get_logger().info(
                        f'Waiting for first status from robot [{self.name}] '
                        'before localization check')
                    last_status_wait_log = now
                if not self._sleep(0.5, execution):
                    return False
                continue

            localized = robot_status.get('localized')
            if localized is True:
                self.node.get_logger().info(
                    f'Robot [{self.name}] is already localized; skipping '
                    'hot-localize')
                self.map_name = target_map
                return True

            if localized is not False:
                self.node.get_logger().info(
                    f'Robot [{self.name}] status does not include a '
                    f'definitive localized=false value; '
                    f'status localized={localized}')
                if not self._sleep(0.5, execution):
                    return False
                continue

            if not self._ensure_undocked(execution):
                return False

            if self.api.localize(next_pose_robot, robot_encoding_id=self.name):
                self.map_name = target_map
                return True

            if not self._sleep(0.5, execution):
                return False

        return False

    def _ensure_undocked(self, execution=None) -> bool:
        robot_status = self.api.get_robot_status(robot_encoding_id=self.name)
        while robot_status is None and self._execution_is_active(execution):
            if not self._sleep(0.5, execution):
                return False
            robot_status = self.api.get_robot_status(
                robot_encoding_id=self.name)

        if robot_status is None:
            return False

        status = robot_status.get('status')
        if status == RobotRuntimeStatusText.RESTING.value:
            return True

        if status not in (
                RobotRuntimeStatusText.DOCKED.value,
                RobotRuntimeStatusText.CHARGING.value,
        ):
            if not self._wait_until_ready(execution):
                return False
            robot_status = self.api.get_robot_status(
                robot_encoding_id=self.name)

            if robot_status is not None and robot_status.get('status') == \
                    RobotRuntimeStatusText.RESTING.value:
                return True

        remaining_retries = self.navigation_max_retries
        while self._execution_is_active(execution):
            self.node.get_logger().info(
                f'Requesting robot [{self.name}] to undock')
            undocking_command_sent = False

            while self._execution_is_active(execution):
                if not undocking_command_sent:
                    if not self.api.send_undocking_command(self.name):
                        if remaining_retries <= 0:
                            self.node.get_logger().error(
                                f'Robot [{self.name}] failed to accept '
                                'undocking; no retries left')
                            return False
                        remaining_retries -= 1
                        self.node.get_logger().info(
                            f'Robot [{self.name}] failed to accept '
                            f'undocking; retrying ({remaining_retries} '
                            'retries left)')
                        if not self._sleep(1.0, execution):
                            return False
                        break
                    undocking_command_sent = True
                    self.command_handle.start_operation_attempt(execution)

                undock_status = self._operation_result(
                    execution, self.api.undock_robot(self.name))
                if undock_status == CommandResult.SUCCESS:
                    self.node.get_logger().info(
                        f'Robot [{self.name}] completed undocking')
                    return True

                if undock_status in (
                    CommandResult.ERROR,
                    CommandResult.TIMEOUT,
                ):
                    failure = (
                        'timed out undocking'
                        if undock_status == CommandResult.TIMEOUT
                        else 'failed to undock'
                    )
                    if remaining_retries <= 0:
                        self.node.get_logger().error(
                            f'Robot [{self.name}] {failure}; '
                            'no retries left')
                        return False
                    remaining_retries -= 1
                    self.node.get_logger().info(
                        f'Robot [{self.name}] {failure}; retrying '
                        f'({remaining_retries} retries left)')
                    break

                if not self._sleep(0.5, execution):
                    return False

        return False

    def _destination_rmf_coord(self, destination) -> RmfCoord:
        position = destination.position
        return RmfCoord(
            x=float(position[0]),
            y=float(position[1]),
            orientation_radians=float(position[2]),
        )

    def _dock_name_from_action_description(self, description) -> str | None:
        if isinstance(description, str) and description:
            return description

        if isinstance(description, dict):
            for key in (
                'dock_name',
                'dock',
                'waypoint',
                'waypoint_name',
                'name',
            ):
                dock_name = description.get(key)
                if isinstance(dock_name, str) and dock_name:
                    return dock_name

        charger_waypoint = self.rmf_config.get('charger', {}).get('waypoint')
        if isinstance(charger_waypoint, str) and charger_waypoint:
            return charger_waypoint

        return None

    def _navigate_worker(self, destination, execution):
        target_map = destination.map
        target_pose_rmf = self._destination_rmf_coord(destination)
        remaining_retries = self.navigation_max_retries

        while self._execution_is_active(execution):
            if not self._prepare_map(target_map, execution):
                if not self._sleep(1.0, execution):
                    return
                continue

            if not self._wait_until_ready(execution):
                if self._execution_is_active(execution):
                    self._fail_execution(
                        execution,
                        f'Robot [{self.name}] was not idle before navigating '
                        f'to {target_pose_rmf}')
                return

            transform = self.transforms.get(target_map)
            if transform is None:
                self._fail_execution(
                    execution,
                    f'Missing map transform for [{target_map}]')
                return

            target_pose_robot = transform.rmf_meters_to_robot(
                rmf_coord=target_pose_rmf)

            if not self._ensure_undocked(execution):
                if self._execution_is_active(execution):
                    self._fail_execution(
                        execution,
                        f'Robot [{self.name}] was not ready to navigate '
                        f'to {target_pose_rmf} as it cannot be undocked')
                return

            self.node.get_logger().info(
                f'Navigating [{self.name}] to {target_pose_rmf} = '
                f'{target_pose_robot}')

            navigation_command_sent = False
            if self.command_handle.set_state(
                    execution, CommandState.P2P_MOVE):
                self.map_name = target_map

            while self._execution_is_active(execution):
                if not navigation_command_sent:
                    if not self.api.send_navigation_command(
                            self.name,
                            target_pose_robot,
                            self._robot_map_name(target_map)):
                        if remaining_retries <= 0:
                            self._fail_execution(
                                execution,
                                f'Robot [{self.name}] failed to accept '
                                f'navigation to {target_pose_rmf}; '
                                'no retries left')
                            return
                        remaining_retries -= 1
                        self.node.get_logger().info(
                            f'Robot [{self.name}] failed to accept '
                            f'navigation to {target_pose_rmf}; retrying '
                            f'({remaining_retries} retries left)')
                        if not self._sleep(1.0, execution):
                            return
                        break
                    navigation_command_sent = True
                    self.command_handle.start_operation_attempt(execution)

                navigate_status = self._operation_result(
                    execution, self.api.navigate_robot(
                        self.name, target_pose_robot, target_map))
                if navigate_status == CommandResult.SUCCESS:
                    self.node.get_logger().info(
                        f'Robot [{self.name}] reached its destination')
                    dock_name = destination.dock
                    if dock_name is not None:
                        dock_name = str(dock_name)
                        self.node.get_logger().info(
                            f'Navigation destination for [{self.name}] '
                            f'requires docking at [{dock_name}]')
                        self._dock_worker(
                            dock_name,
                            'navigate(destination.dock)',
                            destination,
                            execution)
                        return

                    self._finish_execution(execution)
                    return

                if navigate_status in (
                    CommandResult.ERROR,
                    CommandResult.TIMEOUT,
                ):
                    failure = (
                        'timed out navigating to'
                        if navigate_status == CommandResult.TIMEOUT
                        else 'failed to navigate to'
                    )
                    if remaining_retries <= 0:
                        self._fail_execution(
                            execution,
                            f'Robot [{self.name}] {failure} '
                            f'{target_pose_rmf}; no retries left')
                        return
                    remaining_retries -= 1
                    self.node.get_logger().info(
                        f'Robot [{self.name}] {failure} {target_pose_rmf}; '
                        f'retrying ({remaining_retries} retries left)')
                    break

                if not self._sleep(0.5, execution):
                    return

    def _run_lift_step(self, execution, worker, *arguments):
        '''Run one step at a lift; an adapter error holds the robot.

        The command queue finishes a command that raises an error. At a lift
        that would tell RMF that the robot arrived, and RMF would move the
        lift car.
        '''
        try:
            worker(*arguments)
        except Exception as error:
            self._lift_move_active = False
            self.node.get_logger().error(traceback.format_exc())
            self._hold_lift_leg(
                execution,
                f'Robot [{self.name}] had an adapter error during a step at '
                f'a lift: {error!r}.')

    def _moved_since(self, start: RmfCoord) -> bool:
        self.position = self.get_position()
        return math.hypot(
            self.position.x - start.x,
            self.position.y - start.y) > LIFT_MOVE_MOTION_TOLERANCE_METERS

    def _lift_session_id(self) -> str:
        '''The lift session ID that RMF uses for this robot.'''
        return f'{self.fleet_name}/{self.rmf_name}'

    def _wait_for_lift_car(self, lift: str, level: str, execution):
        '''Wait for the car to be ready; return the last problem, or None.'''
        deadline = self._clock() + LIFT_READY_WAIT_SECONDS
        while True:
            problem = car_not_ready_problem(
                self.lift_states.get(lift),
                level,
                self._lift_session_id(),
                now=self._clock())
            if problem is None or self._clock() >= deadline:
                return problem
            if not self._sleep(0.2, execution):
                return problem

    def _follow_lift_move(
            self, lift: str, level: str) -> tuple[LiftMoveStatus | None, bool]:
        '''Wait for the lift move to end; stop it when the car is not safe.

        The wait does not end when RMF cancels the move, because the robot
        is then in or near the doorway. Returns the end status and whether
        this adapter tried to stop the move.
        '''
        stopping = False
        stop_sent = False
        while rclpy.ok():
            status = self.api.lift_move_status(self.name)
            if status is None:
                return None, stopping
            if status.state == LiftMoveState.TIMED_OUT:
                self._stop_lift_move()
                return status, stopping
            if status.state.ended:
                return status, stopping

            if not stopping:
                problem = car_unsafe_problem(
                    self.lift_states.get(lift),
                    level,
                    self._lift_session_id())
                if problem is not None:
                    self.node.get_logger().error(
                        f'Lift [{lift}] is not safe for robot [{self.name}] '
                        f'({problem.value}); stopping the robot')
                    stopping = True
            if stopping and not stop_sent:
                stop_sent = self.api.abort_lift_move(self.name)

            self.command_handle.wait(0.2)
        return None, stopping

    def _stop_lift_move(self) -> bool:
        '''Send the stop until the WebSocket takes it, up to a limit.'''
        for _ in range(LIFT_STOP_ATTEMPTS):
            if self.api.abort_lift_move(self.name):
                return True
            self.command_handle.wait(1.0)
        self.node.get_logger().error(
            f'The stop command for robot [{self.name}] was not sent')
        return False

    def _lift_move_worker(
            self,
            command: LiftCommand,
            lift: str,
            destination,
            execution):
        if not self.lift_commands:
            self._refuse_lift_leg(
                execution,
                f'Robot [{self.name}] moves into and out of a lift car only '
                'with lift commands, and experimental.lift_commands is off. '
                'The robot did not move.')
            return

        level = destination.map
        if command == LiftCommand.ENTER and self._lift_in_use == lift \
                and level == self.map_name:
            self.node.get_logger().info(
                f'Robot [{self.name}] is already in the car of lift [{lift}]')
            self._finish_execution(execution)
            return

        if self._lift_commands_refusal is not None:
            self._refuse_lift_leg(execution)
            return

        if level != self.map_name:
            self._hold_lift_leg(
                execution,
                f'Robot [{self.name}] is on level [{self.map_name}] but RMF '
                f'asked for a move at lift [{lift}] on level [{level}]. The '
                'robot did not move.')
            return

        if not self._wait_until_ready(execution):
            if self._execution_is_active(execution):
                self._retry_lift_leg(
                    execution,
                    f'Robot [{self.name}] was not idle before the move at '
                    f'lift [{lift}]. The robot did not move.')
            return

        # The map lookup is a slow call. It runs before the lift check, so
        # that the command goes out at once after the check.
        try:
            robot_map = self.api.lift_command_map(
                self.name,
                self._robot_map_name(level),
                self._robot_map_level(level))
        except LiftCommandError as error:
            self._retry_lift_leg(
                execution, f'{error}. The robot did not move.')
            return

        problem = self._wait_for_lift_car(lift, level, execution)
        if not self._execution_is_active(execution):
            return
        if problem is not None:
            self._retry_lift_leg(
                execution,
                f'Lift [{lift}] was not ready for robot [{self.name}] on '
                f'level [{level}] ({problem.value}). The robot did not '
                'move.')
            return

        start_position = self.get_position()
        try:
            self.api.send_lift_command(self.name, command, lift, robot_map)
        except LiftCommandError as error:
            self._retry_lift_leg(
                execution, f'{error}. The robot did not move.')
            return

        self.command_handle.set_state(execution, CommandState.LIFT_MOVE)
        self._lift_move_active = True
        try:
            status, stopping = self._follow_lift_move(lift, level)
        finally:
            self._lift_move_active = False
            self.api.clear_lift_move(self.name)

        if status is not None and status.state == LiftMoveState.SUCCEEDED:
            self._lift_in_use = lift if command == LiftCommand.ENTER else None
            self._lift_leg_failures = 0
            self.node.get_logger().info(
                f'Robot [{self.name}] completed [{command.value}] at lift '
                f'[{lift}]')
            self._finish_execution(execution)
            return

        if status is not None and status.state == LiftMoveState.REFUSED \
                and status.reason_code in _LIFT_COMMANDS_NOT_AVAILABLE:
            self._lift_commands_refusal = (
                f'Robot [{self.name}] '
                f'{_LIFT_COMMANDS_NOT_AVAILABLE[status.reason_code]} '
                f'({status.reason_code}). The robot did not move. It gets no '
                'more lift commands until the adapter starts again. It can '
                'still do work on its current level. Contact LionsBot to '
                'enable lift commands on this robot.')
            self._refuse_lift_leg(execution)
            return

        message = (
            f'Robot [{self.name}] did not complete [{command.value}] at '
            f'lift [{lift}] on level [{level}]: '
            f'{self._lift_failure_text(status, stopping)}.')
        if stopping or status is None or status.aborted or \
                status.state == LiftMoveState.TIMED_OUT:
            self._hold_lift_leg(execution, message)
            return

        if self._moved_since(start_position):
            # After a move that ended part of the way, RMF cannot make a
            # safe plan: it can take the robot as in the car and move it.
            self._hold_lift_leg(
                execution, f'{message} The robot moved before it stopped.')
            return
        self._retry_lift_leg(execution, message)

    @staticmethod
    def _lift_failure_text(
            status: LiftMoveStatus | None, stopping: bool) -> str:
        if stopping:
            return 'the adapter sent a stop because the lift car moved or ' \
                'the lift session changed'
        if status is None:
            return 'the adapter stopped before the move ended'
        if status.state == LiftMoveState.TIMED_OUT:
            return 'the robot sent no end status and the adapter sent a stop'
        if status.state == LiftMoveState.REFUSED:
            return 'the command did not reach the robot ' \
                f'({status.reason or status.error_code}). The robot did ' \
                'not move'
        if status.aborted:
            return 'a stop command ended the move'
        return f'the robot reported a failure (code {status.error_code})'

    def _retry_lift_leg(self, execution, message: str):
        '''Fail a lift move and ask RMF for a new plan, up to a limit.'''
        self._lift_leg_failures += 1
        if self._lift_leg_failures > self.navigation_max_retries:
            self._hold_lift_leg(
                execution,
                f'{message} This is failure {self._lift_leg_failures} in a '
                'row.')
            return

        self.node.get_logger().error(
            f'{message} Asking RMF for a new plan.')
        self.command_handle.abandon(execution)
        self._request_replan()

    def _hold_lift_leg(self, execution, message: str):
        '''Fail a lift move and keep the robot where it is.

        The move is not reported to RMF as finished: RMF would take the
        robot as arrived and move the lift car.
        '''
        text = (
            f'{message} The robot stays where it is. Check the robot and '
            'the lift, then cancel or restart the task.')
        self.node.get_logger().error(text)
        self._lift_leg_failures = 0
        self.command_handle.abandon(execution)
        self._report_to_rmf(text)

    def _refuse_lift_leg(self, execution, message=None):
        '''Fail a lift move of a robot that cannot use lift commands.

        Outside a lift car, the task is cancelled so that RMF releases the
        lift and gives the robot other work. In a car, the robot stays.
        '''
        message = message or self._lift_commands_refusal
        self.node.get_logger().error(message)
        self.command_handle.abandon(execution)
        self._report_to_rmf(message)
        self.position = self.get_position()
        if self._lift_at_position() is None:
            self._cancel_current_task(message)

    def _cancel_current_task(self, reason: str):
        robot_update_handle = self._rmf_robot_handle()
        if robot_update_handle is None:
            return
        task_id = robot_update_handle.current_task_id()
        if not task_id:
            return

        def on_cancellation(task_was_found: bool):
            self.node.get_logger().info(
                f'Cancelled task [{task_id}] of robot [{self.name}]: '
                f'task found={task_was_found}')

        robot_update_handle.cancel_task(task_id, [reason], on_cancellation)

    def _request_replan(self):
        robot_update_handle = self._rmf_robot_handle()
        if robot_update_handle is not None:
            robot_update_handle.replan()

    def _report_to_rmf(self, text: str):
        robot_update_handle = self._rmf_robot_handle()
        if robot_update_handle is not None:
            robot_update_handle.log_error(text)

    def _rmf_robot_handle(self):
        if self.update_handle is None:
            return None
        return self.update_handle.more()

    def _cabin_seed_pose(self, estimate) -> RmfCoord:
        '''The pose to give the robot after a lift ride, in RMF coordinates.

        The position is the cabin waypoint of the new level. The heading is
        the robot's heading, which a lift ride does not change.
        '''
        position = estimate.position
        x, y = float(position[0]), float(position[1])
        if estimate.graph_index is not None:
            location = self.graph.get_waypoint(estimate.graph_index).location
            x, y = float(location[0]), float(location[1])
        return RmfCoord(x=x, y=y, orientation_radians=float(position[2]))

    def _localize_worker(self, estimate, execution):
        level = estimate.map
        if level == self.map_name:
            self._finish_execution(execution)
            return

        transform = self.transforms.get(level)
        if transform is None:
            self._hold_lift_leg(
                execution,
                f'Missing map transform for [{level}]; robot [{self.name}] '
                'cannot change to that level.')
            return

        seed_pose_robot = transform.rmf_meters_to_robot(
            rmf_coord=self._cabin_seed_pose(estimate))
        self.command_handle.set_state(execution, CommandState.LOCALIZING)
        remaining_retries = self.navigation_max_retries

        while self._execution_is_active(execution):
            if self.api.change_map(
                    robot_encoding_id=self.name,
                    map_name=self._robot_map_name(level),
                    map_level=self._robot_map_level(level)) and \
                    self.api.localize(
                        seed_pose_robot, robot_encoding_id=self.name):
                self.map_name = level
                self.node.get_logger().info(
                    f'Robot [{self.name}] is on level [{level}] at '
                    f'{seed_pose_robot}')
                self._finish_execution(execution)
                return

            if remaining_retries <= 0:
                self._hold_lift_leg(
                    execution,
                    f'Robot [{self.name}] did not change to level [{level}] '
                    'and set its position after the lift ride.')
                return
            remaining_retries -= 1
            self.node.get_logger().info(
                f'Robot [{self.name}] did not change to level [{level}]; '
                f'retrying ({remaining_retries} retries left)')
            if not self._sleep(1.0, execution):
                return

    def _dock_worker(
            self,
            dock_name: str,
            source: str,
            dock_destination,
            execution):
        dock_waypoint = self.graph.find_waypoint(dock_name)
        if dock_waypoint is None:
            self._fail_execution(
                execution,
                f'Unable to find dock waypoint [{dock_name}]')
            return

        self.node.get_logger().info(
            f'_dock_worker() for [{self.name}] dock=[{dock_name}] '
            f'source=[{source}] map=[{self.map_name}] '
            f'waypoint_index={dock_waypoint.index} '
            f'waypoint_location={dock_waypoint.location} '
            f'charger={dock_waypoint.charger}')

        if not dock_waypoint.charger:
            self._fail_execution(
                execution,
                f'Dock waypoint [{dock_name}] is not marked as a charger')
            return

        target_map = (
            (dock_destination.map if dock_destination is not None else None)
            or self.map_name
        )

        while self._execution_is_active(execution):
            if self._prepare_map(target_map, execution):
                break
            if not self._sleep(1.0, execution):
                return

        if dock_destination is not None:
            destination_pose_rmf = self._destination_rmf_coord(
                dock_destination)
            # dock_start takes the heading of the dock marker on the robot
            # map, which points out of the charger. RMF arrives at the dock
            # facing into the charger, so reverse the arrival heading.
            reversed_heading = MapTransform.wrap_orientation(
                destination_pose_rmf.orientation_radians + math.pi)
        else:
            current_pose_rmf = self.get_position()
            reversed_heading = MapTransform.wrap_orientation(
                current_pose_rmf.orientation_radians + math.pi)
            if reversed_heading is None:
                reversed_heading = 0.0

        dock_position_rmf = RmfCoord(
            x=dock_waypoint.location[0],
            y=dock_waypoint.location[1],
            orientation_radians=reversed_heading,
        )
        transform = self.transforms.get(target_map)
        if transform is None:
            self._fail_execution(
                execution,
                f'Missing map transform for [{target_map}]')
            return

        dock_position_robot = transform.rmf_meters_to_robot(dock_position_rmf)

        dock_pose_content = DockProcessContent(
            x=dock_position_robot.x,
            y=dock_position_robot.y,
            orientation_radians=MapTransform.wrap_orientation(
                dock_position_robot.orientation_radians),
            dock_name=dock_name,
        )

        self.command_handle.set_state(execution, CommandState.DOCKING)

        remaining_retries = self.navigation_max_retries
        while self._execution_is_active(execution):
            self.node.get_logger().info(
                f'Requesting robot [{self.name}] to dock at '
                f'[{dock_name}]')
            docking_command_sent = False

            while self._execution_is_active(execution):
                if not docking_command_sent:
                    if not self.api.send_docking_command(
                            robot_encoding_id=self.name,
                            content=dock_pose_content):
                        if remaining_retries <= 0:
                            self._fail_execution(
                                execution,
                                f'Robot [{self.name}] failed to accept '
                                f'docking at [{dock_name}]; '
                                'no retries left')
                            return
                        remaining_retries -= 1
                        self.node.get_logger().info(
                            f'Robot [{self.name}] failed to accept docking at '
                            f'[{dock_name}]; retrying '
                            f'({remaining_retries} retries left)')
                        if not self._sleep(1.0, execution):
                            return
                        break
                    docking_command_sent = True
                    self.command_handle.start_operation_attempt(execution)

                dock_status = self._operation_result(
                    execution, self.api.dock_robot(self.name))
                if dock_status == CommandResult.SUCCESS:
                    self.node.get_logger().info(
                        f'Robot [{self.name}] completed docking')
                    self._finish_execution(execution)
                    return

                if dock_status in (
                    CommandResult.ERROR,
                    CommandResult.TIMEOUT,
                ):
                    failure = (
                        'timed out docking at'
                        if dock_status == CommandResult.TIMEOUT
                        else 'failed to dock at'
                    )
                    if remaining_retries <= 0:
                        self._fail_execution(
                            execution,
                            f'Robot [{self.name}] {failure} '
                            f'[{dock_name}]; no retries left')
                        return
                    remaining_retries -= 1
                    self.node.get_logger().info(
                        f'Robot [{self.name}] {failure} [{dock_name}]; '
                        f'retrying ({remaining_retries} retries left)')
                    break

                if not self._sleep(0.5, execution):
                    return

    def _clean_worker(self, zone: str, source: str, execution):
        self.node.get_logger().info(
            f'_clean_worker() entered for [{self.name}] zone=[{zone}] '
            f'source=[{source}] map=[{self.map_name}]')
        dock_path = self.dock_paths.get(zone)
        command_map = self.map_name
        robot_zone_name = zone
        if dock_path is not None:
            command_map = dock_path['level_name']
            robot_zone_name = dock_path.get('robot_zone_name', zone)
            self.node.get_logger().info(
                f'_clean_worker() overriding schedule for [{self.name}] '
                f'zone=[{zone}] level=[{dock_path["level_name"]}] '
                f'path={dock_path["path"]}')
            execution.override_schedule(
                dock_path['level_name'],
                dock_path['path'],
            )

        if not self._ensure_undocked(execution):
                if self._execution_is_active(execution):
                    self._fail_execution(
                        execution,
                        f'Robot [{self.name}] was not ready to clean '
                        f'zone: {zone} as it cannot be undocked')
                return
        
        if not self._wait_until_ready(execution):
            if self._execution_is_active(execution):
                self._fail_execution(
                    execution,
                    f'Robot [{self.name}] was not idle before cleaning '
                    f'zone: {zone}')
            return

        while self._execution_is_active(execution):
            if self._prepare_map(command_map, execution):
                break
            if not self._sleep(1.0, execution):
                return

        transform = self.transforms.get(command_map)
        if transform is None:
            self._fail_execution(
                execution,
                f'Missing map transform for [{command_map}]')
            return

        clean_start_pose_robot = self.api.position(self.name)
        if clean_start_pose_robot is None:
            self._fail_execution(
                execution,
                f'Unable to capture cleaning start pose for robot '
                f'[{self.name}]')
            return

        clean_start_pose_rmf = transform.robot_to_rmf_meters(
            robot_coord=clean_start_pose_robot)
        if clean_start_pose_rmf.orientation_radians is None:
            self._fail_execution(
                execution,
                f'Cleaning start pose for robot [{self.name}] has no heading')
            return

        return_pose_rmf = RmfCoord(
            x=clean_start_pose_rmf.x,
            y=clean_start_pose_rmf.y,
            orientation_radians=MapTransform.wrap_orientation(
                # reverse orientation for return path
                clean_start_pose_rmf.orientation_radians + math.pi),
        )
        self.node.get_logger().info(
            f'Captured cleaning start pose for [{self.name}] on map '
            f'[{command_map}]: start={clean_start_pose_rmf}, '
            f'return={return_pose_rmf}')

        remaining_retries = self.navigation_max_retries
        while self._execution_is_active(execution):
            self.node.get_logger().info(
                f'Requesting robot [{self.name}] to clean [{zone}] '
                f'on map [{command_map}] using robot zone [{robot_zone_name}]')
            cleaning_command_sent = False
            seen_cleaning = False

            while self._execution_is_active(execution):
                if not cleaning_command_sent:
                    if not self.api.send_cleaning_command(
                            self.name,
                            robot_zone_name,
                            self._robot_map_name(command_map),
                            self._robot_map_level(command_map)):
                        if remaining_retries <= 0:
                            self._fail_execution(
                                execution,
                                f'Robot [{self.name}] failed to accept '
                                f'cleaning [{zone}]; no retries left')
                            return
                        remaining_retries -= 1
                        self.node.get_logger().info(
                            f'Robot [{self.name}] failed to accept cleaning '
                            f'[{zone}]; retrying '
                            f'({remaining_retries} retries left)')
                        if not self._sleep(1.0, execution):
                            return
                        break
                    cleaning_command_sent = True
                    self.command_handle.set_state(
                        execution, CommandState.CLEANING)
                    self.command_handle.start_operation_attempt(execution)

                robot_status = self.api.get_robot_status(self.name)
                runtime_status = (
                    robot_status.get('status')
                    if isinstance(robot_status, dict) else None)
                if runtime_status in _CLEANING_STATUSES:
                    # The robot reports that it works on the clean, so the
                    # timeout counts only the time after it stops doing so.
                    seen_cleaning = True
                    self.command_handle.start_operation_attempt(execution)

                clean_status = self._operation_result(
                    execution, self.api.process_completed(self.name))
                # Some robots send no clean_end_status. After the robot was
                # seen cleaning, a docking status means the clean ended and
                # the robot goes home by itself, so it gets no more commands.
                if seen_cleaning and clean_status == CommandResult.RUNNING \
                        and runtime_status in _DOCKING_STATUSES:
                    self.node.get_logger().info(
                        f'Robot [{self.name}] reports {runtime_status} after '
                        f'cleaning [{zone}]; the clean is finished')
                    self._finish_execution(execution)
                    return
                # Resting can also be a short gap, so it counts as the end
                # only when it outlasts the timeout with no clean_end_status.
                if seen_cleaning and clean_status == CommandResult.TIMEOUT \
                        and runtime_status == \
                        RobotRuntimeStatusText.RESTING.value:
                    clean_status = CommandResult.SUCCESS
                if clean_status == CommandResult.SUCCESS:
                    self.node.get_logger().info(
                        f'Robot [{self.name}] completed cleaning')
                    self._return_to_clean_start_waypoint(
                        command_map,
                        return_pose_rmf,
                        execution)
                    return

                if clean_status in (
                    CommandResult.ERROR,
                    CommandResult.TIMEOUT,
                ):
                    failure = (
                        'timed out cleaning'
                        if clean_status == CommandResult.TIMEOUT
                        else 'failed to clean'
                    )
                    if remaining_retries <= 0:
                        self._fail_execution(
                            execution,
                            f'Robot [{self.name}] {failure} [{zone}]; '
                            'no retries left')
                        return
                    remaining_retries -= 1
                    self.node.get_logger().info(
                        f'Robot [{self.name}] {failure} [{zone}]; retrying '
                        f'({remaining_retries} retries left)')
                    break

                if not self._sleep(0.5, execution):
                    return

    def _return_to_clean_start_waypoint(
            self,
            target_map: str,
            return_pose_rmf: RmfCoord,
            execution):
        if not self._wait_until_ready(execution):
            if self._execution_is_active(execution):
                self._fail_execution(
                    execution,
                    f'Robot [{self.name}] was not idle before returning to '
                    'its cleaning start pose')
            return

        transform = self.transforms.get(target_map)
        if transform is None:
            self._fail_execution(
                execution,
                f'Missing map transform for [{target_map}]')
            return

        return_pose_robot = transform.rmf_meters_to_robot(
            rmf_coord=return_pose_rmf)
        remaining_retries = self.navigation_max_retries

        while self._execution_is_active(execution):
            self.node.get_logger().info(
                f'Returning robot [{self.name}] to its cleaning start pose '
                f'{return_pose_rmf} = {return_pose_robot}')
            navigation_command_sent = False
            self.command_handle.set_state(
                execution, CommandState.P2P_MOVE)

            while self._execution_is_active(execution):
                if not navigation_command_sent:
                    if not self.api.send_navigation_command(
                            self.name,
                            return_pose_robot,
                            self._robot_map_name(target_map)):
                        if remaining_retries <= 0:
                            self._fail_execution(
                                execution,
                                f'Robot [{self.name}] failed to accept return '
                                'navigation after cleaning; no retries left')
                            return
                        remaining_retries -= 1
                        self.node.get_logger().info(
                            f'Robot [{self.name}] failed to accept return '
                            f'navigation after cleaning; retrying '
                            f'({remaining_retries} retries left)')
                        if not self._sleep(1.0, execution):
                            return
                        break
                    navigation_command_sent = True
                    self.command_handle.start_operation_attempt(execution)

                navigate_status = self._operation_result(
                    execution, self.api.navigate_robot(
                        self.name, return_pose_robot, target_map))
                if navigate_status == CommandResult.SUCCESS:
                    self.node.get_logger().info(
                        f'Robot [{self.name}] returned to its cleaning '
                        'start pose')
                    self._finish_execution(execution)
                    return

                if navigate_status in (
                    CommandResult.ERROR,
                    CommandResult.TIMEOUT,
                ):
                    failure = (
                        'timed out returning after cleaning'
                        if navigate_status == CommandResult.TIMEOUT
                        else 'failed to return after cleaning'
                    )
                    if remaining_retries <= 0:
                        self._fail_execution(
                            execution,
                            f'Robot [{self.name}] {failure}; '
                            'no retries left')
                        return
                    remaining_retries -= 1
                    self.node.get_logger().info(
                        f'Robot [{self.name}] {failure}; retrying '
                        f'({remaining_retries} retries left)')
                    break

                if not self._sleep(0.5, execution):
                    return
