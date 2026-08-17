import math
import os
import threading
import time
import uuid

import numpy as np
import rclpy
import rmf_adapter.easy_full_control as rmf_easy
import requests

from .enums.enums import CommandResult
from .enums.enums import CommandKind
from .enums.enums import CommandState
from .enums.enums import OperationStatus
from .enums.enums import RobotRuntimeStatusText
from .CommandHandle import CommandHandle
from .models.DockProcessContent import DockProcessContent
from .RobotClientAPI import RobotAPI
from .utils.Coordinate import RmfCoord
from .utils.MapTransform import MapTransform

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
    ):
        self.name = name
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
        return rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(
                destination, execution
            ),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(
                category, description, execution
            ),
        )

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

    def _send_docked_not_charging_alert(self, elapsed_sec, robot_status):
        if not self.rmf_web_api_token:
            self.node.get_logger().error(
                'Cannot create RMF Web alert [docked_not_charging] for '
                f'[{self.name}]: RMF_WEB_API_TOKEN is not configured')
            return

        payload = {
            'id': f'docked-not-charging-{self.name}-{uuid.uuid4()}',
            'unix_millis_alert_time': int(time.time() * 1000),
            'title': 'Robot docked but not charging',
            'subtitle': f'{self.fleet_name}/{self.name}',
            'message': (
                f'Robot [{self.name}] has reported Docked for '
                f'{elapsed_sec:.0f}s but is not charging.'),
            'display': True,
            'tier': 'warning',
            'responses_available': ['Acknowledge'],
            'alert_parameters': [
                {'name': 'robot', 'value': self.name},
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
                'Failed to create RMF Web alert [docked_not_charging] for '
                f'[{self.name}]: {error!r}')
            return

        self.node.get_logger().warning(
            f'Created RMF Web alert [docked_not_charging] for '
            f'robot [{self.name}]')

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
            self._navigate_worker(event.destination, event.execution)
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
            reversed_heading = MapTransform.wrap_orientation(
                destination_pose_rmf.orientation_radians)
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

                clean_status = self._operation_result(
                    execution, self.api.process_completed(self.name))
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
