# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import argparse
import asyncio
import faulthandler
import math
import os
import sys
import tempfile
import threading
import time
import traceback

import numpy as np
from fleet_adapter.LionsbotRobot import LionsbotRobot
import rclpy
from rclpy.duration import Duration
import rclpy.node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
import rmf_adapter as adpt
from rmf_adapter import Adapter
import rmf_adapter.easy_full_control as rmf_easy

from rmf_charger_msgs.msg import ChargerRequest
from rmf_fleet_msgs.msg import ClosedLanes
from rmf_fleet_msgs.msg import LaneRequest
from rmf_fleet_msgs.msg import ModeRequest
from rmf_fleet_msgs.msg import RobotMode
from rmf_fleet_msgs.msg import SpeedLimitRequest
import yaml

from .utils.logging import configure_debug_file_logging
from .utils.logging import install_ros_logger_file_mirror
from .RobotClientAPI import RobotAPI
from .utils.Coordinate import LionsbotCoord
from .utils.MapTransform import MapTransform
from .utils.robot_config import robot_uuids_from_config

def initialize_map_transform(
        map_transform_config: dict) -> dict[str, MapTransform]:
    transforms: dict[str, MapTransform] = {}

    for map_ref, transform_method in map_transform_config.items():
        level_transform = transform_method.get('level_transform', {})
        level_tx_pixels = level_transform.get('tx_pixels', 0)
        level_ty_pixels = level_transform.get('ty_pixels', 0)
        floorplan_scale_meters_per_pixel = level_transform.get('scale', 1)

        if 'transform_values' in transform_method:
            values = transform_method['transform_values']
            transforms[map_ref] = MapTransform.compute(
                tx_meters=values['tx_meters'],
                ty_meters=values['ty_meters'],
                rotation_in_radians=math.radians(
                    values['rotation_degrees']),
                lbmap_translation_scale_factor=values['scale'],
                level_tx_pixels=level_tx_pixels,
                level_ty_pixels=level_ty_pixels,
                floorplan_scale_meters_per_pixel=(
                    floorplan_scale_meters_per_pixel))
    return transforms


def make_clean_action_paths(dock_summary_yaml, fleet_name: str):
    action_paths = {}
    for dock_name, dock_info in dock_summary_yaml.get(fleet_name, {}).items():
        action_paths[dock_name] = {
            'map_name': dock_info['level_name'],
            'path': dock_info.get('path', []),
            'finish_waypoint': dock_info.get(
                'finish_waypoint',
                dock_info.get('robot_zone_name', dock_name)),
        }

    return action_paths

def make_fleet_configuration(
        config_yaml,
        nav_graph_path,
        server_uri,
        dock_summary_yaml,
        debug_config_path=None):
    fleet_config = config_yaml['rmf_fleet']
    update_frequencies = []
    max_delays = []
    robots = {}
    for robot_name, robot_config in config_yaml['robots'].items():
        rmf_config = robot_config['rmf_config']
        robots[robot_name] = {'charger': rmf_config['charger']['waypoint']}
        update_frequencies.append(
            rmf_config.get('robot_state_update_frequency', 1.0))

        max_delay = robot_config.get('robot_config', {}).get('max_delay')
        if max_delay is not None:
            max_delays.append(max_delay)

    update_frequency = max(update_frequencies) if update_frequencies else 1.0
    update_period = 1.0 / update_frequency
    default_max_delay = max(max_delays) if max_delays else 10.0
    task_capabilities = fleet_config['task_capabilities']

    actions = []
    if task_capabilities.get('clean'):
        actions.append('clean')
    actions.append('docking')

    finishing_request = task_capabilities.get('finishing_request', 'nothing')
    publish_fleet_state = fleet_config.get('publish_fleet_state', True)
    if isinstance(publish_fleet_state, bool):
        publish_fleet_state = update_frequency if publish_fleet_state else 0.0

    fleet_manager_yaml = {
        'prefix': 'unused',
        'user': 'unused',
        'password': 'unused',
    }
    clean_action_paths = make_clean_action_paths(
        dock_summary_yaml, fleet_config['name'])
    if clean_action_paths:
        fleet_manager_yaml['action_paths'] = {
            'clean': clean_action_paths,
        }

    easy_yaml = {
        'rmf_fleet': {
            'name': fleet_config['name'],
            'limits': fleet_config['limits'],
            'profile': fleet_config['profile'],
            'reversible': fleet_config['reversible'],
            'battery_system': fleet_config['battery_system'],
            'mechanical_system': fleet_config['mechanical_system'],
            'ambient_system': fleet_config['ambient_system'],
            'tool_system': fleet_config['tool_system'],
            'recharge_threshold': fleet_config['recharge_threshold'],
            'recharge_soc': fleet_config['recharge_soc'],
            'max_delay': default_max_delay,
            'publish_fleet_state': publish_fleet_state,
            'account_for_battery_drain': (
                fleet_config['account_for_battery_drain']),
            'task_capabilities': {
                'loop': task_capabilities.get('loop', False),
                'delivery': task_capabilities.get('delivery', False),
                'clean': task_capabilities.get('clean', False),
            },
            'actions': actions,
            'finishing_request': finishing_request,
            'robots': robots,
            'robot_state_update_frequency': update_frequency,
        },
        'fleet_manager': fleet_manager_yaml,
    }

    temp_path = None
    try:
        if debug_config_path is not None:
            temp_path = debug_config_path
            with open(temp_path, 'w') as f:
                yaml.safe_dump(easy_yaml, f)
        else:
            with tempfile.NamedTemporaryFile(
                    mode='w', suffix='.yaml', delete=False) as f:
                yaml.safe_dump(easy_yaml, f)
                temp_path = f.name

        easy_config = rmf_easy.FleetConfiguration.from_config_files(
            temp_path, nav_graph_path, server_uri=server_uri)
        assert easy_config, 'Failed to create EasyFullControl fleet config'
    finally:
        if temp_path is not None and debug_config_path is None:
            os.unlink(temp_path)

    return easy_config, easy_config.graph, None, update_period

def make_dock_paths(dock_summary_yaml, fleet_name: str):
    dock_paths = {}
    for dock_name, dock_info in dock_summary_yaml.get(fleet_name, {}).items():
        dock_paths[dock_name] = {
            'level_name': dock_info['level_name'],
            'robot_zone_name': dock_info.get('robot_zone_name', dock_name),
            'path': [
                np.array([p[0], p[1], p[2]], dtype=np.float64)
                for p in dock_info.get('path', [])
            ],
        }

    return dock_paths

def start_add_fleet_robots_thread(
        config_yaml,
        fleet_name,
        node,
        nav_graph,
        fleet_config,
        fleet_handle,
        transforms,
        dock_paths,
        api,
        robots,
        robots_lock,
    ):
    # initialize_robot is retried until each robot is ready. Keep this state
    # outside it so a missing status does not defeat the log throttle.
    last_status_wait_log = {}

    def initialize_robot(
        robot_name: str,
        robot_config: dict,
        fleet_name: str,
        node,
        nav_graph,
        fleet_config,
        transforms,
        dock_paths,
        api,
    ):
        if not api.is_subscribed(robot_name):
            node.get_logger().info(f'Subscribing to robot: {robot_name}')
            if not api.subscribe_to_robot(robot_name, time.time_ns() / 1000000):
                node.get_logger().debug(
                    f'Subscribe failed for {robot_name}, retrying...')
                return None

        robot_position: LionsbotCoord = api.position(robot_name)
        if robot_position is None:
            node.get_logger().debug(
                f'Unable to get robot [{robot_name}] position; retrying')
            return None

        rmf_config = robot_config['rmf_config']
        start_config = rmf_config['start']
        rmf_level = start_config.get('rmf_level', start_config.get('map_name'))
        if rmf_level is None:
            node.get_logger().error(
                f'Missing rmf_level for robot [{robot_name}]')
            return None

        robot_map_name = start_config.get('robot_map_name', rmf_level)
        robot_map_level = start_config.get(
            'robot_map_level',
            start_config.get('robot_level'))
        if robot_map_level is None:
            robot_map_level = rmf_level
        transform = transforms.get(rmf_level)
        if transform is None:
            node.get_logger().error(f'Missing map transform for [{rmf_level}]')
            return None

        robot_position_rmf = transform.robot_to_rmf_meters(robot_position)
        if robot_position_rmf.orientation_radians is None:
            node.get_logger().debug(
                f'Robot [{robot_name}] position has no orientation; retrying')
            return None

        # Do not perform map changes or create a CommandHandle worker until
        # the robot has supplied its first touchscreen status. This function is
        # retried every 0.2 seconds while a robot is unavailable.
        robot_status = api.get_robot_status(robot_encoding_id=robot_name)
        if robot_status is None:
            now = time.monotonic()
            if now - last_status_wait_log.get(robot_name, 0.0) >= 10.0:
                node.get_logger().info(
                    f'Waiting for first status from robot [{robot_name}] '
                    'before localization check')
                last_status_wait_log[robot_name] = now
            return None

        api.robot_current_map[robot_name] = robot_map_name

        node.get_logger().info(f'Changing to start map of robot: {robot_name}')
        if not api.change_map(
                robot_encoding_id=robot_name,
                map_name=robot_map_name,
                map_level=robot_map_level):
            node.get_logger().info(
                f'Changing map [{robot_map_name}] failed for robot [{robot_name}]')
            return None

        robot_map_names = dict(rmf_config.get('robot_map_names', {}))
        robot_map_levels = dict(rmf_config.get('robot_map_levels', {}))
        for level_name, robot_map in rmf_config.get('robot_maps', {}).items():
            if isinstance(robot_map, dict):
                if 'name' in robot_map:
                    robot_map_names[level_name] = robot_map['name']
                if 'level' in robot_map:
                    robot_map_levels[level_name] = robot_map['level']
            else:
                robot_map_names[level_name] = robot_map

        robot_map_names.setdefault(rmf_level, robot_map_name)
        robot_map_levels.setdefault(rmf_level, robot_map_level)

        configuration = fleet_config.get_known_robot_configuration(robot_name)
        robot = LionsbotRobot(
            name=robot_name,
            fleet_name=fleet_name,
            config=robot_config,
            rmf_config=rmf_config,
            node=node,
            nav_graph=nav_graph,
            transforms=transforms,
            map_name=rmf_level,
            robot_map_names=robot_map_names,
            robot_map_levels=robot_map_levels,
            position=robot_position_rmf,
            configuration=configuration,
            dock_paths=dock_paths,
            api=api,
        )

        localization_position = None
        undocked_for_localize = False

        while rclpy.ok():
            robot_status = api.get_robot_status(robot_encoding_id=robot_name)
            if robot_status is None:
                now = time.monotonic()
                if now - last_status_wait_log.get(robot_name, 0.0) >= 10.0:
                    node.get_logger().info(
                        f'Waiting for status from robot [{robot_name}] during '
                        'localization')
                    last_status_wait_log[robot_name] = now
                time.sleep(0.5)
                continue

            localized = robot_status.get('localized')
            if localized is True:
                node.get_logger().info(
                    f'Robot [{robot_name}] is already localized; skipping '
                    'hot-localize')
                break

            if localized is not False:
                node.get_logger().info(
                    f'Robot [{robot_name}] status does not include a definitive '
                    f'localized=false value; status localized={localized}')
                time.sleep(0.5)
                return None

            if localization_position is None:
                localization_starting_point = start_config.get(
                    'localization_starting_point')
                if localization_starting_point is None:
                    node.get_logger().error(
                        f'Missing localization_starting_point for robot '
                        f'[{robot_name}]')
                    return None

                try:
                    localization_position = LionsbotCoord(
                        x=float(localization_starting_point['x']),
                        y=float(localization_starting_point['y']),
                        orientation_radians=float(
                            localization_starting_point['heading']),
                    )
                except (KeyError, TypeError, ValueError):
                    node.get_logger().error(
                        f'Invalid localization_starting_point for robot '
                        f'[{robot_name}]; expected x, y, and heading')
                    return None

            if not undocked_for_localize:
                if not robot._ensure_undocked():
                    node.get_logger().error(
                        f'Unable to undock robot [{robot_name}] before '
                        'hot-localize')
                    return None
                undocked_for_localize = True

            node.get_logger().info(f'Localizing robot: {robot_name}')
            is_localized = api.localize(
                localization_position, robot_encoding_id=robot_name)
            if not is_localized:
                node.get_logger().info(
                    f'Localizing robot [{robot_name}] failed at '
                    f'{localization_position}')
            time.sleep(0.5)
            robot_status = api.get_robot_status(robot_encoding_id=robot_name)
            if robot_status is None:
                now = time.monotonic()
                if now - last_status_wait_log.get(robot_name, 0.0) >= 10.0:
                    node.get_logger().info(
                        f'Waiting for status from robot [{robot_name}] during '
                        'localization')
                    last_status_wait_log[robot_name] = now
                time.sleep(0.5)
                continue

        return robot

    def apply_robot_update_settings(robot: LionsbotRobot):
        if robot.update_handle is None:
            return

        # TODO: understand how the undering cpp threadpool works
        # Wait up to 1 second for the underlying C++ pointer to stabilize
        robot_update_handle = None
        attempts = 0
        while robot_update_handle is None and attempts < 10:
            robot_update_handle = robot.update_handle.more()
            if robot_update_handle is None:
                time.sleep(0.1) # Sleep for 100ms
                attempts += 1

        if robot_update_handle is None:
            robot.node.get_logger().error(
                f"Timed out waiting for underlying C++ handle for [{robot.name}]."
            )
            return

        max_delay = robot.config.get('robot_config', {}).get('max_delay')
        if max_delay is not None:
            robot.node.get_logger().info(
                f'Setting max delay for [{robot.name}] to {max_delay}s')
            robot_update_handle.set_maximum_delay(max_delay)

        if robot.charger_waypoint_index is not None:
            robot_update_handle.set_charger_waypoint(robot.charger_waypoint_index)

    def add_fleet_robots():
        missing_robots = dict(config_yaml['robots'])
        while rclpy.ok() and missing_robots:
            time.sleep(0.2)
            if not api.is_connected():
                now = time.monotonic()
                if now - last_status_wait_log.get(0, 0.0) >= 10.0:
                    node.get_logger().warning(
                        'Robot API telemetry not connected; robot registration '
                        'is waiting')
                    last_status_wait_log[0] = now
                
            for robot_name in list(missing_robots.keys()):
                node.get_logger().debug(f'Connecting to robot: {robot_name}')
                robot = initialize_robot(
                    robot_name=robot_name,
                    robot_config=missing_robots[robot_name],
                    fleet_name=fleet_name,
                    node=node,
                    nav_graph=nav_graph,
                    fleet_config=fleet_config,
                    transforms=transforms,
                    dock_paths=dock_paths,
                    api=api,
                )
                if robot is None:
                    continue

                update_handle = fleet_handle.add_robot(
                    robot.name,
                    robot.make_state(),
                    robot.configuration,
                    robot.make_callbacks(),
                )
                
                if update_handle is None:
                    node.get_logger().error(
                        f'Failed to add robot [{robot_name}] to RMF fleet')
                    continue

                robot.update_handle = update_handle
                apply_robot_update_settings(robot)

                with robots_lock:
                    robots[robot_name] = robot

                node.get_logger().info(
                    f'Successfully added new robot: {robot_name}')
                del missing_robots[robot_name]
    add_robots_thread = threading.Thread(target=add_fleet_robots, daemon=True)
    add_robots_thread.start()

def start_update_thread(
    node,
    robots,
    robots_lock,
    fleet_handle,
    update_period,
    reassign_task_interval,
):
    def update_loop():
        last_task_replan = node.get_clock().now()
        last_update_failure_log = {}
        asyncio.set_event_loop(asyncio.new_event_loop())
        while rclpy.ok():
            now = node.get_clock().now()
            with robots_lock:
                update_jobs = {
                    robot.name: asyncio.get_event_loop().run_in_executor(
                        None, robot.update)
                    for robot in robots.values()
                }

            if update_jobs:
                asyncio.get_event_loop().run_until_complete(
                    asyncio.wait(update_jobs.values())
                )
                for robot_name, update_job in update_jobs.items():
                    try:
                        update_job.result()
                    except Exception as error:
                        # One bad robot update must be visible, but a rapidly
                        # repeating fault should not flood the adapter log.
                        failure_key = (robot_name, type(error))
                        last_logged = last_update_failure_log.get(
                            failure_key, 0.0)
                        now_monotonic = time.monotonic()
                        if now_monotonic - last_logged >= 10.0:
                            node.get_logger().error(
                                f'Robot [{robot_name}] update failed: '
                                f'{error!r}\n{traceback.format_exc()}')
                            last_update_failure_log[failure_key] = \
                                now_monotonic

            interval_sec = (
                now.nanoseconds - last_task_replan.nanoseconds) / 1e9
            if interval_sec > reassign_task_interval:
                fleet_handle.more().reassign_dispatched_tasks()
                last_task_replan = now

            next_wakeup = now + Duration(
                nanoseconds=int(update_period * 1e9))
            while rclpy.ok() and node.get_clock().now() < next_wakeup:
                time.sleep(0.001)

    update_thread = threading.Thread(target=update_loop, daemon=True)
    update_thread.start()

def ros_connections(node, robots, robots_lock, fleet_handle):
    fleet_name = fleet_handle.more().fleet_name

    transient_qos = QoSProfile(
        history=History.KEEP_LAST,
        depth=1,
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL,
    )

    closed_lanes_pub = node.create_publisher(
        ClosedLanes, 'closed_lanes', qos_profile=transient_qos)
    closed_lanes = set()

    def lane_request_cb(msg):
        if msg.fleet_name and msg.fleet_name != fleet_name:
            node.get_logger().info(
                f'Ignoring lane request for fleet [{msg.fleet_name}]')
            return

        if msg.open_lanes:
            node.get_logger().info(f'Opening lanes: {msg.open_lanes}')
        if msg.close_lanes:
            node.get_logger().info(f'Closing lanes: {msg.close_lanes}')

        fleet_handle.more().open_lanes(msg.open_lanes)
        fleet_handle.more().close_lanes(msg.close_lanes)

        for lane_idx in msg.close_lanes:
            closed_lanes.add(lane_idx)
        for lane_idx in msg.open_lanes:
            closed_lanes.discard(lane_idx)

        state_msg = ClosedLanes()
        state_msg.fleet_name = fleet_name
        state_msg.closed_lanes = list(closed_lanes)
        closed_lanes_pub.publish(state_msg)

    def speed_limit_request_cb(msg):
        if msg.fleet_name is None or msg.fleet_name != fleet_name:
            return

        requests = []
        for limit in msg.speed_limits:
            requests.append(adpt.fleet_update_handle.SpeedLimitRequest(
                limit.lane_index, limit.speed_limit))

        fleet_handle.more().limit_lane_speeds(requests)
        fleet_handle.more().remove_speed_limits(msg.remove_limits)

    def mode_request_cb(msg):
        if (
            msg.fleet_name is None
            or msg.fleet_name != fleet_name
            or msg.robot_name is None
        ):
            return

        if msg.mode.mode != RobotMode.MODE_IDLE:
            return

        with robots_lock:
            robot = robots.get(msg.robot_name)
        if robot is not None:
            robot.finish_action()
    
    def set_robot_charger_waypoint(robot: LionsbotRobot, charger_name: str):
        waypoint = robot.graph.find_waypoint(charger_name)
        if waypoint is None:
            robot.node.get_logger().error(
                f'Unable to reassign [{robot.name}] charger; waypoint '
                f'[{charger_name}] does not exist')
            return False

        if not waypoint.charger:
            robot.node.get_logger().error(
                f'Unable to reassign [{robot.name}] charger; waypoint '
                f'[{charger_name}] is not marked as a charger')
            return False

        if robot.update_handle is None:
            robot.node.get_logger().error(
                f'Unable to reassign [{robot.name}] charger; robot has no '
                'RMF update handle')
            return False

        robot_update_handle = robot.update_handle.more()
        if robot_update_handle is None:
            robot.node.get_logger().error(
                f'Unable to reassign [{robot.name}] charger; RMF update handle '
                'is not ready')
            return False

        robot_update_handle.set_charger_waypoint(waypoint.index)
        robot.charger_waypoint_index = waypoint.index
        robot.rmf_config.setdefault('charger', {})['waypoint'] = charger_name
        robot.node.get_logger().info(
            f'Reassigned charger for [{robot.name}] to [{charger_name}] '
            f'(waypoint_index={waypoint.index})')
        return True


    def charger_request_cb(msg):
        if msg.fleet_name and msg.fleet_name != fleet_name:
            node.get_logger().info(
                f'Ignoring charger request for fleet [{msg.fleet_name}]')
            return

        if not msg.robot_name:
            node.get_logger().error(
                'Ignoring charger request with no robot_name')
            return

        if not msg.charger_name:
            node.get_logger().error(
                f'Ignoring charger request for robot [{msg.robot_name}] '
                'with no charger_name')
            return

        with robots_lock:
            robot = robots.get(msg.robot_name)

        if robot is None:
            node.get_logger().error(
                f'Ignoring charger request for unknown robot '
                f'[{msg.robot_name}]')
            return

        set_robot_charger_waypoint(robot, msg.charger_name)

    lane_request_sub = node.create_subscription(
        LaneRequest,
        'lane_closure_requests',
        lane_request_cb,
        qos_profile=qos_profile_system_default,
    )
    speed_limit_request_sub = node.create_subscription(
        SpeedLimitRequest,
        'speed_limit_requests',
        speed_limit_request_cb,
        qos_profile=qos_profile_system_default,
    )
    action_execution_notice_sub = node.create_subscription(
        ModeRequest,
        'action_execution_notice',
        mode_request_cb,
        qos_profile=qos_profile_system_default,
    )
    charger_request_sub = node.create_subscription(
        ChargerRequest,
        'charger_requests',
        charger_request_cb,
        qos_profile=qos_profile_system_default,
    )

    return [
        lane_request_sub,
        speed_limit_request_sub,
        action_execution_notice_sub,
        charger_request_sub,
    ]

def main(argv=sys.argv):
    faulthandler.enable()
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    # Parse config files and maps
    parser = argparse.ArgumentParser(
        prog='fleet_adapter',
        description='Configure and spin up the fleet adapter')
    parser.add_argument('-c', '--config_file', type=str, required=True,
                        help='Path to the config.yaml file')
    parser.add_argument('-n', '--nav_graph', type=str, required=True,
                        help='Path to the nav_graph for this fleet adapter')
    parser.add_argument('-s', '--server_uri', type=str, required=False,
                        default='',
                        help='URI of the api server to transmit state and '
                             'task information.')
    parser.add_argument('--use_sim_time', '-sim', action='store_true',
                        help='Use sim time, default: false')
    parser.add_argument('-d', '--dock_summary_file', type=str, required=True,
                        help='Path to the dock_summary.yaml file')
    parser.add_argument('--debug_easy_config_path', type=str, default=None,
                        help='Write the generated EasyFullControl config YAML '
                             'to this path for debugging.')
    parser.add_argument('--debug_log_file', type=str, default=None,
                        help='Write DEBUG and above logs to this file without '
                             'raising console log verbosity. Defaults to a '
                             'new timestamped file for each run.')
    args = parser.parse_args(args_without_ros[1:])

    debug_log_path = configure_debug_file_logging(args.debug_log_file)
    print('Starting fleet adapter...')

    with open(args.config_file, 'r') as f:
        config_yaml = yaml.safe_load(f)

    robot_uuids = robot_uuids_from_config(config_yaml)

    with open(args.dock_summary_file, 'r') as f:
        dock_summary_yaml = yaml.safe_load(f)

    server_uri = args.server_uri if args.server_uri else "ws://localhost:8000/_internal"

    # Initialize fleet based on config.yaml
    fleet_config, nav_graph, _, update_period = make_fleet_configuration(
        config_yaml,
        args.nav_graph,
        server_uri,
        dock_summary_yaml,
        args.debug_easy_config_path)

    fleet_name = fleet_config.fleet_name
    node = rclpy.node.Node(f'{fleet_name}_command_handle')
    install_ros_logger_file_mirror(node)
    node.get_logger().info(f'Debug log file: {debug_log_path}')

    #  Initialize adapter
    try:
        adapter = Adapter.make(f'{fleet_name}_fleet_adapter')
    except Exception as error:
        node.get_logger().error(
            f'Exception while initializing fleet adapter: {error!r}')
        node.get_logger().error(traceback.format_exc())
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if not adapter:
        node.get_logger().error(
            'Unable to initialize fleet adapter. '
            'Adapter.make() returned no adapter. '
            'Please ensure RMF Schedule Node is running and reachable. '
            f'fleet_name={fleet_name!r}, '
            f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "<unset>")!r}, '
            f'RMW_IMPLEMENTATION={os.environ.get("RMW_IMPLEMENTATION", "<default>")!r}')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if args.use_sim_time:
        param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        node.set_parameters([param])
        adapter.node.use_sim_time()

    adapter.start()
    time.sleep(1.0)

    fleet_handle = adapter.add_easy_fleet(fleet_config)
    if hasattr(fleet_handle.more(), 'set_planner_cache_reset_size'):
        fleet_handle.more().set_planner_cache_reset_size(2500)

    if not config_yaml['rmf_fleet']['publish_fleet_state']:
        fleet_handle.more().fleet_state_topic_publish_period(None)

    transforms = initialize_map_transform(config_yaml['map_transform'])
    dock_paths = make_dock_paths(dock_summary_yaml, fleet_name)
    node.get_logger().info(
        f'Advertised clean zones: {list(dock_paths.keys())}')

    fleet_manager = config_yaml['rmf_fleet']['fleet_manager']
    api = RobotAPI(
        fleet_manager['prefix'],
        fleet_manager['user'],
        fleet_manager['password'],
        robot_uuids=robot_uuids,
    )

    robots = {}
    robots_lock = threading.Lock()
    start_add_fleet_robots_thread(
        config_yaml,
        fleet_name,
        node,
        nav_graph,
        fleet_config,
        fleet_handle,
        transforms,
        dock_paths,
        api,
        robots,
        robots_lock,
    )

    reassign_task_interval = config_yaml['rmf_fleet'].get(
        'reassign_task_interval', 60)
    
    start_update_thread(
        node,
        robots,
        robots_lock,
        fleet_handle,
        update_period,
        reassign_task_interval)
    
    ros_connections(node, robots, robots_lock, fleet_handle)

    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    rclpy_executor.spin()

    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
