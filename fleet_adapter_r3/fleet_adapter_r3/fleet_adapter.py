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

import sys
import argparse

import yaml
import nudged
import time
import threading
import copy
import math

import rclpy
import rclpy.node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from rmf_task_msgs.msg import TaskProfile, TaskType
from rmf_fleet_msgs.msg import ModeRequest, PathRequest, Location, \
    RobotState, RobotMode, DockSummary, Dock, DockParameter

from functools import partial
from typing import Dict

from .enums.enums import Topic
from .RobotCommandHandle import RobotCommandHandle
from .RobotClientAPI import RobotAPI
from .utils.MapTransform import MapTransform
from .utils.Coordinate import RmfCoord
from .utils.Coordinate import LionsbotCoord


# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------


def initialize_fleet(config_yaml, nav_graph_path, node, use_sim_time, server_uri, dock_summary_yaml, safe_nav_flag):
    # Dock summary
    def make_location(p, level_name):
        location = Location()
        location.x = p[0]
        location.y = p[1]
        location.yaw = p[2]
        location.level_name = level_name
        return location

    transient_qos = QoSProfile(
        history=History.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=1,
        reliability=Reliability.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        durability=Durability.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    dock_summary_publisher = node.create_publisher(
        DockSummary, Topic.DOCK_SUMMARY.value, qos_profile=transient_qos)

    dock_summary = DockSummary()
    dock_map = {}
    for fleet_name, docking_info in dock_summary_yaml.items():
        dock_sub_map = {}
        dock = Dock()
        dock.fleet_name = fleet_name
        for dock_name, dock_waypoints in docking_info.items():
            param = DockParameter()
            param.start = dock_name
            finish_waypoint = dock_waypoints.get("finish_waypoint")
            if finish_waypoint is None:
                # for backwards compatibility
                finish_waypoint = dock_name
            param.finish = finish_waypoint
            for point in dock_waypoints["path"]:
                location = make_location(
                    point, dock_waypoints["level_name"])
                param.path.append(location)
            dock.params.append(param)
            dock_sub_map[dock_name] = param.path
        dock_summary.docks.append(dock)
        dock_map[fleet_name] = dock_sub_map
    time.sleep(2)
    dock_summary_publisher.publish(dock_summary)

    # Profile and traits
    fleet_config = config_yaml['rmf_fleet']
    profile = traits.Profile(geometry.make_final_convex_circle(
        fleet_config['profile']['footprint']),
        geometry.make_final_convex_circle(fleet_config['profile']['vicinity']))
    vehicle_traits = traits.VehicleTraits(
        linear=traits.Limits(*fleet_config['limits']['linear']),
        angular=traits.Limits(*fleet_config['limits']['angular']),
        profile=profile)
    vehicle_traits.differential.reversible = fleet_config['reversible']

    # Battery system
    voltage = fleet_config['battery_system']['voltage']
    capacity = fleet_config['battery_system']['capacity']
    charging_current = fleet_config['battery_system']['charging_current']
    battery_sys = battery.BatterySystem.make(
        voltage, capacity, charging_current)

    # Mechanical system
    mass = fleet_config['mechanical_system']['mass']
    moment = fleet_config['mechanical_system']['moment_of_inertia']
    friction = fleet_config['mechanical_system']['friction_coefficient']
    mech_sys = battery.MechanicalSystem.make(mass, moment, friction)

    # Power systems
    ambient_power_sys = battery.PowerSystem.make(
        fleet_config['ambient_system']['power'])
    tool_power_sys = battery.PowerSystem.make(
        fleet_config['tool_system']['power'])

    # Power sinks
    motion_sink = battery.SimpleMotionPowerSink(battery_sys, mech_sys)
    ambient_sink = battery.SimpleDevicePowerSink(
        battery_sys, ambient_power_sys)
    tool_sink = battery.SimpleDevicePowerSink(battery_sys, tool_power_sys)

    nav_graph = graph.parse_graph(nav_graph_path, vehicle_traits)

    # Adapter
    fleet_name = fleet_config['name']
    adapter = adpt.Adapter.make(f'{fleet_name}_fleet_adapter')
    if use_sim_time:
        adapter.node.use_sim_time()
    assert adapter, ("Unable to initialize fleet adapter. Please ensure "
                     "RMF Schedule Node is running")
    adapter.start()
    time.sleep(1.0)

    fleet_handle = adapter.add_fleet(fleet_name, vehicle_traits, nav_graph, server_uri)

    if not fleet_config['publish_fleet_state']:
        fleet_handle.fleet_state_publish_period(None)
    # Account for battery drain
    drain_battery = fleet_config['account_for_battery_drain']
    recharge_threshold = fleet_config['recharge_threshold']
    recharge_soc = fleet_config['recharge_soc']
    finishing_request = fleet_config['task_capabilities']['finishing_request']
    node.get_logger().info(f"Finishing request: [{finishing_request}]")
    # Set task planner params
    ok = fleet_handle.set_task_planner_params(
        battery_sys,
        motion_sink,
        ambient_sink,
        tool_sink,
        recharge_threshold,
        recharge_soc,
        drain_battery,
        finishing_request)
    assert ok, ("Unable to set task planner params")

    task_capabilities = []
    if fleet_config['task_capabilities']['loop']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Loop tasks")
        task_capabilities.append(TaskType.TYPE_LOOP)
    if fleet_config['task_capabilities']['delivery']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Delivery tasks")
        task_capabilities.append(TaskType.TYPE_DELIVERY)
    if fleet_config['task_capabilities']['clean']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Clean tasks")
        task_capabilities.append(TaskType.TYPE_CLEAN)

    # Callable for validating requests that this fleet can accommodate
    def _task_request_check(task_capabilities, msg: TaskProfile):
        if msg.description.task_type in task_capabilities:
            return True
        else:
            return False

    fleet_handle.accept_task_requests(
        partial(_task_request_check, task_capabilities))

    transforms = initialize_map_transform(config_yaml['map_transform'])

    def _updater_inserter(cmd_handle, update_handle):
        """Insert a RobotUpdateHandle."""
        cmd_handle.update_handle = update_handle

    # Initialize robot API for this fleet
    api = RobotAPI(
        fleet_config['fleet_manager']['prefix'],
        fleet_config['fleet_manager']['user'],
        fleet_config['fleet_manager']['password'],
    )

    # Initialize robots for this fleet

    missing_robots = config_yaml['robots']

    def _add_fleet_robots():
        robots = {}
        while len(missing_robots) > 0:
            time.sleep(0.2)
            for robot_name in list(missing_robots.keys()):
                node.get_logger().debug(f"Connecting to robot: {robot_name}")
                robot_position: LionsbotCoord = api.position(robot_name)

                if robot_position is None:
                    node.get_logger().debug(f"Unable to get robot {robot_name} position. Skipping {robot_name}")
                    continue

                robot_start_map = config_yaml['robots'][robot_name]['rmf_config']['start']['map_name']

                robot_position_rmf: RmfCoord = transforms[robot_start_map].robot_to_rmf_meters(robot_position)
                
                if robot_position_rmf.orientation_radians is not None:
                    node.get_logger().info(f"Initializing robot: {robot_name}")
                    robots_config = config_yaml['robots'][robot_name]
                    rmf_config = robots_config['rmf_config']
                    robot_config = robots_config['robot_config']
                    initial_waypoint = rmf_config['start']['waypoint']
                    initial_orientation = rmf_config['start']['orientation']

                    starts = []
                    time_now = adapter.now()

                    if (initial_waypoint is not None) and \
                            (initial_orientation is not None):
                        node.get_logger().info(
                            f"Using provided initial waypoint "
                            f"[{initial_waypoint}] "
                            f"and orientation [{initial_orientation:.2f}] to "
                            f"initialize starts for robot [{robot_name}]")
                        # Get the waypoint index for initial_waypoint
                        initial_waypoint_index = nav_graph.find_waypoint(
                            initial_waypoint).index
                        starts = [plan.Start(time_now,
                                             initial_waypoint_index,
                                             initial_orientation)]
                    else:
                        node.get_logger().info(
                            f"Running compute_plan_starts for robot: "
                            "{robot_name}")
                        starts = plan.compute_plan_starts(
                            nav_graph,
                            rmf_config['start']['map_name'],
                            [robot_position_rmf.x, robot_position_rmf.y, robot_position_rmf.orientation_radians],
                            time_now)

                    if starts is None or len(starts) == 0:
                        node.get_logger().error(
                            f"Unable to determine StartSet for {robot_name}")
                        continue

                    node.get_logger().info(f"Subscribing to robot: {robot_name}")
                    api.subscribe_to_robot(robot_name, time.time_ns() / 1000000)

                    if safe_nav_flag:
                        waypoints_info = {}
                        with open(nav_graph_path, 'r') as file:
                            data = yaml.safe_load(file)
                            data = data['levels'][rmf_config['start']['map_name']]['vertices']
                            for waypoint in data:
                                waypoints_info[waypoint[2]['name']] = [waypoint[0], waypoint[1]]

                    api.robot_current_building[robot_name] = rmf_config['start']['building_name']
                    api.robot_current_map[robot_name] = rmf_config['start']['map_name']

                    node.get_logger().info(f"Changing to start map of robot: {robot_name}")
                    response = api.change_map(robot_name=robot_name, map_name=rmf_config['start']['map_name'])
                    if response is False:
                        node.get_logger().info(f"Changing map failed of robot: {robot_name}")
                        continue

                    node.get_logger().info(f"Localizing robot: {robot_name}")
                    dock_home_position_arr = copy.copy(robot_config['dock_home_position'])
                    dock_home_position = LionsbotCoord(
                        x=dock_home_position_arr[0], 
                        y=dock_home_position_arr[1], 
                        orientation_radians=dock_home_position_arr[2])
                    while True:
                        if api.get_robot_status(robot_name=robot_name)['localized']:
                            break
                        else:
                            is_localized = api.localize(dock_home_position, robot_name=robot_name)

                            time.sleep(0.5)

                            if is_localized:
                                continue
                            else:
                                node.get_logger().info(
                                    f"Localizing robot failed: {robot_name} at {dock_home_position}")

                    robot = RobotCommandHandle(
                        name=robot_name,
                        fleet_name=fleet_name,
                        config=robot_config,
                        node=node,
                        graph=nav_graph,
                        vehicle_traits=vehicle_traits,
                        transforms=transforms,
                        map_name=rmf_config['start']['map_name'],
                        start=starts[0],
                        position=robot_position_rmf,
                        charger_waypoint=rmf_config['charger']['waypoint'],
                        update_frequency=rmf_config.get(
                            'robot_state_update_frequency', 1),
                        adapter=adapter,
                        api=api,
                        use_safe_nav=safe_nav_flag,
                        waypoints_info=waypoints_info)

                    if robot.initialized:
                        robots[robot_name] = robot
                        # Add robot to fleet
                        fleet_handle.add_robot(robot,
                                               robot_name,
                                               profile,
                                               [starts[0]],
                                               partial(_updater_inserter,
                                                       robot))
                        node.get_logger().info(
                            f"Successfully added new robot: {robot_name}")

                    else:
                        node.get_logger().error(
                            f"Failed to initialize robot: {robot_name}")

                    del missing_robots[robot_name]

                else:
                    pass
                    node.get_logger().debug(
                        f"{robot_name} not found, trying again...")
        return

    add_robots = threading.Thread(target=_add_fleet_robots, args=())
    add_robots.start()
    return adapter

def initialize_map_transform(map_transform_config: dict) -> dict[str, MapTransform]:
    transforms: dict[str, MapTransform] = {}
    
    for map_ref, transform_method in map_transform_config.items():
        # Get level transform values is provided
        level_transform = transform_method.get('level_transform', {})
        level_tx_pixels = level_transform.get('tx_pixels', 0)
        level_ty_pixels = level_transform.get('ty_pixels', 0)
        floorplan_scale_meters_per_pixel = level_transform.get('scale', 1)

        if 'transform_values' in transform_method:
            values = transform_method['transform_values']
            transforms[map_ref] = MapTransform.compute(
                tx_meters=values['tx_meters'], 
                ty_meters=values['ty_meters'], 
                rotation_in_radians=math.radians(values['rotation_degrees']), 
                lbmap_translation_scale_factor=values['scale'], 
                level_tx_pixels=level_tx_pixels,
                level_ty_pixels=level_ty_pixels,
                floorplan_scale_meters_per_pixel=floorplan_scale_meters_per_pixel)
        elif 'reference_coordinates' in transform_method:
            reference_coordinates = transform_method['reference_coordinates']
            transforms[map_ref] = MapTransform.estimate(
                rmf_coords=reference_coordinates['rmf'], 
                robot_coords=reference_coordinates['robot'],
                level_tx_pixels=level_tx_pixels, 
                level_ty_pixels=level_ty_pixels,
                floorplan_scale_meters_per_pixel=floorplan_scale_meters_per_pixel)
    
    return transforms

# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("-s", "--server_uri", type=str, required=False, default="",
                        help="URI of the api server to transmit state and task information.")
    parser.add_argument("--use_sim_time", action="store_true",
                        help='Use sim time, default: false')
    parser.add_argument("-d", "--dock_summary_file", type=str, required=True,
                        help="Path to the dock_summary.yaml file")
    parser.add_argument("--use_safe_nav", action="store_true", required=False, default=False,
                        help="Boolean flag for navigating via robot location points")
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet adapter...")

    config_path = args.config_file
    dock_summary_path = args.dock_summary_file
    nav_graph_path = args.nav_graph

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    with open(dock_summary_path, "r") as f:
        dock_summary_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = config_yaml['rmf_fleet']['name']
    node = rclpy.node.Node(f'{fleet_name}_command_handle')

    # Enable sim time for testing offline
    if args.use_sim_time:
        print('USING SIM')
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])

    if args.server_uri == "":
        server_uri = None
    else:
        server_uri = args.server_uri

    if args.use_safe_nav:
        print('USING SAFE_NAV : Navigating via Robot Localization Points')
        print('Please ensure that Robot Map has waypoints same as RMF Map.')
        safe_nav_flag = True
    else:
        safe_nav_flag = False
    
    print(f"[safe_nav_flag] set to {safe_nav_flag}")

    adapter = initialize_fleet(
        config_yaml,
        nav_graph_path,
        node,
        args.use_sim_time,
        server_uri,
        dock_summary_yaml,
        safe_nav_flag)

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
