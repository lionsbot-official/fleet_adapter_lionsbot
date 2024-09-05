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

from rclpy.duration import Duration

import rmf_adapter as adpt
import rmf_adapter.plan as plan

from .enums.enums import Topic
from .enums.enums import ResponseCode
from .enums.enums import ActiveMissionType
from .enums.enums import RobotStatus
from .enums.enums import NavigationStatus

from .models.DockProcessContent import DockProcessContent

import numpy as np

import threading
import math
import json
import enum
import time

from datetime import timedelta
from typing import List, Dict
from .utils.MapTransform import MapTransform
from .utils.Coordinate import RmfCoord
from .utils.Coordinate import LionsbotCoord
from .RobotClientAPI import RobotAPI 

from rmf_task_msgs.msg import ApiRequest, ApiResponse

# States for RobotCommandHandle's state machine used when guiding robot along
# a new path
class RobotState(enum.IntEnum):
    IDLE = 0
    WAITING = 1
    MOVING = 2
    PAUSED = 3

# Custom wrapper for Plan::Waypoint 
class PlanWaypoint():
    def __init__(self, path_index, waypoint:plan.Waypoint):
        # index of Plan::Waypoint in the waypoints in follow_new_path
        self.path_index = path_index
        self.position = RmfCoord(
            x=waypoint.position[0], 
            y=waypoint.position[1], 
            orientation_radians=waypoint.position[2])
        self.time = waypoint.time
        self.graph_index = waypoint.graph_index
        self.approach_lanes = waypoint.approach_lanes

class RobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 fleet_name,
                 config,
                 node,
                 graph,
                 vehicle_traits,
                 transforms: Dict[str, MapTransform],
                 map_name,
                 start,
                 position: RmfCoord,
                 charger_waypoint,
                 update_frequency,
                 adapter,
                 api:RobotAPI,
                 use_safe_nav,
                 waypoints_info):
        adpt.RobotCommandHandle.__init__(self)
        self.name = name
        self.fleet_name = fleet_name
        self.config = config
        self.node = node
        self.graph = graph
        self.vehicle_traits = vehicle_traits
        self.transforms = transforms
        self.map_name = map_name
        # Get the index of the charger waypoint
        waypoint = self.graph.find_waypoint(charger_waypoint)
        assert waypoint, f"Charger waypoint {charger_waypoint} \
          does not exist in the navigation graph"
        self.charger_waypoint_index = waypoint.index
        self.charger_is_set = False
        self.update_frequency = update_frequency
        self.update_handle: adpt.RobotUpdateHandle = None
        self.battery_soc = 1.0
        self.api: RobotAPI = api
        self.position: RmfCoord = position 
        self.initialized = False
        self.state = RobotState.IDLE
        self.dock_name = ""
        self.adapter = adapter

        self.requested_waypoints: List[plan.waypoint] = []
        self.remaining_waypoints: List[PlanWaypoint] = []
        self.path_finished_callback = None
        self.next_arrival_estimator = None
        self.path_index = 0
        self.docking_finished_callback = None

        self.use_safe_nav = use_safe_nav
        self.waypoint_network = waypoints_info

        # RMF location trackers
        self.last_known_lane_index = None
        self.last_known_waypoint_index = None
        # if robot is waiting at a waypoint. This is a Graph::Waypoint index
        self.on_waypoint = None
        # if robot is travelling on a lane. This is a Graph::Lane index
        self.on_lane = None
        self.target_waypoint: PlanWaypoint = None 
        # The graph index of the waypoint the robot is currently docking into
        self.dock_waypoint_index = None

        # Threading variables
        self._lock = threading.Lock()
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._dock_thread = None
        self._quit_dock_event = threading.Event()

        self.node.get_logger().info(f"The robot is starting at: {self.position}")

        # Update tracking variables
        if start.lane is not None:  # If the robot is on a lane
            self.last_known_lane_index = start.lane
            self.on_lane = start.lane
            self.last_known_waypoint_index = start.waypoint
        else:  # Otherwise, the robot is on a waypoint
            self.last_known_waypoint_index = start.waypoint
            self.on_waypoint = start.waypoint

        self.state_update_timer = self.node.create_timer(
            1.0 / self.update_frequency,
            self.update)

        # Subscribe to task api requests topic
        self.robot_state_subscription = self.node.create_subscription(
            ApiRequest, Topic.LB_TASKS_REQUEST.value, self.lb_task_api_requests_payload_handler, 10)
        
        self.initialized = True

    def lb_task_api_requests_payload_handler(self, msg: ApiRequest):
        payload = json.loads(msg.json_msg)
        self.node.get_logger().info(f'Received request payload: {payload}')

        request_type = payload.get('type', None)
        if request_type is not None:
            with self._lock:
                if request_type == 'pause_task_request' and \
                        payload['robot_name'] == self.name and payload['fleet'] == self.fleet_name and \
                        self.state == RobotState.MOVING:
                    self.node.get_logger().info(f'Requesting robot to pause: {self.name}')
                    self.state = RobotState.PAUSED
                elif request_type == 'continue_task_request' and \
                        payload['robot_name'] == self.name and payload['fleet'] == self.fleet_name and \
                        self.state == RobotState.PAUSED:
                    self.node.get_logger().info(f'Requesting robot to resume: {self.name}')
                    if self.api.resume_robot(robot_name=self.name):
                        self.node.get_logger().info(f'Resuming robot: {self.name}')
                        self.state = RobotState.MOVING
                    else:
                        self.node.get_logger().info(f'Resuming robot failed: {self.name}')

    def sleep_for(self, seconds):
        goal_time = \
            self.node.get_clock().now() + Duration(nanoseconds=1e9 * seconds)
        while (self.node.get_clock().now() <= goal_time):
            time.sleep(0.001)

    def clear(self):
        with self._lock:
            self.requested_waypoints = []
            self.remaining_waypoints = []
            self.path_finished_callback = None
            self.next_arrival_estimator = None
            self.docking_finished_callback = None
            self.state = RobotState.IDLE

    def stop(self):
        # Stop the robot. Tracking variables should remain unchanged.
        while True:
            self.node.get_logger().info("Requesting robot to stop...")
            mission_status = self.api.get_mission_status(robot_name=self.name)
            self.node.get_logger().info(f'STOPPING ROBOT: {mission_status}')
            if self.api.stop(self.name):
                break
            self.sleep_for(0.1)
        if self._follow_path_thread is not None:
            self._quit_path_event.set()
            if self._follow_path_thread.is_alive():
                self._follow_path_thread.join()
            self._follow_path_thread = None
            self.clear()
        elif self._dock_thread is not None:
            self._quit_dock_event.set()
            if self._dock_thread.is_alive():
                self._dock_thread.join()
            self._dock_thread = None
            self.clear()

    def follow_new_path(
            self,
            waypoints,
            next_arrival_estimator,
            path_finished_callback):

        self.stop()
        self._quit_path_event.clear()
        self._quit_dock_event.clear()

        self.node.get_logger().info("Received new path to follow...")

        if self.use_safe_nav:
            name_waypoints = []
            # Discard first waypoint due to non-match with existing waypoints.
            waypoints = waypoints[1:]

            for x in range(len(waypoints)):
                is_waypoint = False
                for waypoint_name in self.waypoint_network:
                    if (waypoints[x].position[0] == self.waypoint_network[waypoint_name][0] and
                        waypoints[x].position[1] == self.waypoint_network[waypoint_name][1]):
                        is_waypoint = True
                        self.node.get_logger().info(f'{x}. waypoint received = {waypoint_name}')
                        name_waypoints.append(waypoint_name)
                if not is_waypoint:
                    self.node.get_logger().info(f'{x}. waypoint received = {waypoints[x].position}')

        self.remaining_waypoints = self.get_remaining_waypoints(waypoints)
        assert next_arrival_estimator is not None
        assert path_finished_callback is not None
        self.next_arrival_estimator = next_arrival_estimator
        self.path_finished_callback = path_finished_callback

        def _follow_path():
            target_pose_rmf: RmfCoord = None
            target_pose_robot: LionsbotCoord = None
            while (
                    self.remaining_waypoints or
                    self.state == RobotState.MOVING or
                    self.state == RobotState.WAITING or 
                    self.state == RobotState.PAUSED):
                # Check if we need to abort
                if self._quit_path_event.is_set():
                    self.node.get_logger().info("Aborting previously followed "
                                                "path")
                    return

                # State machine
                if self.state == RobotState.IDLE:
                    self.node.get_logger().info('robot state = IDLING')
                    # Assign the next waypoint
                    self.target_waypoint = self.remaining_waypoints[0]
                    self.path_index = self.remaining_waypoints[0].path_index
                    # Move robot to next waypoint in the current map
                    current_map_name = self.map_name
                    next_pose_map_name = None
                    if self.target_waypoint.graph_index is not None:
                        next_pose_map_name = self.graph.get_waypoint(self.target_waypoint.graph_index).map_name

                    # Check if map has changed, localize if true
                    if next_pose_map_name is not None:
                        map_changed = current_map_name != next_pose_map_name
                        if map_changed:
                            current_pose_robot = self.api.position(self.name)

                            current_pose_rmf:RmfCoord = self.transforms[current_map_name].robot_to_rmf_meters(
                                robot_coord=current_pose_robot) 
        
                            next_pose_robot:LionsbotCoord = self.transforms[next_pose_map_name].rmf_meters_to_robot(
                                rmf_coord=current_pose_rmf) 
                            
                            self.api.change_map(robot_name=self.name, map_name=next_pose_map_name)

                            while True:
                                if self.api.localize(position=next_pose_robot, robot_name=self.name):
                                    current_map_name = next_pose_map_name
                                    self.map_name = current_map_name
                                    break

                    self.node.get_logger().info(f'Attempting to navigate in map: {current_map_name}')

                    if self.use_safe_nav:
                        # Move robot to next waypoint
                        for waypoint_name in self.waypoint_network:
                            if (self.target_waypoint.position.x == self.waypoint_network[waypoint_name][0] and
                                self.target_waypoint.position.y == self.waypoint_network[waypoint_name][1]):
                                target_pose_robot = self.api.get_waypoint_position(self.name, self.map_name, waypoint_name)
                    else:
                        target_pose_rmf = self.target_waypoint.position
                        target_pose_robot: LionsbotCoord = self.transforms[current_map_name].rmf_meters_to_robot(
                            rmf_coord=target_pose_rmf
                        )

                    robot_status = self.api.get_robot_status(robot_name=self.name)
                    while robot_status is None:
                        robot_status = self.api.get_robot_status(robot_name=self.name)

                    # Make sure the robot has completed navigation before making next command
                    while True:
                        if self._quit_path_event.is_set():
                            self.node.get_logger().info('Aborting navigation')
                            return

                        mission_status = self.api.get_mission_status(robot_name=self.name)
                        while mission_status is None:
                            mission_status = self.api.get_mission_status(robot_name=self.name)

                        if mission_status['missionStatus']['activeMissionType'] == ActiveMissionType.IDLE.value:
                            break

                    if self.target_waypoint.graph_index == self.on_waypoint \
                        or self.remaining_waypoints[-1].graph_index == self.on_waypoint:
                        self.remaining_waypoints = self.remaining_waypoints[1:]
                        with self._lock:
                                self.node.get_logger().info(
                                    f"Robot [{self.name}] already at its target waypoint / end waypoint")
                                self.state = RobotState.WAITING
                                self.last_known_waypoint_index = self.on_waypoint

                                duration = self.api.navigation_remaining_duration(self.name)

                                if self.path_index is not None:
                                    self.next_arrival_estimator(
                                        self.path_index, timedelta(seconds=duration))
                        continue
                    
                    if robot_status['docked'] or robot_status['status'] == RobotStatus.DOCKED.value:
                        while True:
                            if self._quit_dock_event.is_set():
                                self.node.get_logger().info('Aborting undock')
                                return

                            self.node.get_logger().info(f'Requesting robot {self.name} to undock')
                            if self.api.undock_robot(robot_name=self.name, time_stamp=time.time_ns() / 1000000):
                                self.node.get_logger().info('Robot has received undock command...')
                                break

                            self.sleep_for(10)

                        while True:
                            if self._quit_dock_event.is_set():
                                self.node.get_logger().info('Aborting undock')
                                return

                            self.node.get_logger().info("Robot is undocking...")
                            if self.api.undocking_completed(robot_name=self.name):
                                break

                            self.sleep_for(6.0)

                        self.node.get_logger().info("Undocking completed")

                    self.node.get_logger().info(
                        f'NAVIGATING TO {target_pose_rmf} = {target_pose_robot}')
                    response = self.api.navigate(self.name, target_pose_robot, current_map_name)

                    if response:
                        self.remaining_waypoints = self.remaining_waypoints[1:]
                        self.state = RobotState.MOVING
                    else:
                        self.node.get_logger().info(
                            f"Robot {self.name} failed to navigate to {target_pose_rmf} = {target_pose_robot}. "
                            f"Retrying...")
                        self.sleep_for(0.1)

                elif self.state == RobotState.WAITING:
                    self.sleep_for(0.1)
                    time_now = self.adapter.now()
                    with self._lock:
                        if self.target_waypoint is not None:
                            waypoint_wait_time = self.target_waypoint.time
                            if (waypoint_wait_time < time_now):
                                self.state = RobotState.IDLE
                            elif self.path_index is not None:
                                self.node.get_logger().info(f"Waiting for {(waypoint_wait_time - time_now).seconds}s")
                                self.next_arrival_estimator(self.path_index, timedelta(seconds=0.0))

                elif self.state == RobotState.MOVING:
                    self.node.get_logger().info(f'robot state = MOVING to target: {self.target_waypoint.position}')
                    # Check if we have reached the target
                    with self._lock:
                        navigate_status = self.api.navigation_completed(self.name)
                        if navigate_status == NavigationStatus.NAVIGATION_SUCCESS:
                            self.node.get_logger().info(f"Robot [{self.name}] has reached its target waypoint")
                            self.state = RobotState.WAITING
                            if (self.target_waypoint.graph_index is not None):
                                self.on_waypoint = self.target_waypoint.graph_index
                                self.last_known_waypoint_index = self.on_waypoint
                            else:
                                self.on_waypoint = None  # still on a lane
                        else:
                            if navigate_status == NavigationStatus.NAVIGATION_ERROR or \
                                navigate_status == NavigationStatus.EMPTY:
                                self.node.get_logger().info(f"Robot [{self.name}] has failed to reach its target waypoint")
                                self.state = RobotState.IDLE
                                self.remaining_waypoints.insert(0, self.target_waypoint)  
                            
                            lane = self.get_current_lane()
                            if lane is not None:
                                self.on_waypoint = None
                                self.on_lane = lane
                            else:
                                # The robot may either be on the previous
                                # waypoint or the target one
                                if self.target_waypoint.graph_index is not None and \
                                    math.dist((self.position.x, self.position.y), (target_pose_rmf.x, target_pose_rmf.y)) < 0.5:
                                    self.on_waypoint = self.target_waypoint.graph_index
                                elif self.last_known_waypoint_index is not None and \
                                    math.dist((self.position.x, self.position.y), self.graph.get_waypoint(
                                        self.last_known_waypoint_index).location) < 0.5:
                                    self.on_waypoint = self.last_known_waypoint_index
                                else:
                                    self.on_lane = None  # update_off_grid()
                                    self.on_waypoint = None

                            self.sleep_for(0.5)

                        duration = self.api.navigation_remaining_duration(self.name)

                        if self.path_index is not None:
                            self.next_arrival_estimator(
                                self.path_index, timedelta(seconds=duration))

                elif self.state == RobotState.PAUSED:
                    self.node.get_logger().info(f'Requesting robot to pause: {self.name}')
                    if self.api.pause_robot(robot_name=self.name):
                        self.node.get_logger().info(f'Pausing robot: {self.name}')
                    else:
                        self.node.get_logger().info(f'Pausing robot failed: {self.name}')
                        self.state = RobotState.MOVING

                    while not self.state == RobotState.MOVING:
                        # Prevent replanning
                        if self.path_index is not None:
                            self.next_arrival_estimator(self.path_index, timedelta(seconds=duration))
                        self.node.get_logger().info(f'Robot paused navigation: {self.name}')
                        self.sleep_for(0.6)
                    self.node.get_logger().info(f'Robot resumed navigation: {self.name}')

            self.node.get_logger().info('robot FINISHED follow-path')
            self.path_finished_callback()
            self.node.get_logger().info(f"Robot {self.name} has successfully navigated along requested path.")

        self._follow_path_thread = threading.Thread(target=_follow_path)
        self._follow_path_thread.start()

    def dock(
            self,
            dock_name,
            docking_finished_callback):
        ''' Docking is very specific to each application. Hence, the user will
            need to customize this function accordingly. In this example, we
            assume the dock_name is the same as the name of the waypoints that
            the robot is trying to dock into. We then call api.start_process()
            to initiate the robot specific process. This could be to start a
            cleaning process or load/unload a cart for delivery.
        '''

        self._quit_dock_event.clear()
        if self._dock_thread is not None:
            self._dock_thread.join()

        self.dock_name = dock_name
        assert docking_finished_callback is not None
        self.docking_finished_callback = docking_finished_callback

        # Get the waypoint that the robot is trying to dock into
        dock_waypoint = self.graph.find_waypoint(self.dock_name)
        assert (dock_waypoint)
        self.dock_waypoint_index = dock_waypoint.index

        def _dock():
            dock_point = self.graph.find_waypoint(dock_name)
            if dock_point.charger:
                dock_position_rmf = RmfCoord(x=dock_point.location[0], y=dock_point.location[1])

                dock_position_robot = self.transforms[self.map_name].rmf_meters_to_robot(dock_position_rmf)

                dock_pose_content = DockProcessContent(x=dock_position_robot.x, y=dock_position_robot.y,
                                                       orientation_radians=0.0,
                                                       dock_name=dock_name)

                while True:
                    if self._quit_dock_event.is_set():
                        self.node.get_logger().info('Aborting docking')
                        return

                    self.node.get_logger().info(
                        f"Requesting robot {self.name} to dock at {dock_name}")
                    docking = self.api.dock_robot(robot_name=self.name,
                                                  time_stamp=time.time_ns() / 1000000,
                                                  content=dock_pose_content)
                    if docking:
                        break

                    self.sleep_for(1)

                with self._lock:
                    self.on_waypoint = None
                    self.on_lane = None
                self.sleep_for(2.0)

                while True:
                    if self._quit_dock_event.is_set():
                        self.node.get_logger().info('Aborting docking')
                        return

                    self.node.get_logger().info("Robot is docking...")
                    if self.api.docking_completed(robot_name=self.name):
                        self.node.get_logger().info("Robot completed docking...")
                        break

                    robot_status = self.api.get_robot_status(robot_name=self.name)
                    mission_status = self.api.get_mission_status(robot_name=self.name)
                    if robot_status['status'] == RobotStatus.RESTING.value and ResponseCode.UNABLE_TO_PLAN_PATH in mission_status['alertIds']:
                        self.node.get_logger().info(
                            f"Re-requesting robot {self.name} to dock at {dock_name}")
                        self.api.dock_robot(robot_name=self.name,
                                            time_stamp=time.time_ns() / 1000000,
                                            content=dock_pose_content)

                    self.sleep_for(5.0)

                with self._lock:
                    self.on_waypoint = self.dock_waypoint_index
                    self.dock_waypoint_index = None
                    self.docking_finished_callback()
                    self.node.get_logger().info("Docking completed")
            else:
                while True:
                    if self._quit_dock_event.is_set():
                        self.node.get_logger().info('Aborting docking')
                        return

                    self.node.get_logger().info(
                        f"Requesting robot {self.name} to clean at {self.dock_name}")
                    if self.api.start_process(self.name, self.dock_name, self.map_name):
                        break

                with self._lock:
                    self.state = RobotState.MOVING
                    self.on_waypoint = None
                    self.on_lane = None
                time.sleep(1.0)

                while True:
                    if self._quit_dock_event.is_set():
                        self.node.get_logger().info('Aborting docking')
                        return

                    self.node.get_logger().info("Robot is cleaning...")
                    if self.api.process_completed(robot_name=self.name):
                        break
                    elif self.state == RobotState.PAUSED:
                        self.node.get_logger().info(f'Requesting robot to pause: {self.name}')
                        if self.api.pause_robot(robot_name=self.name):
                            self.node.get_logger().info(f'Pausing robot: {self.name}')
                        else:
                            self.node.get_logger().info(f'Pausing robot failed: {self.name}')
                            self.state = RobotState.MOVING

                        while self.api.robot_cleaning_paused(robot_name=self.name):
                            self.node.get_logger().info("Robot is paused while cleaning...")
                            self.sleep_for(0.6)
                            
                        self.node.get_logger().info(f'Robot resumed cleaning: {self.name}')
                        with self._lock:
                            self.state = RobotState.MOVING

                    self.sleep_for(2.0)

                with self._lock:
                    self.state = RobotState.IDLE
                    self.on_waypoint = self.dock_waypoint_index
                    self.dock_waypoint_index = None
                    self.docking_finished_callback()
                    self.node.get_logger().info("Cleaning completed")

        self._dock_thread = threading.Thread(target=_dock)
        self._dock_thread.start()

    def get_position(self) -> RmfCoord:
        ''' This helper function returns the live position of the robot in the
        RMF coordinate frame'''
        position = self.api.position(self.name)

        if position is not None:
            return self.transforms[self.map_name].robot_to_rmf_meters(robot_coord=position)
        else:
            self.node.get_logger().error(
                "Unable to retrieve live position from robot.")
            return self.position

    def get_battery_soc(self):
        battery_soc = self.api.battery_soc(self.name)
        if battery_soc is not None:
            return battery_soc
        else:
            self.node.get_logger().error(
                "Unable to retrieve battery data from robot.")
            return self.battery_soc

    def update(self):
        self.position = self.get_position()
        self.battery_soc = self.get_battery_soc()
        self.node.get_logger().info(f'Current robot position: {self.position}')
        if self.update_handle is not None:
            self.update_state()

    def update_state(self):
        self.update_handle.update_battery_soc(self.battery_soc)
        if not self.charger_is_set:
            if ("max_delay" in self.config.keys()):
                max_delay = self.config["max_delay"]
                self.node.get_logger().info(
                    f"Setting max delay to {max_delay}s")
                self.update_handle.set_maximum_delay(max_delay)
            if (self.charger_waypoint_index < self.graph.num_waypoints):
                self.update_handle.set_charger_waypoint(
                    self.charger_waypoint_index)
            else:
                self.node.get_logger().warn(
                    "Invalid waypoint supplied for charger. "
                    "Using default nearest charger in the map")
            self.charger_is_set = True
        # Update position
        with self._lock:
            position_coord = [self.position.x, self.position.y, self.position.orientation_radians]
            if (self.on_waypoint is not None):  # if robot is on a waypoint
                self.update_handle.update_current_waypoint(
                    self.on_waypoint, self.position.orientation_radians)
            elif (self.on_lane is not None):  # if robot is on a lane
                # We only keep track of the forward lane of the robot.
                # However, when calling this update it is recommended to also
                # pass in the reverse lane so that the planner does not assume
                # the robot can only head forwards. This would be helpful when
                # the robot is still rotating on a waypoint.
                forward_lane = self.graph.get_lane(self.on_lane)
                entry_index = forward_lane.entry.waypoint_index
                exit_index = forward_lane.exit.waypoint_index
                reverse_lane = self.graph.lane_from(exit_index, entry_index)
                lane_indices = [self.on_lane]
                if reverse_lane is not None:  # Unidirectional graph
                    lane_indices.append(reverse_lane.index)
                self.update_handle.update_current_lanes(
                    position_coord, lane_indices)
            elif (self.dock_waypoint_index is not None):
                self.update_handle.update_off_grid_position(
                    position_coord, self.dock_waypoint_index)
            # if robot is merging into a waypoint
            elif (self.target_waypoint is not None and
                  self.target_waypoint.graph_index is not None):
                self.update_handle.update_off_grid_position(
                    position_coord, self.target_waypoint.graph_index)
            else:  # if robot is lost
                self.update_handle.update_lost_position(
                    self.map_name, position_coord)

    def get_current_lane(self):
        def projection(current_position,
                       target_position,
                       lane_entry,
                       lane_exit):
            px, py, _ = current_position
            p = np.array([px, py])
            t = np.array(target_position)
            entry = np.array(lane_entry)
            exit = np.array(lane_exit)
            return np.dot(p - t, exit - entry)

        if self.target_waypoint is None:
            return None
        approach_lanes = self.target_waypoint.approach_lanes
        # Spin on the spot
        if approach_lanes is None or len(approach_lanes) == 0:
            return None
        # Determine which lane the robot is currently on
        for lane_index in approach_lanes:
            lane = self.graph.get_lane(lane_index)
            p0 = self.graph.get_waypoint(lane.entry.waypoint_index).location
            p1 = self.graph.get_waypoint(lane.exit.waypoint_index).location
            p = [self.position.x, self.position.y, self.position.orientation_radians]
            before_lane = projection(p, p0, p0, p1) < 0.0
            after_lane = projection(p, p1, p0, p1) >= 0.0
            if not before_lane and not after_lane:  # The robot is on this lane
                return lane_index
        return None


    # ------------------------------------------------------------------------------
    # Helper functions
    # ------------------------------------------------------------------------------
    def get_remaining_waypoints(self, waypoints: List[plan.Waypoint]) -> List[PlanWaypoint]:
        '''
        The function returns a list where each element is a PlanWaypoint. This function
        may be modified if waypoints in a path need to be filtered.
        '''
        assert (len(waypoints) > 0)
        remaining_waypoints = []

        for i in range(len(waypoints)):
            remaining_waypoints.append(PlanWaypoint(i, waypoints[i]))
        return remaining_waypoints
    