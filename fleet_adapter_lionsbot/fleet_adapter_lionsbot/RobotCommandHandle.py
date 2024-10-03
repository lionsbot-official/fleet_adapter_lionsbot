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

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

import rmf_adapter as adpt
import rmf_adapter.plan as plan
import rmf_adapter.schedule as schedule

from std_msgs.msg import String
from rmf_fleet_msgs.msg import DockSummary
from rmf_task_msgs.msg import ApiRequest, ApiResponse


import numpy as np

import json
import threading
import math
import copy
import enum
import time

from typing import List, Tuple

from datetime import timedelta


# States for RobotCommandHandle's state machine used when guiding robot along
# a new path
class RobotState(enum.IntEnum):
    IDLE = 0
    WAITING = 1
    MOVING = 2


class RobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 fleet_name,
                 config,
                 node,
                 graph,
                 vehicle_traits,
                 transforms,
                 map_name,
                 start,
                 position,
                 charger_waypoint,
                 update_frequency,
                 adapter,
                 api,
                 custom_cmd_node=None):
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
        self.update_handle = None  # RobotUpdateHandle
        self.battery_soc = 1.0
        self.api = api
        self.position = position  # (x,y,theta) in RMF coordinates (meters, radians)
        self.initialized = False
        self.state = RobotState.IDLE
        self.dock_name = ""
        self.adapter = adapter

        self.requested_waypoints = []  # RMF Plan waypoints
        self.remaining_waypoints = []
        self.path_finished_callback = None
        self.next_arrival_estimator = None
        self.path_index = 0
        self.docking_finished_callback = None

        # Perform Action
        self.action_execution = None
        self.stubborness = None
        self.action_category = None
        self.latest_clean_percentage = None
        self.current_process = None
        self.clean_percentage_threshold = 0 # Default value
        self.is_paused = False
        self.interruption = None

        if 'clean_percentage_threshold' in self.config:
            self.clean_percentage_threshold =\
                self.config['clean_percentage_threshold']

        # RMF location tracker
        self.last_known_lane_index = None
        self.last_known_waypoint_index = None
        # if robot is waiting at a waypoint. This is a Graph::Waypoint index
        self.on_waypoint = None
        # if robot is travelling on a lane. This is a Graph::Lane index
        self.on_lane = None
        self.target_waypoint = None  # this is a Plan::Waypoint
        # The graph index of the waypoint the robot is currently docking into
        self.dock_waypoint_index = None

        # Threading variables
        self._lock = threading.Lock()
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._dock_thread = None
        self._quit_dock_event = threading.Event()
        self.current_level = self.api.current_map_info()['level']

        self.node.get_logger().info(
            f"The robot is starting at: [{self.position[0]:.2f}, "
            f"{self.position[1]:.2f}, {self.position[2]:.2f}]")

        # Update tracking variables
        if start.lane is not None:  # If the robot is on a lane
            self.last_known_lane_index = start.lane
            self.on_lane = start.lane
            self.last_known_waypoint_index = start.waypoint
        else:  # Otherwise, the robot is on a waypoint
            self.last_known_waypoint_index = start.waypoint
            self.on_waypoint = start.waypoint

        if custom_cmd_node:
            self.custom_cmd_node = custom_cmd_node
        else:
            self.custom_cmd_node = self.node

        self.position_update_timer = self.node.create_timer(
            1.0 / self.update_frequency,
            self.update_position)

        # Second timer doesnt run when creating two timers with a single node
        # running on a multithreaded executor
        self.status_update_timer = self.custom_cmd_node.create_timer(
            4.0 / self.update_frequency,
            self.update_status)

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.estop_pub = self.custom_cmd_node.create_publisher(
            String, '/estop', 10)

        self.clean_info_pub = self.custom_cmd_node.create_publisher(
            String, '/clean_info', 10)

        self.custom_api_request_subscription = self.custom_cmd_node.create_subscription(
            ApiRequest, '/custom_api_requests', self.api_request_cb, 10)

        self.custom_api_response_pub = self.custom_cmd_node.create_publisher(
            ApiResponse, '/custom_api_responses', 10)

        self.finish_manual_task_sub = self.custom_cmd_node.create_subscription(
            String, '/finish_manual_task', self.finish_manual_task_cb, transient_qos)

        self.initialized = True

    def init_handler(self, update_handle):
        self.update_handle = update_handle
        self.update_handle.set_action_executor(self._action_executor)
        self.participant = self.update_handle.get_unstable_participant()

    def api_request_cb(self, msg: ApiRequest):
        payload = json.loads(msg.json_msg)
        self.node.get_logger().info(f'Received request payload: {payload}')

        request_type = payload.get('type', None)
        if request_type is not None:
            if not payload['robot_name'] == self.name or not payload['fleet'] == self.fleet_name:
                return
            if request_type == 'pause_task_request':
                self.node.get_logger().info(f'Requesting robot to pause: {self.name}')
                if self.action_execution is not None:
                    self.node.get_logger().info(f"Pausing robot [{self.name}] action [{self.action_category}]")
                    self.api.pause()
                elif self._follow_path_thread is not None:
                    self.interruption = self.update_handle.interrupt(
                        ["Interruption"],
                        lambda: self.node.get_logger().info(f"Interrupting Robot [{self.name}]")
                    )
                self.is_paused = True
            elif request_type == 'resume_task_request':
                self.node.get_logger().info(f'Requesting robot to resume: {self.name}')
                if self.action_execution is not None:
                    self.api.resume()
                    self.node.get_logger().info(f'Resuming robot [{self.name}] action [{self.action_category}]')
                elif self.interruption is not None:
                    self.interruption.resume(['Resume'])
                self.is_paused = False
            elif request_type == 'manual_task' and \
                    payload['robot_name'] == self.name and payload['fleet'] == self.fleet_name:
                self.node.get_logger().info(f'Requesting robot to stop manual mode: {self.name}')
                self.manual_mode = False
            response_msg = ApiResponse()
            response = {}
            response['success'] = True
            response_msg.request_id = msg.request_id
            response_msg.json_msg = json.dumps(response)
            self.custom_api_response_pub.publish(response_msg)

    def sleep_for(self, seconds):
        goal_time =\
          self.node.get_clock().now() + Duration(nanoseconds=1e9*seconds)
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
        if self._follow_path_thread is not None:
            self._quit_path_event.set()
            if self._follow_path_thread.is_alive():
                self._follow_path_thread.join()
            self._follow_path_thread = None
            self.clear()
        while True:
            self.node.get_logger().info("Requesting robot to stop...")
            if self.api.stop():
                break
            self.sleep_for(0.1)

    def follow_new_path(
        self,
        waypoints,
        next_arrival_estimator,
        path_finished_callback):

        self.stop()
        self._quit_path_event.clear()

        self.node.get_logger().info("Received new path to follow...")

        self.remaining_waypoints = self.get_remaining_waypoints(waypoints)
        assert next_arrival_estimator is not None
        assert path_finished_callback is not None
        self.next_arrival_estimator = next_arrival_estimator
        self.path_finished_callback = path_finished_callback

        def _follow_path():
            target_pose = []
            while (
                self.remaining_waypoints or
                self.state == RobotState.MOVING or
                self.state == RobotState.WAITING):
                # Check if we need to abort
                if self._quit_path_event.is_set():
                    self.node.get_logger().info("Aborting previously followed "
                                                "path")
                    return
                # State machine
                if self.state == RobotState.IDLE:
                    # Assign the next waypoint
                    self.target_waypoint = self.remaining_waypoints[0][1]
                    self.path_index = self.remaining_waypoints[0][0]
                    # Move robot to next waypoint
                    target_pose = self.target_waypoint.position
                    [x, y, yaw] = self.transforms[self.current_level]["tf"].to_robot_map(
                        target_pose[:3])
                    theta = math.degrees(yaw)
                    # ------------------------ #
                    # IMPLEMENT YOUR CODE HERE #
                    # Ensure x, y, theta are in units that api.navigate() #
                    if self.api.is_docked():
                        if self.target_waypoint.graph_index is not None:
                            wp = self.graph.get_waypoint(self.target_waypoint.graph_index)
                            if wp.charger:
                                self.remaining_waypoints = self.remaining_waypoints[1:]
                                with self._lock:
                                    self.node.get_logger().info(
                                        f"Robot [{self.name}] Skipping Charger Wayoint!"
                                        f"waypoint")
                                    self.state = RobotState.WAITING
                                    if (self.target_waypoint.graph_index is not None):
                                        self.on_waypoint = \
                                            self.target_waypoint.graph_index
                                        self.last_known_waypoint_index = \
                                            self.on_waypoint
                                    else:
                                        self.on_waypoint = None  # still on a lane
                                        self.next_arrival_estimator(
                                                self.path_index, timedelta(seconds=0.0))
                            else:
                                while self.api.is_docked():
                                    self.node.get_logger().info(
                                        "Robot is docked attemtping to undock"
                                    )
                                    success = self.api.undock()
                                    time.sleep(1.0)
                # ------------------------ #

                    else:
                        response = self.api.navigate([x, -y, yaw])
                        self.node.get_logger().info(f"{self.name} Success sending navigate command {response}")
                        time.sleep(1.0)
                        navigate_success = self.api.is_navigating()
                        if navigate_success:
                            self.remaining_waypoints = self.remaining_waypoints[1:]
                            self.state = RobotState.MOVING

                        else:
                            self.node.get_logger().info(
                                f"Robot {self.name} failed to navigate to "
                                f"[{x:.0f}, {y:.0f}, {theta:.0f}] coordinates. "
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
                            else:
                                if self.path_index is not None:
                                    self.node.get_logger().info(
                                        f"Waiting for "
                                        f"{(waypoint_wait_time - time_now).seconds}s")
                                    self.next_arrival_estimator(
                                        self.path_index, timedelta(seconds=0.0))

                elif self.state == RobotState.MOVING:
                    self.sleep_for(0.5)
                    # Check if we have reached the target
                    with self._lock:
                        if (self.api.navigation_completed()):
                            self.node.get_logger().info(
                                f"Robot [{self.name}] has reached its target "
                                f"waypoint")
                            self.state = RobotState.WAITING
                            if (self.target_waypoint.graph_index is not None):
                                self.on_waypoint = \
                                    self.target_waypoint.graph_index
                                self.last_known_waypoint_index = \
                                    self.on_waypoint
                            else:
                                self.on_waypoint = None  # still on a lane
                        elif not self.api.is_navigating():
                            self.node.get_logger().info(
                                f"Robot {self.name} is supposed to be moving but has no navigation mission!")
                            if not self.api.is_in_critical():
                                self.node.get_logger().info(
                                    f"Robot {self.name} is not estopped sending it a navigation command again!")
                                # send it to navigate to the same target waypoint again
                                [x, y, yaw] = self.transforms[self.current_level]["tf"].to_robot_map(
                                    target_pose[:3])
                                response = self.api.navigate([x, -y, yaw])
                        elif self.api.is_app_started(10,0.5):
                                self.node.get_logger().info(
                                    f"Robot {self.name} is somehow stuck in APP_STARTED stopping robot!")
                                self.api.stop()
                        else:
                            # Update the lane the robot is on
                            lane = self.get_current_lane()
                            if lane is not None:
                                self.on_waypoint = None
                                self.on_lane = lane
                            else:
                                # The robot may either be on the previous
                                # waypoint or the target one
                                if self.target_waypoint.graph_index is not \
                                    None and self.dist(self.position, target_pose) < 0.5:
                                    self.on_waypoint = self.target_waypoint.graph_index
                                elif self.last_known_waypoint_index is not \
                                    None and self.dist(
                                    self.position, self.graph.get_waypoint(
                                      self.last_known_waypoint_index).location) < 0.5:
                                    self.on_waypoint = self.last_known_waypoint_index
                                else:
                                    self.on_lane = None  # update_off_grid()
                                    self.on_waypoint = None
                        # ------------------------ #
                        # IMPLEMENT YOUR CODE HERE #
                        # If your robot does not have an API to report the
                        # remaining travel duration, replace the API call
                        # below with an estimation
                        # ------------------------ #
                        duration = self.api.navigation_remaining_duration()
                        if self.path_index is not None:
                            self.next_arrival_estimator(
                                self.path_index, timedelta(seconds=duration))
            self.path_finished_callback()
            self.node.get_logger().info(
                f"Robot {self.name} has successfully navigated along "
                f"requested path.")

        self._follow_path_thread = threading.Thread(
            target=_follow_path)
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
        assert(dock_waypoint)
        self.dock_waypoint_index = dock_waypoint.index

        def _dock():
            # Request the robot to start the relevant process
            self.node.get_logger().info(
                f"Requesting robot {self.name} to dock at {self.dock_name}")
            self.api.dock(dock_name)

            with self._lock:
                self.on_waypoint = None
                self.on_lane = None
            self.sleep_for(0.1)
            # ------------------------ #
            # IMPLEMENT YOUR CODE HERE #
            # With whatever logic you need for docking #
            # ------------------------ #
            while (not self.api.is_docked()):
                # Check if we need to abort
                if self._quit_dock_event.is_set():
                    self.node.get_logger().info("Aborting docking")
                    return
                self.node.get_logger().info(f"Robot {self.name} is docking...")
                self.sleep_for(0.2)

            with self._lock:
                self.on_waypoint = self.dock_waypoint_index
                self.dock_waypoint_index = None
                self.docking_finished_callback()
                self.node.get_logger().info(f"[{self.name}] Docking completed")

        self._dock_thread = threading.Thread(target=_dock)
        self._dock_thread.start()

    def get_position(self):
        ''' This helper function returns the live position of the robot in the
        RMF coordinate frame'''
        position = self.api.position()
        if position is not None:
            x, y, theta = self.transforms[self.current_level]['tf'].to_rmf_map(
                [position[0], -position[1], position[2]])
            # ------------------------ #
            # IMPLEMENT YOUR CODE HERE #
            # Ensure x, y are in meters and theta in radians #
            # ------------------------ #
            # Wrap theta between [-pi, pi]. Else arrival estimate will
            # assume robot has to do full rotations and delay the schedule
            if theta > np.pi:
                theta = theta - (2 * np.pi)
            if theta < -np.pi:
                theta = (2 * np.pi) + theta
            return [x, y, theta]
        else:
            self.node.get_logger().error(
                "Unable to retrieve position from robot.")
            return self.position

    def get_battery_soc(self):
        battery_soc = self.api.battery_soc()
        if battery_soc is not None:
            return battery_soc
        else:
            self.node.get_logger().error(
                "Unable to retrieve battery data from robot.")
            return self.battery_soc

    def update_status(self):
        if self.api.is_charging():
            self.update_handle.override_status("charging")
        if not self.api.is_localized():
            self.node.get_logger().warn(f"Robot {self.name} is not localized")
            self.update_handle.override_status("uninitialized")
        if self.api.is_estopped():
            self.node.get_logger().info(f"Robot {self.name} is E-stopped")
            self.notify_estop()

    def update_position(self):
        self.position = self.get_position()
        self.battery_soc = self.get_battery_soc()
        current_map_info = self.api.current_map_info()
        if current_map_info is not None:
            current_level = current_map_info.get('level', None)
            if current_level is not None:
                self.current_level = current_level
        if self.update_handle is not None:
            self.update_state()
        if (self.action_execution):
            self.check_perform_action()

    # Get start sets, for update_position(startsets)
    def get_start_sets(self):
        return plan.compute_plan_starts(
            self.graph,
            self.current_level,
            self.position,
            self.adapter.now(),
            max_merge_waypoint_distance = 0.5,
            max_merge_lane_distance = self.config.get('max_merge_lane_distance', 15.0))

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
            if (self.on_waypoint is not None):  # if robot is on a waypoint
                self.update_handle.update_current_waypoint(
                    self.on_waypoint, self.position[2])
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
                    self.position, lane_indices)
            elif (self.dock_waypoint_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.dock_waypoint_index)
            # if robot is merging into a waypoint
            elif (self.target_waypoint is not None and
                self.target_waypoint.graph_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.target_waypoint.graph_index)
            else:  # if robot is lost
                self.update_handle.update_lost_position(
                    self.map_name, self.position)

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
            p = self.position
            before_lane = projection(p, p0, p0, p1) < 0.0
            after_lane = projection(p, p1, p0, p1) >= 0.0
            if not before_lane and not after_lane:  # The robot is on this lane
                return lane_index
        return None

    def dist(self, A, B):
        ''' Euclidian distance between A(x,y) and B(x,y)'''
        assert(len(A) > 1)
        assert(len(B) > 1)
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

    def get_remaining_waypoints(self, waypoints: list):
        '''
        The function returns a list where each element is a tuple of the index
        of the waypoint and the waypoint present in waypoints. This function
        may be modified if waypoints in a path need to be filtered.
        '''
        assert(len(waypoints) > 0)
        remaining_waypoints = []

        for i in range(len(waypoints)):
            remaining_waypoints.append((i, waypoints[i]))
        return remaining_waypoints

    def notify_estop(self):
        msg = String()
        msg.data = self.name
        self.estop_pub.publish(msg)

    # For custom tasks
    def _action_executor(self,
                         category: str,
                         description: dict,
                         execution:
                         adpt.robot_update_handle.ActionExecution):
        with self._lock:
            # Check task category
            assert(category in ["clean", "manual"])

            self.action_category = category
            if (category == "clean"):
                # TODO(KW): Use JSON schema
                # Validation instead
                if not description["clean_task_name"]:
                    return False
                # TODO(KW): Implement a certain number of retries
                if self.api.start_process(description["clean_task_name"]):
                    # might take a while before mission status changes so
                    # we wait for a while just to be safe
                    time.sleep(0.5)
                    self.latest_clean_percentage = 0
                    self.check_task_completion =\
                        lambda : self.api.process_completed()
                    self.state = RobotState.MOVING
                    self.current_process = description['clean_task_name']
                    self.set_cleaning_trajectory(self.current_process)

                # If starting clean was not successful return
                else:
                    self.node.get_logger().error(
                        f"Failed to initiate cleaning action for robot [{self.name}]")
                    execution.error(f"Failed to initiate cleaning action for robot {self.name}")
                    execution.finished()
                    return

            elif (category == "manual"):
                self.manual_mode = True
                self.check_task_completion = lambda : not self.manual_mode
                self.current_process = "manual"
                self.state = RobotState.MOVING

            # Start Perform Action
            self.node.get_logger().warn(f"Robot [{self.name}] starts [{category}] action")
            self.start_action_time = self.adapter.now()
            self.on_waypoint = None
            self.on_lane = None
            self.action_execution = execution
            self.stubborness = self.update_handle.unstable_be_stubborn()
            self.node.get_logger().warn(f"Robot [{self.name}] starts [{category}] action")
            # TODO(KW): Determine an appropriate value to set the nominal
            # velocity during cleaning task.
            # self.vehicle_traits.linear.nominal_velocity = xxx

    def check_perform_action(self):
        self.node.get_logger().info(f"Executing perform action [{self.action_category}]")
        action_ok = self.action_execution.okay()
        if self.check_task_completion() or not action_ok:
            if action_ok:
                if self.action_category == 'clean':
                    if self.latest_clean_percentage  < self.clean_percentage_threshold:
                        self.handle_clean_failed()
                        return
                self.node.get_logger().info(
                    f"action [{self.action_category}] is completed")
                starts = self.get_start_sets()
                if starts is not None:
                    self.update_handle.update_position(starts)
                self.action_execution.finished()

            else:
                self.node.get_logger().warn(
                    f"action [{self.action_category}] is killed/canceled")
            self.stubborness.release()
            self.stubborness = None
            self.action_execution = None
            self.start_action_time = None
            self.latest_clean_percentage = None
            self.check_task_completion = None
            self.current_process = None
            self.current_clean_path = None
            return
        assert(self.participant)
        assert(self.start_action_time)

        if self.action_category == "clean":
            # Check mission status and update clean percentage.
            if self.api.is_cleaning():

                # TODO (KW): Perhaps we can start setting the clean trajectory the first time
                # this evluates to True so that it can better match the robot's position.
                print(f"[{self.name}] Starting to clean!")

                # NOTE: The schema for mission status from websockets is different compared
                # to the one that you get from the restful APIs.
                progress = self.api.get_clean_progress(self.current_process)
                # To prevent the percentage getting stuck at 100 when
                # progress attribute not yet updated.
                if self.latest_clean_percentage < progress:
                    self.latest_clean_percentage = progress
                    self.node.get_logger().info(\
                        f"Update cleaning progress to {self.latest_clean_percentage}")

                # Get remaining clean path and set the trajectory
                remaining_clean_path = self.get_remaining_clean_path(
                    self.latest_clean_percentage, self.current_clean_percentages, self.current_clean_path)

                trajectory = schedule.make_trajectory(
                    self.vehicle_traits,
                    self.adapter.now(),
                    remaining_clean_path)
                route = schedule.Route(self.map_name, trajectory)
                self.participant.set_itinerary([route])

        # TODO(KW): Use a more accurate estimate
        total_action_time = timedelta(hours=1.0)
        remaining_time = total_action_time - (self.adapter.now() - self.start_action_time)
        print(f"Still performing action, Estimated remaining time: [{remaining_time}]")
        self.action_execution.update_remaining_time(remaining_time)

        # starts = self.get_start_sets()
        # if starts is not None:
        #     self.update_handle.update_position(starts)
        # else:
        #     self.node.get_logger().error(f"Cant get startset during perform action")
        #     self.update_handle.update_off_grid_position(
        #         self.position, self.target_waypoint.graph_index)


    def set_cleaning_trajectory(self, process):
        robot_positions = self.api.get_path_from_zone(process)
        assert(len(robot_positions) % 2 == 0)
        rmf_positions = []
        for i in range(0, len(robot_positions), 2):
            robot_pose = [robot_positions[i], robot_positions[i+1], 0]
            rmf_pose = self.transforms[self.current_level]['tf'].to_rmf_map(
                [robot_pose[0], -robot_pose[1], robot_pose[2]])
            rmf_positions.append(rmf_pose)

        self.current_clean_path = rmf_positions
        current_total_clean_distance = sum([self.dist(rmf_positions[i], rmf_positions[i+1]) for i in range(0, (len(rmf_positions) - 1), 2)])
        initial_percentage = 0

        self.current_clean_percentages = []
        initial_percentage = 0.0
        for i in range(0, (len(rmf_positions) -1)):
            initial_percentage += (self.dist(rmf_positions[i], rmf_positions[i+1])/current_total_clean_distance)*100
            self.current_clean_percentages.append(initial_percentage)

        trajectory = schedule.make_trajectory(
            self.vehicle_traits,
            self.adapter.now(),
            rmf_positions)
        route = schedule.Route(self.map_name, trajectory)
        self.participant.set_itinerary([route])

    def get_clean_path_index(self, percentage: float, clean_path_percentages: List[float]) -> int:
        if percentage <= 0.0:
            return 0
        if percentage <= clean_path_percentages[0]:
            return 1
        for i in range(0, len(clean_path_percentages) - 1):
            if percentage > clean_path_percentages[i] and percentage <= clean_path_percentages[i+1]:
                return i + 2

    def get_remaining_clean_path(self, percentage: float, clean_path_percentages: List[float], clean_path: List[Tuple[int, int, int]]) -> List[Tuple[int, int, int]]:
        if percentage is None:
            percentage = 0.0
        clean_path_index = self.get_clean_path_index(percentage, clean_path_percentages)
        # Robot has not start clean path
        return [self.position, *clean_path[clean_path_index:]]

    def handle_clean_failed(self):
        print(f"Only cleaned up to {self.latest_clean_percentage}!")
        clean_info_msg = String()
        clean_info_msg.data = \
            f"Robot [{self.name}] only cleaned up to [{self.latest_clean_percentage}]."
        self.clean_info_pub.publish(clean_info_msg)
        self.latest_clean_percentage = 0
        process = self.current_process
        if self.api.start_process(process):
            print(f"RE-DOING CLEAN TASK {process}")
            self.set_cleaning_trajectory(process)
            return
        print("FAILED TO REDO CLEAN TASK! RETRYING")

    def finish_manual_task_cb(self, msg: String):
        if msg.data != self.name:
            return
        self.custom_cmd_node.get_logger().info(
            f"Robot {self.name} is finishing manual task.")
        self.manual_mode = False