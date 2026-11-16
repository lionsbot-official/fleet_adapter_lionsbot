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

"""A stand-in RMF lift adapter for tests at a site with no real lift.

It moves no real lift. It answers rmf_lift_msgs/LiftRequest on lift_requests
and publishes rmf_lift_msgs/LiftState on lift_states for each lift of the
building map. Never run it where a real lift adapter runs.
"""

import sys
import time

# The values of rmf_lift_msgs/LiftRequest and rmf_lift_msgs/LiftState.
REQUEST_END_SESSION = 0
REQUEST_AGV_MODE = 1
DOOR_CLOSED = 0
DOOR_OPEN = 2
MOTION_STOPPED = 0
MOTION_UP = 1
MOTION_DOWN = 2
MODE_HUMAN = 1
MODE_AGV = 2


class MockLift:
    """The state of one lift car. It needs no ROS."""

    def __init__(self, name, floors, travel_seconds, clock=time.monotonic):
        self.name = name
        self.floors = list(floors)
        self.current_floor = self.destination_floor = self.floors[0]
        self.door_state = DOOR_CLOSED
        self.motion_state = MOTION_STOPPED
        self.current_mode = MODE_HUMAN
        self.session_id = ''
        self._travel_seconds = travel_seconds
        self._clock = clock
        self._arrival_time = 0.0
        self._requested_door_state = DOOR_CLOSED

    def request(self, session_id, request_type, destination_floor, door_state):
        """Apply one LiftRequest. Only the session that holds the car counts."""
        self.update()
        if self.session_id not in ('', session_id):
            return
        if request_type == REQUEST_END_SESSION:
            self.session_id = ''
            self.current_mode = MODE_HUMAN
            self._requested_door_state = DOOR_CLOSED
        elif destination_floor in self.floors:
            self.session_id = session_id
            self.current_mode = (
                MODE_AGV if request_type == REQUEST_AGV_MODE else MODE_HUMAN)
            self._requested_door_state = door_state
            # A car that moves takes a new destination after it stops. RMF
            # repeats its request.
            if self.motion_state == MOTION_STOPPED and \
                    destination_floor != self.current_floor:
                up = self.floors.index(destination_floor) > \
                    self.floors.index(self.current_floor)
                self.destination_floor = destination_floor
                self.motion_state = MOTION_UP if up else MOTION_DOWN
                self._arrival_time = self._clock() + self._travel_seconds
        self.update()

    def update(self):
        """Bring the state to the present time."""
        if self.motion_state != MOTION_STOPPED and \
                self._clock() >= self._arrival_time:
            self.current_floor = self.destination_floor
            self.motion_state = MOTION_STOPPED
        self.door_state = (
            self._requested_door_state
            if self.motion_state == MOTION_STOPPED else DOOR_CLOSED)


def main(argv=sys.argv):
    import rclpy
    from rclpy.qos import QoSDurabilityPolicy
    from rclpy.qos import QoSProfile
    from rmf_building_map_msgs.msg import BuildingMap
    from rmf_lift_msgs.msg import LiftRequest
    from rmf_lift_msgs.msg import LiftState

    rclpy.init(args=argv)
    node = rclpy.create_node('mock_lift_adapter')
    travel_seconds = node.declare_parameter('travel_seconds', 3.0).value
    lifts = {}
    logged = {}

    def on_map(building_map):
        for lift in building_map.lifts:
            lifts.setdefault(
                lift.name, MockLift(lift.name, lift.levels, travel_seconds))

    def on_request(request):
        lift = lifts.get(request.lift_name)
        if lift is not None:
            lift.request(
                request.session_id,
                request.request_type,
                request.destination_floor,
                request.door_state)

    def publish_states():
        for lift in lifts.values():
            lift.update()
            publisher.publish(LiftState(
                lift_time=node.get_clock().now().to_msg(),
                lift_name=lift.name,
                available_floors=lift.floors,
                current_floor=lift.current_floor,
                destination_floor=lift.destination_floor,
                door_state=lift.door_state,
                motion_state=lift.motion_state,
                available_modes=[MODE_HUMAN, MODE_AGV],
                current_mode=lift.current_mode,
                session_id=lift.session_id))
            text = (
                f'MOCK lift [{lift.name}]: floor [{lift.current_floor}] '
                f'destination [{lift.destination_floor}] '
                f'door_state={lift.door_state} '
                f'motion_state={lift.motion_state} '
                f'session [{lift.session_id}]')
            if logged.get(lift.name) != text:
                logged[lift.name] = text
                node.get_logger().info(text)

    publisher = node.create_publisher(LiftState, 'lift_states', 10)
    node.create_subscription(LiftRequest, 'lift_requests', on_request, 10)
    node.create_subscription(
        BuildingMap, 'map', on_map,
        QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    node.create_timer(1.0, publish_states)
    node.get_logger().warning(
        'This is a mock lift adapter. It moves no real lift.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
