"""The real fleet adapter with a fake robot in place of the LionsBot cloud.

Everything of the adapter runs as in production (fleet_adapter.main,
LionsbotRobot, CommandHandle, LiftGuard, the lift_states subscription). Only
RobotAPI is replaced: the fake robot contacts nothing, jumps to each target
after a short time, and answers the lift commands.

Usage: the arguments of the fleet_adapter executable.
"""

import sys
import threading
import time

from fleet_adapter.enums.enums import CommandResult
from fleet_adapter.enums.enums import LiftCommand
from fleet_adapter.enums.enums import LiftMoveState
from fleet_adapter.enums.enums import LiftMoveStatus
from fleet_adapter.enums.enums import OperationStatus
import fleet_adapter.fleet_adapter as adapter_main
from fleet_adapter.utils.Coordinate import RmfCoord
import yaml

MOVE_SECONDS = 1.0
LIFT_MOVE_SECONDS = 2.0
START_WAYPOINT = 'start_L1'


def argument(flag):
    return sys.argv[sys.argv.index(flag) + 1]


CONFIG = yaml.safe_load(open(argument('-c')))
NAV_GRAPH = yaml.safe_load(open(argument('-n')))
TRANSFORMS = adapter_main.initialize_map_transform(CONFIG['map_transform'])
RMF_CONFIG = next(iter(CONFIG['robots'].values()))['rmf_config']
LEVEL_OF_ROBOT_MAP = {
    RMF_CONFIG['start']['robot_map_name']: RMF_CONFIG['start']['rmf_level'],
    **{m['name']: level for level, m in RMF_CONFIG['robot_maps'].items()}}


def say(text):
    print(f'FAKE ROBOT [{time.time():.3f}]: {text}', flush=True)


def waypoint_on_robot_map(name, heading):
    for level, graph in NAV_GRAPH['levels'].items():
        for x, y, properties in graph['vertices']:
            if properties.get('name') == name:
                return TRANSFORMS[level].rmf_meters_to_robot(
                    RmfCoord(x=x, y=y, orientation_radians=heading))
    raise KeyError(name)


class FakeRobotAPI:
    def __init__(self, prefix, user, password, robot_uuids=None):
        self.robot_current_map = {}
        self._lock = threading.Lock()
        self._pose = waypoint_on_robot_map(START_WAYPOINT, 0.0)
        self._robot_map = None
        self._move = None
        self._lift_move = None

    # Connection and telemetry
    def is_connected(self):
        return True

    def is_subscribed(self, robot_encoding_id):
        return True

    def subscribe_to_robot(self, robot_encoding_id, time_stamp):
        return True

    def set_xy_goal_tolerance(self, robot_encoding_id, tolerance):
        pass

    def position(self, robot_encoding_id):
        with self._lock:
            return self._pose

    def get_robot_status(self, robot_encoding_id):
        return {'status': 'Resting', 'localized': True, 'batterySoc': 100,
                'charging': False}

    def battery_soc(self, robot_encoding_id):
        return 1.0

    # Maps and pose
    def change_map(self, robot_encoding_id, map_name, map_level=None):
        if self._robot_map != map_name:
            say(f'change_map to robot map [{map_name}] level [{map_level}]')
        self._robot_map = map_name
        self.robot_current_map[robot_encoding_id] = map_name
        return True

    def localize(self, pose, robot_encoding_id):
        say(f'localize at {pose} on robot map [{self._robot_map}]')
        with self._lock:
            self._pose = pose
        return True

    # Point-to-point moves
    def send_navigation_command(self, robot_encoding_id, pose, map_name):
        say(f'navigation (P2P) to {pose} on robot map [{map_name}]')
        self._move = (time.monotonic() + MOVE_SECONDS, pose)
        return True

    def navigate_robot(self, robot_encoding_id, pose, map_name):
        arrival, target = self._move
        if time.monotonic() < arrival:
            return OperationStatus(CommandResult.RUNNING, MOVE_SECONDS)
        with self._lock:
            self._pose = target
        return OperationStatus(CommandResult.SUCCESS)

    def stop(self, robot_encoding_id, timeout_seconds=None):
        say('stop')
        return True

    # Lift commands
    def lift_command_map(self, robot_encoding_id, map_name, map_level):
        return {'name': map_name, 'level': map_level}

    def send_lift_command(self, robot_encoding_id, command, lift, robot_map):
        level = LEVEL_OF_ROBOT_MAP[robot_map['name']]
        place = 'cabin' if command == LiftCommand.ENTER else 'approach'
        target = waypoint_on_robot_map(
            f'{lift}_{level}_{place}', self._pose.orientation_radians)
        say(f'{command.value} lift [{lift}] on robot map '
            f'[{robot_map["name"]}], moving to the {place} point {target}')
        self._lift_move = (
            time.monotonic() + LIFT_MOVE_SECONDS, target, command)
        return 'fake-request'

    def lift_move_status(self, robot_encoding_id):
        if self._lift_move is None:
            return None
        arrival, target, command = self._lift_move
        if time.monotonic() < arrival:
            return LiftMoveStatus(
                LiftMoveState.RUNNING, command, 'fake-request')
        with self._lock:
            if self._pose is not target:
                say(f'{command.end_status}: success')
            self._pose = target
        return LiftMoveStatus(
            LiftMoveState.SUCCEEDED, command, 'fake-request')

    def abort_lift_move(self, robot_encoding_id):
        say('ABORT of the lift move')
        return True

    def clear_lift_move(self, robot_encoding_id):
        self._lift_move = None


adapter_main.RobotAPI = FakeRobotAPI
sys.exit(adapter_main.main(sys.argv))
