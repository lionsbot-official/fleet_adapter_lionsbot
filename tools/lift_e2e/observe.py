"""Send the robot to another level and check each step of the lift trip.

Usage: observe.py <waypoint>=<level> [<waypoint>=<level> ...]

For each pair it sends one patrol task to the waypoint through the task
dispatcher (task_api_requests), as RMF Web does, and waits for the robot to be
idle on that level. It prints each change on the lift topics and of the robot's
place. The exit code is 0 when every check passes.
"""

import json
import sys
import time

import rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rmf_fleet_msgs.msg import FleetState
from rmf_lift_msgs.msg import LiftRequest
from rmf_lift_msgs.msg import LiftState
from rmf_task_msgs.msg import ApiRequest
from rmf_task_msgs.msg import ApiResponse

TRANSIENT = QoSProfile(
    depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
printed = {}
robot = {}
responses = []
# (topic, session_id, request_type, destination_floor) of each lift request
# and ('lift_states', session_id, door_state, current_floor) of each state.
seen = set()


def say(key, text):
    if printed.get(key) != text:
        printed[key] = text
        print(f'[{time.time():.3f}] {key}: {text}', flush=True)


def on_request(topic):
    def callback(m):
        # The lift supervisor repeats an end-session request for a free lift.
        if m.session_id == '':
            return
        seen.add((topic, m.session_id, m.request_type, m.destination_floor))
        say(topic, f'lift={m.lift_name} session_id={m.session_id!r} '
            f'request_type={m.request_type} '
            f'destination_floor={m.destination_floor!r} '
            f'door_state={m.door_state}')
    return callback


def on_state(m):
    if m.motion_state == LiftState.MOTION_STOPPED:
        seen.add(('lift_states', m.session_id, m.door_state, m.current_floor))
    say('lift_states', f'lift={m.lift_name} '
        f'current_floor={m.current_floor!r} '
        f'destination_floor={m.destination_floor!r} '
        f'door_state={m.door_state} motion_state={m.motion_state} '
        f'current_mode={m.current_mode} session_id={m.session_id!r}')


def on_fleet(m):
    for r in m.robots:
        robot.update(
            name=f'{m.name}/{r.name}', task_id=r.task_id,
            level=r.location.level_name)
        say('fleet_states', f'robot={r.name} task_id={r.task_id!r} '
            f'level={r.location.level_name!r} '
            f'x={r.location.x:.1f} y={r.location.y:.1f}')


def spin_until(condition, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if condition():
            return True
    return False


def check(text, passed):
    print(f'{"PASS" if passed else "FAIL"}: {text}', flush=True)
    return passed


rclpy.init()
node = rclpy.create_node('lift_e2e_observer')
node.create_subscription(
    LiftRequest, 'adapter_lift_requests',
    on_request('adapter_lift_requests'), TRANSIENT)
node.create_subscription(
    LiftRequest, 'lift_requests', on_request('lift_requests'), TRANSIENT)
node.create_subscription(LiftState, 'lift_states', on_state, 10)
node.create_subscription(FleetState, 'fleet_states', on_fleet, 10)
node.create_subscription(
    ApiResponse, 'task_api_responses',
    lambda m: responses.append(m.json_msg), TRANSIENT)
task_pub = node.create_publisher(ApiRequest, 'task_api_requests', TRANSIENT)

if not spin_until(lambda: robot, 60):
    sys.exit('FAIL: no robot in fleet_states')
session = robot['name']
passed = True
for number, argument in enumerate(sys.argv[1:]):
    place, level = argument.split('=')
    start_level = robot['level']
    seen.clear()
    request = {
        'type': 'dispatch_task_request',
        'request': {
            'category': 'patrol',
            'description': {'places': [place], 'rounds': 1},
            'unix_millis_earliest_start_time': 0}}
    task_pub.publish(ApiRequest(
        request_id=f'lift_e2e_{number}', json_msg=json.dumps(request)))
    spin_until(lambda: len(responses) > number, 20)
    spin_until(lambda: robot['task_id'] != '', 60)
    spin_until(lambda: robot['task_id'] == '' and (
        'lift_states', '', LiftState.DOOR_CLOSED, level) in seen, 240)
    print(f'Task to {place}:')
    for floor in (start_level, level):
        request = (session, LiftRequest.REQUEST_AGV_MODE, floor)
        passed &= check(
            f'the fleet adapter requested the lift to {floor} in session '
            f'{session}', ('adapter_lift_requests', *request) in seen)
        passed &= check(
            'the lift supervisor forwarded it',
            ('lift_requests', *request) in seen)
        passed &= check(
            f'the lift stopped at {floor} with open doors in that session',
            ('lift_states', session, LiftState.DOOR_OPEN, floor) in seen)
    passed &= check(
        'the fleet adapter ended the lift session', (
            'lift_requests', session, LiftRequest.REQUEST_END_SESSION,
            level) in seen)
    passed &= check(
        'the lift is free', printed['lift_states'].endswith("session_id=''"))
    passed &= check(
        f'the robot is on {level} with no task',
        robot['level'] == level and robot['task_id'] == '')
sys.exit(0 if passed else 1)
