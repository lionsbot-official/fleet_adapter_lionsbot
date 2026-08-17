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

"""Unit tests for robot UUID validation and WebSocket resubscriptions."""

import json
import threading

import pytest

from .. import RobotClientAPI as robot_client_api
from ..RobotClientAPI import RobotAPI
from ..utils.robot_config import robot_uuids_from_config


class FakeWebSocket:
    """Record subscribe messages and optionally fail selected robots."""

    def __init__(self, failed_robots=None):
        self.failed_robots = failed_robots or set()
        self.messages = []

    def send(self, message):
        self.messages.append(message)
        robot_id = json.loads(message)['robot_id']
        if robot_id in self.failed_robots:
            raise RuntimeError('simulated send failure')


def make_api(subscribed_robots):
    """Build a RobotAPI instance without starting its network threads."""
    api = object.__new__(RobotAPI)
    api._lock = threading.Lock()
    api._subscribed_robots = set(subscribed_robots)
    api._pending_resubscriptions = set()
    api._last_status_ws_message_at = None
    api._last_pose_ws_message_at = None
    api.robot_status_ws_connection = FakeWebSocket()
    api.robot_pose_ws_connection = FakeWebSocket()
    return api


def test_stale_telemetry_websocket_is_marked_disconnected(monkeypatch):
    api = make_api(set())
    api._status_ws_ready = threading.Event()
    api._pose_ws_ready = threading.Event()
    api._status_ws_ready.set()
    api._pose_ws_ready.set()
    api._last_status_ws_message_at = 100.0
    api._last_pose_ws_message_at = 110.0
    monkeypatch.setattr(robot_client_api.time, 'monotonic', lambda: 116.0)
    monkeypatch.setattr(RobotAPI, 'TELEMETRY_STALE_SECONDS', 10.0)

    assert api._clear_stale_telemetry_websockets() == ('robotstatus',)
    assert not api._status_ws_ready.is_set()
    assert api._pose_ws_ready.is_set()


def test_robot_uuids_from_config_rejects_missing_or_empty_uuid():
    config = {
        'robots': {
            'missing': {'robot_config': {}},
            'empty': {'robot_config': {'uuid': '  '}},
            'null': {'robot_config': {'uuid': None}},
        },
    }

    with pytest.raises(ValueError, match='missing, empty, null'):
        robot_uuids_from_config(config)


def test_robot_uuids_from_config_includes_every_robot():
    config = {
        'robots': {
            'one': {'robot_config': {'uuid': 'uuid-one'}},
            'two': {'robot_config': {'uuid': 'uuid-two'}},
        },
    }

    assert robot_uuids_from_config(config) == {
        'one': 'uuid-one',
        'two': 'uuid-two',
    }


def test_websocket_restart_marks_subscriptions_pending():
    api = make_api({'one', 'two'})

    api._mark_all_for_resubscription()

    assert not api._subscribed_robots
    assert api._pending_resubscriptions == {'one', 'two'}


def test_failed_resubscription_is_retained_for_next_retry(monkeypatch):
    api = make_api({'one', 'two'})
    api.robot_status_ws_connection.failed_robots = {'two'}
    monkeypatch.setattr(robot_client_api.time, 'sleep', lambda _: None)

    api._mark_all_for_resubscription()

    assert not api._resubscribe_pending_robots()
    assert api._subscribed_robots == {'one'}
    assert api._pending_resubscriptions == {'two'}

    api.robot_status_ws_connection.failed_robots.clear()

    assert api._resubscribe_pending_robots()
    assert api._subscribed_robots == {'one', 'two'}
    assert not api._pending_resubscriptions


def test_supervisor_retries_pending_subscriptions_without_reconnecting(
        monkeypatch):
    api = make_api(set())
    api._pending_resubscriptions = {'two'}
    api.robot_status_ws_connection.failed_robots = {'two'}
    api._status_ws_ready = threading.Event()
    api._pose_ws_ready = threading.Event()
    api._status_ws_ready.set()
    api._pose_ws_ready.set()
    api.token_expiry = None
    monkeypatch.setattr(robot_client_api.time, 'sleep', lambda _: None)

    class Wakeup:
        def __init__(self):
            self.waits = []

        def wait(self, timeout):
            self.waits.append(timeout)
            if len(self.waits) == 1:
                api.robot_status_ws_connection.failed_robots.clear()
            else:
                raise StopIteration

        def clear(self):
            pass

    wakeup = Wakeup()
    api._connection_wakeup = wakeup
    api._next_token_refresh_time = lambda: robot_client_api.time.monotonic() + 60
    api.request_token = lambda: pytest.fail('unexpected token request')
    api.check_connection = lambda **_: pytest.fail('unexpected reconnect')
    api._disconnect_websockets = lambda: pytest.fail('unexpected disconnect')

    with pytest.raises(StopIteration):
        api._connection_supervisor_loop()

    assert wakeup.waits[0] == api.RECONNECT_INITIAL_BACKOFF_SECONDS
    assert len(wakeup.waits) == 2
    assert api._subscribed_robots == {'two'}
    assert not api._pending_resubscriptions
