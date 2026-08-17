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


'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''
import json
import math
import websocket
import requests
from typing import Dict

from .utils.Coordinate import LionsbotCoord
from .utils import constants

from .enums.enums import CommandResult
from .enums.enums import RobotRuntimeStatusText
from .enums.enums import OperationEndStatus
from .enums.enums import OperationStatus

from .models.NavigateContent import NavigateContent
from .models.CleanProcessContent import CleanProcessContent
from .models.DockProcessContent import DockProcessContent
import time
from datetime import datetime
from datetime import timezone

from .utils.logging import debug_logger
import threading

logger = debug_logger('RobotClientAPI')


class RobotAPI:
    STOP_TIMEOUT_SECONDS = 300
    REQUEST_TIMEOUT_SECONDS = 10.0
    FAILED_P2P_POSE_WAIT_SECONDS = 2.0
    TOKEN_REFRESH_INTERVAL_SECONDS = 6 * 60 * 60
    RECONNECT_INITIAL_BACKOFF_SECONDS = 1.0
    RECONNECT_MAX_BACKOFF_SECONDS = 60.0
    # Detect connections that stop receiving telemetry without invoking the
    # websocket-client error or close callbacks.
    TELEMETRY_STALE_SECONDS = 15.0
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(
        self,
        prefix: str,
        user: str,
        password: str,
        robot_uuids: Dict[str, str] | None = None,
    ):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.robot_uuids = robot_uuids or {}
        self.robot_names_by_uuid = {
            robot_uuid: robot_encoding_id
            for robot_encoding_id, robot_uuid in self.robot_uuids.items()
        }
        self.token = None
        self.token_expiry = None
        self.robot_status_ws_connection = None
        self.robot_pose_ws_connection = None

        self.robot_current_map = {}
        self.robot_status = {}
        self.robot_pose: Dict[str, LionsbotCoord] = {}
        self.robot_operation_end_status = {}
        self._operation_success_fallback_seen_at = {}
        self._pending_failed_navigation_end_status = {}
        self._pending_failed_navigation_end_status_since = {}

        self.robot_xy_goal_tolerances = {}
        self.operation_end_status_grace_seconds = 60

        self._lock = threading.Lock()
        self._token_lock = threading.Lock()
        self._subscribed_robots: set[str] = set()
        # Robots whose subscribe request failed during a reconnect must remain
        # candidates for the next reconnect attempt.
        self._pending_resubscriptions: set[str] = set()
        self._status_ws_ready = threading.Event()
        self._pose_ws_ready = threading.Event()
        self._last_status_ws_message_at: float | None = None
        self._last_pose_ws_message_at: float | None = None
        self._connection_wakeup = threading.Event()

        self.check_connection()
        threading.Thread(
            target=self._connection_supervisor_loop,
            name='lionsbot-connection-supervisor',
            daemon=True).start()

    def _robot_uuid(self, robot_encoding_id: str) -> str:
        return self.robot_uuids[robot_encoding_id]

    def set_xy_goal_tolerance(
            self,
            robot_encoding_id: str,
            xy_goal_tolerance: float):
        self.robot_xy_goal_tolerances[robot_encoding_id] = xy_goal_tolerance

    def _within_goal_tolerance_radius(
            self,
            robot_encoding_id: str,
            pose: LionsbotCoord,
            robot_pose: LionsbotCoord) -> bool:
        xy_goal_tolerance = self.robot_xy_goal_tolerances[robot_encoding_id]
        dx = pose.x - robot_pose.x
        dy = pose.y - robot_pose.y
        return math.sqrt(dx * dx + dy * dy) <= xy_goal_tolerance

    def _robot_state_key(self, robot_uuid: str) -> str:
        return self.robot_names_by_uuid.get(robot_uuid, robot_uuid)

    @staticmethod
    def _mask_token(token: str | None) -> str:
        if not token:
            return '<none>'
        if len(token) <= 16:
            return '<short_token>'
        return f'{token[:8]}...{token[-8:]}'

    @staticmethod
    def _redact_mapping(
            data: dict | None,
            secret_keys: tuple = ('authorization', 'password', 'token')) -> dict:
        if data is None:
            return {}

        redacted = {}
        for key, value in data.items():
            if key.lower() in secret_keys:
                redacted[key] = '***'
            elif isinstance(value, dict):
                redacted[key] = RobotAPI._redact_mapping(value, secret_keys)
            elif isinstance(value, list):
                redacted[key] = [
                    RobotAPI._redact_mapping(item, secret_keys)
                    if isinstance(item, dict) else item
                    for item in value
                ]
            else:
                redacted[key] = value

        return redacted

    @staticmethod
    def _json_for_log(data):
        if isinstance(data, (dict, list)):
            return json.dumps(data, ensure_ascii=False)
        return data

    @staticmethod
    def _int_status_value(value):
        try:
            return int(value)
        except (TypeError, ValueError):
            return value

    @staticmethod
    def _coordinate_as_int(value) -> int:
        return int(round(value))

    @staticmethod
    def _is_touchscreen_robot_status(robot_status: dict | None) -> bool:
        return robot_status is not None and \
            robot_status.get('operation_fb') == 'touchscreen_robot_status'

    @staticmethod
    def _robot_status_value(robot_status: dict | None) -> str | None:
        if not RobotAPI._is_touchscreen_robot_status(robot_status):
            return None
        return robot_status.get('status')

    @staticmethod
    def _is_docked_status(robot_status: dict | None) -> bool:
        return RobotAPI._robot_status_value(robot_status) == \
            RobotRuntimeStatusText.DOCKED.value

    @staticmethod
    def _is_charging_status(robot_status: dict | None) -> bool:
        return isinstance(robot_status, dict) and (
            robot_status.get('charging') is True or
            robot_status.get('status') ==
            RobotRuntimeStatusText.CHARGING.value)

    @staticmethod
    def _cached_touchscreen_robot_status(
            operation_fb: str,
            robot_status: dict,
            timestamp: int | None) -> dict:
        state = RobotAPI._int_status_value(robot_status.get('state'))
        response_codes = robot_status.get(
            'response_codes',
            robot_status.get('responseCodes', []))
        alert_ids = [
            code.get('code') if isinstance(code, dict) else code
            for code in response_codes
        ]
        battery_info = robot_status.get('battery_info', {})
        return {
            'operation_fb': operation_fb,
            'eta': (
                robot_status.get(
                    'time_to_complete',
                    # estimated time is often underestimated, so we double it to be safe
                    robot_status.get('timeToComplete', 0)) or 0) * 2,
            'alertIds': alert_ids,
            'progress': robot_status.get(
                'mission_progress',
                robot_status.get('missionProgress')),
            'localized': robot_status.get(
                'localized',
                robot_status.get('localised')),
            'batterySoc': robot_status.get(
                'battery_soc',
                battery_info.get('soc')),
            # This boolean is distinct from the vendor display status, which
            # may remain "Docked" while charging.
            'charging': robot_status.get('charging'),
            'state': state,
            'status': robot_status.get('status'),
            'timestamp': timestamp,
        }

    # Static Variables
    ROBOT_DOCKING_ACTIVE_STATUSES = {
        RobotRuntimeStatusText.DOCKING_MOVE.value,
        RobotRuntimeStatusText.DOCKING_IN_PROGRESS.value,
    }
    OPERATION_P2P = 'p2p'
    OPERATION_CLEANING = 'cleaning'
    OPERATION_DOCK = 'dock'

    # Websocket Functions
    def check_connection(self, refresh_token: bool = True) -> bool:
        if refresh_token and not self.request_token():
            return False
        if self.token is None or self.token_expiry is None or \
                self.token_expiry <= time.time():
            return False

        self._status_ws_ready.clear()
        self._pose_ws_ready.clear()
        threading.Thread(
            target=self.connect_to_robot_status_ws, daemon=True).start()
        threading.Thread(
            target=self.connect_to_robot_position_ws, daemon=True).start()

        deadline = time.monotonic() + RobotAPI.REQUEST_TIMEOUT_SECONDS
        for event, name in ((self._status_ws_ready, 'robotstatus'),
                            (self._pose_ws_ready, 'robotpose')):
            if not event.wait(max(0.0, deadline - time.monotonic())):
                logger.error('WebSocket [%s] did not open within %.1fs',
                             name, RobotAPI.REQUEST_TIMEOUT_SECONDS)
                return False
        return True

    def is_connected(self) -> bool:
        """Return whether both robot telemetry WebSockets are ready."""
        return (self._status_ws_ready.is_set() and
                self._pose_ws_ready.is_set())

    def _clear_stale_telemetry_websockets(self) -> tuple[str, ...]:
        """Mark ready telemetry WebSockets disconnected when their data is stale."""
        now = time.monotonic()
        with self._lock:
            sockets = (
                ('robotstatus', self._status_ws_ready,
                 self._last_status_ws_message_at),
                ('robotpose', self._pose_ws_ready,
                 self._last_pose_ws_message_at),
            )
            stale = tuple(
                (name, ready) for name, ready, last_message_at in sockets
                if ready.is_set() and last_message_at is not None and
                now - last_message_at > self.TELEMETRY_STALE_SECONDS)
            for _, ready in stale:
                ready.clear()
            return tuple(name for name, _ in stale)

    def _disconnect_websockets(self):
        for connection in (
                self.robot_status_ws_connection,
                self.robot_pose_ws_connection):
            if connection is not None:
                try:
                    connection.close()
                except Exception as error:
                    logger.warning('Error closing WebSocket during reconnect: %r',
                                   error)

    def _mark_all_for_resubscription(self):
        """Mark every current-session subscription for restoration."""
        with self._lock:
            self._pending_resubscriptions.update(self._subscribed_robots)
            self._subscribed_robots.clear()

    def _resubscribe_pending_robots(self) -> bool:
        """Retry subscriptions that are absent from the current session."""
        with self._lock:
            pending_robots = set(self._pending_resubscriptions)

        all_restored = True
        for robot_encoding_id in pending_robots:
            if not self.subscribe_to_robot(
                    robot_encoding_id, time.time_ns() / 1000000):
                logger.warning('Unable to resubscribe robot [%s]',
                               robot_encoding_id)
                with self._lock:
                    self._pending_resubscriptions.add(robot_encoding_id)
                all_restored = False
            else:
                with self._lock:
                    self._pending_resubscriptions.discard(robot_encoding_id)

        return all_restored

    def _next_token_refresh_time(self) -> float:
        """Refresh every six hours, or before an earlier server expiry."""
        refresh_in = self.TOKEN_REFRESH_INTERVAL_SECONDS
        if self.token_expiry is not None:
            refresh_in = min(refresh_in, max(0.0, self.token_expiry -
                                              time.time() - 60.0))
        return time.monotonic() + refresh_in

    def _connection_supervisor_loop(self):
        """Own authentication, telemetry reconnects, and resubscriptions."""
        backoff = self.RECONNECT_INITIAL_BACKOFF_SECONDS
        next_token_refresh = self._next_token_refresh_time()

        while True:
            stale_websockets = self._clear_stale_telemetry_websockets()
            if stale_websockets:
                logger.warning(
                    'Robot telemetry WebSocket(s) stopped receiving data: %s',
                    ', '.join(stale_websockets))
            connected = (self._status_ws_ready.is_set() and
                         self._pose_ws_ready.is_set())
            refresh_due = time.monotonic() >= next_token_refresh

            if not connected or refresh_due:
                # Refreshing the token requires a fresh pair of authenticated
                # WebSocket sessions. A partial connection is also rebuilt as a
                # pair. Session-bound subscriptions must be restored afterwards.
                self._mark_all_for_resubscription()
                self._disconnect_websockets()
                self._status_ws_ready.clear()
                self._pose_ws_ready.clear()

                if not self.request_token() or \
                        not self.check_connection(refresh_token=False):
                    logger.warning('Robot telemetry WebSocket reconnect failed; '
                                   'retrying in %.1fs', backoff)
                    self._connection_wakeup.wait(backoff)
                    self._connection_wakeup.clear()
                    backoff = min(
                        backoff * 2, self.RECONNECT_MAX_BACKOFF_SECONDS)
                    continue

                next_token_refresh = self._next_token_refresh_time()

            with self._lock:
                has_pending_resubscriptions = bool(
                    self._pending_resubscriptions)
            if has_pending_resubscriptions:
                if not self._resubscribe_pending_robots():
                    logger.warning('Robot telemetry subscriptions are incomplete; '
                                   'retrying in %.1fs', backoff)
                    self._connection_wakeup.wait(backoff)
                    self._connection_wakeup.clear()
                    backoff = min(
                        backoff * 2, self.RECONNECT_MAX_BACKOFF_SECONDS)
                    continue

                logger.info('Robot telemetry subscriptions restored')

            backoff = self.RECONNECT_INITIAL_BACKOFF_SECONDS
            self._connection_wakeup.wait(
                min(1.0, next_token_refresh - time.monotonic()))
            self._connection_wakeup.clear()

    
    def request_token(self):
        '''Login without Authorization; store token from JSON field "token".'''
        path = constants.SECURITY_PATH
        payload = {'email': self.user, 'password': self.password, 'applicationName': 'DASHBOARD'}

        with self._token_lock:
            try:
                logger.debug(
                    'HTTP POST login request body=%s',
                    RobotAPI._redact_mapping(payload))
                r = self._post(
                    path=f'{constants.OPEN_API_PREFIX}{path}',
                    headers=None,
                    json=payload,
                )
                r.raise_for_status()
                data = r.json()
                logger.debug(
                    'HTTP POST login response body=%s',
                    RobotAPI._redact_mapping(data))
                token = data['token']
                expiry_raw = data['tokenExpiryIsoUtcTime']
                for fmt in ("%Y-%m-%dT%H:%M:%S.%fZ", "%Y-%m-%dT%H:%M:%SZ"):
                    try:
                        token_expiry = datetime.strptime(expiry_raw, fmt).replace(tzinfo=timezone.utc)
                        break
                    except ValueError:
                        continue
                else:
                    raise ValueError(f'Unrecognized tokenExpiryIsoUtcTime format: {expiry_raw!r}')

                self.token = token
                self.token_expiry = token_expiry.timestamp()
                logger.info(
                    'Login OK: token=%s expires_utc=%s (in %.0fs)',
                    RobotAPI._mask_token(token),
                    expiry_raw,
                    self.token_expiry - time.time(),
                )
            except requests.exceptions.RequestException as request_error:
                logger.error('Login request error: %s', request_error)
            except (KeyError, ValueError) as parse_err:
                logger.error('Login response parse error: %s', parse_err)

        return self.token is not None and self.token_expiry is not None and \
            self.token_expiry > time.time()

    def _bearer_headers(self) -> dict[str, str]:
        '''HTTP REST only: Authorization: Bearer <token> (see Postman REST requests).'''
        return {'Authorization': f'Bearer {self.token}'}

    def _ws_subprotocols(self) -> list[str]:
        '''Token as WebSocket subprotocol → Sec-WebSocket-Protocol header on handshake.'''
        return [self.token]

    def _log_ws_auth(self, channel: str, ws_url: str, subprotocols: list[str]):
        logger.debug(
            'WS CONNECT [%s] url=%s origin=https://%s offered_subprotocols=%s',
            channel,
            ws_url,
            self.prefix,
            [RobotAPI._mask_token(p) for p in subprotocols],
        )
        if self.token is None:
            logger.warning('WS CONNECT [%s]: token is None — handshake will fail', channel)

    def _log_http_send(self, method: str, url: str, headers=None, json_payload=None):
        safe_headers = RobotAPI._redact_mapping(headers)
        safe_payload = (
            RobotAPI._redact_mapping(json_payload)
            if isinstance(json_payload, dict) else json_payload)

        logger.info(
            'HTTP SEND [%s] full_request=%s',
            method,
            RobotAPI._json_for_log({
                'method': method,
                'url': url,
                'headers': safe_headers,
                'json': safe_payload,
            }),
        )

    def refresh_expired_token(self):
        if self.token_expiry is None or self.token_expiry <= time.time():
            self.request_token()

    def _build_https_url(self, path: str) -> str:
        return f'https://{self.prefix}{path}'

    def _build_wss_url(self, path: str) -> str:
        return f'wss://{self.prefix}{path}'

    def _get(self, path: str, headers=None):
        url = self._build_https_url(path)
        self._log_http_send('GET', url, headers=headers)
        return requests.get(url, headers=headers, timeout=RobotAPI.REQUEST_TIMEOUT_SECONDS)

    def _get_json_with_auth_retry(
            self,
            path: str,
            max_attempts: int = 3,
            retry_delay_seconds: float = 1.0):
        for attempt in range(max_attempts):
            self.refresh_expired_token()
            headers = self._bearer_headers()

            try:
                r = self._get(path=path, headers=headers)
                if r.status_code == 401 and attempt == 0:
                    logger.warning(
                        'HTTP GET %s returned 401; refreshing token and '
                        'retrying once',
                        path)
                    self.request_token()
                    continue

                r.raise_for_status()
                return r.json()
            except requests.exceptions.Timeout as timeout_error:
                logger.error(
                    'HTTP GET %s timed out after %ss on attempt %s/%s: %s',
                    path,
                    RobotAPI.REQUEST_TIMEOUT_SECONDS,
                    attempt + 1,
                    max_attempts,
                    timeout_error)
            except requests.exceptions.ConnectionError as connection_error:
                logger.error(
                    'HTTP GET %s connection error on attempt %s/%s: %s',
                    path,
                    attempt + 1,
                    max_attempts,
                    connection_error)
            except requests.exceptions.HTTPError as http_err:
                status_code = (
                    http_err.response.status_code
                    if http_err.response is not None else None)
                logger.error(
                    'HTTP GET %s error on attempt %s/%s: %s',
                    path,
                    attempt + 1,
                    max_attempts,
                    http_err)
                if status_code is None or status_code < 500:
                    return None

            if attempt + 1 < max_attempts:
                time.sleep(retry_delay_seconds)

        return None

    def _post(self, path: str, headers=None, json=None):
        url = self._build_https_url(path)
        self._log_http_send('POST', url, headers=headers, json_payload=json)
        return requests.post(url, headers=headers, json=json, timeout=RobotAPI.REQUEST_TIMEOUT_SECONDS)

    def _put(self, path: str, headers=None, json=None):
        url = self._build_https_url(path)
        self._log_http_send('PUT', url, headers=headers, json_payload=json)
        return requests.put(url, headers=headers, json=json, timeout=RobotAPI.REQUEST_TIMEOUT_SECONDS)

    def connect_to_robot_status_ws(self):
        self.refresh_expired_token()
        if self.token is None:
            logger.error('Cannot open robotstatus WebSocket: no token after login')
            return

        def on_message(wsc, message):
            if wsc is self.robot_status_ws_connection:
                with self._lock:
                    self._last_status_ws_message_at = time.monotonic()
            json_message = json.loads(message)

            with self._lock:
                operation_fb = json_message.get('operation_fb')
                if operation_fb == 'ping':
                    return
                robot_id = json_message.get('robot_id')
                if robot_id is None:
                    return
                robot_encoding_id = self._robot_state_key(robot_id)

                if operation_fb == 'touchscreen_robot_status':
                    logger.debug('WS RECV status full_message=%s', message)
                    robot_status = json_message['content']

                    status_snapshot = RobotAPI._cached_touchscreen_robot_status(
                        operation_fb,
                        robot_status,
                        json_message.get('timestamp'))
                    self.robot_status[robot_encoding_id] = status_snapshot
                elif operation_fb in (
                    OperationEndStatus.P2P_END_STATUS,
                    OperationEndStatus.CLEAN_END_STATUS,
                    OperationEndStatus.DOCK_END_STATUS,
                    OperationEndStatus.UNDOCK_END_STATUS,
                ):
                    logger.debug('WS RECV end_status full_message=%s', message)
                    self.robot_operation_end_status[robot_encoding_id] = json_message

        def on_error(wsc, error):
            logger.info(error)
            if wsc is self.robot_status_ws_connection:
                self._status_ws_ready.clear()
                self._connection_wakeup.set()

        def on_close(wsc, close_status_code, close_msg):
            if wsc is self.robot_status_ws_connection:
                self._status_ws_ready.clear()
                self._connection_wakeup.set()
            logger.info("### Status Websocket Connecton Closed ###")

        def on_open(wsc):
            if wsc is self.robot_status_ws_connection:
                with self._lock:
                    self._last_status_ws_message_at = time.monotonic()
                self._status_ws_ready.set()
            logger.info("Opened Status Websocket Connection")

        path = f'{constants.WS_OPEN_API_PREFIX}/robotstatus'
        subprotocols = self._ws_subprotocols()
        status_ws_url = self._build_wss_url(path)
        self._log_ws_auth('robotstatus', status_ws_url, subprotocols)
        self.robot_status_ws_connection = websocket.WebSocketApp(
            status_ws_url,
            subprotocols=subprotocols,
            on_open=on_open,
            on_close=on_close,
            on_error=on_error,
            on_message=on_message,
        )
        self.robot_status_ws_connection.run_forever(suppress_origin=True)

    def connect_to_robot_position_ws(self):
        self.refresh_expired_token()
        if self.token is None:
            logger.error('Cannot open robotpose WebSocket: no token after login')
            return

        def on_message(wsc, message):
            if wsc is self.robot_pose_ws_connection:
                with self._lock:
                    self._last_pose_ws_message_at = time.monotonic()
            json_message = json.loads(message)
            with self._lock:
                if json_message.get('operation_fb') == 'ping':
                    return
                robot_id = json_message.get('robot_id')
                if json_message.get('operation_fb') == 'robot_pose' and robot_id is not None:
                    logger.debug('WS RECV pose full_message=%s', message)
                    robot_encoding_id = self._robot_state_key(robot_id)
                    robot_pose = json_message['content']
                    self.robot_pose[robot_encoding_id] = LionsbotCoord(
                        x=robot_pose['x'],
                        y=robot_pose['y'],
                        # LionsBot pose feedback uses degrees, while all
                        # downstream adapter and RMF orientation values use
                        # radians.
                        orientation_radians=math.radians(
                            robot_pose['heading']))

        def on_error(wsc, error):
            logger.error(error)
            if wsc is self.robot_pose_ws_connection:
                self._pose_ws_ready.clear()
                self._connection_wakeup.set()

        def on_close(wsc, close_status_code, close_msg):
            if wsc is self.robot_pose_ws_connection:
                self._pose_ws_ready.clear()
                self._connection_wakeup.set()
            logger.info("### Position Websocket Connecton Closed ###")

        def on_open(wsc):
            if wsc is self.robot_pose_ws_connection:
                with self._lock:
                    self._last_pose_ws_message_at = time.monotonic()
                self._pose_ws_ready.set()
            logger.info("Opened Position Websocket Connection")

        path = f'{constants.WS_OPEN_API_PREFIX}/robotpose'
        subprotocols = self._ws_subprotocols()
        pose_ws_url = self._build_wss_url(path)
        self._log_ws_auth('robotpose', pose_ws_url, subprotocols)
        self.robot_pose_ws_connection = websocket.WebSocketApp(
            pose_ws_url,
            subprotocols=subprotocols,
            on_open=on_open,
            on_close=on_close,
            on_error=on_error,
            on_message=on_message,
        )
        # TODO: check if origin should be set
        self.robot_pose_ws_connection.run_forever(suppress_origin=True)

    def is_subscribed(self, robot_encoding_id: str) -> bool:
        with self._lock:
            return robot_encoding_id in self._subscribed_robots

    def subscribe_to_robot(self, robot_encoding_id: str, time_stamp: float):
        payload = {'operation_cmd': 'subscribe', 'robot_id': robot_encoding_id}
        subscribe_status_message = json.dumps(payload)
        subscribe_pose_message = json.dumps(payload)

        if self.robot_status_ws_connection is None or self.robot_pose_ws_connection is None:
            logger.error('WS SEND subscribe skipped: WebSocket connection not established')
            return False

        with self._lock:
            if robot_encoding_id in self._subscribed_robots:
                return True
            try:
                logger.debug('WS SEND [robotstatus] full_message=%s',
                             subscribe_status_message)
                self.robot_status_ws_connection.send(subscribe_status_message)
                logger.debug('WS SEND [robotpose] full_message=%s',
                             subscribe_pose_message)
                self.robot_pose_ws_connection.send(subscribe_pose_message)
            except Exception as error:
                logger.error('WS SEND subscribe failed for [%s]: %r',
                             robot_encoding_id, error)
                return False
            self._subscribed_robots.add(robot_encoding_id)

        time.sleep(2.5)

        return True

    def _build_ws_command_payload(self, operation_cmd: str, robot_encoding_id: str, time_stamp: float, content: dict):
        return {
            'operation_cmd': operation_cmd,
            'robot_id': robot_encoding_id,
            'time_stamp': time_stamp,
            'content': content
        }

    # ------------------------------------------------------------------------------
    # Robot Information Accessors
    # ------------------------------------------------------------------------------
    def position(self, robot_encoding_id: str) -> LionsbotCoord:
        ''' Return Coordinate:LionsbotCoord expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        position = self.robot_pose.get(robot_encoding_id, None)

        return position

    def get_robot_info(self, robot_encoding_id: str):
        path = (
            f'{constants.OPEN_API_PREFIX}/robot/by-encoding-id/'
            f'{robot_encoding_id}')
        return self._get_json_with_auth_retry(path)

    def get_robot_status(self, robot_encoding_id: str):
        return self.robot_status.get(robot_encoding_id, None)

    def battery_soc(self, robot_encoding_id: str):
        robot_status = self.get_robot_status(robot_encoding_id)
        if robot_status is None:
            return None

        battery_soc = robot_status.get('batterySoc')
        if battery_soc is None:
            return None
        return battery_soc / 100


    def _get_robot_maps(self, robot_encoding_id: str):
        robot_uuid = self._robot_uuid(robot_encoding_id)
        path = f'{constants.OPEN_API_PREFIX}/robot/{robot_uuid}/map'
        return self._get_json_with_auth_retry(path)

    @staticmethod
    def _map_level_value(map_data: dict):
        for key in (
                'level',
                'levelName',
                'level_name',
                'floor',
                'floorName',
                'floor_name',
                'floorNumber',
                'floor_number'):
            value = map_data.get(key)
            if value is not None:
                return value
        return None

    @staticmethod
    def _normalize_map_level(level):
        if level is None:
            return None
        return str(level).strip()
    
    @staticmethod
    def robot_is_available(robot_status: dict | None) -> bool:
        if robot_status is None:
            return False

        # A docked robot is not reusable while it is explicitly reported as
        # charging. The vendor's display status may remain "Docked" in that
        # state. Older payloads may omit this field, so preserve the existing
        # availability behavior when it is absent.
        return robot_status.get('status') in (
            RobotRuntimeStatusText.RESTING.value,
            RobotRuntimeStatusText.DOCKED.value,
        )

    @classmethod
    def _map_level_matches(cls, map_data: dict, map_level) -> bool:
        requested_level = cls._normalize_map_level(map_level)
        candidate_level = cls._normalize_map_level(
            cls._map_level_value(map_data))
        return candidate_level == requested_level

    def get_map(
            self,
            map_name: str,
            robot_encoding_id: str,
            map_level=None):
        data = self._get_robot_maps(robot_encoding_id)
        if data is None:
            return None

        maps = data['workSiteMaps']
        name_matches = [m for m in maps if m['name'] == map_name]
        if map_level is None:
            logger.error(
                f'Map level is required when looking up map [{map_name}] '
                f'for robot [{robot_encoding_id}]')
            return None

        for m in name_matches:
            if self._map_level_matches(m, map_level):
                logger.info(
                    f'Found map [{map_name}] level [{map_level}] '
                    f'for robot [{robot_encoding_id}]')
                return m

        logger.error(
            f'Unable to find map [{map_name}] level [{map_level}] '
            f'for robot [{robot_encoding_id}]; '
            f'name_matches={len(name_matches)}')
        return None

    def get_zones_by_map(self, map_id: str):
        path = f'{constants.OPEN_API_PREFIX}/robot/map/{map_id}/zones'
        return self._get_json_with_auth_retry(path)

    def get_zone_equalizers(self, map_id: str, robot_encoding_id: str):
        path = (
            f'{constants.OPEN_API_PREFIX}/robot/map/{map_id}'
            f'/equalizer-configs?robotId={self._robot_uuid(robot_encoding_id)}')
        return self._get_json_with_auth_retry(path)

    # ------------------------------------------------------------------------------
    # Robot Operations
    # ------------------------------------------------------------------------------
    def send_navigation_command(
            self,
            robot_encoding_id: str,
            pose: LionsbotCoord,
            map_name: str) -> bool:
        '''Send a P2P navigation command to the robot.'''
        self.refresh_expired_token()

        navigate_content = NavigateContent(
            heading_radians=pose.orientation_radians,
            x=RobotAPI._coordinate_as_int(pose.x),
            y=RobotAPI._coordinate_as_int(pose.y),
            waypoint='',
            waypoint_id=''
        )
        time_stamp = time.time_ns() / 1000000
        with self._lock:
            self.robot_operation_end_status[robot_encoding_id] = None
        payload = self._build_ws_command_payload(
            operation_cmd='p2p_start',
            robot_encoding_id=robot_encoding_id,
            time_stamp=time_stamp,
            content=navigate_content.__dict__)
        navigate_message = json.dumps(payload)
        logger.debug('WS SEND [robotstatus] full_message=%s',
                     navigate_message)
        try:
            self.robot_status_ws_connection.send(navigate_message)
        except Exception as err:
            logger.info(f'Navigation command send failed: {err}')
            return False
        self.robot_current_map[robot_encoding_id] = map_name
        return True

    def navigate_robot(
            self,
            robot_encoding_id: str,
            pose: LionsbotCoord,
            map_name: str) -> OperationStatus:
        '''Report the current navigation status for the robot.'''
        self.refresh_expired_token()

        with self._lock:
            robot_status = self.get_robot_status(
                robot_encoding_id=robot_encoding_id)
            robot_pose = self.robot_pose.get(robot_encoding_id)
            operation_end_status = self.robot_operation_end_status.get(
                robot_encoding_id)

        if operation_end_status is not None and \
                operation_end_status.get('operation_fb') == \
                OperationEndStatus.P2P_END_STATUS:
            if operation_end_status.get('content', {}).get('status') is False:
                pending_navigation = getattr(
                    self,
                    '_pending_failed_navigation_end_status',
                    {})
                self._pending_failed_navigation_end_status = \
                    pending_navigation
                if robot_encoding_id not in pending_navigation:
                    self._pending_failed_navigation_end_status[
                        robot_encoding_id] = robot_pose
                    self._pending_failed_navigation_end_status_since[
                        robot_encoding_id] = time.monotonic()
                    return OperationStatus(CommandResult.RUNNING)

                pending_pose = pending_navigation[robot_encoding_id]
                waiting_since = \
                    self._pending_failed_navigation_end_status_since.get(
                        robot_encoding_id, time.monotonic())
                if robot_pose is pending_pose and \
                        time.monotonic() - waiting_since < \
                        self.FAILED_P2P_POSE_WAIT_SECONDS:
                    return OperationStatus(CommandResult.RUNNING)

                if robot_pose is None:
                    return OperationStatus(CommandResult.RUNNING)

                self._pending_failed_navigation_end_status.pop(
                    robot_encoding_id, None)
                self._pending_failed_navigation_end_status_since.pop(
                    robot_encoding_id, None)
                if self._within_goal_tolerance_radius(
                        robot_encoding_id,
                        pose,
                        robot_pose):
                    with self._lock:
                        if self.robot_operation_end_status.get(
                                robot_encoding_id) is operation_end_status:
                            self.robot_operation_end_status[
                                robot_encoding_id] = None
                    return OperationStatus(CommandResult.SUCCESS)
                with self._lock:
                    if self.robot_operation_end_status.get(
                            robot_encoding_id) is operation_end_status:
                        self.robot_operation_end_status[robot_encoding_id] = None
                return OperationStatus(CommandResult.ERROR)
            self._pending_failed_navigation_end_status.pop(
                robot_encoding_id, None)
            self._pending_failed_navigation_end_status_since.pop(
                robot_encoding_id, None)
            with self._lock:
                if self.robot_operation_end_status.get(
                        robot_encoding_id) is operation_end_status:
                    self.robot_operation_end_status[robot_encoding_id] = None
            return OperationStatus(CommandResult.SUCCESS)

        eta_seconds = None
        if robot_status is not None:
            eta = robot_status.get('eta')
            try:
                eta_seconds = float(eta)
            except (TypeError, ValueError):
                eta_seconds = None

        return OperationStatus(CommandResult.RUNNING, eta_seconds)

    def send_cleaning_command(
            self,
            robot_encoding_id: str,
            clean_zone_name: str,
            map_name: str,
            map_level=None) -> bool:
        self.refresh_expired_token()
        try:
            clean_process_content = self.build_clean_process_content(
                robot_encoding_id=robot_encoding_id,
                map_name=map_name,
                clean_zone_name=clean_zone_name,
                map_level=map_level)
        except Exception as err:
            logger.info(
                f'Clean command build failed for robot '
                f'[{robot_encoding_id}] zone [{clean_zone_name}]: {err}')
            return False

        if clean_process_content is None:
            return False

        payload = {
            'operation_cmd': 'clean_start',
            'robot_id': robot_encoding_id,
            'content': clean_process_content.__dict__
        }
        clean_message = json.dumps(payload)
        with self._lock:
            self.robot_operation_end_status[robot_encoding_id] = None
        logger.debug('WS SEND [robotstatus] full_message=%s', clean_message)
        try:
            self.robot_status_ws_connection.send(clean_message)
        except Exception as err:
            logger.info(f'Clean command send failed: {err}')
            return False

        return True

    def build_clean_process_content(
            self,
            robot_encoding_id: str,
            map_name: str,
            clean_zone_name: str,
            map_level=None):
        map_data = self.get_map(
            map_name=map_name,
            robot_encoding_id=robot_encoding_id,
            map_level=map_level)
        if map_data is None:
            logger.info(
                f'Unable to build clean command for robot '
                f'[{robot_encoding_id}]: map [{map_name}] level '
                f'[{map_level}] is unavailable')
            return None

        map_id = map_data['id']
        map_zones = self.get_zones_by_map(map_id=map_id)
        if map_zones is None:
            logger.info(
                f'Unable to build clean command for robot '
                f'[{robot_encoding_id}]: zones for map [{map_id}] are '
                f'unavailable')
            return None
        filtered_zones = [
            zone for zone in map_zones
            if zone.get('name') == clean_zone_name]
        if not filtered_zones:
            logger.info(
                f'Unable to build clean command for robot '
                f'[{robot_encoding_id}]: zone [{clean_zone_name}] was not '
                f'found on map [{map_id}]')
            return None
        clean_zone = filtered_zones[0]

        all_zone_equalizers = self.get_zone_equalizers(
            map_id=map_id,
            robot_encoding_id=robot_encoding_id)
        if all_zone_equalizers is None:
            logger.info(
                f'Unable to build clean command for robot '
                f'[{robot_encoding_id}]: equalizers for map [{map_id}] are '
                f'unavailable')
            return None

        zone_all_equalizer_response = all_zone_equalizers.get(
            'zoneAllEqualizerResponse')
        if zone_all_equalizer_response is None:
            logger.info(
                f'Unable to build clean command for robot '
                f'[{robot_encoding_id}]: equalizer response for map '
                f'[{map_id}] is missing zoneAllEqualizerResponse')
            return None
        filtered_zone_equalizers = [
            equalizer for equalizer in zone_all_equalizer_response
            if equalizer.get('name') == clean_zone_name]
        if not filtered_zone_equalizers:
            logger.info(
                f'Unable to build clean command for robot '
                f'[{robot_encoding_id}]: equalizer for zone '
                f'[{clean_zone_name}] was not found')
            return None
        zone_equalizer = filtered_zone_equalizers[0]
        selected_mode_name = zone_equalizer['selectedModeName']

        filtered_modes = list(filter(
            lambda x: x.get('name') == selected_mode_name,
            zone_equalizer.get('modes', [])))
        if not filtered_modes:
            logger.info(
                f'Unable to build clean command for robot '
                f'[{robot_encoding_id}]: selected equalizer mode '
                f'[{selected_mode_name}] was not found')
            return None
        selected_zone_equalizers = filtered_modes[0]

        configs = {}
        for config in selected_zone_equalizers['configs']:
            configs[config['configName']] = config['value']

        zones = [{
            'configs': configs,
            'name': clean_zone['name']
        }]

        clean_process_content = CleanProcessContent(
            zones=zones
        )

        return clean_process_content

    def stop(
            self,
            robot_encoding_id: str,
            operation_family: str | None = None,
            timeout_seconds: float | None = None):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        self.refresh_expired_token()

        robot_status = self.get_robot_status(robot_encoding_id=robot_encoding_id)
        if robot_status is None:
            return False

        if RobotAPI.robot_is_available(robot_status):
            return True

        operation_family = self._resolve_operation_family(
            operation_family,
            robot_status)
        operation_cmd = 'operation_stop'
        success_statuses = {RobotRuntimeStatusText.RESTING.value}
        success_grace_seconds = 30.0
        succeed_on_received = False
        if operation_family == RobotAPI.OPERATION_P2P:
            end_status = OperationEndStatus.P2P_END_STATUS
        elif operation_family == RobotAPI.OPERATION_CLEANING:
            end_status = OperationEndStatus.CLEAN_END_STATUS
        elif operation_family == RobotAPI.OPERATION_DOCK:
            end_status = OperationEndStatus.DOCK_END_STATUS
            succeed_on_received = True
        else:
            return False

        with self._lock:
            self.robot_operation_end_status[robot_encoding_id] = None
        if not self._send_robot_status_command(robot_encoding_id, operation_cmd):
            return False
        return self._wait_for_operation_end_status(
            robot_encoding_id,
            end_status,
            succeed_on_received=succeed_on_received,
            success_statuses=success_statuses,
            success_grace_seconds=success_grace_seconds,
            timeout_seconds=timeout_seconds)

    def process_completed(self, robot_encoding_id: str) -> OperationStatus:
        '''Report the current cleaning status for the robot.'''
        self.refresh_expired_token()

        with self._lock:
            robot_status = self.get_robot_status(
                robot_encoding_id=robot_encoding_id)
        operation_end_status = self._take_operation_end_status(
            robot_encoding_id, OperationEndStatus.CLEAN_END_STATUS)

        if operation_end_status is not None and \
                operation_end_status.get('operation_fb') == \
                OperationEndStatus.CLEAN_END_STATUS:
            if operation_end_status.get('content', {}).get('status') is False:
                return OperationStatus(CommandResult.ERROR)
            return OperationStatus(CommandResult.SUCCESS)

        eta_seconds = None
        if robot_status is not None and \
                robot_status['status'] == RobotRuntimeStatusText.CLEANING.value:
            eta = robot_status.get('eta')
            try:
                eta_seconds = float(eta)
            except (TypeError, ValueError):
                eta_seconds = None

        return OperationStatus(CommandResult.RUNNING, eta_seconds)

    def _resolve_operation_family(
            self,
            operation_family: str | None,
            robot_status: dict | None = None) -> str | None:
        if operation_family in (
                RobotAPI.OPERATION_P2P,
                RobotAPI.OPERATION_CLEANING,
                RobotAPI.OPERATION_DOCK):
            return operation_family
        if operation_family == 'clean':
            return RobotAPI.OPERATION_CLEANING
        if operation_family == 'docking':
            return RobotAPI.OPERATION_DOCK

        if robot_status is None:
            return None

        status = robot_status.get('status')
        if status in (
                RobotRuntimeStatusText.MOVING.value,
                RobotRuntimeStatusText.MOVING_PAUSED.value):
            return RobotAPI.OPERATION_P2P
        if status in (
                RobotRuntimeStatusText.CLEANING.value,
                RobotRuntimeStatusText.CLEANING_PAUSED.value):
            return RobotAPI.OPERATION_CLEANING
        if status in (
                RobotRuntimeStatusText.DOCKING_MOVE.value,
                RobotRuntimeStatusText.DOCKING_IN_PROGRESS.value,
                RobotRuntimeStatusText.DOCKING_PAUSED.value):
            return RobotAPI.OPERATION_DOCK
        return None

    def _send_robot_status_command(
            self,
            robot_encoding_id: str,
            operation_cmd: str) -> bool:
        self.refresh_expired_token()
        payload = self._build_ws_command_payload(
            operation_cmd=operation_cmd,
            robot_encoding_id=robot_encoding_id,
            time_stamp=time.time_ns() / 1000000,
            content={'status': True})
        message = json.dumps(payload)
        logger.debug('WS SEND [robotstatus] full_message=%s', message)
        try:
            self.robot_status_ws_connection.send(message)
        except Exception as err:
            logger.info(f'Robot status command [{operation_cmd}] failed: {err}')
            return False
        return True

    def _wait_for_robot_status(
            self,
            robot_encoding_id: str,
            expected_statuses: set[str]) -> bool:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            robot_status = self.get_robot_status(robot_encoding_id=robot_encoding_id)
            if robot_status is not None and \
                    robot_status.get('status') in expected_statuses:
                return True
            time.sleep(0.5)
        return False

    def _operation_success_fallback_key(
            self,
            robot_encoding_id: str,
            operation_end_status: str) -> tuple[str, str]:
        return (robot_encoding_id, operation_end_status)

    def _clear_operation_success_fallback(
            self,
            robot_encoding_id: str,
            operation_end_status: str):
        key = self._operation_success_fallback_key(
            robot_encoding_id,
            operation_end_status)
        self._operation_success_fallback_seen_at.pop(key, None)

    def _take_operation_end_status(
            self,
            robot_encoding_id: str,
            expected_operation_fb: str):
        """Atomically read and consume a matching operation end status."""
        with self._lock:
            result = self.robot_operation_end_status.get(robot_encoding_id)
            if result is not None and \
                    result.get('operation_fb') == expected_operation_fb:
                self.robot_operation_end_status[robot_encoding_id] = None
                return result
        return None

    def _operation_success_after_grace(
            self,
            robot_encoding_id: str,
            operation_end_status: str,
            now: float) -> bool:
        key = self._operation_success_fallback_key(
            robot_encoding_id,
            operation_end_status)
        success_seen_at = self._operation_success_fallback_seen_at.get(key)
        if success_seen_at is None:
            self._operation_success_fallback_seen_at[key] = now
            return self.operation_end_status_grace_seconds <= 0
        return now - success_seen_at >= \
            self.operation_end_status_grace_seconds

    def _wait_for_operation_end_status(
            self,
            robot_encoding_id: str,
            operation_end_status: str,
            succeed_on_received: bool = False,
            success_statuses: set[str] | None = None,
            success_grace_seconds: float = 0.0,
            timeout_seconds: float | None = None) -> bool:
        if timeout_seconds is None:
            timeout_seconds = RobotAPI.STOP_TIMEOUT_SECONDS
        deadline = time.monotonic() + max(0.0, timeout_seconds)
        success_seen_at = None
        while time.monotonic() < deadline:
            result = self._take_operation_end_status(
                robot_encoding_id, operation_end_status)
            if result is not None:
                if succeed_on_received:
                    return True
                return result.get('content', {}).get('status') is True

            if success_statuses is not None:
                robot_status = self.get_robot_status(
                    robot_encoding_id=robot_encoding_id)
                if robot_status is not None and \
                        robot_status.get('status') in success_statuses:
                    if success_grace_seconds <= 0:
                        return True
                    if success_seen_at is None:
                        success_seen_at = time.monotonic()
                    elif time.monotonic() - success_seen_at >= \
                            success_grace_seconds:
                        return True
                else:
                    success_seen_at = None
            time.sleep(0.5)
        return False

    def send_docking_command(
            self,
            robot_encoding_id: str,
            content: DockProcessContent) -> bool:
        payload = self._build_ws_command_payload(
            operation_cmd='dock_start',
            robot_encoding_id=robot_encoding_id,
            time_stamp=time.time_ns() / 1000000,
            content={
                **content.__dict__,
                'x': RobotAPI._coordinate_as_int(content.x),
                'y': RobotAPI._coordinate_as_int(content.y),
            })
        dock_message = json.dumps(payload)
        with self._lock:
            self.robot_operation_end_status[robot_encoding_id] = None
        logger.debug('WS SEND [robotstatus] full_message=%s', dock_message)
        try:
            self.robot_status_ws_connection.send(dock_message)
        except Exception as err:
            logger.info(f'Docking command send failed: {err}')
            return False
        return True

    def dock_robot(self, robot_encoding_id: str) -> OperationStatus:
        self.refresh_expired_token()

        with self._lock:
            robot_status = self.get_robot_status(
                robot_encoding_id=robot_encoding_id)
        operation_end_status = self._take_operation_end_status(
            robot_encoding_id, OperationEndStatus.DOCK_END_STATUS)

        if operation_end_status is not None and \
                operation_end_status.get('operation_fb') == \
                OperationEndStatus.DOCK_END_STATUS:
            if operation_end_status.get('content', {}).get('status') is False:
                self._clear_operation_success_fallback(
                    robot_encoding_id,
                    OperationEndStatus.DOCK_END_STATUS)
                return OperationStatus(CommandResult.ERROR)
            self._clear_operation_success_fallback(
                robot_encoding_id,
                OperationEndStatus.DOCK_END_STATUS)
            return OperationStatus(CommandResult.SUCCESS)

        if RobotAPI._is_charging_status(robot_status) or RobotAPI._is_docked_status(robot_status):
            self._clear_operation_success_fallback(
                robot_encoding_id,
                OperationEndStatus.DOCK_END_STATUS)
            return OperationStatus(CommandResult.SUCCESS)

        eta_seconds = None
        if robot_status is not None:
            eta = robot_status.get('eta')
            try:
                eta_seconds = float(eta)
            except (TypeError, ValueError):
                eta_seconds = None

        return OperationStatus(CommandResult.RUNNING, eta_seconds)

    def send_undocking_command(self, robot_encoding_id: str) -> bool:
        payload = self._build_ws_command_payload(
            operation_cmd='undock_start',
            robot_encoding_id=robot_encoding_id,
            time_stamp=time.time_ns() / 1000000,
            content={'status': 'true'})
        undock_message = json.dumps(payload)
        with self._lock:
            self.robot_operation_end_status[robot_encoding_id] = None
        logger.debug('WS SEND [robotstatus] full_message=%s', undock_message)
        try:
            self.robot_status_ws_connection.send(undock_message)
        except Exception as err:
            logger.info(f'Undocking command send failed: {err}')
            return False
        return True

    def undock_robot(self, robot_encoding_id: str) -> OperationStatus:
        self.refresh_expired_token()

        now = time.monotonic()
        with self._lock:
            robot_status = self.get_robot_status(
                robot_encoding_id=robot_encoding_id)
        operation_end_status = self._take_operation_end_status(
            robot_encoding_id, OperationEndStatus.UNDOCK_END_STATUS)

        if operation_end_status is not None and \
                operation_end_status.get('operation_fb') == \
                OperationEndStatus.UNDOCK_END_STATUS:
            if operation_end_status.get('content', {}).get('status') is False:
                self._clear_operation_success_fallback(
                    robot_encoding_id,
                    OperationEndStatus.UNDOCK_END_STATUS)
                return OperationStatus(CommandResult.ERROR)
            self._clear_operation_success_fallback(
                robot_encoding_id,
                OperationEndStatus.UNDOCK_END_STATUS)
            return OperationStatus(CommandResult.SUCCESS)

        if robot_status is not None and \
                robot_status['status'] == RobotRuntimeStatusText.RESTING.value:
            undocked_success = self._operation_success_after_grace(
                robot_encoding_id,
                OperationEndStatus.UNDOCK_END_STATUS,
                now)
            if undocked_success:
                return OperationStatus(CommandResult.SUCCESS)
            return OperationStatus(CommandResult.RUNNING)

        eta_seconds = None
        if robot_status is not None:
            eta = robot_status.get('eta')
            try:
                eta_seconds = float(eta)
            except (TypeError, ValueError):
                eta_seconds = None

        return OperationStatus(CommandResult.RUNNING, eta_seconds)

    def localize(self, dock_home_position: LionsbotCoord, robot_encoding_id: str) -> bool:
        self.refresh_expired_token()

        robot_uuid = self._robot_uuid(robot_encoding_id)
        path = f'{constants.OPEN_API_PREFIX}/robot/command/hot-localize/{robot_uuid}'
        headers = self._bearer_headers()

        heading = dock_home_position.orientation_radians

        payload = {
            'x': RobotAPI._coordinate_as_int(dock_home_position.x),
            'y': RobotAPI._coordinate_as_int(dock_home_position.y),
            'heading': heading,
        }

        try:
            r = self._put(path=path, headers=headers, json=payload)
            r.raise_for_status()
            data = r.json()

            if data['success'] and data['percentage'] > 25:
                return True
        except requests.exceptions.Timeout as timeout_error:
            logger.info(
                'HTTP PUT %s timed out after %ss: %s',
                path, RobotAPI.REQUEST_TIMEOUT_SECONDS, timeout_error)
        except requests.exceptions.ConnectionError as connection_error:
            logger.info(f'Connection error: {connection_error}')
        except requests.exceptions.HTTPError as http_err:
            logger.info(f'HTTP error: {http_err}')

        return False

    def change_map(self, robot_encoding_id: str, map_name: str, map_level=None):
        self.refresh_expired_token()

        map_data = self.get_map(
            map_name=map_name,
            robot_encoding_id=robot_encoding_id,
            map_level=map_level)
        if map_data is None:
            return False

        map_id = map_data['id']

        robot_uuid = self._robot_uuid(robot_encoding_id)
        path = f'{constants.OPEN_API_PREFIX}/robot/{robot_uuid}/map/{map_id}'
        headers = self._bearer_headers()

        try:
            r = self._put(path=path, headers=headers)
            r.raise_for_status()

            timeout = 10
            while True:
                robot_maps = self._get_robot_maps(
                    robot_encoding_id=robot_encoding_id)
                if robot_maps is None:
                    return False

                current_map_id = robot_maps['currentMapId']
                if current_map_id == map_id:
                    logger.debug("Required map set to: %s", current_map_id)
                    return True
                else:
                    logger.debug("Current map still at: %s", current_map_id)

                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    return False
        except requests.exceptions.Timeout as timeout_error:
            logger.info(
                'HTTP PUT %s timed out after %ss: %s',
                path, RobotAPI.REQUEST_TIMEOUT_SECONDS, timeout_error)
        except requests.exceptions.ConnectionError as connection_error:
            logger.info(f'Connection error: {connection_error}')
        except requests.exceptions.HTTPError as http_err:
            logger.info(f'HTTP error: {http_err}')

        return False
