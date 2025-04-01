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
import threading

import websocket
import requests
from urllib.error import HTTPError
import numpy as np
import math

from .enums.enums import RobotStatus
from .enums.enums import RobotMissionStatus
from .enums.enums import ResponseCode
from .enums.enums import NavigateStatus
from .enums.enums import NavigationCompleteStatus

from .models.EqualizerConfig import EqualizerConfig
from .models.NavigateContent import NavigateContent
from .models.CleanProcessContent import CleanProcessContent
from .models.DockProcessContent import DockProcessContent
import time
from datetime import datetime
from datetime import timezone

from .models.StopProcessContent import StopProcessContent
from .models.Zone import Zone

from .utils.MapTransform import MapTransform
from .utils.Coordinate import RmfCoord
from .utils.Coordinate import LionsbotCoord
from typing import Dict, List


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, user: str, password: str):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.connected = False
        self.token = None
        self.token_expiry = None
        self.robot_status_ws_connection = None
        self.robot_pose_ws_connection = None
        self.robot = None
        self.connected = None

        self.robot_current_map = {}
        self.robot_current_building = {}
        self.robot_status = {}
        self.robot_pose: Dict[str, LionsbotCoord] = {}
        self.robot_mission = {}
        self.robot_operation = {}

        self._lock = threading.Lock()

        # Test connectivity
        connected = self.check_connection()
        self.connected = connected
        if connected:
            self.connected = True
        else:
            self.connected = False

    # ------------------------------------------------------------------------------
    # Websocket Functions
    # ------------------------------------------------------------------------------

    def check_connection(self):
        self.request_token()
        if self.token is None:
            return False

        connect_to_robot_status_thread = threading.Thread(target=self.connect_to_robot_status_ws, daemon=True)
        connect_to_robot_pose_thread = threading.Thread(target=self.connect_to_robot_position_ws, daemon=True)

        connect_to_robot_status_thread.start()
        connect_to_robot_pose_thread.start()

        time.sleep(3)

        return True

    def request_token(self):
        path = '/security/auth'
        payload = {'email': self.user, 'password': self.password, 'applicationName': 'DASHBOARD'}

        try:
            r = requests.post(f'https://{self.prefix}{path}', json=payload)
            r.raise_for_status()
            data = r.json()
            token = data['token']
            token_expiry = datetime.strptime(data['tokenExpiryIsoUtcTime'], "%Y-%m-%dT%H:%M:%S.%fZ").replace(tzinfo=timezone.utc)

            self.token = token
            self.token_expiry = token_expiry.timestamp()
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return None
    
    def refresh_expired_token(self):
        if self.token_expiry is None or self.token_expiry <= time.time():
            self.request_token()

    def connect_to_robot_status_ws(self):
        self.refresh_expired_token()

        def on_message(wsc, message):
            json_message = json.loads(message)
            with self._lock:
                if json_message['operation_fb'] == 'robot_status':
                    robot_status = json_message['content']
                    self.robot_status[json_message['robot_id']] = {
                        'eta': robot_status['estimated_time_to_finish'],
                        'alertIds': [robot_status['alert_id']],
                        'docked': robot_status['docked'],
                        'progress': robot_status['progress'],
                        'localized': robot_status['localized'],
                        'soc': robot_status['battery_info']['soc'],
                        'state': robot_status['operation_state'],
                        'status': robot_status['operation_status']
                    }
                elif json_message['operation_fb'] == 'operation_status':
                    operation_status = json_message['content']
                    self.robot_mission[json_message['robot_id']] = {
                        'missionStatus': {
                            'activeMissionType': operation_status['activeMissionType'],
                            'mission': {
                                'status': operation_status['mission']['status'], 
                                'x': operation_status['mission'].get('x', None), 
                                'y': operation_status['mission'].get('y', None), 
                            }
                        },
                        'eta': self.robot_status[json_message['robot_id']]['eta'],
                        'alertIds': self.robot_status[json_message['robot_id']]['alertIds'],
                        'progress': self.robot_status[json_message['robot_id']]['progress']
                    }
                else:
                    robot_operation = self.robot_operation.get(json_message['robot_id'], None)
                    if robot_operation is not None and \
                            robot_operation['operation'] == json_message['operation_fb'] and \
                            robot_operation['time_stamp'] < time.time_ns() / 1000000:
                        operation_status = json_message['content']
                        self.robot_operation[json_message['robot_id']]['status'] = operation_status['status']

        def on_error(wsc, error):
            print(error)

        def on_close(wsc, close_status_code, close_msg):
            print("### closed ###")

        def on_open(wsc):
            print("Opened connection")

        path = '/ws/openapi/robotstatus/v3.1'
        subprotocols = [self.token]

        self.robot_status_ws_connection = websocket.WebSocketApp(f'wss://{self.prefix}{path}',
                                                                 subprotocols=subprotocols,
                                                                 on_open=on_open,
                                                                 on_close=on_close,
                                                                 on_error=on_error,
                                                                 on_message=on_message)
        self.robot_status_ws_connection.run_forever(origin=f'https://{self.prefix}')

    def connect_to_robot_position_ws(self):
        self.refresh_expired_token()

        def on_message(wsc, message):
            json_message = json.loads(message)
            with self._lock:
                if json_message['operation_fb'] == 'robot_pose':
                    robot_pose = json_message['content']
                    self.robot_pose[json_message['robot_id']] = LionsbotCoord(robot_pose['x'],
                                                                 robot_pose['y'],
                                                                 math.radians(robot_pose['heading']))

        def on_error(wsc, error):
            print(error)

        def on_close(wsc, close_status_code, close_msg):
            print("### closed ###")

        def on_open(wsc):
            print("Opened connection")

        path = '/ws/openapi/robotpose'
        subprotocols = [self.token]

        self.robot_pose_ws_connection = websocket.WebSocketApp(f'wss://{self.prefix}{path}',
                                                               subprotocols=subprotocols,
                                                               on_open=on_open,
                                                               on_close=on_close,
                                                               on_error=on_error,
                                                               on_message=on_message)
        self.robot_pose_ws_connection.run_forever(origin=f'https://{self.prefix}')

    def subscribe_to_robot(self, robot_encoding_id: str, time_stamp: float):
        payload = {'operation_cmd': 'subscribe', 'robot_encoding_id': robot_encoding_id, 'time_stamp': time_stamp}
        subscribe_status_message = json.dumps(payload)
        print(f'subscribe payload: {subscribe_status_message}')

        payload = {'operation_cmd': 'subscribe', 'robot_encoding_id': robot_encoding_id}
        subscribe_pose_message = json.dumps(payload)
        print(f'subscribe payload: {subscribe_pose_message}')

        self.robot_status_ws_connection.send(subscribe_status_message)
        self.robot_pose_ws_connection.send(subscribe_pose_message)

        time.sleep(2.5)

        return True

    # ------------------------------------------------------------------------------
    # Robot Information Accessors
    # ------------------------------------------------------------------------------

    def position(self, robot_name: str) -> LionsbotCoord:
        ''' Return LionsbotCoord expressed in the robot's coordinate frame or
            None if any errors are encountered'''    
        position = self.robot_pose.get(robot_name, None)

        if position is None:
            self.refresh_expired_token()

            path = f'/openapi/v1/robot/{robot_name}/position'
            headers = {'Authorization': f'Bearer {self.token}'}

            try:
                r = requests.get(f'https://{self.prefix}{path}', headers=headers)
                r.raise_for_status()
                data = r.json()

                x = data['x']
                y = data['y']
                orientation_radians = math.radians(data['heading'])

                if x is None or y is None or orientation_radians is None:
                    return None

                position = LionsbotCoord(x=x, y=y, orientation_radians=orientation_radians)
                self.robot_pose[robot_name] = position

                return position
            except requests.exceptions.ConnectionError as connection_error:
                print(f'Connection error: {connection_error}')
            except HTTPError as http_err:
                print(f'HTTP error: {http_err}')

            return None

        return position

    def get_mission_status(self, robot_name: str):
        mission_status = self.robot_mission.get(robot_name, None)

        if mission_status is None:
            self.refresh_expired_token()

            path = f'/openapi/v1/robot/{robot_name}/mission-status'
            headers = {'Authorization': f'Bearer {self.token}'}

            try:
                r = requests.get(f'https://{self.prefix}{path}', headers=headers)
                r.raise_for_status()
                data = r.json()

                self.robot_mission[robot_name] = data

                return data
            except requests.exceptions.ConnectionError as connection_error:
                print(f'Connection error: {connection_error}')
            except HTTPError as http_err:
                print(f'HTTP error: {http_err}')

            return None

        return mission_status

    def get_robot_info(self, robot_name: str):
        self.refresh_expired_token()

        path = f'/openapi/v1/robot/{robot_name}'
        headers = {'Authorization': f'Bearer {self.token}'}

        try:
            r = requests.get(f'https://{self.prefix}{path}', headers=headers)
            r.raise_for_status()
            data = r.json()

            return data
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return None

    def get_robot_status(self, robot_name: str):
        robot_status = self.robot_status.get(robot_name, None)

        if robot_status is None:
            self.refresh_expired_token()

            path = f'/openapi/v1/robot/{robot_name}/status'
            headers = {'Authorization': f'Bearer {self.token}'}

            try:
                r = requests.get(f'https://{self.prefix}{path}', headers=headers)
                r.raise_for_status()
                data = r.json()

                self.robot_status[robot_name] = data

                return data
            except requests.exceptions.ConnectionError as connection_error:
                print(f'Connection error: {connection_error}')
            except HTTPError as http_err:
                print(f'HTTP error: {http_err}')

            return None

        return robot_status

    def navigation_remaining_duration(self, robot_name: str):
        self.refresh_expired_token()

        mission_status = self.get_mission_status(robot_name)
        while mission_status is None:
            time.sleep(1)
            mission_status = self.get_mission_status(robot_name=robot_name)

        if mission_status is None or mission_status['eta'] is None:
            return 0.0

        return mission_status['eta']

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
        self.refresh_expired_token()

        robot_status = self.get_robot_status(robot_name)
        while robot_status is None:
            robot_status = self.get_robot_status(robot_name)

        if robot_status is None:
            return None

        return robot_status['soc'] / 100

    def get_zones_by_map(self, map_id: str):
        self.refresh_expired_token()

        path = f'/openapi/v1/worksitemap/{map_id}/zone'
        headers = {'Authorization': f'Bearer {self.token}'}

        try:
            r = requests.get(f'https://{self.prefix}{path}', headers=headers)
            r.raise_for_status()
            data = r.json()

            return data
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return None

    def get_zone_equalizers(self, process_id: str, robot_name: str):
        self.refresh_expired_token()

        path = f'/openapi/v1/worksitemap/section/zone/equalizer/{process_id}/{robot_name}'
        headers = {'Authorization': f'Bearer {self.token}'}

        try:
            r = requests.get(f'https://{self.prefix}{path}', headers=headers)
            r.raise_for_status()
            data = r.json()

            return data
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return None

    def get_robot_maps(self, robot_name: str):
        self.refresh_expired_token()

        path = f'/openapi/v1/worksite/robot/maps/{robot_name}'
        headers = {'Authorization': f'Bearer {self.token}'}

        try:
            r = requests.get(f'https://{self.prefix}{path}', headers=headers)
            r.raise_for_status()
            data = r.json()

            return data
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return None

    def get_map(self, map_name: str, robot_name: str):
        self.refresh_expired_token()

        path = f'/openapi/v1/worksite/robot/maps/{robot_name}'
        headers = {'Authorization': f'Bearer {self.token}'}

        try:
            r = requests.get(f'https://{self.prefix}{path}', headers=headers)
            r.raise_for_status()
            data = r.json()

            robot_current_building = self.robot_current_building[robot_name]
            for m in data['workSiteMaps']:
                if m['name'] == f'{map_name}_{robot_current_building}' and m['level'] == f'{map_name}':
                    return m
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return None

    # ------------------------------------------------------------------------------
    # Robot Operations
    # ------------------------------------------------------------------------------
    def navigate(self, robot_name: str, pose: LionsbotCoord, map_name: str) -> NavigateStatus:
        ''' Request the robot to navigate to pose:LionsbotCoord (robot's coordinate convention). This function
            should return True if the robot has accepted the request, else False'''
        self.refresh_expired_token()

        navigate_content = NavigateContent(
            heading_radians=pose.orientation_radians,
            x=pose.x,
            y=pose.y,
            waypoint='Custom',
            waypoint_id=''
        )

        self.navigate_robot(robot_encoding_id=robot_name, time_stamp=time.time_ns() / 1000000, content=navigate_content)

        timeout = 5
        while True:
            robot_mission_status = self.get_mission_status(robot_name=robot_name)
            if robot_mission_status is None:
                return NavigateStatus.NONE

            robot_mission_details = robot_mission_status['missionStatus']['mission'] 
            mission_status = robot_mission_details['status']
            
            x = robot_mission_details.get('x', pose.x)
            y = robot_mission_details.get('y', pose.y)

            # Ensure mission status is from the mission moving to the current waypoint
            # Margin of error round to 1e-04 between rmf waypoint coordinates and robot mission status coordinates
            # as there are times mission status feedback rounds of position to whole number
            if mission_status == RobotMissionStatus.ROBOT_MOVING.value \
                and math.isclose(x, pose.x, rel_tol=1e-04) and math.isclose(y, pose.y, rel_tol=1e-04):
                self.robot_current_map[robot_name] = map_name
                return NavigateStatus.MOVING
            elif mission_status == RobotMissionStatus.CMD_REJECTED.value \
                and math.isclose(x, pose.x, rel_tol=1e-04) and math.isclose(y, pose.y, rel_tol=1e-04) \
                and ResponseCode.DESTINATION_TOO_NEAR in robot_mission_status['alertIds']: 
                return NavigateStatus.TOO_CLOSE
            else:
                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    return NavigateStatus.FAILED
                                
    def start_process(self, robot_name: str, process: str, map_name: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''
        self.refresh_expired_token()

        self.clean(robot_encoding_id=robot_name,
                   process=process,
                   map_name=map_name,
                   time_stamp=time.time_ns() / 1000000)

        timeout = 5
        while True:
            robot_operation = self.robot_operation.get(robot_name, None)
            if robot_operation is None:
                return False

            elif robot_operation['operation'] == 'start_work' and robot_operation['status'] is not None:
                with self._lock:
                    if robot_operation['status']:
                        self.robot_operation[robot_name] = None
                        return True

                    self.robot_operation[robot_name] = None
                    return False

            time.sleep(1)
            timeout -= 1
            if timeout == 0:
                with self._lock:
                    self.robot_operation[robot_name] = None
                return False

    def stop(self, robot_name: str):
        self.refresh_expired_token()

        robot_status = self.get_robot_status(robot_name=robot_name)
        if robot_status['docked'] or robot_status['status'] in [RobotStatus.RESTING.value]:
            return True

        robot_mission_status = self.get_mission_status(robot_name=robot_name)
        if robot_mission_status is None:
            return False

        if robot_mission_status['missionStatus']['mission']['status'] == RobotMissionStatus.APP_STARTED.value:
            self.e_stop_robot(robot_name=robot_name, time_stamp=time.time_ns() / 1000000)
        else:
            operation = None
            if robot_mission_status['missionStatus']['activeMissionType'] == 'MOVING':
                operation = 2
            elif robot_mission_status['missionStatus']['activeMissionType'] == 'WORKING':
                operation = 1

            if operation is not None:
                stop_process_content = StopProcessContent(
                    robot_type='LeoScrub',
                    operation_code=operation,
                    status_code=2
                )

                self.stop_robot(robot_name=robot_name, time_stamp=time.time_ns() / 1000000,
                                content=stop_process_content)
            else:
                self.e_stop_robot(robot_name=robot_name, time_stamp=time.time_ns() / 1000000)

        time.sleep(0.5)

        timeout = 5
        while True:
            robot_mission_status = self.get_mission_status(robot_name=robot_name)
            if robot_mission_status is None:
                return False
            elif robot_mission_status['missionStatus']['activeMissionType'] == 'IDLE' or \
                    robot_mission_status['missionStatus']['mission']['status'] in RobotAPI.TERMINAL_MISSION_STATUSES:
                return True
            else:
                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    self.robot_mission[robot_name] = None
                    return False

    def navigation_completed(self, robot_name: str) -> NavigationCompleteStatus:
        self.refresh_expired_token()

        mission_status = self.get_mission_status(robot_name)

        if mission_status is None:
            return NavigationCompleteStatus.NONE

        if mission_status['missionStatus']['mission']['status'] == RobotMissionStatus.MOVING_FINISHED.value \
                or (ResponseCode.DESTINATION_TOO_NEAR in mission_status['alertIds']
                    and mission_status['missionStatus']['mission']['status'] == RobotMissionStatus.CMD_REJECTED.value):
            return NavigationCompleteStatus.MOVING_COMPLETED

        return NavigationCompleteStatus.MOVING_INPROGRESS

    def process_completed(self, robot_name: str):
        self.refresh_expired_token()

        mission_status = self.get_mission_status(robot_name)

        if mission_status is None:
            return False

        if mission_status['missionStatus']['mission']['status'] == RobotMissionStatus.CLEANING_FINISHED.value:
            return True

        return False

    def undocking_completed(self, robot_name: str):
        self.refresh_expired_token()

        robot_status = self.get_robot_status(robot_name=robot_name)
        if robot_status is not None and not robot_status['docked']: 
            return True

        mission_status = self.get_mission_status(robot_name)
        if mission_status is None:
            return False

        if mission_status['missionStatus']['activeMissionType'] == 'IDLE' \
                and mission_status['missionStatus']['mission']['status'] == RobotMissionStatus.ROBOT_DOCKING_STOPPED.value:
            return True
        return False

    def docking_completed(self, robot_name: str):
        self.refresh_expired_token()

        robot_status = self.get_robot_status(robot_name)
        while robot_status is None:
            robot_status = self.get_robot_status(robot_name)

        if robot_status is None:
            return False

        if robot_status['docked']:
            return True
        return False

    def navigate_robot(self, robot_encoding_id: str, time_stamp: float, content: NavigateContent):
        payload = {'operation_cmd': 'mode_point2nav',
                   'robot_encoding_id': robot_encoding_id,
                   'time_stamp': time_stamp,
                   'content': content.__dict__}
        navigate_message = json.dumps(payload)
        self.robot_status_ws_connection.send(navigate_message)

        return True

    def clean(self, robot_encoding_id: str, process: str, map_name: str, time_stamp: float):
        clean_process_content = self.build_clean_process_content(robot_encoding_id=robot_encoding_id,
                                                                 map_name=map_name,
                                                                 process=process)

        payload = {'operation_cmd': 'start_work',
                   'robot_encoding_id': robot_encoding_id,
                   'time_stamp': time_stamp,
                   'content': clean_process_content.__dict__}
        clean_message = json.dumps(payload)

        with self._lock:
            self.robot_operation[robot_encoding_id] = {
                'operation': 'start_work',
                'time_stamp': time_stamp,
                'status': None
            }
        self.robot_status_ws_connection.send(clean_message)

        return True

    def build_clean_process_content(self, robot_encoding_id: str, map_name: str, process: str):
        m = self.get_map(map_name=map_name, robot_name=robot_encoding_id)
        if m is None:
            return None

        map_id = m['id']
        map_level = m['level']

        zones = self.get_zones_by_map(map_id=map_id)
        while zones is None:
            zones = self.get_zones_by_map(map_id=map_id)
        filtered_zones = list(filter(lambda x: x['name'] == process, zones))

        process_details = filtered_zones[0]

        all_zone_equalizers = self.get_zone_equalizers(process_details['id'], robot_name=robot_encoding_id)
        while all_zone_equalizers is None:
            all_zone_equalizers = self.get_zone_equalizers(process_details['id'], robot_name=robot_encoding_id)

        selected_mode_id = all_zone_equalizers['selectedModeId']

        filtered_zone_equalizers = list(filter(lambda x: x['id'] == selected_mode_id, all_zone_equalizers['modes']))
        selected_zone_equalizers = filtered_zone_equalizers[0]

        configs = []
        for config in selected_zone_equalizers['configs']:
            configs.append(EqualizerConfig(config['configName'], config['value']).__dict__)

        zones = [Zone(area=process_details['area'],
                      configs=configs,
                      selected_mode_name=selected_zone_equalizers['name'],
                      zone_name=process_details['name'],
                      zone_id=process_details['id']).__dict__]

        clean_process_content = CleanProcessContent(
            robot_type='LeoScrub',
            working_type='point2clean',
            section_id='',
            section_name='',
            map_id=map_id,
            map_name=map_name,
            map_level=map_level,
            zones=zones,
            operator=self.user
        )

        return clean_process_content

    def stop_robot(self, robot_name: str, time_stamp: float, content: StopProcessContent):
        payload = {'operation_cmd': 'operation_interrupt',
                   'robot_encoding_id': robot_name,
                   'time_stamp': time_stamp,
                   'content': content.__dict__}
        stop_message = json.dumps(payload)
        print(f'stop robot payload: {stop_message}')
        self.robot_status_ws_connection.send(stop_message)

        return True

    def e_stop_robot(self, robot_name: str, time_stamp: float):
        payload = {'operation_cmd': 'mode_estop',
                   'robot_encoding_id': robot_name,
                   'time_stamp': time_stamp,
                   'content': {'status': 'true'}}
        estop_message = json.dumps(payload)
        print(f'estop robot payload: {estop_message}')
        self.robot_status_ws_connection.send(estop_message)

        return True

    def pause_robot(self, robot_name: str):
        time_stamp = time.time_ns() / 1000000

        robot_mission_status = self.get_mission_status(robot_name=robot_name)
        if robot_mission_status is None:
            return False

        operation = None
        if robot_mission_status['missionStatus']['activeMissionType'] == 'MOVING':
            operation = 2
        elif robot_mission_status['missionStatus']['activeMissionType'] == 'WORKING':
            operation = 1

        if operation is not None:
            pause_process_content = {'robot_type': 'LeoScrub',
                                     'operation': operation,
                                     'status': 1}

            payload = {'operation_cmd': 'operation_interrupt',
                       'robot_encoding_id': robot_name,
                       'time_stamp': time_stamp,
                       'content': pause_process_content}
            pause_message = json.dumps(payload)
            print(f'pause robot payload: {pause_message}')

            with self._lock:
                self.robot_operation[robot_name] = {
                    'operation': 'operation_interrupt',
                    'time_stamp': time_stamp,
                    'status': None
                }
            self.robot_status_ws_connection.send(pause_message)

            timeout = 5
            while True:
                robot_operation = self.robot_operation.get(robot_name, None)
                robot_status = self.get_robot_status(robot_name=robot_name)

                if robot_operation is None:
                    return False

                elif robot_status['status'] in RobotAPI.ROBOT_STATUS_PAUSED:
                    with self._lock:
                        self.robot_operation[robot_name] = None
                        return True

                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    with self._lock:
                        self.robot_operation[robot_name] = None
                    return False
        else:
            return False

    def resume_robot(self, robot_name: str):
        time_stamp = time.time_ns() / 1000000

        robot_mission_status = self.get_mission_status(robot_name=robot_name)
        if robot_mission_status is None:
            return False

        operation = None
        if robot_mission_status['missionStatus']['activeMissionType'] == 'MOVING':
            operation = 2
        elif robot_mission_status['missionStatus']['activeMissionType'] == 'WORKING':
            operation = 1

        if operation is not None:
            pause_process_content = {'robot_type': 'LeoScrub',
                                     'operation': operation,
                                     'status': 0}

            payload = {'operation_cmd': 'operation_interrupt',
                       'robot_encoding_id': robot_name,
                       'time_stamp': time_stamp,
                       'content': pause_process_content}
            pause_message = json.dumps(payload)
            print(f'pause robot payload: {pause_message}')

            with self._lock:
                self.robot_operation[robot_name] = {
                    'operation': 'operation_interrupt',
                    'time_stamp': time_stamp,
                    'status': None
                }
            self.robot_status_ws_connection.send(pause_message)

            timeout = 5
            while True:
                robot_operation = self.robot_operation.get(robot_name, None)
                robot_status = self.get_robot_status(robot_name=robot_name)

                if robot_operation is None:
                    return False

                elif robot_status['status'] in RobotAPI.ROBOT_STATUS_ACITVE:
                    with self._lock:
                        # if robot_operation['status']:
                        self.robot_operation[robot_name] = None
                        return True

                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    with self._lock:
                        self.robot_operation[robot_name] = None
                    return False
        else:
            return False

    def robot_cleaning(self, robot_name:str):
        robot_status = self.get_robot_status(robot_name=robot_name)
        if robot_status is None: 
            return False
        
        return robot_status['status'] == RobotStatus.CLEANING.value
    
    def robot_cleaning_paused(self, robot_name:str):
        robot_status = self.get_robot_status(robot_name=robot_name)
        if robot_status is None: 
            return False
        
        return robot_status['status'] == RobotStatus.CLEANING_PAUSED.value
    
    def dock_robot(self, robot_name: str, time_stamp: float, content: DockProcessContent):
        if self.get_robot_status(robot_name=robot_name)['docked']:
            return True

        payload = {'operation_cmd': 'mode_dock',
                   'robot_encoding_id': robot_name,
                   'time_stamp': time_stamp,
                   'content': content.__dict__}
        dock_message = json.dumps(payload)
        print(f'dock robot payload: {dock_message}')

        with self._lock:

            self.robot_operation[robot_name] = {
                'operation': 'mode_dock',
                'time_stamp': time_stamp,
                'status': None
            }
        self.robot_status_ws_connection.send(dock_message)

        timeout = 5
        while True:
            robot_operation = self.robot_operation.get(robot_name, None)

            if robot_operation is None:
                return False

            elif robot_operation['operation'] == 'mode_dock' and robot_operation['status'] is not None:
                with self._lock:
                    if robot_operation['status']:
                        self.robot_operation[robot_name] = None
                        return True

                    self.robot_operation[robot_name] = None
                    return False
            elif self.docking_completed(robot_name=robot_name):
                return True

            time.sleep(1)
            timeout -= 1
            if timeout == 0:
                with self._lock:
                    self.robot_operation[robot_name] = None
                return False

    def undock_robot(self, robot_name: str):
        self.refresh_expired_token()

        path = f'/openapi/v1/robot/command/undock/{robot_name}'
        headers = {'Authorization': f'Bearer {self.token}'}

        try:
            r = requests.put(f'https://{self.prefix}{path}', headers=headers)
            r.raise_for_status()
            data = r.json()

            if data['docked']:
                return True
            
            ''' 
                undock using the websocket is currently unavailable, currently use the undock api instead 
            '''
            with self._lock:
                self.robot_operation[robot_name] = {
                    'operation': 'mode_undock',
                    'time_stamp': time.time_ns() / 1000000,
                    'status': None
                }

            timeout = 5
            while True:
                robot_operation = self.robot_operation.get(robot_name, None)

                if robot_operation is None:
                    return False

                elif robot_operation['operation'] == 'mode_undock' and robot_operation['status'] is not None:
                    with self._lock:
                        if robot_operation['status']:
                            self.robot_operation[robot_name] = None
                            return True

                        self.robot_operation[robot_name] = None
                        return False
                elif self.undocking_completed(robot_name=robot_name):
                    return True

                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    with self._lock:
                        self.robot_operation[robot_name] = None
                    return False

        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return False

    def localize(self, position: LionsbotCoord, robot_name: str):
        self.refresh_expired_token()

        path = f'/openapi/v1/robot/command/hot-localize/{robot_name}'
        headers = {'Authorization': f'Bearer {self.token}'}

        payload = {'x': position.x, 'y': position.y, 'heading': position.orientation_radians}

        try:
            r = requests.post(f'https://{self.prefix}{path}', headers=headers, json=payload)
            r.raise_for_status()
            data = r.json()

            return data['localized']
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return False

    def change_map(self, robot_name: str, map_name: str):
        self.refresh_expired_token()

        m = self.get_map(map_name=map_name, robot_name=robot_name)
        if m is None:
            return False

        map_id = m['id']

        path = f'/openapi/v1/robot/{robot_name}/changemap/{map_id}?timestamp={time.time_ns() / 1000000}'
        headers = {'Authorization': f'Bearer {self.token}'}

        try:
            r = requests.put(f'https://{self.prefix}{path}', headers=headers)
            r.raise_for_status()

            timeout = 4
            while True:
                robot_maps = self.get_robot_maps(robot_name=robot_name)
                if robot_maps is None:
                    return False

                if robot_maps['currentWorksiteMapId'] == map_id:
                    return True

                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    return False
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return False
    
    # ------------------------------------------------------------------------------
    # Static Variables
    # ------------------------------------------------------------------------------
    TERMINAL_MISSION_STATUSES = {RobotMissionStatus.CLEANING_FINISHED.value,
                                RobotMissionStatus.MOVING_TO_WORK_STOPPED.value,
                                RobotMissionStatus.APP_CLEANING_STOPPED.value,
                                RobotMissionStatus.CLEANING_STOPPED.value,
                                RobotMissionStatus.ROBOT_EXITED.value,
                                RobotMissionStatus.APP_WAITING_FOR_LIFT_STOPPED.value,
                                RobotMissionStatus.APP_ENTERING_LIFT_STOPPED.value,
                                RobotMissionStatus.APP_IN_LIFT_STOPPED.value,
                                RobotMissionStatus.APP_EXITING_LIFT_STOPPED.value,
                                RobotMissionStatus.APP_EXITED_LIFT_FINISHED_STOPPED.value,
                                RobotMissionStatus.E_STOP_PRESSED.value,
                                RobotMissionStatus.IN_CRITICAL.value,
                                RobotMissionStatus.APP_STOPPED.value,
                                RobotMissionStatus.MOVING_STOPPED.value,
                                RobotMissionStatus.MOVING_FINISHED.value,
                                RobotMissionStatus.APP_MOVING_TO_DOCK_STOPPED.value,
                                RobotMissionStatus.ROBOT_MOVING_TO_DOCK_STOPPED.value,
                                RobotMissionStatus.APP_DOCKING_STOPPED.value}
    ROBOT_STATUS_ACITVE = {RobotStatus.CLEANING.value, RobotStatus.MOVING.value}
    ROBOT_STATUS_PAUSED = {RobotStatus.CLEANING_PAUSED.value, RobotStatus.MOVING_PAUSED.value}