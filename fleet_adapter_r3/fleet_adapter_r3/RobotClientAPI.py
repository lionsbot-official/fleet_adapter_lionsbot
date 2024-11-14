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
import websocket
import requests
from urllib.error import HTTPError
import numpy as np
import math
from typing import Dict

from .utils.Coordinate import LionsbotCoord
from .utils import constants

from .enums.enums import ActiveMissionType
from .enums.enums import NavigationStatus
from .enums.enums import RobotStatus
from .enums.enums import ResponseCode
from .enums.enums import RobotMissionStatus

from .models.NavigateContent import NavigateContent
from .models.CleanProcessContent import CleanProcessContent
from .models.DockProcessContent import DockProcessContent
import time

from .models.StopProcessContent import StopProcessContent
from .models.Zone import Zone
import jwt
import threading


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
        self.token_details = None
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
    # Static Variables
    # ------------------------------------------------------------------------------
    TERMINAL_MISSION_STATUSES = {RobotMissionStatus.CLEANING_FINISHED.value,
                                 RobotMissionStatus.MOVING_FINISHED.value,
                                 RobotMissionStatus.APP_STOPPED.value,
                                 RobotMissionStatus.MOVING_STOPPED.value,
                                 RobotMissionStatus.MOVING_FINISHED.value,
                                 RobotMissionStatus.E_STOP_PRESSED.value,
                                 RobotMissionStatus.IN_CRITICAL.value,
                                 RobotMissionStatus.DOCKED.value,
                                 RobotMissionStatus.APP_MOVING_TO_DOCK_STOPPED.value,
                                 RobotMissionStatus.APP_DOCKING_STOPPED.value,
                                 RobotMissionStatus.DOCKING_STOPPED.value,
                                 RobotMissionStatus.APP_MOVING_TO_WORK_STOPPED.value,
                                 RobotMissionStatus.MOVING_TO_WORK_STOPPED.value,
                                 RobotMissionStatus.APP_CLEANING_STOPPED.value,
                                 RobotMissionStatus.CLEANING_STOPPED.value}
    
    ROBOT_ACTIVE_STATUSES = {RobotStatus.CLEANING.value, 
                            RobotStatus.MOVING.value, 
                            RobotStatus.RESTING.value}
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
        path = constants.SECURITY_PATH
        payload = {'email': self.user, 'password': self.password, 'applicationName': 'DASHBOARD'}

        try:
            r = requests.post(f'https://{self.prefix}{path}', json=payload)
            r.raise_for_status()
            data = r.json()
            token = data['token']

            self.token = token

            decoded = jwt.decode(self.token, options={"verify_signature": False})
            self.token_details = decoded
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return None

    def refresh_expired_token(self):
        if self.token_details is None or self.token_details['exp'] <= time.time():
            self.request_token()

    def connect_to_robot_status_ws(self):
        self.refresh_expired_token()

        def on_message(wsc, message):
            json_message = json.loads(message)

            with self._lock:
                if json_message['operation_fb'] == 'touchscreen_robot_status':
                    robot_status = json_message['content']
                    self.robot_status[json_message['robot_id']] = {
                        'eta': robot_status['timeToComplete'],
                        'alertIds': robot_status['alertIds'],
                        'docked': robot_status['docked'],
                        'progress': robot_status['missionProgress'],
                        'localized': robot_status['localised'],
                        'batterySoc': robot_status['batteryInfo']['soc'],
                        'state': robot_status['state'],
                        'status': robot_status['status']
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
            print("### Status Websocket Connecton Closed ###")

        def on_open(wsc):
            print("Opened Status Websocket Connection")

        path = f'{constants.WS_OPEN_API_PREFIX}/robotstatus/v3.1'
        subprotocols = [self.token]

        self.robot_status_ws_connection = websocket.WebSocketApp(f'wss://{self.prefix}{path}',
                                                                 subprotocols=subprotocols,
                                                                 on_open=on_open,
                                                                 on_close=on_close,
                                                                 on_error=on_error,
                                                                 on_message=on_message)
        self.robot_status_ws_connection.run_forever()

    def connect_to_robot_position_ws(self):
        self.refresh_expired_token()

        def on_message(wsc, message):
            json_message = json.loads(message)
            with self._lock:
                if json_message['operation_fb'] == 'robot_pose':
                    robot_pose = json_message['content']
                    self.robot_pose[json_message['robot_id']] = LionsbotCoord(x=robot_pose['x'],
                                                                y=robot_pose['y'],
                                                                orientation_radians=math.radians(robot_pose['heading']))

        def on_error(wsc, error):
            print(error)

        def on_close(wsc, close_status_code, close_msg):
            print("### Position Websocket Connecton Closed ###")

        def on_open(wsc):
            print("Opened Position Websocket Connection")

        path = f'{constants.WS_OPEN_API_PREFIX}/robotpose'
        subprotocols = [self.token]

        self.robot_pose_ws_connection = websocket.WebSocketApp(f'wss://{self.prefix}{path}',
                                                               subprotocols=subprotocols,
                                                               on_open=on_open,
                                                               on_close=on_close,
                                                               on_error=on_error,
                                                               on_message=on_message)
        self.robot_pose_ws_connection.run_forever()

    def subscribe_to_robot(self, robot_encoding_id: str, time_stamp: float):
        payload = {'operation_cmd': 'subscribe', 'robot_encoding_id': robot_encoding_id, 'time_stamp': time_stamp}
        subscribe_status_message = json.dumps(payload)
        print(f'Status subscribe payload: {subscribe_status_message}')

        payload = {'operation_cmd': 'subscribe', 'robot_encoding_id': robot_encoding_id}
        subscribe_pose_message = json.dumps(payload)
        print(f'Position subscribe payload: {subscribe_pose_message}')

        self.robot_status_ws_connection.send(subscribe_status_message)
        self.robot_pose_ws_connection.send(subscribe_pose_message)

        time.sleep(2.5)

        return True

    # ------------------------------------------------------------------------------
    # Robot Information Accessors
    # ------------------------------------------------------------------------------
    def position(self, robot_name: str) -> LionsbotCoord:
        ''' Return Coordinate:LionsbotCoord expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        position = self.robot_pose.get(robot_name, None)

        if position is None:
            self.refresh_expired_token()

            path = f'{constants.OPEN_API_PREFIX}/robot/{robot_name}/position'
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
                
                position = LionsbotCoord(x, y, orientation_radians)
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

            path = f'{constants.OPEN_API_PREFIX}/robot/{robot_name}/mission-status'
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

        path = f'{constants.OPEN_API_PREFIX}/robot/{robot_name}'
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

            path = f'{constants.OPEN_API_PREFIX}/robot/{robot_name}/status'
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
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        self.refresh_expired_token()

        mission_status = self.get_mission_status(robot_name=robot_name)
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

        return robot_status['batterySoc'] / 100

    def get_robot_maps(self, robot_name: str):
        self.refresh_expired_token()

        path = f'{constants.OPEN_API_PREFIX}/worksite/robot/maps/{robot_name}'
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

        path = f'{constants.OPEN_API_PREFIX}/worksite/robot/maps/{robot_name}'
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

    def get_zones_by_map(self, map_id: str):
        self.refresh_expired_token()

        path = f'{constants.OPEN_API_PREFIX}/worksitemap/{map_id}/zone'
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

        path = f'{constants.OPEN_API_PREFIX}/worksitemap/section/zone/equalizer/{process_id}/{robot_name}'
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

    # ------------------------------------------------------------------------------
    # Robot Operations
    # ------------------------------------------------------------------------------
    def navigate(self, robot_name: str, pose: LionsbotCoord, map_name: str):
        ''' Request the robot to navigate to pose:LionsbotCoord 
            Return True if the robot has accepted the request, else False'''
        self.refresh_expired_token()

        navigate_content = NavigateContent(
            heading_radians=pose.orientation_radians,
            x=pose.x,
            y=pose.y,
            waypoint='Custom Move Point',
            waypoint_id=''
        )

        self.navigate_robot(robot_encoding_id=robot_name, time_stamp=time.time_ns() / 1000000, content=navigate_content)

        time_window_seconds = 5
        while time_window_seconds > 0:
            robot_mission_status = self.get_mission_status(robot_name=robot_name)
            if robot_mission_status is None:
                return False
            
            robot_mission_details = robot_mission_status['missionStatus']['mission'] 
            mission_status = robot_mission_details['status']
            
            x = robot_mission_details.get('x', pose.x)
            y = robot_mission_details.get('y', pose.y)

            # Ensure mission status is from the mission moving to the current waypoint
            # Margin of error round to 1 between rmf waypoint coordinates and robot mission status coordinates
            # as there are times mission status feedback rounds of position to whole number
            is_correct_mission = int(x) == int(pose.x) and int(y) == int(pose.y) 

            curr_x = self.robot_pose.get(robot_name).x
            curr_y = self.robot_pose.get(robot_name).y
            is_robot_within_target_coord = abs(x - curr_x) <= 10 and abs(y - curr_y) <= 10

            # When coordinate is too near, possible to receive MOVING -> MOVING_FINISHED in quick succession. In this case, 
            # we take navigation to be successful 
            # if robot encountered error but robot is already near target position, treat as success as robot will not send
            # P2P_STARTED anymore when position is too close to target
            if (mission_status == RobotMissionStatus.MOVING.value or mission_status == RobotMissionStatus.MOVING_FINISHED.value) \
                and is_correct_mission and (ResponseCode.P2P_STARTED in robot_mission_status['alertIds'] or is_robot_within_target_coord):
                self.robot_current_map[robot_name] = map_name
                return True
            
            time.sleep(1)
            time_window_seconds -= 1

        return False

    def navigate_robot(self, robot_encoding_id: str, time_stamp: float, content: NavigateContent):
        payload = {'operation_cmd': 'p2p_start',
                   'robot_encoding_id': robot_encoding_id,
                   'time_stamp': time_stamp,
                   'content': content.__dict__}
        navigate_message = json.dumps(payload)
        self.robot_status_ws_connection.send(navigate_message)

        return True

    def navigation_completed(self, robot_name: str) -> NavigationStatus:
        self.refresh_expired_token()
        
        with self._lock:
            mission_status = self.get_mission_status(robot_name=robot_name)
            robot_status = self.get_robot_status(robot_name=robot_name)

        if mission_status is None or robot_status is None:
            return NavigationStatus.EMPTY
        
        robot_mission_status = mission_status['missionStatus']['mission']['status']

        if robot_mission_status == RobotMissionStatus.MOVING_FINISHED.value:
            # 5 seconds to wait for any error code
            # This is necessary as mission status and response codes are from two separate message payloads
            retries = 5
            while retries > 0:
                # within this time window, if any error code comes in, navigation was not completed
                if ResponseCode.P2P_STARTED not in mission_status['alertIds']:
                    return NavigationStatus.NAVIGATION_ERROR
                
                time.sleep(1)
                retries -= 1

            # P2P_STARTED indicates moving started and finished successfully without any errors in between
            return NavigationStatus.NAVIGATION_SUCCESS
                
        else:
            return NavigationStatus.NAVIGATING if robot_status['status'] == RobotStatus.MOVING.value \
            else NavigationStatus.NAVIGATION_ERROR

    def start_process(self, robot_name: str, process: str, map_name: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''
        self.refresh_expired_token()

        self.clean(robot_encoding_id=robot_name,
                   clean_zone_name=process,
                   map_name=map_name,
                   time_stamp=time.time_ns() / 1000000)

        timeout = 5
        while True:
            robot_mission_status = self.get_mission_status(robot_name=robot_name)
            if robot_mission_status is None:
                return False

            if robot_mission_status['missionStatus']['mission']['status'] == RobotMissionStatus.CLEANING.value:
                return True
            else:
                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    return False

    def clean(self, robot_encoding_id: str, clean_zone_name: str, map_name: str, time_stamp: float):
        clean_process_content = self.build_clean_process_content(robot_encoding_id=robot_encoding_id,
                                                                 map_name=map_name,
                                                                 clean_zone_name=clean_zone_name)

        payload = {'operation_cmd': 'clean_start',
                   'robot_encoding_id': robot_encoding_id,
                   'time_stamp': time_stamp,
                   'content': clean_process_content.__dict__}
        clean_message = json.dumps(payload)
        self.robot_status_ws_connection.send(clean_message)

        return True

    def build_clean_process_content(self, robot_encoding_id: str, map_name: str, clean_zone_name: str):
        robot_info = self.get_robot_info(robot_encoding_id)
        robot_type = robot_info['robotType']

        map_data = self.get_map(map_name=map_name, robot_name=robot_encoding_id)
        if map_data is None:
            return None

        map_id = map_data['id']
        map_level = map_data['level']
        map_zones = self.get_zones_by_map(map_id=map_id)

        while map_zones is None:
            map_zones = self.get_zones_by_map(map_id=map_id)
        
        filtered_zones = list(filter(lambda x: x['name'] == clean_zone_name, map_zones))

        clean_zone = filtered_zones[0]

        all_zone_equalizers = self.get_zone_equalizers(clean_zone['id'], robot_name=robot_encoding_id)
        while all_zone_equalizers is None:
            all_zone_equalizers = self.get_zone_equalizers(clean_zone['id'], robot_name=robot_encoding_id)

        selected_mode_id = all_zone_equalizers['selectedModeId']

        filtered_zone_equalizers = list(filter(lambda x: x['id'] == selected_mode_id, all_zone_equalizers['modes']))
        selected_zone_equalizers = filtered_zone_equalizers[0]

        configs = {}
        for config in selected_zone_equalizers['configs']:
            configs[config['configName']] = config['value']

        zones = [Zone(area=clean_zone['area'],
                      configs=configs,
                      selected_mode_name=selected_zone_equalizers['name'],
                      zone_name=clean_zone['name'],
                      zone_id=clean_zone['id']).__dict__]

        clean_process_content = CleanProcessContent(
            mode=0,
            robot_type=robot_type,
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

    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        self.refresh_expired_token()

        robot_status = self.get_robot_status(robot_name=robot_name)
        if robot_status['docked'] or robot_status['status'] == RobotStatus.RESTING.value:
            return True

        stop_process_content = StopProcessContent(
            status='true'
        )

        self.stop_robot_moving(robot_name=robot_name, time_stamp=time.time_ns() / 1000000,
                               content=stop_process_content)
        self.stop_robot_cleaning(robot_name=robot_name, time_stamp=time.time_ns() / 1000000,
                                 content=stop_process_content)
        self.stop_robot_docking(robot_name=robot_name, time_stamp=time.time_ns() / 1000000,
                                content=stop_process_content)

        time.sleep(0.5)

        timeout = 5
        while True:
            robot_mission_status = self.get_mission_status(robot_name=robot_name)
            if robot_mission_status is None:
                return False

            if robot_mission_status['missionStatus']['activeMissionType'] == ActiveMissionType.IDLE.value or \
                robot_mission_status['missionStatus']['mission']['status'] in RobotAPI.TERMINAL_MISSION_STATUSES:
                return True
            else:
                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    self.robot_mission[robot_name] = None
                    return False

    def stop_robot_moving(self, robot_name: str, time_stamp: float, content: StopProcessContent):
        payload = {'operation_cmd': 'p2p_stop',
                   'robot_encoding_id': robot_name,
                   'time_stamp': time_stamp,
                   'content': content.__dict__}
        stop_message = json.dumps(payload)
        print(f'stop robot moving payload: {stop_message}')
        self.robot_status_ws_connection.send(stop_message)

        return True

    def stop_robot_cleaning(self, robot_name: str, time_stamp: float, content: StopProcessContent):
        payload = {'operation_cmd': 'clean_stop',
                   'robot_encoding_id': robot_name,
                   'time_stamp': time_stamp,
                   'content': content.__dict__}
        stop_message = json.dumps(payload)
        print(f'stop robot cleaning payload: {stop_message}')
        self.robot_status_ws_connection.send(stop_message)

        return True

    def stop_robot_docking(self, robot_name: str, time_stamp: float, content: StopProcessContent):
        payload = {'operation_cmd': 'dock_stop',
                   'robot_encoding_id': robot_name,
                   'time_stamp': time_stamp,
                   'content': content.__dict__}
        stop_message = json.dumps(payload)
        print(f'stop robot docking payload: {stop_message}')
        self.robot_status_ws_connection.send(stop_message)

        return True

    def process_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed cleaning. Else False.'''
        self.refresh_expired_token()

        mission_status = self.get_mission_status(robot_name)

        if mission_status is None:
            return False

        if mission_status['missionStatus']['mission']['status'] == RobotMissionStatus.CLEANING_FINISHED.value :
            return True

        return False

    def undocking_completed(self, robot_name: str):
        self.refresh_expired_token()

        robot_status = self.get_robot_status(robot_name)
        while robot_status is None:
            robot_status = self.get_robot_status(robot_name)

        if robot_status is None:
            return False

        if not robot_status['docked'] or robot_status['status'] == RobotStatus.RESTING.value:
            return True
        return False

    def docking_completed(self, robot_name: str):
        self.refresh_expired_token()

        robot_status = self.get_robot_status(robot_name)
        while robot_status is None:
            robot_status = self.get_robot_status(robot_name)

        if robot_status is None:
            return False
        
        if robot_status['docked'] or robot_status['status'] == RobotStatus.DOCKED.value:
            return True
        return False

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

        payload = {
            'operation_cmd': None,
            'robot_encoding_id': robot_name,
            'time_stamp': time_stamp,
            'content': {'status': True}
        }

        robot_status = self.get_robot_status(robot_name=robot_name)

        operation_cmd = None

        if robot_status['status'] == RobotStatus.MOVING.value:
            operation_cmd = 'p2p_pause'
        elif robot_status['status'] == RobotStatus.CLEANING.value:
            operation_cmd = 'clean_pause'
        else:
            return False

        payload['operation_cmd'] = operation_cmd
        pause_message = json.dumps(payload)
        print(f'pause robot payload: {pause_message}')

        with self._lock:
            self.robot_operation[robot_name] = {
                'operation': operation_cmd,
                'time_stamp': time_stamp,
                'status': None
            }
        self.robot_status_ws_connection.send(pause_message)

        timeout = 5
        while True:
            robot_operation = self.robot_operation.get(robot_name, None)

            if robot_operation is None:
                return False
            elif self.robot_cleaning_paused(robot_name) or self.robot_moving_paused(robot_name):
                with self._lock:
                    self.robot_operation[robot_name] = None
                    return True

            time.sleep(1)
            timeout -= 1
            if timeout == 0:
                with self._lock:
                    self.robot_operation[robot_name] = None
                return False

    def resume_robot(self, robot_name: str):
        time_stamp = time.time_ns() / 1000000

        payload = {
            'operation_cmd': None,
            'robot_encoding_id': robot_name,
            'time_stamp': time_stamp,
            'content': {'status': True}
        }

        operation_cmd = None

        if self.robot_moving_paused(robot_name):
            operation_cmd = 'p2p_continue'
        elif self.robot_cleaning_paused(robot_name=robot_name):
            operation_cmd = 'clean_continue'
        else:
            return False

        payload['operation_cmd'] = operation_cmd
        resume_message = json.dumps(payload)
        print(f'resume robot payload: {resume_message}')

        with self._lock:
            self.robot_operation[robot_name] = {
                'operation': operation_cmd,
                'time_stamp': time_stamp,
                'status': None
            }
        self.robot_status_ws_connection.send(resume_message)

        timeout = 5
        while True:
            robot_operation = self.robot_operation.get(robot_name, None)
            robot_status = self.get_robot_status(robot_name=robot_name)

            if robot_operation is None:
                return False

            elif robot_status['status'] in RobotAPI.ROBOT_ACTIVE_STATUSES:
                with self._lock:
                    self.robot_operation[robot_name] = None
                    return True

            time.sleep(1)
            timeout -= 1
            if timeout == 0:
                with self._lock:
                    self.robot_operation[robot_name] = None
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

    def robot_moving_paused(self, robot_name:str):
        robot_status = self.get_robot_status(robot_name=robot_name)
        if robot_status is None: 
            return False
        
        return robot_status['status'] == RobotStatus.MOVING_PAUSED.value   

    def dock_robot(self, robot_name: str, time_stamp: float, content: DockProcessContent):
        if self.get_robot_status(robot_name=robot_name)['docked']:
            return True

        payload = {'operation_cmd': 'dock_start',
                   'robot_encoding_id': robot_name,
                   'time_stamp': time_stamp,
                   'content': content.__dict__}
        dock_message = json.dumps(payload)
        print(f'dock robot payload: {dock_message}')

        with self._lock:
            self.robot_operation[robot_name] = {
                'operation': 'dock_start',
                'time_stamp': time_stamp,
                'status': None
            }
            
        self.robot_status_ws_connection.send(dock_message)

        timeout = 5
        while True:
            robot_operation = self.robot_operation.get(robot_name, None)

            if robot_operation is None:
                return False
            elif robot_operation['operation'] == 'dock_start' and robot_operation['status'] is not None:
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

    def undock_robot(self, robot_name: str, time_stamp: float):
        payload = {'operation_cmd': 'undock_start',
                   'robot_encoding_id': robot_name,
                   'time_stamp': time_stamp,
                   'content': {'status': 'true'}}
        dock_message = json.dumps(payload)
        print(f'undock robot payload: {dock_message}')

        with self._lock:
            self.robot_operation[robot_name] = {
                'operation': 'undock_start',
                'time_stamp': time_stamp,
                'status': None
            }
        self.robot_status_ws_connection.send(dock_message)

        timeout = 5
        while True:
            robot_operation = self.robot_operation.get(robot_name, None)

            if robot_operation is None:
                return False

            elif robot_operation['operation'] == 'undock_start' and robot_operation['status'] is not None:
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

    def stop_docking(self, robot_name: str, time_stamp: float):
        payload = {'operation_cmd': 'dock_stop',
                   'robot_encoding_id': robot_name,
                   'time_stamp': time_stamp,
                   'content': {'status': 'true'}}
        stop_dock_message = json.dumps(payload)
        print(f'stop docking payload: {stop_dock_message}')

        with self._lock:
            self.robot_operation[robot_name] = {
                'operation': 'dock_stop',
                'time_stamp': time_stamp,
                'status': None
            }
        self.robot_status_ws_connection.send(stop_dock_message)

        timeout = 5
        while True:
            robot_operation = self.robot_operation.get(robot_name, None)

            if robot_operation is None:
                return False

            elif robot_operation['operation'] == 'dock_stop' and robot_operation['status'] is not None:
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

    def localize(self, position: LionsbotCoord, robot_name: str) -> bool:
        self.refresh_expired_token()

        path = f'{constants.OPEN_API_PREFIX}/robot/command/hot-localize/{robot_name}'
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

        map_data = self.get_map(map_name=map_name, robot_name=robot_name)
        if map_data is None:
            return False

        map_id = map_data['id']

        path = f'{constants.OPEN_API_PREFIX}/robot/{robot_name}/changemap/{map_id}?timestamp={time.time_ns() / 1000000}'
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
