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

import jwt
import json
import math
import time
import requests
import websocket

from threading import Thread
from typing import Dict, Any, Callable, Optional

from .models import EqualizerConfig
from .models import NavigateContent
from .models import CleanProcessContent
from .models import DockProcessContent
from .models import StopProcessContent
from .models import Zone

from urllib.error import HTTPError

class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, user: str, password: str, robot_id: str):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.connected = False
        self.online = False
        self.robot_id = robot_id

        # Authentication
        self.token = None
        self.token_details = None

        # Test connectivity
        self.robot_status_websocket = None
        self.robot_status_websocket_thread = None
        self.robot_status_websocket_connected = False
        self.start_websockets()
        connected = self.check_connection()

        if connected:
            print("Successfully able to query API server")
        else:
            print("Unable to query API server")

    def get_time_stamp(self):
        return time.time_ns() /1000000

    def start_websockets(self):
        self.connect_and_subscribe()

    def check_connection(self):
        ''' Return True if connection to the robot API server is successful
            and websockets are connected
        '''
        return self.check_online() and self.robot_status_websocket_connected


    def connect_to_status_ws(self):
        self.refresh_expired_token()

        def on_message(wsc, message):
            return
        def on_error(wsc, error):
            print(error)

        def on_close(wsc, close_status_code, close_msg):
            self.robot_status_websocket_connected = False
            print(f"### {self.robot_id} status websocket closed! ###")
            reconnect_thread = Thread(target = self.connect_and_subscribe, daemon=True)
            print(f"### {self.robot_id} Starting recconnect to status websocket thread on close! ###")
            reconnect_thread.start()

        def on_open(wsc):
            self.robot_status_websocket_connected = True
            print(f"### {self.robot_id} status websocket opened connection")

        path = '/ws/openapi/robotstatus/v3.1'
        subprotocols = [self.token]

        self.robot_status_websocket = websocket.WebSocketApp(f'wss://{self.prefix}{path}',
                                                                subprotocols=subprotocols,
                                                                on_open=on_open,
                                                                on_close=on_close,
                                                                on_error=on_error,
                                                                on_message=on_message)
        self.robot_status_websocket.run_forever()
        return

    def connect_and_subscribe(self):
        subscribe_payload =subscribe_payload = {'operation_cmd': 'subscribe', 'robot_encoding_id': self.robot_id, 'time_stamp': self.get_time_stamp()}
        self.connect_websocket()
        try:
            self.robot_status_websocket.send(json.dumps(subscribe_payload))
        except Exception as e:
            print(f"Exception trying to subscribe to robot {self.robot_id} with payload {subscribe_payload}")

    def connect_websocket(self):
        if self.robot_status_websocket_connected:
            return
        if self.robot_status_websocket_thread is not None:
            try:
                self.robot_status_websocket_thread.join()
            except Exception as e:
                print(f"{e}")
            self.robot_status_websocket_thread = None
        self.robot_status_websocket_thread = Thread(target=self.connect_to_status_ws, daemon=True)
        self.robot_status_websocket_thread.start()
        while not self.robot_status_websocket_connected:
            print(f"Waiting to connect to websocket for robot {self.robot_id}")
            time.sleep(0.5)

    def check_online(self):
        data = self.robot()
        if data is not None:
            return data.get('state', False)
        return False

    def request_token(self) -> bool:
        path = '/security/auth'
        payload = {'email': self.user, 'password': self.password, 'applicationName': 'DASHBOARD'}

        try:
            r = requests.post(f'https://{self.prefix}{path}', json=payload)
            r.raise_for_status()
            data = r.json()
            token = data['token']

            self.token = token

            decoded = jwt.decode(self.token, options={"verify_signature": False})
            self.token_details = decoded
            return True
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        return False

    def refresh_expired_token(self):
        if self.token_details is None or self.token_details['exp'] <= time.time():
            self.request_token()

    def robot(self) -> Dict[str, Any]:
        self.request_token()
        path = f'/openapi/v1/robot/{self.robot_id}/position'
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

    def status(self):
        self.refresh_expired_token()

        path = f'/openapi/v1/robot/{self.robot_id}/status'
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

    def mission_status(self):
        self.refresh_expired_token()

        path = f'/openapi/v1/robot/{self.robot_id}/mission-status'
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

    def is_charging(self) -> bool:
        status = self.status()
        if status is None:
            return False
        if status['status'] == 'Charging':
            return True
        return False

    def position(self):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        self.refresh_expired_token()

        path = f'/openapi/v1/robot/{self.robot_id}/position'
        headers = {'Authorization': f'Bearer {self.token}'}

        try:
            r = requests.get(f'https://{self.prefix}{path}', headers=headers)
            r.raise_for_status()
            data = r.json()

            x = data['x']
            y = data['y']
            theta = math.radians(data['heading'])

            if x is None or y is None or theta is None:
                return None

            return [x, y, theta]
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return None


    def navigate(self, pose):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
        self.refresh_expired_token()

        navigate_content = NavigateContent(
            heading=pose[2],
            x=pose[0],
            y=pose[1],
            waypoint='Custom',
            waypoint_id=''
        )
        payload = {'operation_cmd': 'mode_point2nav',
                   'robot_encoding_id': self.robot_id,
                   'time_stamp': self.get_time_stamp(),
                   'content': navigate_content.__dict__}
        try:
            self.robot_status_websocket.send(json.dumps(payload))
            return True
        except Exception as e:
            print(f"Exception sending Navigate command to robot {self.robot_id}: {e}")
        return False

    def is_navigating(self):
        """
        Checks if the robot is currently on a navigate mission
        it can be that it is but is paused
        """
        status = self.status()
        if status is None:
            return False
        if status['status'] == 'MOVING':
            return True
        mission_status = self.mission_status()
        if mission_status is None:
            return False
        if mission_status['missionStatus']['activeMissionType'] == 'MOVING':
            return True
        # The robot's next target waypoint may be too close. We return True in this
        # case so that the state machine would change to the MOVING state where
        # navigation_complete() would return True immediately.
        if mission_status['missionStatus']['mission']['status'] == 'CMD_REJECTED':
            return True
        return False

    def is_app_started(self, count: int=1, sleep: float=1.0) -> bool:
        """
        This function checks if the mission status stays at APP_STARTED for [count] number
        of times and sleeps for [sleep] seconds every iteration. If not it returns False immediately.
        """
        for i in range(count):
            mission_status = self.mission_status()
            if mission_status['missionStatus']['activeMissionType'] == 'MOVING':
                mission = mission_status['missionStatus'].get('mission', None)
                if mission is not None:
                    if mission.get('status', None) != 'APP_STARTED':
                        return False
            time.sleep(sleep)

        return True

    def start_process(self, process: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''
        clean_process_content = self.build_clean_process_content(process=process)

        payload = {'operation_cmd': 'start_work',
                   'robot_encoding_id': self.robot_id,
                   'time_stamp': self.get_time_stamp(),
                   'content': clean_process_content.__dict__}
        clean_message = json.dumps(payload)

        try:
            self.robot_status_websocket.send(clean_message)
            return True
        except Exception as e:
            print(f"Exception sending Clean command to robot {self.robot_id}: {e}")
        return False

    def get_clean_progress(self, process: str) -> float:
        mission_status = self.mission_status()
        if mission_status is None:
            return 0.0
        # zone is null when the robot is estopped during a clean task so this would return 0.0
        if 'zone' in mission_status['missionStatus']['mission']:
            if mission_status['missionStatus']['mission']['zone'] is not None:
                if mission_status['missionStatus']['mission']['zone'].get('name') == process:
                    return mission_status['missionStatus']['mission']['zone'].get('progress', 0.0)
        return 0.0

    def is_cleaning(self) -> bool:
        mission_status = self.mission_status()
        if mission_status is None:
            return False
        if mission_status['missionStatus']['activeMissionType'] == 'WORKING':
            return True
        return False

    def is_estopped(self) -> bool:
        status = self.status()
        if status is None:
            return False
        alert_ids = status.get('alertIds', None)
        if 2202 in alert_ids or 2203 in alert_ids or 2206 in alert_ids:
            return True
        return False

    def is_in_critical(self) -> bool:
        status = self.status()
        if status is None:
            return False
        if status.get('status') == 'Critical':
            return True
        return False

    def stop(self):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        self.refresh_expired_token()

        robot_status = self.status()
        if robot_status is None:
            return False

        if robot_status['status'] in ['Resting', 'Sleeping']:
            return True

        robot_mission_status = self.mission_status()
        if robot_mission_status is None:
            return False

        # Not exactly sure why to call e stop when status is APP_STARTED
        if robot_mission_status['missionStatus']['mission']['status'] == 'APP_STARTED':
            self.e_stop()
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

                self._stop(content=stop_process_content)
            else:
                return self.e_stop()


    def _stop(self, content: StopProcessContent):
        payload = {'operation_cmd': 'operation_interrupt',
                   'robot_encoding_id': self.robot_id,
                   'time_stamp': self.get_time_stamp(),
                   'content': content.__dict__}
        stop_message = json.dumps(payload)
        print(f'stop robot payload: {stop_message}')
        try:
            self.robot_status_websocket.send(stop_message)
            return True
        except Exception as e:
            print(f"Exception trying to stop robot {self.robot_id}: {e}")

        return True

    def e_stop(self):
        payload = {'operation_cmd': 'mode_estop',
                   'robot_encoding_id': self.robot_id,
                   'time_stamp': self.get_time_stamp(),
                   'content': {'status': 'true'}}
        estop_message = json.dumps(payload)
        print(f'estop robot payload: {estop_message}')
        try:
            self.robot_status_websocket.send(estop_message)
        except Exception as e:
            print(f"Exception trying to E stop robot {self.robot_id} : {e}")
        return True

    def pause(self):
        time_stamp = self.get_time_stamp()

        robot_mission_status = self.mission_status()
        if robot_mission_status is None:
            return False

        operation = None
        if robot_mission_status['missionStatus']['activeMissionType'] == 'MOVING':
            operation = 2
        elif robot_mission_status['missionStatus']['activeMissionType'] == 'WORKING':
            operation = 1

        if operation is None:
            return False

        pause_process_content = {'robot_type': 'LeoScrub',
                                    'operation': operation,
                                    'status': 1}

        payload = {'operation_cmd': 'operation_interrupt',
                    'robot_encoding_id': self.robot_id,
                    'time_stamp': time_stamp,
                    'content': pause_process_content}
        pause_message = json.dumps(payload)
        print(f'pause robot payload: {pause_message}')
        try:
            self.robot_status_websocket.send(pause_message)
            return True
        except Exception as e:
            print(f"Exception trying to pause robot {self.robot_id}: {e}")
        return False

    def resume(self):
        time_stamp = self.get_time_stamp()

        robot_mission_status = self.mission_status()
        if robot_mission_status is None:
            return False

        operation = None
        if robot_mission_status['missionStatus']['activeMissionType'] == 'MOVING':
            operation = 2
        elif robot_mission_status['missionStatus']['activeMissionType'] == 'WORKING':
            operation = 1

        if operation is None:
            return False

        pause_process_content = {'robot_type': 'LeoScrub',
                                    'operation': operation,
                                    'status': 0}

        payload = {'operation_cmd': 'operation_interrupt',
                    'robot_encoding_id': self.robot_id,
                    'time_stamp': time_stamp,
                    'content': pause_process_content}
        pause_message = json.dumps(payload)
        print(f'pause robot payload: {pause_message}')

        try:
            self.robot_status_websocket.send(pause_message)
            return True
        except Exception as e:
            print(f"Exception trying to resume robot {self.robot_id}: {e}")
        return False

    def navigation_remaining_duration(self):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        mission_status = self.mission_status()
        if mission_status is not None:
            if mission_status['missionStatus']['mission']['status'] == 'ROBOT_MOVING':
                return mission_status['eta']
        return 0.0

    def navigation_completed(self):
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''
        mission_status = self.mission_status()
        if mission_status is not None:
            if mission_status['missionStatus']['mission']['status'] == 'CMD_REJECTED':
                return True
            if mission_status['missionStatus']['mission']['status'] == 'ROBOT_MOVING_FINISHED':
                return True
        return False

    def process_completed(self):
        ''' Return True if the robot has successfully completed its previous
            process request. Else False.'''
        status = self.status()
        if status is not None:
            if status['status'] in ['Resting', 'Sleeping']:
                return True
        return False

    def process_remaining_duration(self):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        mission_status = self.mission_status()
        if mission_status is not None:
            if mission_status['missionStatus']['activeMissionType'] == 'WORKING':
                return mission_status['eta']
        return 0.0

    def navigation_remaining_duration(self):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        mission_status = self.mission_status()
        if mission_status is not None:
            if mission_status['missionStatus']['mission']['status'] == 'ROBOT_MOVING':
                return mission_status['eta']
        return 0.0

    def battery_soc(self):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
        self.refresh_expired_token()

        robot_status = self.status()
        if robot_status is not None:
            return robot_status['batterySoc'] / 100
        return None

    def build_clean_process_content(self, process: str):
        map_id = self.current_worksite_id()

        zones = self.get_zones_by_map(map_id=map_id)
        while zones is None:
            zones = self.get_zones_by_map(map_id=map_id)
        filtered_zones = list(filter(lambda x: x['name'] == process, zones))

        process_details = filtered_zones[0]

        all_zone_equalizers = self.get_zone_equalizers(process_details['id'])
        while all_zone_equalizers is None:
            all_zone_equalizers = self.get_zone_equalizers(process_details['id'])

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
            map_name='map',
            map_level='L1',
            zones=zones,
            operator='bruce'
        )

        return clean_process_content


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

    def is_localized(self) -> bool:
        status = self.status()
        if status is None:
            return False
        return status['localized']

    def hot_localize(self, x: int, y: int, theta: float):
        self.refresh_expired_token()

        path = f'/openapi/v1/robot/command/hot-localize/{self.robot_id}'
        headers = {'Authorization': f'Bearer {self.token}'}

        payload = {'x': x, 'y': y, 'heading': theta}

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

    def is_docked(self) -> bool:
        '''
        Returns a boolean indicating if the robot is docked or not.
        If the robot status is None this returns False
        '''

        # NOTE that if the robot is offline. This returns False by default.
        status = self.status()
        if status is not None:
            return status['docked']
        return False

    def dock(self, dock_name: str):
        if self.is_docked():
            return True

        dock_info = self.get_dock_info(dock_name)
        xy = []
        xy_string = dock_info['coordinates']
        for coord in xy_string.split(' '):
            xy.append(int(coord))
        dock_content = DockProcessContent(x=xy[0], y=xy[1],theta=dock_info['angle'],
                                          dock_name=dock_name)


        payload = {'operation_cmd': 'mode_dock',
                   'robot_encoding_id': self.robot_id,
                   'time_stamp': self.get_time_stamp(),
                   'content': dock_content.__dict__}
        dock_message = json.dumps(payload)
        print(f'dock robot payload: {dock_message}')

        try:
            self.robot_status_websocket.send(dock_message)
            return True
        except Exception as e:
            print(f'Exception when trying to dock robot {self.robot_id}: {e}')
        return False


    def undock(self):
        '''
        Commands the robot to undock. This method does not do a close loop observation
        to check if the robot has actually undocked.
        '''
        self.refresh_expired_token()

        path = f'/openapi/v1/robot/command/undock/{self.robot_id}'
        headers = {'Authorization': f'Bearer {self.token}'}

        try:
            r = requests.put(f'https://{self.prefix}{path}', headers=headers)
            r.raise_for_status()
            data = r.json()
            if data['docked']:
                return True
            return False

        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return False

    # MAP OPERATIONS
    def get_robot_maps(self) -> Dict[str, Any]:
        self.refresh_expired_token()

        path = f'/openapi/v1/worksite/robot/maps/{self.robot_id}'
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

    def current_worksite_id(self) -> str | None:
        '''
        return current worksite id
        '''
        map_data = self.get_robot_maps()
        if map_data is not None:
            return map_data.get('currentWorksiteMapId')

    def current_map(self) -> str:
        '''
        Returns the current worksite name.
        '''
        map_data = self.get_robot_maps()
        if map_data is not None:
            current_worksite_id = map_data['currentWorksiteMapId']
            for map in map_data['workSiteMaps']:
                if map['id'] == current_worksite_id:
                    return map['name']
        return None

    def current_map_info(self) -> Dict[str, Any] | None:
        map_data = self.get_robot_maps()
        if map_data is not None:
            current_worksite_id = map_data['currentWorksiteMapId']
            for map in map_data['workSiteMaps']:
                if map['id'] == current_worksite_id:
                    return map

        return None

    def get_zones_by_map(self, map_id: str) -> Dict[str, Any]:
        '''
        returns a list of zones according to the worksite id provided
        '''
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

    def get_path_from_zone(self, zone_name: str):
            '''
            Get a path from the from the cleaning zone of the current worksite.
            '''
            current_worksite_id = self.current_worksite_id()
            zones = self.get_zones_by_map(current_worksite_id)
            if zones is None:
                return None
            for zone in zones:
                if zone.get('name') == zone_name:
                    print(f"Length {len(zone.get('paths'))}")
                    print(f"Type {type(zone.get('paths'))}")
                    print(f"Got paths [{zone.get('paths')}]")
                    ls_paths = [int(coord) for coord in zone.get('paths').split(' ')]
                    return ls_paths

    def change_map(self, map_name: str) -> bool:
        self.refresh_expired_token()

        map_data = self.get_robot_maps()
        if map_data is None:
            return False

        # get map id using map_name
        map_id = None
        for map in map_data:
            if map['name'] == map_name:
                map_id = map['id']
                break

        if map_id is None:
            print(f"No such map of name {map_name} exists for robot {self.robot_id}")
            return False

        path = f'/openapi/v1/robot/{self.robot_id}/changemap/{map_id}?timestamp={self.get_time_stamp()}'
        headers = {'Authorization': f'Bearer {self.token}'}

        try:
            r = requests.put(f'https://{self.prefix}{path}', headers=headers)
            r.raise_for_status()
            if r.json()["currentMapId"] == map_id:
                return True
            return False
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return False

    def get_docks(self) -> Dict[str, Any]:
        current_map_id = self.current_worksite_id()

        self.refresh_expired_token()

        path = f'/openapi/v1/worksitemap/dock/{current_map_id}'
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

    def get_dock_info(self, dock_name) -> Dict[str, Any] | None:
        docks_info = self.get_docks()
        if docks_info is not None:
            for dock in docks_info:
                if dock_name == dock['name']:
                    return dock
        return  None

    def get_zone_equalizers(self, process_id: str):
        self.refresh_expired_token()

        path = f'/openapi/v1/worksitemap/section/zone/equalizer/{process_id}/{self.robot_id}'
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