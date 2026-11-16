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

"""Robot configuration validation helpers."""

import re

ROBOTS_KEY = 'robots'
ROBOT_CONFIG_KEY = 'robot_config'
UUID_KEY = 'uuid'
RMF_NAME_KEY = 'rmf_name'

_ROS_TOPIC_TOKEN = re.compile(r'[A-Za-z_][A-Za-z0-9_]*')


class InvalidRmfRobotNameError(ValueError):
    """A configured `rmf_name` is not a valid ROS topic name token."""

    def __init__(self, vendor_name: str, rmf_name):
        super().__init__(
            f'{ROBOT_CONFIG_KEY}.{RMF_NAME_KEY} of robot [{vendor_name}] must '
            'hold only letters, digits and underscores, and must not start '
            f'with a digit; got {rmf_name!r}')
        self.vendor_name = vendor_name
        self.rmf_name = rmf_name


class DuplicateRmfRobotNameError(ValueError):
    """Two or more configured robots have the same RMF name."""

    def __init__(self, duplicates: dict[str, list[str]]):
        shared = '; '.join(
            f'[{rmf_name}] for {", ".join(vendor_names)}'
            for rmf_name, vendor_names in duplicates.items())
        super().__init__(
            'Every robot requires its own RMF name; set '
            f'{ROBOT_CONFIG_KEY}.{RMF_NAME_KEY} to tell these robots apart: '
            f'{shared}')
        self.duplicates = duplicates


def robot_uuids_from_config(config_yaml: dict) -> dict[str, str]:
    """Return configured robot UUIDs, rejecting incomplete robot entries."""
    robot_uuids = {}
    invalid_robots = []
    for robot_name, robot_config in config_yaml[ROBOTS_KEY].items():
        robot_uuid = robot_config.get(ROBOT_CONFIG_KEY, {}).get(UUID_KEY)
        if not isinstance(robot_uuid, str) or not robot_uuid.strip():
            invalid_robots.append(robot_name)
            continue
        robot_uuids[robot_name] = robot_uuid

    if invalid_robots:
        raise ValueError(
            'Every robot requires a non-empty robot_config.uuid; missing or '
            f'invalid UUID for: {", ".join(invalid_robots)}')

    return robot_uuids


def _is_ros_topic_token(name) -> bool:
    """Return whether `name` can be one segment of a ROS 2 topic name."""
    return isinstance(name, str) and bool(_ROS_TOPIC_TOKEN.fullmatch(name))


def rmf_robot_name(vendor_name: str, robot_config: dict) -> str:
    """Return a name RMF can safely use for `vendor_name`.

    RMF composes per-robot ROS topics from the robot name, e.g.
    `rmf/dynamic_event/begin/<fleet>/<robot>`. ROS 2 topic segments allow only
    alphanumerics and underscores and may not start with a digit, but LionsBot
    encoding IDs contain hyphens (`R3-2200888-SCR`), which aborts the adapter
    with InvalidTopicNameError.

    The vendor ID stays the identifier used against the LionsBot API; this is
    only the name RMF sees. Set `robot_config.rmf_name` to choose it yourself;
    a value that is not a valid topic segment is rejected.
    """
    explicit = robot_config.get(ROBOT_CONFIG_KEY, {}).get(RMF_NAME_KEY)
    if explicit is not None:
        if not _is_ros_topic_token(explicit):
            raise InvalidRmfRobotNameError(vendor_name, explicit)
        return explicit

    safe = re.sub(r'[^A-Za-z0-9_]', '_', vendor_name)
    if not safe or not (safe[0].isalpha() or safe[0] == '_'):
        safe = f'r_{safe}'
    return safe


def rmf_robot_names_from_config(config_yaml: dict) -> dict[str, str]:
    """Return the RMF name of each configured robot, rejecting shared names."""
    rmf_names = {}
    vendor_names_by_rmf_name = {}
    for vendor_name, robot_config in config_yaml[ROBOTS_KEY].items():
        rmf_name = rmf_robot_name(vendor_name, robot_config)
        rmf_names[vendor_name] = rmf_name
        vendor_names_by_rmf_name.setdefault(rmf_name, []).append(vendor_name)

    duplicates = {
        rmf_name: vendor_names
        for rmf_name, vendor_names in vendor_names_by_rmf_name.items()
        if len(vendor_names) > 1
    }
    if duplicates:
        raise DuplicateRmfRobotNameError(duplicates)

    return rmf_names
