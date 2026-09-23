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


def robot_uuids_from_config(config_yaml: dict) -> dict[str, str]:
    """Return configured robot UUIDs, rejecting incomplete robot entries."""
    robot_uuids = {}
    invalid_robots = []
    for robot_name, robot_config in config_yaml['robots'].items():
        robot_uuid = robot_config.get('robot_config', {}).get('uuid')
        if not isinstance(robot_uuid, str) or not robot_uuid.strip():
            invalid_robots.append(robot_name)
            continue
        robot_uuids[robot_name] = robot_uuid

    if invalid_robots:
        raise ValueError(
            'Every robot requires a non-empty robot_config.uuid; missing or '
            f'invalid UUID for: {", ".join(invalid_robots)}')

    return robot_uuids


def rmf_robot_name(vendor_name: str, robot_config: dict) -> str:
    """Return a name RMF can safely use for `vendor_name`.

    RMF composes per-robot ROS topics from the robot name, e.g.
    `rmf/dynamic_event/begin/<fleet>/<robot>`. ROS 2 topic segments allow only
    alphanumerics and underscores and may not start with a digit, but LionsBot
    encoding IDs contain hyphens (`R3-2200888-SCR`), which aborts the adapter
    with InvalidTopicNameError.

    The vendor ID stays the identifier used against the LionsBot API; this is
    only the name RMF sees. Set `robot_config.rmf_name` to choose it yourself.
    """
    explicit = robot_config.get('robot_config', {}).get('rmf_name')
    if isinstance(explicit, str) and explicit.strip():
        return explicit.strip()

    safe = re.sub(r'[^A-Za-z0-9_]', '_', vendor_name)
    if not safe or not (safe[0].isalpha() or safe[0] == '_'):
        safe = f'r_{safe}'
    return safe
