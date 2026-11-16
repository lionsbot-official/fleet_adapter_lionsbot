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

"""Stand-ins for the ROS 2 packages on a machine that does not have them.

The robot logic imports rclpy and the RMF adapter bindings, which exist only
in a ROS 2 environment. These tests drive that logic through mocks, so a
module with the same name is enough. A real installation is never replaced.
"""

import importlib
import sys
from unittest.mock import MagicMock

ROS_MODULES = ('rclpy', 'rmf_adapter', 'rmf_adapter.easy_full_control')


def _install_stand_in(name: str):
    try:
        importlib.import_module(name)
    except ImportError:
        stand_in = MagicMock(name=name)
        stand_in.ok.return_value = True
        sys.modules[name] = stand_in
        parent, _, child = name.rpartition('.')
        if parent:
            setattr(sys.modules[parent], child, stand_in)


for _name in ROS_MODULES:
    _install_stand_in(_name)
