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

"""RMF Web addresses, from the server URI that the adapter is given."""

import socket
from urllib.parse import urlparse


def alerts_url(server_uri: str) -> str:
    """Return the alerts endpoint of the RMF Web API server of server_uri."""
    return f'http://{urlparse(server_uri).netloc}/alerts/request'


def reachable(server_uri: str, timeout: float = 3.0) -> bool:
    """Return True when a TCP connection to the host and port opens."""
    uri = urlparse(server_uri)
    try:
        with socket.create_connection((uri.hostname, uri.port or 80), timeout):
            return True
    except OSError:
        return False
