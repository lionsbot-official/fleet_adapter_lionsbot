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

import socket

import pytest

from ..utils.rmf_web import alerts_url
from ..utils.rmf_web import reachable


@pytest.mark.parametrize('server_uri, expected', [
    ('ws://localhost:8000/_internal', 'http://localhost:8000/alerts/request'),
    ('ws://10.0.0.5:8000/_internal', 'http://10.0.0.5:8000/alerts/request'),
])
def test_alerts_url_is_on_the_server_of_the_server_uri(server_uri, expected):
    assert alerts_url(server_uri) == expected


def test_reachable_when_a_server_listens():
    with socket.socket() as server:
        server.bind(('127.0.0.1', 0))
        server.listen()
        port = server.getsockname()[1]
        assert reachable(f'ws://127.0.0.1:{port}/_internal')


def test_not_reachable_when_nothing_listens():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    assert not reachable(f'ws://127.0.0.1:{port}/_internal', timeout=1.0)
