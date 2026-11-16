import importlib.util
import json
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / 'launch' / 'check_network.py'
spec = importlib.util.spec_from_file_location('check_network', SCRIPT)
check_network = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check_network)

IP_ADDR = json.dumps([
    {'ifname': 'lo', 'flags': ['LOOPBACK', 'UP'],
     'addr_info': [{'family': 'inet', 'local': '127.0.0.1'}]},
    {'ifname': 'wlp0s20f3', 'flags': ['BROADCAST', 'MULTICAST', 'UP'],
     'addr_info': [{'family': 'inet', 'local': '192.0.2.10'},
                   {'family': 'inet6', 'local': 'fe80::1'}]},
    {'ifname': 'tun0', 'flags': ['POINTOPOINT', 'NOARP', 'UP'],
     'addr_info': [{'family': 'inet', 'local': '198.51.100.48'}]},
    {'ifname': 'enx0', 'flags': ['BROADCAST', 'MULTICAST'],
     'addr_info': []},
])


def test_reads_the_interfaces_that_are_up():
    found = check_network.interfaces(IP_ADDR)

    assert found == [
        check_network.Interface('wlp0s20f3', '192.0.2.10', True),
        check_network.Interface('tun0', '198.51.100.48', False),
    ]


def test_reads_the_multicast_route():
    assert check_network.route_device('[{"dev": "wlp0s20f3"}]') == 'wlp0s20f3'
    assert check_network.route_device('[]') is None


def test_working_multicast_has_no_problem():
    found = check_network.interfaces(IP_ADDR)

    assert check_network.problems(found, 'wlp0s20f3', lambda _: True) == []


def test_a_firewall_that_drops_multicast_gets_the_ufw_command():
    found = check_network.interfaces(IP_ADDR)

    (issue,) = check_network.problems(found, 'wlp0s20f3', lambda _: False)

    assert 'sudo ufw allow in on wlp0s20f3 to 239.255.0.1' in issue


def test_a_vpn_that_takes_multicast_is_named():
    found = check_network.interfaces(IP_ADDR)

    (issue,) = check_network.problems(found, 'tun0', lambda _: True)

    assert 'VPN tunnel tun0' in issue
