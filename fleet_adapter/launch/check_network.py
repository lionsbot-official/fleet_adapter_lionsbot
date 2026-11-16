#!/usr/bin/env python3
"""Check that ROS 2 nodes on this machine can find each other.

Cyclone DDS finds other nodes with multicast to 239.255.0.1. Two things on a
laptop stop that, and then RMF just waits for /rmf_traffic_schedule:

- a firewall such as ufw drops multicast that comes in on the network card,
- a VPN takes all traffic, multicast too, into its tunnel.

This script finds both and prints the fix. It uses the Python standard library
and the `ip` command only. It exits with 1 when it finds a problem.
"""

import json
import socket
import subprocess
import sys
from typing import NamedTuple

DDS_GROUP = '239.255.0.1'
TUNNEL_PREFIXES = ('tun', 'tap', 'wg', 'ppp', 'utun')


class Interface(NamedTuple):
    name: str
    address: str
    multicast: bool


def interfaces(ip_addr_json: str) -> list[Interface]:
    """The IPv4 interfaces that are up, without loopback, from `ip -j addr`."""
    found = []
    for item in json.loads(ip_addr_json):
        flags = item.get('flags') or []
        if 'UP' not in flags or 'LOOPBACK' in flags:
            continue
        for info in item.get('addr_info') or []:
            if info.get('family') == 'inet':
                found.append(Interface(
                    item['ifname'], info['local'], 'MULTICAST' in flags))
    return found


def route_device(ip_route_json: str) -> str | None:
    """The interface that `ip -j route get 239.255.0.1` sends multicast on."""
    routes = json.loads(ip_route_json or '[]')
    return routes[0].get('dev') if routes else None


def multicast_reaches_itself(address: str, timeout: float = 1.0) -> bool:
    """Send one multicast packet on the interface and wait for it to come back."""
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        receiver.bind(('', 0))
        port = receiver.getsockname()[1]
        receiver.setsockopt(
            socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
            socket.inet_aton(DDS_GROUP) + socket.inet_aton(address))
        receiver.settimeout(timeout)
        sender.setsockopt(
            socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(address))
        sender.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        sender.sendto(b'rmf-check', (DDS_GROUP, port))
        return receiver.recv(64) == b'rmf-check'
    except OSError:
        return False
    finally:
        receiver.close()
        sender.close()


def problems(found: list[Interface], device: str | None, probe) -> list[str]:
    """The problems to report. probe(address) is True when multicast works."""
    issues = []
    if device and device.startswith(TUNNEL_PREFIXES):
        issues.append(
            f'Multicast for ROS 2 goes into the VPN tunnel {device}. ROS 2 nodes on '
            'this machine cannot find each other. Turn off the VPN, or set it to '
            'not take all traffic, then start RMF again.')
        return issues
    # Cyclone DDS uses the interface that multicast is routed on.
    for interface in found:
        if interface.name != device or not interface.multicast:
            continue
        if not probe(interface.address):
            issues.append(
                f'Multicast does not work on {interface.name} ({interface.address}). '
                'A firewall may block it. With ufw, run:\n'
                f'  sudo ufw allow in on {interface.name} to {DDS_GROUP}')
    return issues


def _ip(*args: str) -> str:
    result = subprocess.run(
        ['ip', '-j', *args], capture_output=True, text=True, check=False)
    return result.stdout if result.returncode == 0 else '[]'


def main() -> int:
    found = interfaces(_ip('addr'))
    device = route_device(_ip('route', 'get', DDS_GROUP))
    issues = problems(found, device, multicast_reaches_itself)
    for issue in issues:
        print(f'Network problem: {issue}')
    if not issues:
        print('Network check: multicast for ROS 2 works.')
    return 1 if issues else 0


if __name__ == '__main__':
    sys.exit(main())
