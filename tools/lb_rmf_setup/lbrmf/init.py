"""Write a new fleet config from the template, with the values from the LionsBot API.

The person who sets up a site gives the robot serial and the building name. This
fills in the rest of what the config template asks for. It leaves the map
transform and the charger for `export`, which reads them from the building map
and the robot map.
"""

import re
from collections.abc import Mapping
from typing import NamedTuple
from urllib.parse import urlparse

from lbrmf import errors
from lbrmf.building import emit_scalar


class InitValues(NamedTuple):
    fleet_name: str
    robot: str
    robot_id: str
    rmf_level: str
    map_name: str
    map_level: str
    api_host: str


# Each template line to fill: a pattern of the line and the value it takes.
_TEMPLATE_LINES = (
    (r"^  <rmf_level>:$", "  {rmf_level}:"),
    (r"^  name: <fleet_name>$", "  name: {fleet_name}"),
    (r"^    prefix: \S+$", "    prefix: {api_host}"),
    (r"^  <lionsbot_robot_name>:$", "  {robot}:"),
    (r"^      uuid: <robot_uuid>$", "      uuid: {robot_id}"),
    (r"^        rmf_level: <rmf_level>$", "        rmf_level: {rmf_level}"),
    (r"^        robot_map_name: <lionsbot_map_name>$", "        robot_map_name: {map_name}"),
    (r"^        robot_map_level: <lionsbot_map_level>$", "        robot_map_level: {map_level}"),
)

_SAFE_NAME = re.compile(r"[^a-z0-9]+")


def robot_type(family: str | None, kind: str | None) -> str:
    """A short fleet type such as r3vac, from the robot family and type."""
    text = _SAFE_NAME.sub("", f"{family or ''}{kind or ''}".lower())
    return text or "robot"


def api_host(base_url: str) -> str:
    host = urlparse(base_url).hostname
    if not host:
        raise errors.UsageError(f"{base_url} is not a URL")
    return host


def render_config(template: str, values: InitValues) -> str:
    """The template with each placeholder of a new site filled in.

    It stops when the template has changed so a line is not there, so a new
    template never gives a config with a placeholder left in it by mistake.
    """
    # Quote a value that YAML would not read back as the same text, such as level 8.
    fields = {key: emit_scalar(value) for key, value in values._asdict().items()}
    for pattern, line in _TEMPLATE_LINES:
        template, count = re.subn(
            pattern, line.format(**fields).replace("\\", "\\\\"), template, count=1, flags=re.M
        )
        if count != 1:
            raise errors.MalformedConfigError(
                f"the config template has no line like {pattern.strip('^$')}"
            )
    return template


def update_env(text: str, settings: Mapping[str, str]) -> str:
    """Set each KEY=value in a dotenv text. Keep every other line and comment."""
    lines = text.splitlines()
    for key, value in settings.items():
        pattern = re.compile(rf"^{re.escape(key)}=")
        for index, line in enumerate(lines):
            if pattern.match(line):
                lines[index] = f"{key}={value}"
                break
        else:
            lines.append(f"{key}={value}")
    return "\n".join(lines) + "\n"
