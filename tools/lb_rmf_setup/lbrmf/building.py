import json
import math
import re
from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import IntEnum
from types import MappingProxyType
from typing import Any, NamedTuple

import yaml

from lbrmf import errors
from lbrmf.frame import Measurement, Point


class ParamType(IntEnum):
    STRING = 1
    INT = 2
    DOUBLE = 3
    BOOL = 4


class Param(NamedTuple):
    type: ParamType
    value: Any


NO_PARAMS: Mapping[str, Param] = MappingProxyType({})


class Vertex(NamedTuple):
    x: float
    y: float
    z: float
    name: str
    params: Mapping[str, Param] = NO_PARAMS


@dataclass
class LevelEdit:
    replaced: dict[int, Vertex] = field(default_factory=dict)
    appended: list[Vertex] = field(default_factory=list)
    lanes: list[list[Any]] = field(default_factory=list)


class _Key:
    LEVELS = "levels"
    NAME = "name"
    VERTICES = "vertices"
    MEASUREMENTS = "measurements"
    DISTANCE = "distance"
    LANES = "lanes"
    LIFTS = "lifts"
    LAYERS = "layers"
    ELEVATION = "elevation"
    FIDUCIALS = "fiducials"
    REFERENCE_LEVEL = "reference_level_name"


# Level lists whose items join two vertices, and lists of shapes with a vertex list.
DRAWN_PAIRS = ("walls", "measurements", "doors")
DRAWN_SHAPES = ("floors", "holes")

PLAIN_SCALAR = re.compile(r"[A-Za-z_][A-Za-z0-9_.\-]*")
YAML_KEYWORDS = frozenset({"yes", "no", "true", "false", "on", "off", "null", "nan", "inf"})


# yaml-cpp, which traffic editor uses, writes a double with 17 significant digits.
def _emit_float(value: float) -> str:
    if not math.isfinite(value):
        raise errors.NonFiniteNumberError(f"cannot write {value} to the building file")
    text = f"{value:.17g}"
    # PyYAML reads an exponent without a dot as a string.
    if "e" in text and "." not in text:
        text = text.replace("e", ".0e")
    return text


def emit_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return _emit_float(value)
    text = str(value)
    if PLAIN_SCALAR.fullmatch(text) and text.lower() not in YAML_KEYWORDS:
        return text
    return json.dumps(text, ensure_ascii=False)


def emit_vertex(vertex: Vertex) -> str:
    parts = [emit_scalar(vertex.x), emit_scalar(vertex.y), emit_scalar(vertex.z)]
    parts.append(emit_scalar(vertex.name))
    if vertex.params:
        params = ", ".join(
            f"{emit_scalar(key)}: [{int(param.type)}, {emit_scalar(param.value)}]"
            for key, param in sorted(vertex.params.items())
        )
        parts.append(f"{{{params}}}")
    return f"[{', '.join(parts)}]"


def _emit_flow(value: Any) -> str:
    if isinstance(value, Mapping):
        return (
            "{"
            + ", ".join(f"{emit_scalar(k)}: {_emit_flow(v)}" for k, v in sorted(value.items()))
            + "}"
        )
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_emit_flow(item) for item in value) + "]"
    return emit_scalar(value)


# Traffic editor writes nested mappings as blocks and lists such as level doors on one line.
def _emit_block(mapping: Mapping[str, Any], indent: str) -> list[str]:
    lines = []
    for key, value in sorted(mapping.items()):
        if isinstance(value, Mapping) and value:
            lines.append(f"{indent}{emit_scalar(key)}:")
            lines += _emit_block(value, indent + "  ")
        else:
            lines.append(f"{indent}{emit_scalar(key)}: {_emit_flow(value)}")
    return lines


def emit_lifts(lifts: Mapping[str, Mapping[str, Any]], indent: str) -> str:
    return "\n".join(_emit_block(lifts, indent))


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _parse_param(name: str, raw: Any) -> Param:
    if not isinstance(raw, list) or len(raw) != 2 or raw[0] not in set(ParamType):
        raise errors.MalformedBuildingError(f"vertex param {name} is {raw!r}, not [type, value]")
    return Param(ParamType(raw[0]), raw[1])


def _parse_vertex(level: str, index: int, raw: Any) -> Vertex:
    if (
        not isinstance(raw, list)
        or len(raw) < 4
        or not all(_is_number(value) for value in raw[:3])
        or not isinstance(raw[3], str)
    ):
        raise errors.MalformedBuildingError(
            f"level {level} vertex {index} is {raw!r}, not [x, y, z, name, params]"
        )
    params: Mapping[str, Param] = NO_PARAMS
    if len(raw) > 4 and raw[4]:
        if not isinstance(raw[4], dict):
            raise errors.MalformedBuildingError(f"level {level} vertex {index} params")
        params = {key: _parse_param(key, value) for key, value in raw[4].items()}
    return Vertex(raw[0], raw[1], raw[2], raw[3], params)


# Called only after the parsed data shows that the key exists.
def _entry(node: yaml.MappingNode, key: str) -> tuple[yaml.Node, yaml.Node]:
    return next((key_node, value) for key_node, value in node.value if key_node.value == key)


class BuildingFile:
    def __init__(self, text: str, data: Mapping[str, Any], root: yaml.MappingNode):
        self._text = text
        self._data = data
        self._root = root

    @classmethod
    def parse(cls, text: str) -> "BuildingFile":
        try:
            data = yaml.safe_load(text)
            root = yaml.compose(text, Loader=yaml.SafeLoader)
        except yaml.YAMLError as err:
            raise errors.MalformedBuildingError(f"not valid YAML: {err}") from err
        if not isinstance(data, dict) or not isinstance(root, yaml.MappingNode):
            raise errors.MalformedBuildingError("the building file is not a YAML mapping")
        if not isinstance(data.get(_Key.NAME), str):
            raise errors.MalformedBuildingError("the building file has no name")
        if not isinstance(data.get(_Key.LEVELS), dict):
            raise errors.MalformedBuildingError("the building file has no levels mapping")
        return cls(text, data, root)

    @property
    def name(self) -> str:
        return self._data[_Key.NAME]

    @property
    def level_names(self) -> list[str]:
        return list(self._data[_Key.LEVELS])

    @property
    def reference_level(self) -> str:
        """The level RMF places every other level against: the named one, or the first."""
        return self._data.get(_Key.REFERENCE_LEVEL) or next(iter(self.level_names), "")

    def fiducials(self, level: str) -> list[tuple[str, Point]]:
        result = []
        for raw in self._level(level).get(_Key.FIDUCIALS) or []:
            try:
                result.append((str(raw[2]), Point(float(raw[0]), float(raw[1]))))
            except (IndexError, KeyError, TypeError, ValueError) as err:
                raise errors.MalformedBuildingError(
                    f"level {level} fiducial {raw!r} is not [x, y, name]"
                ) from err
        return result

    def _level(self, level: str) -> Mapping[str, Any]:
        levels = self._data[_Key.LEVELS]
        if level not in levels:
            raise errors.LevelNotFoundError(f"level {level} is not in the building file")
        if not isinstance(levels[level], dict) or not isinstance(
            levels[level].get(_Key.VERTICES), list
        ):
            raise errors.MalformedBuildingError(f"level {level} has no vertices list")
        return levels[level]

    def vertices(self, level: str) -> list[Vertex]:
        raw = self._level(level)[_Key.VERTICES]
        return [_parse_vertex(level, index, item) for index, item in enumerate(raw)]

    def measurements(self, level: str) -> list[Measurement]:
        vertices = self.vertices(level)
        result = []
        for raw in self._level(level).get(_Key.MEASUREMENTS) or []:
            try:
                first, second, params = raw[0], raw[1], raw[2]
                meters = params[_Key.DISTANCE][1]
                start, end = vertices[first], vertices[second]
            except (IndexError, KeyError, TypeError) as err:
                raise errors.MalformedBuildingError(
                    f"level {level} measurement {raw!r} is not [vertex, vertex, distance]"
                ) from err
            result.append(Measurement(math.dist((start.x, start.y), (end.x, end.y)), meters))
        return result

    def drawing(self, level: str) -> str | None:
        """The file name of the floor plan image of a level."""
        drawing = self._level(level).get("drawing") or {}
        name = drawing.get("filename") if isinstance(drawing, dict) else None
        return name if isinstance(name, str) and name else None

    def layers(self, level: str) -> dict[str, Any]:
        """The image layers of a level that traffic editor shows over the floor plan."""
        layers = self._level(level).get(_Key.LAYERS) or {}
        return layers if isinstance(layers, dict) else {}

    def lifts(self) -> dict[str, dict[str, Any]]:
        lifts = self._data.get(_Key.LIFTS) or {}
        if not isinstance(lifts, dict):
            raise errors.MalformedBuildingError("lifts is not a mapping")
        return {name: dict(lift or {}) for name, lift in lifts.items()}

    def elevation(self, level: str) -> float:
        value = self._level(level).get(_Key.ELEVATION) or 0
        if not _is_number(value):
            raise errors.MalformedBuildingError(f"level {level} elevation is not a number")
        return float(value)

    def _lifts_span(self, lifts: Mapping[str, Mapping[str, Any]]) -> tuple[int, int, str]:
        entry = next((pair for pair in self._root.value if pair[0].value == _Key.LIFTS), None)
        block = emit_lifts(lifts, "  ")
        if entry is None:
            end = len(self._text.rstrip("\n"))
            return end, end, f"\n{_Key.LIFTS}:\n{block}"
        key_node, value_node = entry
        _, end = self._item_span(value_node)
        return key_node.end_mark.index, end, f":\n{block}"

    def _level_node(self, level: str) -> yaml.MappingNode:
        self._level(level)
        _, levels_node = _entry(self._root, _Key.LEVELS)
        _, level_node = _entry(levels_node, level)
        return level_node

    def _item_span(self, node: yaml.Node) -> tuple[int, int]:
        end = node.end_mark.index
        while end > node.start_mark.index and self._text[end - 1] in " \t\r\n":
            end -= 1
        return node.start_mark.index, end

    def _line_prefix(self, node: yaml.Node) -> str:
        start = node.start_mark.index
        return self._text[start - node.start_mark.column : start]

    def drawn_vertices(self, level: str) -> set[int]:
        """Vertices that a wall, measurement, door, floor or hole uses. They need no lane."""
        data = self._level(level)
        used = set()
        for key in DRAWN_PAIRS:
            for item in data.get(key) or []:
                if isinstance(item, list):
                    used.update(index for index in item[:2] if isinstance(index, int))
        for key in DRAWN_SHAPES:
            for item in data.get(key) or []:
                if isinstance(item, dict):
                    used.update(i for i in item.get(_Key.VERTICES) or [] if isinstance(i, int))
        return used

    def lanes(self, level: str) -> list[Any]:
        return list(self._level(level).get(_Key.LANES) or [])

    def _list_spans(
        self,
        level: str,
        key: str,
        existing: list[str],
        replaced: Mapping[int, str],
        appended: list[str],
    ) -> list[tuple[int, int, str]]:
        if not replaced and not appended:
            return []
        level_node = self._level_node(level)
        if level_node.flow_style:
            raise errors.MalformedBuildingError(
                f"level {level} is a one-line mapping; open and save it in traffic editor"
            )
        entry = next((pair for pair in level_node.value if pair[0].value == key), None)
        if entry is None:
            first_key = level_node.value[0][0]
            indent = " " * first_key.start_mark.column
            _, end = self._item_span(level_node.value[-1][1])
            block = "".join(f"\n{indent}  - {item}" for item in appended)
            return [(end, end, f"\n{indent}{key}:{block}")]
        key_node, node = entry
        items = node.value
        if node.flow_style:
            texts = list(existing)
            for index, text in replaced.items():
                texts[index] = text
            texts += appended
            indent = " " * (key_node.start_mark.column + 2)
            block = "".join(f"\n{indent}- {text}" for text in texts)
            start = node.start_mark.index
            while self._text[start - 1] in " \t":
                start -= 1
            return [(start, node.end_mark.index, block)]
        spans = []
        for index, text in replaced.items():
            start, end = self._item_span(items[index])
            spans.append((start, end, text))
        if appended:
            last = items[-1]
            _, end = self._item_span(last)
            prefix = self._line_prefix(last)
            spans.append((end, end, "".join(f"\n{prefix}{text}" for text in appended)))
        return spans

    def _level_spans(self, level: str, edit: LevelEdit) -> list[tuple[int, int, str]]:
        spans = self._list_spans(
            level,
            _Key.VERTICES,
            [emit_vertex(vertex) for vertex in self.vertices(level)],
            {index: emit_vertex(vertex) for index, vertex in edit.replaced.items()},
            [emit_vertex(vertex) for vertex in edit.appended],
        )
        lanes = self.lanes(level)
        spans += self._list_spans(
            level,
            _Key.LANES,
            [_emit_flow(lane) for lane in lanes],
            {},
            [_emit_flow(i) for i in edit.lanes],
        )
        return spans

    def render(
        self,
        levels: Mapping[str, LevelEdit],
        lifts: Mapping[str, Mapping[str, Any]] | None = None,
    ) -> str:
        spans = []
        if lifts is not None:
            spans.append(self._lifts_span(lifts))
        for level, edit in levels.items():
            spans.extend(self._level_spans(level, edit))
        text = self._text
        for start, end, replacement in sorted(spans, reverse=True):
            text = text[:start] + replacement + text[end:]
        return text
