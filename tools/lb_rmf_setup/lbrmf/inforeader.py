import json
from collections.abc import Mapping, Sequence
from typing import Any

from lbrmf import errors
from lbrmf.frame import MapFrame, Point
from lbrmf.geometry import CleaningArea, Lift, Lobby, MapGeometry, Marker

GEOMETRY_SCHEMA = 1


class _Info:
    META_DATA = "meta_data"
    MARKERS = "markers"
    ZONES = "zones"
    CLEAN_ZONES = "clean"
    CUSTOM_PATHS = "custom_paths"
    DOCK = "dock"
    NAME = "name"
    COORDS = "coords"
    PATHS = "paths"
    ANGLE = "angle"
    RESOLUTION = "resolution"
    ORIGIN_X = "ox"
    ORIGIN_Y = "oy"
    WIDTH = "width"
    HEIGHT = "height"
    LIFT_LOBBIES = "lift_lobbies"
    LIFTS = "lifts"
    DOOR_WIDTH = "door_width"
    CAR_WIDTH = "carriage_width"


class _Geometry:
    SCHEMA = "schema"
    MAP_NAME = "map_name"
    FRAME = "frame"
    MARKERS = "markers"
    AREAS = "areas"
    NAME = "name"
    X = "x"
    Y = "y"
    ANGLE = "angle"
    DOCK = "dock"
    START = "start"
    PATH = "path"
    RESOLUTION = "resolution"
    ORIGIN_X = "origin_x"
    ORIGIN_Y = "origin_y"
    WIDTH = "width"
    HEIGHT = "height"
    LIFTS = "lifts"
    DOOR_WIDTH = "door_width"
    CAR_WIDTH = "car_width"
    LOBBIES = "lobbies"
    HOME_DOCK = "home_dock"


def _load_object(text: str) -> Mapping[str, Any]:
    try:
        data = json.loads(text)
    except json.JSONDecodeError as err:
        raise errors.MalformedMapFileError(f"not valid JSON: {err}") from err
    if not isinstance(data, dict):
        raise errors.MalformedMapFileError("the map file is not a JSON object")
    return data


def _pixel_points(name: str, text: str) -> tuple[Point, ...]:
    try:
        values = [float(value) for value in text.split()]
    except ValueError as err:
        raise errors.MalformedMapFileError(f"{name} has a point that is not a number") from err
    if len(values) % 2:
        raise errors.MalformedMapFileError(f"{name} has an odd number of point values")
    return tuple(Point(x, y) for x, y in zip(values[::2], values[1::2], strict=True))


def _info_frame(meta: Mapping[str, Any] | None) -> MapFrame | None:
    if not meta:
        return None
    try:
        return MapFrame(
            resolution=float(meta[_Info.RESOLUTION]),
            origin_x=float(meta[_Info.ORIGIN_X]),
            origin_y=float(meta[_Info.ORIGIN_Y]),
            width=int(meta[_Info.WIDTH]),
            height=int(meta[_Info.HEIGHT]),
        )
    except (KeyError, TypeError, ValueError) as err:
        raise errors.MalformedMapFileError(f"meta_data has no complete map frame: {err}") from err


def _info_name(item: Mapping[str, Any]) -> str:
    name = item.get(_Info.NAME)
    if not isinstance(name, str) or not name:
        raise errors.MalformedMapFileError(f"an object has no name: {item!r}")
    return name


def _info_marker(kind: str, item: Mapping[str, Any]) -> Marker:
    name = _info_name(item)
    points = _pixel_points(name, item.get(_Info.COORDS, ""))
    if len(points) != 1:
        raise errors.MalformedMapFileError(f"marker {name} does not have one point")
    return Marker(name, points[0], float(item.get(_Info.ANGLE) or 0.0), dock=kind == _Info.DOCK)


def _info_area(item: Mapping[str, Any]) -> CleaningArea:
    name = _info_name(item)
    outline = _pixel_points(name, item.get(_Info.COORDS, ""))
    if not outline:
        raise errors.MalformedMapFileError(f"{name} has no points")
    return CleaningArea(name, outline[0], _pixel_points(name, item.get(_Info.PATHS) or ""))


def _info_point(kind: str, item: Mapping[str, Any]) -> tuple[str, Point]:
    name = _info_name(item)
    points = _pixel_points(name, item.get(_Info.COORDS, ""))
    if len(points) != 1:
        raise errors.MalformedMapFileError(f"{kind} {name} does not have one point")
    return name, points[0]


def _info_lift(item: Mapping[str, Any]) -> Lift:
    name = _info_name(item)
    points = _pixel_points(name, item.get(_Info.COORDS, ""))
    if len(points) != 1:
        raise errors.MalformedMapFileError(f"lift {name} does not have one point")
    return Lift(
        name,
        points[0],
        float(item.get(_Info.DOOR_WIDTH) or 0.0),
        float(item.get(_Info.CAR_WIDTH) or 0.0),
    )


def read_info(text: str, map_name: str) -> MapGeometry:
    data = _load_object(text)
    try:
        markers = tuple(
            _info_marker(kind, item)
            for kind, items in (data.get(_Info.MARKERS) or {}).items()
            for item in items
        )
        zones = (data.get(_Info.ZONES) or {}).get(_Info.CLEAN_ZONES) or []
        custom_paths = data.get(_Info.CUSTOM_PATHS) or []
        areas = tuple(_info_area(item) for item in [*custom_paths, *zones])
        lobby_items = data.get(_Info.LIFT_LOBBIES) or []
        lifts = tuple(
            _info_lift(lift) for lobby in lobby_items for lift in lobby.get(_Info.LIFTS) or []
        )
        lobbies = tuple(Lobby(*_info_point("lobby", lobby)) for lobby in lobby_items)
    except (AttributeError, TypeError, ValueError) as err:
        raise errors.MalformedMapFileError(
            f"the .info file has an unexpected shape: {err}"
        ) from err
    frame = _info_frame(data.get(_Info.META_DATA))
    return MapGeometry(map_name, frame, markers, areas, lifts, lobbies)


def _point(name: str, value: Any) -> Point:
    if not isinstance(value, Sequence) or len(value) != 2:
        raise errors.MalformedMapFileError(f"{name} has a point that is not [x, y]")
    return Point(float(value[0]), float(value[1]))


def _geometry_frame(frame: Mapping[str, Any] | None) -> MapFrame | None:
    if frame is None:
        return None
    return MapFrame(
        resolution=float(frame[_Geometry.RESOLUTION]),
        origin_x=float(frame[_Geometry.ORIGIN_X]),
        origin_y=float(frame[_Geometry.ORIGIN_Y]),
        width=int(frame[_Geometry.WIDTH]),
        height=int(frame[_Geometry.HEIGHT]),
    )


def read_geometry(text: str) -> MapGeometry:
    data = _load_object(text)
    schema = data.get(_Geometry.SCHEMA)
    if schema != GEOMETRY_SCHEMA or isinstance(schema, bool):
        raise errors.UnsupportedGeometrySchemaError(
            f"geometry schema {schema!r} is not {GEOMETRY_SCHEMA}; fetch the map again"
        )
    try:
        map_name = data[_Geometry.MAP_NAME]
        if not isinstance(map_name, str):
            raise errors.MalformedMapFileError("map_name is not a string")
        markers = tuple(
            Marker(
                item[_Geometry.NAME],
                Point(float(item[_Geometry.X]), float(item[_Geometry.Y])),
                float(item[_Geometry.ANGLE]),
                bool(item[_Geometry.DOCK]),
            )
            for item in data[_Geometry.MARKERS]
        )
        areas = tuple(
            CleaningArea(
                item[_Geometry.NAME],
                _point(item[_Geometry.NAME], item[_Geometry.START]),
                tuple(_point(item[_Geometry.NAME], point) for point in item[_Geometry.PATH]),
            )
            for item in data[_Geometry.AREAS]
        )
        frame = _geometry_frame(data[_Geometry.FRAME])
        lifts = tuple(
            Lift(
                item[_Geometry.NAME],
                Point(float(item[_Geometry.X]), float(item[_Geometry.Y])),
                float(item[_Geometry.DOOR_WIDTH]),
                float(item[_Geometry.CAR_WIDTH]),
            )
            for item in data.get(_Geometry.LIFTS) or []
        )
        lobbies = tuple(
            Lobby(item[_Geometry.NAME], Point(float(item[_Geometry.X]), float(item[_Geometry.Y])))
            for item in data.get(_Geometry.LOBBIES) or []
        )
    except (KeyError, TypeError, ValueError) as err:
        raise errors.MalformedMapFileError(f"the geometry file is incomplete: {err}") from err
    home_dock = data.get(_Geometry.HOME_DOCK)
    if home_dock is not None and not isinstance(home_dock, str):
        raise errors.MalformedMapFileError("home_dock is not a string")
    return MapGeometry(map_name, frame, markers, areas, lifts, lobbies, home_dock)
