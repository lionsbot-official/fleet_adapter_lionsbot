from typing import NamedTuple

from lbrmf.frame import MapFrame, Point


class Marker(NamedTuple):
    name: str
    point: Point
    angle: float
    dock: bool


class CleaningArea(NamedTuple):
    name: str
    start: Point
    path: tuple[Point, ...]


class Lift(NamedTuple):
    name: str
    approach: Point
    door_width: float
    car_width: float


class Lobby(NamedTuple):
    name: str
    point: Point


class MapGeometry(NamedTuple):
    map_name: str
    frame: MapFrame | None
    markers: tuple[Marker, ...]
    areas: tuple[CleaningArea, ...]
    lifts: tuple[Lift, ...] = ()
    lobbies: tuple[Lobby, ...] = ()
    # The dock the robot goes home to, when the map says which one it is.
    home_dock: str | None = None
