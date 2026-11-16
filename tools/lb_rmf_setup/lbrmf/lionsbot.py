import re
import struct
import uuid
from collections.abc import Mapping
from typing import Any, NamedTuple

from lbrmf import errors
from lbrmf.api import ApiClient
from lbrmf.frame import MapFrame, Point
from lbrmf.geometry import CleaningArea, Lift, Lobby, MapGeometry, Marker

ROBOTS_PATH = "/openapi/v2/robot"
MAPS_PATH = "/openapi/v2/robot/{robot_id}/map"
MARKERS_PATH = "/openapi/v2/robot/map/{map_id}/markers"
ZONES_PATH = "/openapi/v2/robot/map/{map_id}/zones"
HOME_POI_PATH = "/openapi/v2/robot/map/{map_id}/markers/homepoi"
LIFT_LOBBIES_PATH = "/openapi/v2/robot/lift/lift-lobbies/{map_id}"
MAP_NOT_SYNCED_STATUS = 400
MAP_NOT_SYNCED_CODE = "RSE30153"
PAGE_SIZE = 100
DOCK_MARKER = "dock"
CLEANING_ZONE_TYPES = frozenset({"clean", "custom"})
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
UNSAFE_FILE_CHARACTERS = re.compile(r"[^A-Za-z0-9._-]")


class _Field:
    CONTENT = "content"
    LAST = "last"
    ID = "id"
    ENCODING_ID = "robotEncodingId"
    FAMILY = "robotFamily"
    TYPE = "robotType"
    AUTONOMY = "autonomyVersion"
    WORKSITE = "worksiteName"
    NAME = "name"
    LEVEL = "level"
    BUILDING = "building"
    LAST_MODIFIED = "lastModifiedDate"
    FRAME = "frame"
    IMAGE_LINK = "pngRawLink"
    RESOLUTION = "resolution"
    ORIGIN_X = "originX"
    ORIGIN_Y = "originY"
    WIDTH = "width"
    HEIGHT = "height"
    OCCUPIED = "occupiedThresh"
    FREE = "freeThresh"
    MARKER_TYPE = "type"
    ANGLE = "angle"
    POINT = "point"
    ZONE_TYPE = "type"
    POINTS = "points"
    PATH_POINTS = "pathPoints"
    X = "x"
    Y = "y"
    INCLUDE_PATHS = "includePaths"
    WORK_SITE_MAPS = "workSiteMaps"
    CURRENT_MAP_ID = "currentMapId"
    LIFTS = "lifts"
    LIFT_NAME = "liftName"
    DOOR_WIDTH = "doorWidth"
    CAR_WIDTH = "carriageWidth"
    LOBBY_NAME = "lobbyName"
    PAGE = "page"
    SIZE = "size"


class RobotSummary(NamedTuple):
    id: str
    serial: str
    family: str | None
    type: str | None
    autonomy_version: str | None
    worksite: str | None


class MapSummary(NamedTuple):
    id: str
    name: str
    level: str | None
    building: str | None
    last_modified: str | None
    frame: MapFrame | None
    occupied_thresh: float | None
    free_thresh: float | None
    image_link: str | None


def _unknown_format(what: str) -> errors.ApiResponseError:
    return errors.ApiResponseError(
        f"the API answered {what} in a format this version does not know; "
        "update the setup script, or report it to LionsBot"
    )


def sanitize_file_name(name: str) -> str:
    return UNSAFE_FILE_CHARACTERS.sub("_", name)


def _is_uuid(text: str) -> bool:
    try:
        uuid.UUID(text)
    except ValueError:
        return False
    return True


def _frame(raw: Mapping[str, Any] | None) -> MapFrame | None:
    if not raw:
        return None
    return MapFrame(
        resolution=float(raw[_Field.RESOLUTION]),
        origin_x=float(raw[_Field.ORIGIN_X]),
        origin_y=float(raw[_Field.ORIGIN_Y]),
        width=int(raw[_Field.WIDTH]),
        height=int(raw[_Field.HEIGHT]),
    )


def _png_size(data: bytes) -> tuple[int, int]:
    if not data.startswith(PNG_SIGNATURE) or len(data) < 24:
        raise errors.NotAPngError("the plain map image is not a PNG file")
    return struct.unpack(">II", data[16:24])


class LionsBotMaps:
    def __init__(self, client: ApiClient):
        self.client = client
        # The map each robot uses now, as the last maps() call saw it.
        self.current_map_ids: dict[str, str] = {}

    def robots(self) -> list[RobotSummary]:
        robots = []
        page = 0
        while True:
            body = self.client.get_json(ROBOTS_PATH, {_Field.PAGE: page, _Field.SIZE: PAGE_SIZE})
            try:
                content = body[_Field.CONTENT]
                if not isinstance(content, list):
                    raise TypeError(_Field.CONTENT)
                robots += [
                    RobotSummary(
                        str(item[_Field.ID]),
                        item[_Field.ENCODING_ID],
                        item.get(_Field.FAMILY),
                        item.get(_Field.TYPE),
                        item.get(_Field.AUTONOMY),
                        item.get(_Field.WORKSITE),
                    )
                    for item in content
                ]
                last = bool(body.get(_Field.LAST, True))
            except (KeyError, TypeError, AttributeError) as err:
                raise _unknown_format("the robot list") from err
            if last:
                return robots
            page += 1

    def resolve_robot_id(self, ident: str) -> str:
        if _is_uuid(ident):
            return ident
        for robot in self.robots():
            if robot.serial == ident:
                return robot.id
        raise errors.RobotNotFoundError(f"the account cannot see a robot {ident}")

    def maps(self, robot_id: str) -> list[MapSummary]:
        body = self.client.get_json(MAPS_PATH.format(robot_id=robot_id))
        if isinstance(body, Mapping) and body.get(_Field.CURRENT_MAP_ID):
            self.current_map_ids[robot_id] = str(body[_Field.CURRENT_MAP_ID])
        try:
            items = body[_Field.WORK_SITE_MAPS]
            return [
                MapSummary(
                    id=str(item[_Field.ID]),
                    name=item[_Field.NAME],
                    level=item.get(_Field.LEVEL),
                    building=item.get(_Field.BUILDING),
                    last_modified=item.get(_Field.LAST_MODIFIED),
                    frame=_frame(item.get(_Field.FRAME)),
                    occupied_thresh=(item.get(_Field.FRAME) or {}).get(_Field.OCCUPIED),
                    free_thresh=(item.get(_Field.FRAME) or {}).get(_Field.FREE),
                    image_link=item.get(_Field.IMAGE_LINK),
                )
                for item in items
            ]
        except (KeyError, TypeError, ValueError, AttributeError) as err:
            raise _unknown_format("the map list") from err

    def _pixels(self, frame: MapFrame, raw: Mapping[str, Any]) -> Point:
        return frame.to_pixels(Point(float(raw[_Field.X]), float(raw[_Field.Y])))

    def geometry(self, map_summary: MapSummary) -> tuple[MapGeometry, list[str]]:
        frame = map_summary.frame
        if frame is None:
            raise errors.MapWithoutFrameError(
                f"map {map_summary.name} has no map frame; the robot must sync the map again"
            )
        markers_body = self.client.get_json(MARKERS_PATH.format(map_id=map_summary.id))
        zones_body = self.client.get_json(
            ZONES_PATH.format(map_id=map_summary.id), {_Field.INCLUDE_PATHS: "true"}
        )
        warnings = []
        markers = []
        areas = []
        try:
            for item in markers_body:
                if not item.get(_Field.POINT):
                    warnings.append(f"marker {item[_Field.NAME]} has no point in metres; skipped")
                    continue
                markers.append(
                    Marker(
                        item[_Field.NAME],
                        self._pixels(frame, item[_Field.POINT]),
                        float(item.get(_Field.ANGLE) or 0.0),
                        dock=item.get(_Field.MARKER_TYPE) == DOCK_MARKER,
                    )
                )
            for item in zones_body:
                if item.get(_Field.ZONE_TYPE) not in CLEANING_ZONE_TYPES:
                    continue
                if not item.get(_Field.POINTS):
                    warnings.append(f"zone {item[_Field.NAME]} has no points in metres; skipped")
                    continue
                areas.append(
                    CleaningArea(
                        item[_Field.NAME],
                        self._pixels(frame, item[_Field.POINTS][0]),
                        tuple(self._pixels(frame, p) for p in item.get(_Field.PATH_POINTS) or []),
                    )
                )
        except (KeyError, TypeError, ValueError, AttributeError) as err:
            raise _unknown_format(f"the points of map {map_summary.name}") from err
        home_dock = self._home_dock(map_summary, warnings)
        return (
            MapGeometry(map_summary.name, frame, tuple(markers), tuple(areas), home_dock=home_dock),
            warnings,
        )

    def _home_dock(self, map_summary: MapSummary, warnings: list[str]) -> str | None:
        # The home dock is optional. A map with no home dock answers 404 or an empty body.
        try:
            body = self.client.get_json(HOME_POI_PATH.format(map_id=map_summary.id))
        except errors.NotFoundError:
            return None
        except errors.ApiError as err:
            warnings.append(f"could not read the home dock of map {map_summary.name}: {err}")
            return None
        if not isinstance(body, Mapping) or body.get(_Field.MARKER_TYPE) != DOCK_MARKER:
            return None
        name = body.get(_Field.NAME)
        return name if isinstance(name, str) and name else None

    def image(self, map_summary: MapSummary) -> bytes:
        if not map_summary.image_link or map_summary.frame is None:
            raise errors.MapWithoutImageError(f"map {map_summary.name} has no plain map image")
        data = self.client.download(map_summary.image_link)
        size = _png_size(data)
        expected = (map_summary.frame.width, map_summary.frame.height)
        if size != expected:
            raise errors.ImageSizeError(
                f"the map image is {size[0]} x {size[1]} pixels, the map frame says "
                f"{expected[0]} x {expected[1]}"
            )
        return data

    def _lift_lobbies(self, map_summary: MapSummary) -> Any:
        try:
            return self.client.get_json(LIFT_LOBBIES_PATH.format(map_id=map_summary.id))
        except errors.ApiStatusError as err:
            if (err.status, err.error_code) == (MAP_NOT_SYNCED_STATUS, MAP_NOT_SYNCED_CODE):
                raise errors.MapNotSyncedError(map_summary.name) from err
            raise

    def lifts(self, map_summary: MapSummary) -> tuple[tuple[Lift, ...], tuple[Lobby, ...]]:
        frame = map_summary.frame
        if frame is None:
            raise errors.MapWithoutFrameError(f"map {map_summary.name} has no map frame")
        lobbies = self._lift_lobbies(map_summary)
        try:
            lifts = tuple(
                Lift(
                    lift[_Field.LIFT_NAME],
                    self._pixels(frame, lift[_Field.POINT]),
                    float(lift[_Field.DOOR_WIDTH]),
                    float(lift[_Field.CAR_WIDTH]),
                )
                for lobby in lobbies
                for lift in lobby[_Field.LIFTS]
            )
            lobby_points = tuple(
                Lobby(lobby[_Field.LOBBY_NAME], self._pixels(frame, lobby[_Field.POINT]))
                for lobby in lobbies
                if lobby.get(_Field.POINT)
            )
        except (KeyError, TypeError, ValueError) as err:
            raise _unknown_format(f"the lifts of map {map_summary.name}") from err
        return lifts, lobby_points

    def lift_names(self, map_summary: MapSummary) -> set[str]:
        lobbies = self._lift_lobbies(map_summary)
        try:
            return {lift[_Field.LIFT_NAME] for lobby in lobbies for lift in lobby[_Field.LIFTS]}
        except (KeyError, TypeError) as err:
            raise _unknown_format(f"the lifts of map {map_summary.name}") from err
