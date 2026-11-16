import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any, NamedTuple

from lbrmf import errors


class Point(NamedTuple):
    x: float
    y: float


class Measurement(NamedTuple):
    pixel_length: float
    meters: float


@dataclass(frozen=True)
class MapFrame:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    # Robot map pixel rows count down from the top; the frame's y counts up from the bottom.
    def to_pixels(self, meters: Point) -> Point:
        return Point(
            (meters.x - self.origin_x) / self.resolution,
            self.height - (meters.y - self.origin_y) / self.resolution,
        )

    def to_meters(self, pixels: Point) -> Point:
        return Point(
            self.origin_x + pixels.x * self.resolution,
            self.origin_y + (self.height - pixels.y) * self.resolution,
        )


# Floorplan images run from about 1 mm to 50 cm per pixel; a value outside is a unit mistake.
MIN_FLOORPLAN_SCALE = 0.001
MAX_FLOORPLAN_SCALE = 0.5

TRANSFORM_VALUE_KEYS = ("rotation_degrees", "scale", "tx_meters", "ty_meters")


def _number(config: Mapping[str, Any], key: str) -> float:
    value = config[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise errors.MalformedTransformError(f"{key} is {value!r}, not a number")
    return float(value)


@dataclass(frozen=True)
class TransformValues:
    rotation_degrees: float
    scale: float
    tx_meters: float
    ty_meters: float

    @classmethod
    def from_config(cls, config: Mapping[str, Any] | None) -> "TransformValues":
        if not isinstance(config, Mapping) or set(config) != set(TRANSFORM_VALUE_KEYS):
            raise errors.MalformedTransformError(
                f"transform_values must have exactly {', '.join(TRANSFORM_VALUE_KEYS)}"
            )
        return cls(**{key: _number(config, key) for key in TRANSFORM_VALUE_KEYS})


@dataclass(frozen=True)
class LevelTransform:
    scale: float | None = None
    tx_pixels: float = 0.0
    ty_pixels: float = 0.0

    @classmethod
    def from_config(cls, config: Mapping[str, Any] | None) -> "LevelTransform":
        if not config:
            return cls()
        level = cls(
            scale=_number(config, "scale") if "scale" in config else None,
            tx_pixels=_number(config, "tx_pixels") if "tx_pixels" in config else 0.0,
            ty_pixels=_number(config, "ty_pixels") if "ty_pixels" in config else 0.0,
        )
        # The adapter cannot read the measurements. Without a scale it shifts by pixels as metres.
        if level.scale is None and (level.tx_pixels or level.ty_pixels):
            raise errors.MalformedTransformError(
                "level_transform has tx_pixels or ty_pixels but no scale. "
                "Add scale, the floorplan scale of this level in metres per pixel"
            )
        return level


def wrap_angle(radians: float) -> float:
    return (radians + math.pi) % (2 * math.pi) - math.pi


def floorplan_scale(measurements: Sequence[Measurement]) -> float:
    if not measurements:
        raise errors.NoFloorplanScaleError()
    total = 0.0
    for measurement in measurements:
        if measurement.pixel_length == 0:
            raise errors.ZeroLengthMeasurementError(
                f"a measurement of {measurement.meters} m joins two vertices at the same place"
            )
        total += measurement.meters / measurement.pixel_length
    return total / len(measurements)


class MapTransform:
    """The fleet adapter's MapTransform in closed form; a test keeps the two equal."""

    def __init__(
        self,
        values: TransformValues,
        level: LevelTransform,
        measured_floorplan_scale: float | None,
    ):
        scale = level.scale if level.scale is not None else measured_floorplan_scale
        if scale is None:
            raise errors.NoFloorplanScaleError()
        if not MIN_FLOORPLAN_SCALE <= scale <= MAX_FLOORPLAN_SCALE:
            source = "level_transform.scale" if level.scale is not None else "the measurements"
            raise errors.ImplausibleFloorplanScaleError(
                f"the floorplan scale from {source} is {scale} metres per pixel; a floorplan "
                f"image is {MIN_FLOORPLAN_SCALE} to {MAX_FLOORPLAN_SCALE} metres per pixel. "
                "Delete level_transform from the level in the fleet config, or fix the "
                "measurement in the building file"
            )
        self.floorplan_scale = scale
        self.rotation = math.radians(values.rotation_degrees)
        self._a = values.scale * math.cos(self.rotation)
        self._b = values.scale * math.sin(self.rotation)
        self._tx = values.tx_meters
        self._ty = values.ty_meters
        self._level_shift = Point(-level.tx_pixels * scale, level.ty_pixels * scale)

    def _reference_meters(self, robot: Point) -> Point:
        return Point(
            self._a * robot.x + self._b * robot.y + self._tx,
            self._b * robot.x - self._a * robot.y - self._ty,
        )

    def robot_to_rmf_meters(self, robot: Point) -> Point:
        reference = self._reference_meters(robot)
        return Point(reference.x + self._level_shift.x, reference.y + self._level_shift.y)

    # The adapter places floorplan pixels without the level shift.
    def robot_to_floorplan_pixels(self, robot: Point) -> Point:
        reference = self._reference_meters(robot)
        return Point(reference.x / self.floorplan_scale, -reference.y / self.floorplan_scale)

    def heading(self, robot_angle: float) -> float:
        return wrap_angle(robot_angle - self.rotation)
