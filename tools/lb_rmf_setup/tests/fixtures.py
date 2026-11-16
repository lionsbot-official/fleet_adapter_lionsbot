import struct
import zlib
from pathlib import Path

import yaml

from lbrmf.building import BuildingFile
from lbrmf.frame import (
    LevelTransform,
    MapFrame,
    MapTransform,
    Point,
    TransformValues,
    floorplan_scale,
)
from lbrmf.geometry import CleaningArea, MapGeometry, Marker

TESTDATA = Path(__file__).resolve().parents[1] / "testdata"
OFFICE_NEW = TESTDATA / "office_new"
OFFICE_NEW_BUILDING = OFFICE_NEW / "rmf_office_new.building.yaml"
OFFICE_NEW_LEVEL = "L8"


def _points(values: str) -> tuple[Point, ...]:
    numbers = [float(value) for value in values.split()]
    return tuple(Point(x, y) for x, y in zip(numbers[::2], numbers[1::2], strict=True))


R3SCP_GEOMETRY = MapGeometry(
    map_name="rmf_office_new",
    frame=MapFrame(resolution=0.05, origin_x=-17.56, origin_y=-20.93, width=647, height=1193),
    markers=(
        Marker("refuel_891", Point(337, 796), angle=0.08300105075706732, dock=True),
        Marker("location_394", Point(352, 795), angle=-3.058591602832726, dock=False),
    ),
    areas=(
        CleaningArea(
            "cZone_356",
            start=Point(359, 650),
            path=_points(
                "364 652 366 693 373 690 371 651 378 651 380 694 373 694 377 690 383 689 "
                "382 675 381 669 380 655 362 656 363 663 365 684"
            ),
        ),
    ),
)

R5_GEOMETRY = MapGeometry(
    map_name="rmf_office_new",
    frame=MapFrame(resolution=0.05, origin_x=-27.85, origin_y=-36.8, width=1089, height=1430),
    markers=(
        Marker("charging_1", Point(570, 694), angle=3.141592653589793, dock=True),
        Marker("location_001", Point(556, 694), angle=0.0, dock=False),
    ),
    areas=(
        CleaningArea(
            "cZone_001",
            start=Point(503, 683),
            path=_points(
                "462 686 464 686 496 687 498 687 496 688 463 690 461 689 461 687 463 687 "
                "465 687 477 687 476 688 474 688 472 688 470 688 467 688 465 689 463 689 "
                "462 688"
            ),
        ),
        CleaningArea(
            "cZone_002",
            start=Point(451, 427),
            path=_points(
                "434 428 439 428 441 428 443 429 445 429 447 431 447 458 447 460 446 462 "
                "444 462 426 462 423 462 422 460 422 431 423 429 425 428 431 428 434 428 "
                "435 429 437 431 439 432 441 434 441 436 441 438 441 454 441 456 439 456 "
                "430 456 428 455 428 453 428 437 428 435 429 434 431 434 438 434 440 434 "
                "438 436 437 438 436 440 434 449"
            ),
        ),
    ),
)


def office_new_building() -> BuildingFile:
    return BuildingFile.parse(OFFICE_NEW_BUILDING.read_text())


def office_new_transform(robot_type: str) -> MapTransform:
    config = yaml.safe_load((OFFICE_NEW / f"config_{robot_type}.yaml").read_text())
    level = config["map_transform"][OFFICE_NEW_LEVEL]
    building = office_new_building()
    return MapTransform(
        TransformValues.from_config(level["transform_values"]),
        LevelTransform.from_config(level.get("level_transform")),
        floorplan_scale(building.measurements(OFFICE_NEW_LEVEL)),
    )


def make_png(width: int, height: int) -> bytes:
    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    header = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    pixels = zlib.compress(b"".join(b"\x00" + b"\xff" * width for _ in range(height)))
    return (
        b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", pixels) + chunk(b"IEND", b"")
    )
