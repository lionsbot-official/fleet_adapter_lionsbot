import json

from lbrmf.frame import MapFrame
from lbrmf.geometry import MapGeometry
from lbrmf.inforeader import GEOMETRY_SCHEMA

ROS_MAP_MODE = "trinary"


def ros_map_yaml(image_name: str, frame: MapFrame, occupied: float, free: float) -> str:
    return (
        f"image: {image_name}\n"
        f"mode: {ROS_MAP_MODE}\n"
        f"resolution: {frame.resolution}\n"
        f"origin: [{frame.origin_x}, {frame.origin_y}, 0.0]\n"
        "negate: 0\n"
        f"occupied_thresh: {occupied}\n"
        f"free_thresh: {free}\n"
    )


def geometry_json(geometry: MapGeometry) -> str:
    frame = geometry.frame
    document = {
        "schema": GEOMETRY_SCHEMA,
        "map_name": geometry.map_name,
        "frame": None
        if frame is None
        else {
            "resolution": frame.resolution,
            "origin_x": frame.origin_x,
            "origin_y": frame.origin_y,
            "width": frame.width,
            "height": frame.height,
        },
        "markers": [
            {"name": m.name, "x": m.point.x, "y": m.point.y, "angle": m.angle, "dock": m.dock}
            for m in geometry.markers
        ],
        "areas": [
            {"name": a.name, "start": list(a.start), "path": [list(p) for p in a.path]}
            for a in geometry.areas
        ],
        "lifts": [
            {
                "name": lift.name,
                "x": lift.approach.x,
                "y": lift.approach.y,
                "door_width": lift.door_width,
                "car_width": lift.car_width,
            }
            for lift in geometry.lifts
        ],
        "lobbies": [{"name": b.name, "x": b.point.x, "y": b.point.y} for b in geometry.lobbies],
        "home_dock": geometry.home_dock,
    }
    return json.dumps(document, indent=1) + "\n"
