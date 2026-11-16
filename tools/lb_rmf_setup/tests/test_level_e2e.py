"""On a level that is not the reference level, the adapter and RMF's nav graph agree."""

import itertools
import json
import math

import pytest
import yaml
from test_frame import _adapter_transform

import lb_rmf_setup
from lbrmf.constants import ExitCode

building_map = pytest.importorskip("building_map.building", reason="needs rmf_building_map_tools")

REFERENCE_SCALE = 0.05
REFERENCE_FIDUCIALS = [(300.0, 400.0), (700.0, 900.0)]
VALUES = {"rotation_degrees": 30.0, "scale": 0.05, "tx_meters": 3.0, "ty_meters": 4.0}
ROBOT_POINTS = {"p_a": (100, 200), "p_b": (400, 150), "p_c": (250, 600)}
LANE = {"bidirectional": [4, True], "graph_idx": [2, 0]}


def _level(fiducials, scale):
    return {
        "drawing": {"filename": "floor.png"},
        "elevation": 0,
        "fiducials": [[x, y, f"f{i}"] for i, (x, y) in enumerate(fiducials)],
        "lanes": [],
        "measurements": [[0, 1, {"distance": [3, 5.0]}]],
        "vertices": [[100.0, 100.0, 0, ""], [100.0 + 5.0 / scale, 100.0, 0, ""]],
    }


def _export(site, zoom, turn_degrees, offset, level_transform=None):
    """L2's image shows the same place as L1's, zoomed, turned and moved by offset pixels.

    Without a level_transform the export fills it in from the fiducials.
    """
    turn = math.radians(turn_degrees)
    cos, sin = zoom * math.cos(turn), zoom * math.sin(turn)
    fiducials = [
        (cos * x - sin * y + offset[0], sin * x + cos * y + offset[1])
        for x, y in REFERENCE_FIDUCIALS
    ]
    scale = REFERENCE_SCALE / zoom
    level_config = {"transform_values": VALUES}
    if level_transform is not None:
        level_config["level_transform"] = level_transform
    building_path = site / "site.building.yaml"
    levels = {"L1": _level(REFERENCE_FIDUCIALS, REFERENCE_SCALE), "L2": _level(fiducials, scale)}
    building_path.write_text(
        yaml.safe_dump({"coordinate_system": "reference_image", "name": "site", "levels": levels})
    )
    (site / "config.yaml").write_text(
        yaml.safe_dump({"rmf_fleet": {"name": "r3"}, "map_transform": {"L2": level_config}})
    )
    markers = [
        {"name": name, "x": x, "y": y, "angle": 0.0, "dock": False}
        for name, (x, y) in ROBOT_POINTS.items()
    ]
    (site / "map.geometry.json").write_text(
        json.dumps({"schema": 1, "map_name": "m", "frame": None, "markers": markers, "areas": []})
    )
    arguments = ["--building", str(building_path), "--config", str(site / "config.yaml")]
    geometry = ["--level", "L2", "--from-geometry", str(site / "map.geometry.json")]
    return lb_rmf_setup.main(["export", *arguments, *geometry])


def _max_error(site, *case):
    assert _export(site, *case) == ExitCode.OK
    building_path = site / "site.building.yaml"

    # Traffic editor pass 2: RMF keeps only the vertices that a lane joins.
    building = yaml.safe_load(building_path.read_text())
    vertices = building["levels"]["L2"]["vertices"]
    placed = [index for index, vertex in enumerate(vertices) if vertex[3] in ROBOT_POINTS]
    building["levels"]["L2"]["lanes"] = [[a, b, LANE] for a, b in itertools.pairwise(placed)]
    graph = building_map.Building(building).generate_nav_graphs()["0"]

    nav = {vertex[2]["name"]: vertex[:2] for vertex in graph["levels"]["L2"]["vertices"]}
    assert set(nav) == set(ROBOT_POINTS)
    # 1 is the scale the adapter takes when level_transform has none (fleet_adapter.py).
    coord, adapter = _adapter_transform(site / "config.yaml", "L2", 1)
    errors = []
    for name, point in ROBOT_POINTS.items():
        meters = adapter.robot_to_rmf_meters(coord(*point))
        errors.append(math.dist(nav[name], (meters.x, meters.y)))
    return max(errors)


@pytest.mark.parametrize(
    ("zoom", "offset"),
    [(1, (0, 0)), (1, (250, -120)), (1.25, (250, -120))],
    ids=["same frame", "moved", "moved and zoomed"],
)
def test_adapter_matches_the_nav_graph_with_the_level_transform_of_export(tmp_path, zoom, offset):
    assert _max_error(tmp_path, zoom, 0, offset) < 1e-6
    filled = yaml.safe_load((tmp_path / "config.yaml").read_text())["map_transform"]["L2"]
    assert filled["level_transform"] == pytest.approx(
        {"scale": REFERENCE_SCALE / zoom, "tx_pixels": offset[0], "ty_pixels": offset[1]}
    )


# These pin the limits the config template states. A change that lifts one should update both.
def test_moved_image_with_an_empty_level_transform_is_off_by_the_offset(tmp_path):
    assert _max_error(tmp_path, 1, 0, (250, -120), {}) == pytest.approx(
        math.hypot(250, 120) * REFERENCE_SCALE
    )


def test_export_refuses_a_turned_image(tmp_path, capsys):
    assert _export(tmp_path, 1, 20, (250, -120)) == ExitCode.LOCAL_INPUT
    assert "rotated floor plan is not supported" in capsys.readouterr().err
    assert "level_transform" not in (tmp_path / "config.yaml").read_text()


def test_turned_image_with_a_level_transform_by_hand_is_far_off(tmp_path):
    by_hand = {"scale": REFERENCE_SCALE, "tx_pixels": 250, "ty_pixels": -120}
    assert _max_error(tmp_path, 1, 20, (250, -120), by_hand) > 1
