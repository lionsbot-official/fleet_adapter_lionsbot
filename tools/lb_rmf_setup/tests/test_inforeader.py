import json

import pytest
from fixtures import OFFICE_NEW, R3SCP_GEOMETRY, R5_GEOMETRY

from lbrmf import errors
from lbrmf.frame import MapFrame, Point
from lbrmf.geometry import CleaningArea, MapGeometry, Marker
from lbrmf.inforeader import read_geometry, read_info


@pytest.mark.parametrize(("robot", "expected"), [("r3scp", R3SCP_GEOMETRY), ("r5", R5_GEOMETRY)])
def test_read_info(robot, expected):
    geometry = read_info((OFFICE_NEW / f"{robot}.info").read_text(), "rmf_office_new")

    assert geometry == expected


def test_read_info_markers_of_every_kind_and_custom_paths():
    info = {
        "meta_data": {"resolution": 0.05, "ox": 0, "oy": 0, "width": 10, "height": 20},
        "markers": {
            "dock": [{"name": "d", "coords": "1 2", "angle": 0.5}],
            "glassdoor": [{"name": "g", "coords": "3 4"}],
        },
        "zones": {"clean": [], "restrict": [{"name": "r", "coords": "1 1", "paths": ""}]},
        "custom_paths": [{"name": "p", "coords": "5 6 7 8", "paths": "5 6 7 8"}],
    }

    geometry = read_info(json.dumps(info), "m")

    assert geometry == MapGeometry(
        map_name="m",
        frame=MapFrame(0.05, 0.0, 0.0, 10, 20),
        markers=(
            Marker("d", Point(1, 2), 0.5, dock=True),
            Marker("g", Point(3, 4), 0.0, dock=False),
        ),
        areas=(CleaningArea("p", Point(5, 6), (Point(5, 6), Point(7, 8))),),
    )


@pytest.mark.parametrize(
    "text",
    [
        "[]",
        "{",
        '{"markers": {"dock": [{"name": "d", "coords": "1"}]}}',
        '{"markers": {"dock": [{"coords": "1 2"}]}}',
        '{"zones": {"clean": [{"name": "z", "coords": "1 2", "paths": "1 2 3"}]}}',
        '{"zones": {"clean": [{"name": "z", "coords": "a b", "paths": ""}]}}',
        '{"meta_data": {"resolution": 0.05}}',
        '{"zones": {"clean": [{"name": "z", "coords": "", "paths": ""}]}}',
        '{"markers": ["dock"]}',
        '{"markers": {"poi": [{"name": "p", "coords": "1 2 3 4"}]}}',
        '{"markers": {"dock": [{"name": "d", "coords": "1 2", "angle": "east"}]}}',
    ],
    ids=[
        "not an object",
        "not json",
        "one coordinate",
        "no name",
        "odd path",
        "not numbers",
        "partial frame",
        "zone without points",
        "markers not a mapping",
        "marker with two points",
        "angle not a number",
    ],
)
def test_malformed_info(text):
    with pytest.raises(errors.MalformedMapFileError):
        read_info(text, "m")


def test_info_without_frame():
    assert read_info("{}", "m") == MapGeometry("m", None, (), ())


def _geometry_document(**overrides):
    document = {
        "schema": 1,
        "map_name": "rmf_office_new",
        "frame": {
            "resolution": 0.05,
            "origin_x": -17.56,
            "origin_y": -20.93,
            "width": 647,
            "height": 1193,
        },
        "markers": [
            {"name": "refuel_891", "x": 337, "y": 796, "angle": 0.08300105075706732, "dock": True}
        ],
        "areas": [{"name": "z", "start": [1, 2], "path": [[1, 2], [3, 4]]}],
    }
    document.update(overrides)
    return json.dumps(document)


def test_read_geometry():
    assert read_geometry(_geometry_document()) == MapGeometry(
        map_name="rmf_office_new",
        frame=R3SCP_GEOMETRY.frame,
        markers=(R3SCP_GEOMETRY.markers[0],),
        areas=(CleaningArea("z", Point(1, 2), (Point(1, 2), Point(3, 4))),),
    )


def test_read_geometry_without_frame():
    assert read_geometry(_geometry_document(frame=None)).frame is None


@pytest.mark.parametrize("schema", [2, None, "1"])
def test_unsupported_geometry_schema(schema):
    with pytest.raises(errors.UnsupportedGeometrySchemaError):
        read_geometry(_geometry_document(schema=schema))


@pytest.mark.parametrize(
    "overrides",
    [{"markers": [{"name": "a"}]}, {"areas": [{"name": "z", "start": [1]}]}, {"map_name": 5}],
    ids=["marker without point", "short start", "map name not a string"],
)
def test_malformed_geometry(overrides):
    with pytest.raises(errors.MalformedMapFileError):
        read_geometry(_geometry_document(**overrides))
