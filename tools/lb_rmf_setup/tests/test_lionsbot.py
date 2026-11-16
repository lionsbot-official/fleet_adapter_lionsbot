import json
from unittest import mock

import pytest
from apiserver import Reply, ScriptedServer
from fixtures import OFFICE_NEW, TESTDATA, make_png

from lbrmf import errors
from lbrmf.api import ApiClient, Credentials
from lbrmf.frame import MapFrame
from lbrmf.inforeader import read_info
from lbrmf.lionsbot import LionsBotMaps, MapSummary, RobotSummary, sanitize_file_name

API_DATA = TESTDATA / "api"
MAP_ID = "5d1f6c1a-8e37-4f0e-9b8e-1f2a3b4c5d6e"
ROBOT_ID = "37defd58-0f13-4e68-99be-f7987b3a6900"
LOGIN = "/openapi/v2/edge/auth/login"


def _fixture(name, server=None):
    text = (API_DATA / name).read_text()
    if server is not None:
        text = text.replace("{server}", server.url)
    return json.loads(text)


def _source(server):
    server.script(LOGIN, Reply(200, _fixture("login_response.json")))
    client = ApiClient(
        Credentials.for_password(base_url=server.url, user="u", password="p"),
        sleep=mock.create_autospec(lambda seconds: None),
    )
    return LionsBotMaps(client)


def test_robots_pages_until_the_last():
    first = {**_fixture("robots_page.json"), "last": False}
    with ScriptedServer() as server:
        server.script(
            "/openapi/v2/robot", Reply(200, first), Reply(200, _fixture("robots_page.json"))
        )

        robots = _source(server).robots()

        assert (
            robots
            == [RobotSummary(ROBOT_ID, "R3SCP-B02357", "R3", "ScrubPro", "1.16.2", "Office")] * 2
        )
        assert [r.path for r in server.requests_to("/openapi/v2/robot")] == [
            "/openapi/v2/robot?page=0&size=100",
            "/openapi/v2/robot?page=1&size=100",
        ]


@pytest.mark.parametrize(
    ("ident", "expected_calls"),
    [(ROBOT_ID, 0), ("R3SCP-B02357", 1)],
    ids=["uuid", "serial"],
)
def test_resolve_robot(ident, expected_calls):
    with ScriptedServer() as server:
        server.script("/openapi/v2/robot", Reply(200, _fixture("robots_page.json")))
        source = _source(server)

        assert source.resolve_robot_id(ident) == ROBOT_ID
        assert len(server.requests_to("/openapi/v2/robot")) == expected_calls


def test_unknown_robot():
    with ScriptedServer() as server:
        server.script("/openapi/v2/robot", Reply(200, _fixture("robots_page.json")))

        with pytest.raises(errors.RobotNotFoundError):
            _source(server).resolve_robot_id("R5-0000001")


def test_maps():
    with ScriptedServer() as server:
        server.script(
            f"/openapi/v2/robot/{ROBOT_ID}/map", Reply(200, _fixture("r3scp_maps.json", server))
        )

        maps = _source(server).maps(ROBOT_ID)

        assert maps[0] == MapSummary(
            id=MAP_ID,
            name="rmf_office_new",
            level="8",
            building="Office",
            last_modified="2026-11-01T02:03:04.000+00:00",
            frame=MapFrame(0.05, -17.56, -20.93, 647, 1193),
            occupied_thresh=0.65,
            free_thresh=0.195,
            image_link=f"{server.url}/signed/rmf_office_new.png?X-Signature=abc",
        )
        assert maps[1].frame is None


def _maps_by_name(server):
    server.script(
        f"/openapi/v2/robot/{ROBOT_ID}/map", Reply(200, _fixture("r3scp_maps.json", server))
    )
    source = _source(server)
    return {item.name: item for item in source.maps(ROBOT_ID)}, source


def test_geometry_matches_the_info_file():
    expected = read_info((OFFICE_NEW / "r3scp.info").read_text(), "rmf_office_new")
    with ScriptedServer() as server:
        maps, source = _maps_by_name(server)
        server.script(
            f"/openapi/v2/robot/map/{MAP_ID}/markers", Reply(200, _fixture("r3scp_markers.json"))
        )
        server.script(
            f"/openapi/v2/robot/map/{MAP_ID}/zones", Reply(200, _fixture("r3scp_zones.json"))
        )
        server.script(
            f"/openapi/v2/robot/map/{MAP_ID}/markers/homepoi",
            Reply(200, {"type": "dock", "name": "refuel_891", "angle": 0.083}),
        )

        geometry, warnings = source.geometry(maps["rmf_office_new"])
        assert geometry.home_dock == "refuel_891"

        (zones,) = server.requests_to(f"/openapi/v2/robot/map/{MAP_ID}/zones")
        assert zones.path.endswith("?includePaths=true")
    assert warnings == []
    assert geometry.map_name == expected.map_name
    assert geometry.frame == expected.frame
    assert [(m.name, m.dock, m.angle) for m in geometry.markers] == [
        (m.name, m.dock, m.angle) for m in expected.markers
    ]
    for ours, theirs in zip(geometry.markers, expected.markers, strict=True):
        assert ours.point == pytest.approx(theirs.point, abs=0.02)
    (area,) = geometry.areas
    assert area.name == expected.areas[0].name
    assert area.start == pytest.approx(expected.areas[0].start, abs=0.02)
    flat = [value for point in area.path for value in point]
    expected_flat = [value for point in expected.areas[0].path for value in point]
    assert flat == pytest.approx(expected_flat, abs=0.02)


def test_geometry_of_a_map_without_frame():
    with ScriptedServer() as server:
        maps, source = _maps_by_name(server)

        with pytest.raises(errors.MapWithoutFrameError):
            source.geometry(maps["old_map"])


@pytest.mark.parametrize(
    ("body", "error"),
    [
        (b"not a png at all", errors.NotAPngError),
        (make_png(10, 10), errors.ImageSizeError),
    ],
    ids=["not a png", "wrong size"],
)
def test_image_checks(body, error):
    with ScriptedServer() as server:
        maps, source = _maps_by_name(server)
        server.script("/signed/rmf_office_new.png", Reply(200, body))

        with pytest.raises(error):
            source.image(maps["rmf_office_new"])


def test_image():
    png = make_png(647, 1193)
    with ScriptedServer() as server:
        maps, source = _maps_by_name(server)
        server.script("/signed/rmf_office_new.png", Reply(200, png))

        assert source.image(maps["rmf_office_new"]) == png


def test_image_without_link():
    with ScriptedServer() as server:
        maps, source = _maps_by_name(server)

        with pytest.raises(errors.MapWithoutImageError):
            source.image(maps["old_map"])


@pytest.mark.parametrize(
    ("name", "expected"),
    [
        ("rmf_office_new", "rmf_office_new"),
        ("L3 / east..wing", "L3___east..wing"),
        ("../x", ".._x"),
    ],
)
def test_sanitize_file_name(name, expected):
    assert sanitize_file_name(name) == expected


@pytest.mark.parametrize(
    "body",
    [{"content": "x"}, [1], {"content": [{"id": 1}]}],
    ids=["content not a list", "not a page", "robot without serial"],
)
def test_malformed_robot_page(body):
    with ScriptedServer() as server:
        server.script("/openapi/v2/robot", Reply(200, body))

        with pytest.raises(errors.ApiResponseError):
            _source(server).robots()


def test_malformed_map_list():
    with ScriptedServer() as server:
        server.script(f"/openapi/v2/robot/{ROBOT_ID}/map", Reply(200, [{"name": "no id"}]))

        with pytest.raises(errors.ApiResponseError):
            _source(server).maps(ROBOT_ID)


def test_malformed_markers():
    with ScriptedServer() as server:
        maps, source = _maps_by_name(server)
        server.script(
            f"/openapi/v2/robot/map/{MAP_ID}/markers",
            Reply(200, [{"name": "m", "point": {"x": 1}}]),
        )
        server.script(f"/openapi/v2/robot/map/{MAP_ID}/zones", Reply(200, []))

        with pytest.raises(errors.ApiResponseError):
            source.geometry(maps["rmf_office_new"])


def test_items_without_points_are_skipped_with_a_warning():
    with ScriptedServer() as server:
        maps, source = _maps_by_name(server)
        server.script(
            f"/openapi/v2/robot/map/{MAP_ID}/markers",
            Reply(
                200,
                [
                    {"type": "poi", "name": "no_point"},
                    {"type": "dock", "name": "null_point", "point": None},
                    {"type": "loc_point", "name": "ok", "point": {"x": 0, "y": 0}},
                ],
            ),
        )
        server.script(
            f"/openapi/v2/robot/map/{MAP_ID}/zones",
            Reply(200, [{"type": "clean", "name": "empty_zone", "points": [], "pathPoints": []}]),
        )
        server.script(f"/openapi/v2/robot/map/{MAP_ID}/markers/homepoi", Reply(404, {}))

        geometry, warnings = source.geometry(maps["rmf_office_new"])

    assert [marker.name for marker in geometry.markers] == ["ok"]
    assert geometry.areas == ()
    assert warnings == [
        "marker no_point has no point in metres; skipped",
        "marker null_point has no point in metres; skipped",
        "zone empty_zone has no points in metres; skipped",
    ]


def test_map_list_is_read_from_work_site_maps():
    with ScriptedServer() as server:
        server.script(f"/openapi/v2/robot/{ROBOT_ID}/map", Reply(200, [{"id": "x", "name": "y"}]))

        with pytest.raises(errors.ApiResponseError):
            _source(server).maps(ROBOT_ID)


NOT_SYNCED = Reply(400, {"errorCode": "RSE30153"})


@pytest.mark.parametrize(
    "read", [LionsBotMaps.lifts, LionsBotMaps.lift_names], ids=["lifts", "names"]
)
@pytest.mark.parametrize(
    ("reply", "error"),
    [
        (NOT_SYNCED, errors.MapNotSyncedError),
        (Reply(400, {"errorCode": "RSE00001"}), errors.ApiStatusError),
        (Reply(400, b"not json"), errors.ApiStatusError),
    ],
    ids=["map not synced", "other 400", "400 without json"],
)
def test_lift_lobbies_of_a_map_that_is_not_synced(read, reply, error):
    with ScriptedServer() as server:
        maps, source = _maps_by_name(server)
        server.script(f"/openapi/v2/robot/lift/lift-lobbies/{MAP_ID}", reply)

        with pytest.raises(error):
            read(source, maps["rmf_office_new"])


@pytest.mark.parametrize(
    ("reply", "warned"),
    [
        (Reply(404, {}), False),
        (Reply(200, {"type": "loc_point", "name": "not_a_dock"}), False),
        (Reply(500, {}, {"Retry-After": "0"}), True),
    ],
)
def test_a_map_without_a_readable_home_dock_has_none(reply, warned):
    with ScriptedServer() as server:
        maps, source = _maps_by_name(server)
        server.script(f"/openapi/v2/robot/map/{MAP_ID}/markers", Reply(200, []))
        server.script(f"/openapi/v2/robot/map/{MAP_ID}/zones", Reply(200, []))
        for _ in range(5):
            server.script(f"/openapi/v2/robot/map/{MAP_ID}/markers/homepoi", reply)

        geometry, warnings = source.geometry(maps["rmf_office_new"])

    assert geometry.home_dock is None
    assert any("home dock" in warning for warning in warnings) is warned
