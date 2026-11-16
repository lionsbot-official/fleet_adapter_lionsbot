import math

import pytest
import yaml
from fixtures import (
    OFFICE_NEW,
    OFFICE_NEW_LEVEL,
    R3SCP_GEOMETRY,
    R5_GEOMETRY,
    TESTDATA,
    office_new_building,
    office_new_transform,
)

from lbrmf import errors
from lbrmf.building import BuildingFile, Param, ParamType
from lbrmf.constants import VertexParam
from lbrmf.frame import Point
from lbrmf.geometry import CleaningArea, MapGeometry, Marker
from lbrmf.plan import CleaningPath, make_plan, unified_diff

EMPTY_LEVEL = TESTDATA / "building" / "empty_level.building.yaml"
DOCK_PARAMS = {
    VertexParam.DOCK_NAME: Param(ParamType.STRING, "refuel_891"),
    VertexParam.IS_CHARGER: Param(ParamType.BOOL, True),
    VertexParam.ROBOT_DOCK_NAME: Param(ParamType.STRING, "refuel_891"),
}
ZONE_PARAMS = {
    VertexParam.IS_CLEANING_ZONE: Param(ParamType.BOOL, True),
    VertexParam.ROBOT_ZONE_NAME: Param(ParamType.STRING, "cZone_356"),
}


def _plan(building, geometry=R3SCP_GEOMETRY, robot_type="r3scp", update_positions=False):
    return make_plan(
        building,
        OFFICE_NEW_LEVEL,
        geometry,
        office_new_transform(robot_type),
        update_positions=update_positions,
    )


def _empty_level_plan(geometry, update_positions=False):
    return make_plan(
        BuildingFile.parse(EMPTY_LEVEL.read_text()),
        "L1",
        geometry,
        office_new_transform("r3scp"),
        update_positions=update_positions,
    )


@pytest.mark.parametrize(
    ("robot_type", "geometry"), [("r3scp", R3SCP_GEOMETRY), ("r5", R5_GEOMETRY)]
)
def test_keeps_hand_placed_positions_and_reports_the_move(robot_type, geometry):
    building = office_new_building()
    transform = office_new_transform(robot_type)

    plan = _plan(building, geometry, robot_type)

    assert plan.added == []
    assert plan.changed == []
    assert building.render({OFFICE_NEW_LEVEL: plan.level_edit()}) == building.render({})
    by_name = {vertex.name: vertex for vertex in building.vertices(OFFICE_NEW_LEVEL)}
    starts = {marker.name: marker.point for marker in geometry.markers}
    starts.update({area.name: area.start for area in geometry.areas})
    assert {drift.name for drift in plan.kept_positions} == set(starts)
    for drift in plan.kept_positions:
        target = transform.robot_to_floorplan_pixels(starts[drift.name])
        existing = by_name[drift.name]
        expected = math.dist((existing.x, existing.y), target)
        assert drift.pixels == pytest.approx(expected)
        assert drift.meters == pytest.approx(expected * transform.floorplan_scale)


def test_update_positions_moves_only_the_position():
    building = office_new_building()
    transform = office_new_transform("r3scp")

    plan = _plan(building, update_positions=True)

    assert plan.kept_positions == []
    changed = {change.after.name: change for change in plan.changed}
    assert set(changed) == {"refuel_891", "location_394", "cZone_356"}
    for change in plan.changed:
        assert change.after.params == change.before.params
        assert change.after.z == change.before.z
    expected = transform.robot_to_floorplan_pixels(Point(337, 796))
    assert (changed["refuel_891"].after.x, changed["refuel_891"].after.y) == pytest.approx(expected)


def test_new_names_are_appended_with_their_params():
    plan = _empty_level_plan(R3SCP_GEOMETRY)
    transform = office_new_transform("r3scp")

    by_name = {vertex.name: vertex for vertex in plan.added}

    assert [vertex.name for vertex in plan.added] == ["refuel_891", "location_394", "cZone_356"]
    assert by_name["refuel_891"].params == DOCK_PARAMS
    assert by_name["location_394"].params == {}
    assert by_name["cZone_356"].params == ZONE_PARAMS
    assert all(vertex.z == 0 for vertex in plan.added)
    assert (by_name["cZone_356"].x, by_name["cZone_356"].y) == pytest.approx(
        transform.robot_to_floorplan_pixels(Point(359, 650))
    )


@pytest.mark.parametrize("update_positions", [False, True])
def test_a_second_run_changes_nothing(update_positions):
    first = _empty_level_plan(R5_GEOMETRY, update_positions)
    after_first = BuildingFile.parse(
        BuildingFile.parse(EMPTY_LEVEL.read_text()).render({"L1": first.level_edit()})
    )

    second = make_plan(
        after_first,
        "L1",
        R5_GEOMETRY,
        office_new_transform("r3scp"),
        update_positions=update_positions,
    )

    assert not second.has_changes
    assert after_first.render({"L1": second.level_edit()}) == after_first.render({})


def test_existing_params_are_kept_and_ours_are_merged():
    text = EMPTY_LEVEL.read_text().replace(
        "vertices: []",
        "vertices:\n"
        "      - [10, 20, 1.5, refuel_891, {dock_name: [1, old], speed_limit: [3, 0.5]}]",
    )

    plan = _empty_level_plan_on(text)

    (change,) = plan.changed
    assert change.after.params == {**DOCK_PARAMS, "speed_limit": Param(ParamType.DOUBLE, 0.5)}
    assert (change.after.x, change.after.y, change.after.z) == (10, 20, 1.5)


def _empty_level_plan_on(text):
    return make_plan(
        BuildingFile.parse(text),
        "L1",
        MapGeometry("m", None, markers=R3SCP_GEOMETRY.markers[:1], areas=()),
        office_new_transform("r3scp"),
        update_positions=False,
    )


def test_the_dock_heading_is_reported_not_written():
    plan = _empty_level_plan(R3SCP_GEOMETRY)

    assert plan.dock_headings == {
        "refuel_891": office_new_transform("r3scp").heading(0.08300105075706732)
    }


def test_cleaning_paths_match_the_committed_dock_summary():
    committed = yaml.safe_load((OFFICE_NEW / "dock_summary.yaml").read_text())["r3"]

    plan = _plan(office_new_building())

    (path,) = plan.cleaning_paths
    expected = committed["cZone_356"]
    assert path._replace(path=[]) == CleaningPath(
        name="cZone_356",
        level_name=expected["level_name"],
        finish_waypoint=expected["finish_waypoint"],
        robot_zone_name=expected["robot_zone_name"],
        path=[],
    )
    flat = [value for point in path.path for value in point]
    expected_flat = [value for point in expected["path"] for value in point]
    assert flat == pytest.approx(expected_flat, abs=1e-12)


def test_vertices_not_on_this_map_are_reported_not_deleted():
    plan = _plan(office_new_building())

    assert plan.not_on_map == ["charging_1", "cZone_001", "cZone_002"]


def test_area_without_a_path_is_a_warning():
    geometry = MapGeometry("m", None, markers=(), areas=(CleaningArea("z1", Point(1, 2), ()),))

    plan = _empty_level_plan(geometry)

    assert plan.cleaning_paths[0].path == []
    assert plan.warnings == ["z1 has no cleaning path; its dock summary path is empty"]


@pytest.mark.parametrize(
    "geometry",
    [
        MapGeometry(
            "m",
            None,
            markers=(Marker("a", Point(1, 1), 0.0, dock=False),),
            areas=(CleaningArea("a", Point(1, 1), ()),),
        ),
        MapGeometry(
            "m",
            None,
            markers=(
                Marker("a", Point(1, 1), 0.0, dock=True),
                Marker("a", Point(2, 2), 0.0, dock=False),
            ),
            areas=(),
        ),
    ],
    ids=["marker and zone", "two markers"],
)
def test_name_collision_on_the_map_stops_the_plan(geometry):
    with pytest.raises(errors.NameCollisionError):
        _empty_level_plan(geometry)


def test_duplicate_vertex_name_in_the_building_file():
    text = EMPTY_LEVEL.read_text().replace(
        "vertices: []",
        "vertices:\n      - [1, 2, 0, refuel_891]\n      - [3, 4, 0, refuel_891]",
    )

    with pytest.raises(errors.DuplicateVertexNameError):
        _empty_level_plan_on(text)


def test_empty_names_are_ignored():
    text = EMPTY_LEVEL.read_text().replace(
        "vertices: []", 'vertices:\n      - [1, 2, 0, ""]\n      - [3, 4, 0, ""]'
    )

    plan = _empty_level_plan_on(text)

    assert [vertex.name for vertex in plan.added] == ["refuel_891"]


def test_unified_diff_names_the_file():
    lines = unified_diff("maps/b/x.building.yaml", "a\nb\n", "a\nc\n")

    assert lines == [
        "--- maps/b/x.building.yaml",
        "+++ maps/b/x.building.yaml (after)",
        "@@ -1,2 +1,2 @@",
        " a",
        "-b",
        "+c",
    ]


def test_summary_lines():
    plan = _plan(office_new_building())

    lines = plan.summary()

    assert lines[0] == "level L8: 0 new, 0 changed, 3 unchanged"
    assert any(line.startswith("  kept position: refuel_891 would move ") for line in lines)
    assert any(
        line.startswith("  dock: refuel_891 is reached from location_394. The robot arrives at ")
        for line in lines
    )
    assert "  not on this robot map (kept): charging_1, cZone_001, cZone_002" in lines


PDD_FRAME = R3SCP_GEOMETRY.frame
# A real R3 vacuum map: the dock and the point LionsBot keeps in front of it.
PDD_DOCK = Marker("charging_793", Point(388, 474), angle=2.17042138521799, dock=True)
PDD_APPROACH = Marker("location_303", Point(380, 462), angle=-0.971171268371804, dock=False)


def _docks_geometry(*markers, frame=PDD_FRAME):
    return MapGeometry("map", frame, tuple(markers), ())


@pytest.mark.parametrize(
    ("geometry", "pairs"),
    [
        (R3SCP_GEOMETRY, [("refuel_891", "location_394")]),
        (R5_GEOMETRY, [("charging_1", "location_001")]),
        (_docks_geometry(PDD_DOCK, PDD_APPROACH), [("charging_793", "location_303")]),
    ],
)
def test_finds_the_point_in_front_of_each_dock(geometry, pairs):
    plan = _empty_level_plan(geometry)

    assert [(a.dock, a.approach, a.created) for a in plan.dock_approaches] == [
        (dock, approach, False) for dock, approach in pairs
    ]


def test_the_robot_arrives_facing_the_dock():
    transform = office_new_transform("r3scp")

    plan = _empty_level_plan(_docks_geometry(PDD_DOCK, PDD_APPROACH))

    (approach,) = plan.dock_approaches
    expected = transform.heading(PDD_DOCK.angle) + math.pi
    assert math.cos(approach.arrival_heading) == pytest.approx(math.cos(expected))
    assert math.sin(approach.arrival_heading) == pytest.approx(math.sin(expected))


def test_adds_one_lane_from_the_approach_point_to_the_dock():
    plan = _empty_level_plan(_docks_geometry(PDD_DOCK, PDD_APPROACH))

    names = [vertex.name for vertex in plan.added]
    assert plan.lanes == [
        [
            names.index("location_303"),
            names.index("charging_793"),
            {"bidirectional": [4, True], "graph_idx": [2, 0]},
        ]
    ]


def test_an_existing_dock_lane_is_kept():
    plan = _plan(office_new_building())

    assert plan.lanes == []


@pytest.mark.parametrize(
    "other",
    [
        # Faces the same way as the dock, so it does not face the dock.
        PDD_APPROACH._replace(angle=PDD_DOCK.angle),
        # Beside the dock, not in front of it.
        PDD_APPROACH._replace(point=Point(400, 466)),
        # Too far away.
        PDD_APPROACH._replace(point=Point(360, 433)),
    ],
)
def test_adds_an_approach_point_when_no_point_faces_the_dock(other):
    plan = _empty_level_plan(_docks_geometry(PDD_DOCK, other))
    transform = office_new_transform("r3scp")

    (approach,) = plan.dock_approaches
    assert (approach.approach, approach.created) == ("charging_793_approach", True)
    added = {vertex.name: vertex for vertex in plan.added}
    meters = 0.7 / PDD_FRAME.resolution
    expected = transform.robot_to_floorplan_pixels(
        Point(
            PDD_DOCK.point.x + meters * math.cos(PDD_DOCK.angle),
            PDD_DOCK.point.y - meters * math.sin(PDD_DOCK.angle),
        )
    )
    new = added["charging_793_approach"]
    assert (new.x, new.y) == pytest.approx(expected)
    assert new.params == {}
    assert len(plan.lanes) == 1


def test_a_map_without_a_frame_gets_no_dock_lane():
    plan = _empty_level_plan(_docks_geometry(PDD_DOCK, PDD_APPROACH, frame=None))

    assert plan.dock_approaches == []
    assert plan.lanes == []
    assert any("no frame" in warning for warning in plan.warnings)


def test_a_rounding_move_is_not_reported():
    transform = office_new_transform("r3scp")
    dock, approach = R3SCP_GEOMETRY.markers
    at_dock = transform.robot_to_floorplan_pixels(dock.point)
    at_approach = transform.robot_to_floorplan_pixels(approach.point)
    text = EMPTY_LEVEL.read_text().replace(
        "vertices: []",
        "vertices:\n"
        f"      - [{at_dock.x + 0.1}, {at_dock.y}, 0, refuel_891]\n"
        f"      - [{at_approach.x + 5}, {at_approach.y}, 0, location_394]",
    )

    plan = make_plan(
        BuildingFile.parse(text),
        "L1",
        R3SCP_GEOMETRY._replace(areas=()),
        transform,
        update_positions=False,
    )

    assert [kept.name for kept in plan.kept_positions] == ["location_394"]
