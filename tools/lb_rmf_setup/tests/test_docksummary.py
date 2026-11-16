import pytest
import yaml
from fixtures import OFFICE_NEW, R3SCP_GEOMETRY, office_new_building, office_new_transform

from lbrmf import errors
from lbrmf.docksummary import merge, parse, render
from lbrmf.plan import CleaningPath, make_plan

COMMITTED = OFFICE_NEW / "dock_summary.yaml"
NEW_ZONE = CleaningPath(
    name="cZone_9",
    level_name="L8",
    finish_waypoint="cZone_9",
    robot_zone_name="cZone_9",
    path=[(1.5, -2.25, 0.0), (1.75, -2.5, 0.0)],
)


def _r3scp_paths():
    plan = make_plan(
        office_new_building(),
        "L8",
        R3SCP_GEOMETRY,
        office_new_transform("r3scp"),
        update_positions=False,
    )
    return plan.cleaning_paths


def test_committed_file_round_trips_byte_for_byte():
    text = COMMITTED.read_text()

    assert render(parse(text)) == text


def test_merge_replaces_by_name_and_keeps_the_rest():
    before = parse(COMMITTED.read_text())

    after = merge(before, "r3", [*_r3scp_paths(), NEW_ZONE])

    assert after["r5"] == before["r5"]
    assert set(after["r3"]) == {"cZone_356", "cZone_9"}
    assert after["r3"]["cZone_9"] == {
        "finish_waypoint": "cZone_9",
        "level_name": "L8",
        "path": [[1.5, -2.25, 0.0], [1.75, -2.5, 0.0]],
        "robot_zone_name": "cZone_9",
    }
    flat_after = [value for point in after["r3"]["cZone_356"]["path"] for value in point]
    flat_before = [value for point in before["r3"]["cZone_356"]["path"] for value in point]
    assert flat_after == pytest.approx(flat_before, abs=1e-12)
    assert before["r3"].keys() == {"cZone_356"}


def test_entries_this_export_does_not_name_are_kept():
    before = parse(COMMITTED.read_text())

    after = merge(before, "r5", [NEW_ZONE])

    assert set(after["r5"]) == {"cZone_001", "cZone_002", "cZone_9"}
    assert after["r3"] == before["r3"]


@pytest.mark.parametrize("existing", [None, ""])
def test_new_file(existing):
    after = merge(parse(existing), "r3", [NEW_ZONE])

    assert yaml.safe_load(render(after)) == {
        "r3": {
            "cZone_9": {
                "finish_waypoint": "cZone_9",
                "level_name": "L8",
                "path": [[1.5, -2.25, 0.0], [1.75, -2.5, 0.0]],
                "robot_zone_name": "cZone_9",
            }
        }
    }


def test_a_second_merge_changes_nothing():
    once = render(merge(parse(COMMITTED.read_text()), "r3", [*_r3scp_paths(), NEW_ZONE]))

    twice = render(merge(parse(once), "r3", [*_r3scp_paths(), NEW_ZONE]))

    assert twice == once


@pytest.mark.parametrize(
    "text",
    ["- a\n", "r3: [1, 2]\n", "r3:\n  zone: 5\n", "r3: {zone: {path: [}\n"],
    ids=["not a mapping", "fleet not a mapping", "entry not a mapping", "not yaml"],
)
def test_malformed_dock_summary(text):
    with pytest.raises(errors.MalformedDockSummaryError):
        parse(text)
