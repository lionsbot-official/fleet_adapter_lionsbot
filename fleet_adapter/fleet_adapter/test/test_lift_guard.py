# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the lift car checks before and during a lift move."""

from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from ..enums.enums import LiftCarProblem
from ..LiftGuard import car_not_ready_problem
from ..LiftGuard import car_unsafe_problem
from ..LiftGuard import LIFT_STATE_MAX_AGE_SECONDS
from ..LiftGuard import LiftStateCache

FIXTURE = yaml.safe_load(
    (Path(__file__).parent / 'fixtures' / 'lift_states.yaml').read_text())
LIFT = FIXTURE['lift']
FLOOR = FIXTURE['floor']
SESSION_ID = FIXTURE['session_id']
RECEIVED_AT = 500.0


def case_ids(cases):
    return [case['name'] for case in cases]


def cached_state(state):
    """Return the state as the cache holds it after one lift_states message."""
    cache = LiftStateCache(clock=lambda: RECEIVED_AT)
    if state is not None:
        cache.update(SimpleNamespace(**state))
    return cache.get(LIFT)


def expected_problem(case):
    return LiftCarProblem(case['problem']) if case['problem'] else None


def test_fixture_age_limit_is_the_limit_in_use():
    assert FIXTURE['max_age_seconds'] == LIFT_STATE_MAX_AGE_SECONDS


@pytest.mark.parametrize(
    'case', FIXTURE['ready'], ids=case_ids(FIXTURE['ready']))
def test_car_is_ready_only_at_the_floor_open_stopped_and_ours(case):
    problem = car_not_ready_problem(
        cached_state(case['state']),
        FLOOR,
        SESSION_ID,
        now=RECEIVED_AT + case['age_seconds'])

    assert problem == expected_problem(case)


@pytest.mark.parametrize(
    'case', FIXTURE['unsafe'], ids=case_ids(FIXTURE['unsafe']))
def test_move_is_unsafe_when_the_car_moves_or_the_session_changes(case):
    problem = car_unsafe_problem(
        cached_state(case['state']), FLOOR, SESSION_ID)

    assert problem == expected_problem(case)


def test_cache_keeps_the_latest_state_of_each_lift():
    now = [RECEIVED_AT]
    cache = LiftStateCache(clock=lambda: now[0])
    cache.update(SimpleNamespace(**FIXTURE['ready_state']))
    now[0] += 3.0
    cache.update(SimpleNamespace(
        **{**FIXTURE['ready_state'], 'current_floor': 'L3'}))
    cache.update(SimpleNamespace(
        **{**FIXTURE['ready_state'], 'lift_name': 'LiftB'}))

    assert cache.get(LIFT).current_floor == 'L3'
    assert cache.get(LIFT).received_at == RECEIVED_AT + 3.0
    assert cache.get('LiftB').current_floor == FLOOR
    assert cache.get('LiftC') is None
