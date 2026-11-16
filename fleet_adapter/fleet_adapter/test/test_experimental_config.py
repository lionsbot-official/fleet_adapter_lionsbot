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

"""Unit tests for the `experimental` section of the fleet config."""

from pathlib import Path

import pytest
import yaml

from ..utils.experimental_config import EXPERIMENTAL_KEY
from ..utils.experimental_config import ExperimentalFeatureNotBooleanError
from ..utils.experimental_config import ExperimentalFeatures
from ..utils.experimental_config import ExperimentalSectionNotMappingError
from ..utils.experimental_config import InvalidExperimentalConfigError
from ..utils.experimental_config import LIFT_COMMANDS_KEY
from ..utils.experimental_config import UnknownExperimentalFeatureError
from ..utils.experimental_config import experimental_features_from_config

FIXTURES = Path(__file__).parent / 'fixtures'
CASES = yaml.safe_load((FIXTURES / 'experimental_config.yaml').read_text())
CONFIG_TEMPLATE = (
    Path(__file__).parents[2] / 'configs' / 'config_template.yaml')


def case_ids(cases):
    return [case['name'] for case in cases]


@pytest.mark.parametrize('case', CASES['valid'], ids=case_ids(CASES['valid']))
def test_valid_section_gives_the_lift_commands_flag(case):
    features = experimental_features_from_config(case['config'])

    assert features == ExperimentalFeatures(
        lift_commands=case['lift_commands'])


@pytest.mark.parametrize(
    'case', CASES['not_a_mapping'], ids=case_ids(CASES['not_a_mapping']))
def test_section_that_is_not_a_mapping_is_refused(case):
    with pytest.raises(ExperimentalSectionNotMappingError) as raised:
        experimental_features_from_config(case['config'])

    assert raised.value.value == case['config'][EXPERIMENTAL_KEY]


@pytest.mark.parametrize(
    'case', CASES['unknown_feature'], ids=case_ids(CASES['unknown_feature']))
def test_unknown_feature_is_refused(case):
    with pytest.raises(UnknownExperimentalFeatureError) as raised:
        experimental_features_from_config(case['config'])

    assert raised.value.unknown_keys == case['unknown_keys']


@pytest.mark.parametrize(
    'case', CASES['not_a_boolean'], ids=case_ids(CASES['not_a_boolean']))
def test_flag_that_is_not_a_boolean_is_refused(case):
    with pytest.raises(ExperimentalFeatureNotBooleanError) as raised:
        experimental_features_from_config(case['config'])

    assert raised.value.feature == LIFT_COMMANDS_KEY
    assert raised.value.value == case['value']


@pytest.mark.parametrize('error_type', [
    ExperimentalSectionNotMappingError,
    UnknownExperimentalFeatureError,
    ExperimentalFeatureNotBooleanError,
])
def test_each_refusal_is_an_invalid_experimental_config_error(error_type):
    assert issubclass(error_type, InvalidExperimentalConfigError)
    assert issubclass(InvalidExperimentalConfigError, ValueError)


def test_config_template_keeps_lift_commands_off():
    template = yaml.safe_load(CONFIG_TEMPLATE.read_text())

    assert template[EXPERIMENTAL_KEY] == {LIFT_COMMANDS_KEY: False}
    assert experimental_features_from_config(template) == \
        ExperimentalFeatures(lift_commands=False)
