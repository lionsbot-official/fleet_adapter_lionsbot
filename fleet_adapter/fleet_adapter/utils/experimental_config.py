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

"""Validation of the `experimental` section of the fleet config."""

from dataclasses import dataclass

EXPERIMENTAL_KEY = 'experimental'
LIFT_COMMANDS_KEY = 'lift_commands'

KNOWN_FEATURE_KEYS = (LIFT_COMMANDS_KEY,)


class InvalidExperimentalConfigError(ValueError):
    """The `experimental` section of the fleet config is not valid."""


class ExperimentalSectionNotMappingError(InvalidExperimentalConfigError):
    """The `experimental` section is not a mapping of feature keys."""

    def __init__(self, value):
        super().__init__(
            f'{EXPERIMENTAL_KEY} must be a mapping of feature keys, for '
            f'example "{LIFT_COMMANDS_KEY}: false"; got {value!r}')
        self.value = value


class UnknownExperimentalFeatureError(InvalidExperimentalConfigError):
    """The `experimental` section has a key that is not a feature."""

    def __init__(self, unknown_keys: list[str]):
        super().__init__(
            f'{EXPERIMENTAL_KEY} has unknown keys: '
            f'{", ".join(unknown_keys)}; the known keys are: '
            f'{", ".join(KNOWN_FEATURE_KEYS)}')
        self.unknown_keys = unknown_keys


class ExperimentalFeatureNotBooleanError(InvalidExperimentalConfigError):
    """An experimental feature flag is not `true` or `false`."""

    def __init__(self, feature: str, value):
        super().__init__(
            f'{EXPERIMENTAL_KEY}.{feature} must be true or false; '
            f'got {value!r}')
        self.feature = feature
        self.value = value


@dataclass(frozen=True)
class ExperimentalFeatures:
    """The experimental features that the fleet config turns on."""

    lift_commands: bool = False


def experimental_features_from_config(
        config_yaml: dict) -> ExperimentalFeatures:
    """Return the experimental features, rejecting an invalid section.

    A misspelled key or a value that is not a boolean stops the adapter. If
    the adapter ignored them, the feature would stay off with no message.
    """
    section = config_yaml.get(EXPERIMENTAL_KEY)
    if section is None:
        return ExperimentalFeatures()
    if not isinstance(section, dict):
        raise ExperimentalSectionNotMappingError(section)

    unknown_keys = sorted(
        str(key) for key in section if key not in KNOWN_FEATURE_KEYS)
    if unknown_keys:
        raise UnknownExperimentalFeatureError(unknown_keys)

    if LIFT_COMMANDS_KEY not in section:
        return ExperimentalFeatures()

    lift_commands = section[LIFT_COMMANDS_KEY]
    if not isinstance(lift_commands, bool):
        raise ExperimentalFeatureNotBooleanError(
            LIFT_COMMANDS_KEY, lift_commands)
    return ExperimentalFeatures(lift_commands=lift_commands)
