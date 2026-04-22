################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

# mypy: disable-error-code=import-not-found

"""Tests for AHRS launch-side mounting calibration configuration."""

from __future__ import annotations

from typing import Any

import pytest  # type: ignore[import-not-found]

from oasis_control.launch.ahrs_mounting import AhrsMountingConfig


def test_ahrs_mounting_config_uses_boot_calibration_defaults() -> None:
    config: AhrsMountingConfig = AhrsMountingConfig()

    assert config.parent_frame_id == "base_link"
    assert config.child_frame_id == "imu_link"
    assert config.calibration_duration_sec == 2.0
    assert config.stationary_angular_speed_threshold_rads == 0.35
    assert config.min_sample_count == 10


def test_ahrs_mounting_config_supports_custom_boot_policy() -> None:
    config: AhrsMountingConfig = AhrsMountingConfig(
        parent_frame_id="chassis",
        child_frame_id="imu_sensor",
        calibration_duration_sec=1.5,
        stationary_angular_speed_threshold_rads=0.1,
        min_sample_count=5,
    )

    assert config.parent_frame_id == "chassis"
    assert config.child_frame_id == "imu_sensor"
    assert config.calibration_duration_sec == 1.5
    assert config.stationary_angular_speed_threshold_rads == 0.1
    assert config.min_sample_count == 5


class _FakeNode:
    def __init__(self, **kwargs: Any) -> None:
        self.kwargs: dict[str, Any] = kwargs


class _FakeLaunchDescription:
    def __init__(self) -> None:
        self.actions: list[_FakeNode] = []

    def add_action(self, action: _FakeNode) -> None:
        self.actions.append(action)


def test_add_ahrs_node_configures_boot_mounting_calibration_params(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import oasis_control.launch.control_descriptions as control_descriptions

    monkeypatch.setattr(control_descriptions, "Node", _FakeNode)

    launch_description = _FakeLaunchDescription()
    mounting_config = AhrsMountingConfig(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        calibration_duration_sec=2.0,
        stationary_angular_speed_threshold_rads=0.2,
        min_sample_count=12,
    )

    control_descriptions.ControlDescriptions.add_ahrs_node(
        launch_description,
        "falcon",
        mounting_config=mounting_config,
    )

    assert len(launch_description.actions) == 1

    ahrs_action = launch_description.actions[0].kwargs
    assert ahrs_action["package"] == "oasis_control"
    assert ahrs_action["executable"] == "ahrs"
    assert ahrs_action["parameters"] == [
        {
            "base_frame_id": "base_link",
            "imu_frame_id": "imu_link",
            "mounting_calibration_duration_sec": 2.0,
            "mounting_stationary_angular_speed_threshold_rads": 0.2,
            "mounting_min_sample_count": 12,
        }
    ]
    assert ("accel", "falcon/accel") in ahrs_action["remappings"]
    assert ("ahrs/accel", "falcon/ahrs/accel") in ahrs_action["remappings"]
    assert ("ahrs/gravity", "falcon/ahrs/gravity") in ahrs_action["remappings"]
