################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the deployed ZUPT component launch contract."""

from __future__ import annotations

from typing import Any
from unittest.mock import patch

from launch.launch_description import LaunchDescription
from launch_ros.descriptions import ComposableNode

from oasis_drivers.hardware.config import AhrsConfig
from oasis_drivers.hardware.config import AhrsMountingConfig
from oasis_drivers.hardware.config import Bno086Config
from oasis_drivers.launch.driver_descriptions import DriverDescriptions
from oasis_drivers.launch.driver_nodes import DriverNodes


def test_zupt_component_uses_configured_imu_child_frame() -> None:
    composable_nodes: list[ComposableNode] = []

    DriverDescriptions.add_zupt_detector(
        composable_nodes, host_id="falcon", imu_frame_id="sensor_imu_link"
    )

    assert len(composable_nodes) == 1
    node: ComposableNode = composable_nodes[0]
    node_arguments: dict[str, Any] = getattr(node, "kwargs")
    parameters: dict[str, object] = node_arguments["parameters"][0]
    remappings: dict[str, str] = dict(node_arguments["remappings"])
    assert parameters["imu_frame_id"] == "sensor_imu_link"
    assert remappings == {
        "imu": "falcon/imu",
        "zupt": "falcon/zupt",
        "zupt_flag": "falcon/zupt_flag",
    }


def test_ahrs_launch_passes_mounting_child_frame_to_zupt() -> None:
    mounting: AhrsMountingConfig = AhrsMountingConfig(
        parent_frame_id="vehicle_base",
        child_frame_id="sensor_imu_link",
        calibration_duration_sec=1.0,
        stationary_angular_speed_threshold_rads=0.1,
        min_sample_count=10,
    )
    ahrs: AhrsConfig = AhrsConfig(
        imu=Bno086Config(interrupt_gpio=17),
        mounting=mounting,
        enable_magnetometer=False,
    )

    with (
        patch.object(DriverDescriptions, "add_bno086_imu"),
        patch.object(DriverDescriptions, "add_ahrs_node"),
        patch.object(DriverDescriptions, "add_zupt_detector") as add_zupt,
    ):
        DriverNodes.add_ahrs(
            ld=LaunchDescription(),
            composable_nodes=[],
            host_id="falcon",
            ahrs=ahrs,
        )

    add_zupt.assert_called_once_with([], "falcon", "sensor_imu_link")
