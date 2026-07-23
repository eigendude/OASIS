################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for OASIS control launch descriptions."""

from __future__ import annotations

from typing import Any

from launch.launch_description import LaunchDescription

from oasis_control.launch.control_descriptions import ControlDescriptions


def test_conductor_measurement_remappings_resolve_under_oasis_station() -> None:
    launch_description: LaunchDescription = LaunchDescription()

    ControlDescriptions.add_conductor_manager(
        launch_description,
        host_id="station",
        mcu_node="conductor",
        wol_server_id="station",
        input_provider="station",
        camera_zone="station",
    )

    launch_description_any: Any = launch_description
    conductor_node: Any = launch_description_any.actions[0]
    remappings: dict[str, str] = dict(conductor_node.kwargs["remappings"])
    namespace: str = conductor_node.kwargs["namespace"]

    assert f"/{namespace}/{remappings['supply_voltage']}" == (
        "/oasis/station/supply_voltage"
    )
    assert f"/{namespace}/{remappings['traction_voltage']}" == (
        "/oasis/station/traction_voltage"
    )
    assert f"/{namespace}/{remappings['traction_power']}" == (
        "/oasis/station/traction_power"
    )
    assert f"/{namespace}/{remappings['power_meter_0']}" == (
        "/oasis/station/power/power_meter_0"
    )
    assert f"/{namespace}/{remappings['power_meter_1']}" == (
        "/oasis/station/power/power_meter_1"
    )
