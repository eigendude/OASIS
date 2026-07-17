################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from __future__ import annotations

from oasis_control.hardware.config import HostHardwareConfig
from oasis_control.hardware.config import MCUManagerConfig
from oasis_control.hardware.config import MCUManagerImplementation


def get_host_hardware_config(
    host_id: str,
    zone_id: str,
) -> HostHardwareConfig:
    """Return control-facing hardware for a smarthome host and deployment zone"""

    if not zone_id:
        raise ValueError("zone_id must not be empty")

    if host_id == "airlab":
        return HostHardwareConfig(enable_oled_visualizer=True)
    if host_id == "falcon":
        return HostHardwareConfig(
            enable_ahrs_speedometer=True,
            mcu_manager=MCUManagerConfig(
                node_name="engineer",
                implementation=MCUManagerImplementation.PWM,
                conductor_host="station",
            ),
        )
    if host_id == "jetson":
        return HostHardwareConfig(
            mcu_manager=MCUManagerConfig(
                node_name="engine",
                implementation=MCUManagerImplementation.STANDARD,
            )
        )
    if host_id == "station":
        return HostHardwareConfig(
            mcu_manager=MCUManagerConfig(
                node_name="conductor",
                implementation=MCUManagerImplementation.CONDUCTOR,
                input_provider="megapegasus",
                wol_server_id=host_id,
                motor_voltage_reversed=True,
            ),
            enable_wol_server=True,
        )

    return HostHardwareConfig()
