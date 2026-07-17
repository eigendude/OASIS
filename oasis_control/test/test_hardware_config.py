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

import pytest

from oasis_control.hardware.config import HostHardwareConfig
from oasis_control.hardware.config import MCUManagerImplementation
from oasis_control.hardware.hosts import get_host_hardware_config


def test_airlab_enables_oled_visualization() -> None:
    hardware: HostHardwareConfig = get_host_hardware_config("airlab", "airlab_zone")

    assert hardware.enable_oled_visualizer is True
    assert hardware.mcu_manager is None


def test_falcon_enables_speedometer_and_pwm_manager() -> None:
    hardware: HostHardwareConfig = get_host_hardware_config("falcon", "falcon_zone")

    assert hardware.enable_ahrs_speedometer is True
    assert hardware.mcu_manager is not None
    assert hardware.mcu_manager.node_name == "engineer"
    assert hardware.mcu_manager.implementation is MCUManagerImplementation.PWM
    assert hardware.mcu_manager.conductor_host == "station"


def test_jetson_uses_standard_engine_manager() -> None:
    hardware: HostHardwareConfig = get_host_hardware_config("jetson", "jetson_zone")

    assert hardware.mcu_manager is not None
    assert hardware.mcu_manager.node_name == "engine"
    assert hardware.mcu_manager.implementation is MCUManagerImplementation.STANDARD


def test_station_uses_conductor_manager_and_wol_server() -> None:
    hardware: HostHardwareConfig = get_host_hardware_config("station", "station_zone")

    assert hardware.enable_wol_server is True
    assert hardware.mcu_manager is not None
    assert hardware.mcu_manager.node_name == "conductor"
    assert hardware.mcu_manager.implementation is MCUManagerImplementation.CONDUCTOR
    assert hardware.mcu_manager.input_provider == "megapegasus"
    assert hardware.mcu_manager.wol_server_id == "station"
    assert hardware.mcu_manager.motor_voltage_reversed is True


def test_unknown_host_has_empty_control_hardware() -> None:
    hardware: HostHardwareConfig = get_host_hardware_config("unknown", "zone")

    assert hardware == HostHardwareConfig()


def test_empty_zone_is_rejected() -> None:
    with pytest.raises(ValueError, match="zone_id must not be empty"):
        get_host_hardware_config("falcon", "")
