################################################################################
#
#  Copyright (C) 2021-2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from launch.launch_description import LaunchDescription

from oasis_control.hardware.config import HostHardwareConfig
from oasis_control.hardware.config import MCUManagerConfig
from oasis_control.hardware.config import MCUManagerImplementation
from oasis_control.hardware.hosts import get_host_hardware_config
from oasis_control.launch.control_descriptions import ControlDescriptions
from oasis_drivers.launch.driver_descriptions import DriverDescriptions as Drivers
from oasis_home.utils.smarthome_config import SmarthomeConfig


def generate_launch_description() -> LaunchDescription:
    config: SmarthomeConfig = SmarthomeConfig()
    hardware: HostHardwareConfig = get_host_hardware_config(
        config.HOST_ID,
        config.ZONE_ID,
    )

    print(f"Launching control on {config.HOSTNAME} in zone {config.ZONE_ID}")

    ld: LaunchDescription = LaunchDescription()

    # Smarthome nodes
    if config.HOST_ID == config.HOME_ASSISTANT_ID:
        ControlDescriptions.add_home_manager(
            ld,
            config.SMART_DISPLAY_ZONES,
            config.SMART_DISPLAY_PLUG_ID,
            config.CAMERA_ZONES,
            config.HOME_ASSISTANT_ID,
        )

    # Navigation
    if hardware.enable_ahrs_speedometer:
        ControlDescriptions.add_ahrs_speedometer(ld, config.HOST_ID)

    # Microcontroller nodes
    mcu_manager: MCUManagerConfig | None = hardware.mcu_manager
    if (
        mcu_manager is not None
        and mcu_manager.implementation is MCUManagerImplementation.PWM
    ):
        ControlDescriptions.add_engineer_manager(
            ld,
            config.HOST_ID,
            mcu_manager.node_name,
            conductor_host=mcu_manager.conductor_host or "",
        )
    elif (
        mcu_manager is not None
        and mcu_manager.implementation is MCUManagerImplementation.STANDARD
    ):
        ControlDescriptions.add_engine_manager(
            ld,
            config.HOST_ID,
            mcu_manager.node_name,
        )
    elif (
        mcu_manager is not None
        and mcu_manager.implementation is MCUManagerImplementation.CONDUCTOR
    ):
        ControlDescriptions.add_conductor_manager(
            ld,
            config.HOST_ID,
            mcu_manager.node_name,
            mcu_manager.wol_server_id or config.HOST_ID,
            mcu_manager.input_provider or "",
            motor_voltage_reversed=mcu_manager.motor_voltage_reversed,
        )

    if hardware.enable_wol_server:
        Drivers.add_wol_server(ld, config.HOST_ID)

    return ld
