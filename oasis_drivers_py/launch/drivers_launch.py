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
from launch_ros.descriptions import ComposableNode

from oasis_drivers.hardware.config import HostHardwareConfig
from oasis_drivers.hardware.hosts import get_host_hardware_config
from oasis_drivers.launch.driver_descriptions import DriverDescriptions as Drivers
from oasis_drivers.launch.driver_nodes import DriverNodes
from oasis_home.utils.smarthome_config import SmarthomeConfig


def generate_launch_description() -> LaunchDescription:
    config: SmarthomeConfig = SmarthomeConfig()
    hardware: HostHardwareConfig = get_host_hardware_config(
        config.HOST_ID,
        config.ZONE_ID,
    )

    print(f"Launching drivers on {config.HOSTNAME} in zone {config.ZONE_ID}")

    ld: LaunchDescription = LaunchDescription()
    composable_nodes: list[ComposableNode] = []
    camera_composable_nodes: list[ComposableNode] = []

    Drivers.add_system_monitor(ld, config.HOST_ID)

    if config.HOST_ID in config.SMART_DISPLAY_ZONES:
        Drivers.add_display_server(ld, config.ZONE_ID)

    for camera in hardware.cameras:
        DriverNodes.add_camera(
            ld,
            camera_composable_nodes,
            camera,
            config.ZONE_ID,
            config.KINECT_V2_ZONE_ID,
        )

    if hardware.ahrs is not None:
        DriverNodes.add_ahrs(
            ld,
            composable_nodes,
            config.HOST_ID,
            hardware.ahrs,
        )

    if hardware.power_meter is not None:
        Drivers.add_power_meter_node(
            composable_nodes,
            config.HOST_ID,
            hardware.power_meter.as_parameters(),
        )

    if hardware.ssd1305_display is not None:
        Drivers.add_ssd1305_display_node(
            composable_nodes,
            config.HOST_ID,
            hardware.ssd1305_display.as_parameters(),
        )

    if hardware.mcu is not None:
        DriverNodes.add_mcu(ld, config.HOST_ID, hardware.mcu)

    driver_launch_prefix: str | None = None
    if hardware.ahrs is not None:
        driver_launch_prefix = hardware.ahrs.launch_prefix

    Drivers.add_driver_components(
        ld,
        config.HOST_ID,
        composable_nodes,
        log_level="info",
        launch_prefix=driver_launch_prefix,
    )
    Drivers.add_driver_components(
        ld,
        config.HOST_ID,
        camera_composable_nodes,
        log_level="info",
        container_name=f"camera_container_{config.HOST_ID}",
    )

    return ld
