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

from oasis_drivers.hardware.config import AhrsConfig
from oasis_drivers.hardware.config import CameraConfig
from oasis_drivers.hardware.config import CameraImplementation
from oasis_drivers.hardware.config import MCUConfig
from oasis_drivers.hardware.config import MCUImplementation
from oasis_drivers.launch.driver_descriptions import DriverDescriptions as Drivers
from oasis_drivers.launch.mcu_descriptions import MCUDescriptions as MCU


class DriverNodes:
    @staticmethod
    def add_camera(
        ld: LaunchDescription,
        camera_composable_nodes: list[ComposableNode],
        camera: CameraConfig,
        zone_id: str,
        kinect_v2_zone_id: str,
    ) -> None:
        """Add launch actions for an enabled camera"""

        if not camera.enabled:
            return

        if camera.implementation is CameraImplementation.LIBCAMERA:
            if camera.image_format is None or camera.sensor_mode is None:
                raise ValueError("libcamera format and sensor mode are required")

            libcamera_parameters: dict[str, object] | None = None
            if camera.libcamera is not None:
                libcamera_parameters = camera.libcamera.as_parameters()

            Drivers.add_ros2_camera(
                camera_composable_nodes,
                zone_id,
                camera.image_format,
                camera.image_size_list(),
                camera.sensor_mode,
                jpeg_quality=camera.jpeg_quality,
                camera_frame_id=camera.camera_frame_id,
                libcamera_params=libcamera_parameters,
            )
        elif camera.implementation is CameraImplementation.V4L2:
            if camera.video_device is None:
                raise ValueError("V4L2 video device is required")

            Drivers.add_v4l2_camera(
                ld,
                zone_id,
                camera.video_device,
                camera.image_size_list(),
            )
        elif camera.implementation is CameraImplementation.KINECT_V2:
            Drivers.add_kinect_v2(ld, kinect_v2_zone_id)

    @staticmethod
    def add_ahrs(
        ld: LaunchDescription,
        composable_nodes: list[ComposableNode],
        host_id: str,
        ahrs: AhrsConfig,
    ) -> None:
        """Add the configured AHRS hardware as one cohesive unit"""

        Drivers.add_bno086_imu(
            composable_nodes,
            host_id,
            ahrs.imu.interrupt_gpio,
        )
        Drivers.add_ahrs_node(composable_nodes, host_id, ahrs.mounting)
        if ahrs.enable_zupt:
            Drivers.add_zupt_detector(
                composable_nodes, host_id, ahrs.mounting.child_frame_id
            )
        if ahrs.enable_magnetometer:
            Drivers.add_mmc5983ma_magnetometer(ld, host_id)

    @staticmethod
    def add_mcu(
        ld: LaunchDescription,
        host_id: str,
        mcu: MCUConfig,
    ) -> None:
        """Add the configured serial microcontroller bridge"""

        if mcu.implementation is MCUImplementation.FIRMATA:
            MCU.add_firmata_bridge(
                ld,
                host_id,
                mcu.node_name,
                mcu.serial_port,
            )
        elif mcu.implementation is MCUImplementation.TELEMETRIX:
            MCU.add_telemetrix_bridge(
                ld,
                host_id,
                mcu.node_name,
                mcu.serial_port,
            )
