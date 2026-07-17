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

import os

from oasis_drivers.hardware.config import AhrsConfig
from oasis_drivers.hardware.config import AhrsMountingConfig
from oasis_drivers.hardware.config import Bno086Config
from oasis_drivers.hardware.config import CameraConfig
from oasis_drivers.hardware.config import CameraImplementation
from oasis_drivers.hardware.config import HostHardwareConfig
from oasis_drivers.hardware.config import LibcameraConfig
from oasis_drivers.hardware.config import MCUConfig
from oasis_drivers.hardware.config import MCUImplementation


DEFAULT_IMAGE_SIZE: tuple[int, int] = (640, 480)


def get_host_hardware_config(
    host_id: str,
    zone_id: str,
) -> HostHardwareConfig:
    """Return installed hardware for a smarthome host and deployment zone"""

    if not zone_id:
        raise ValueError("zone_id must not be empty")

    if host_id in ("bar", "kitchen"):
        return HostHardwareConfig(
            cameras=(
                CameraConfig(
                    implementation=CameraImplementation.V4L2,
                    image_size=DEFAULT_IMAGE_SIZE,
                    video_device="/dev/video0",
                ),
            )
        )
    if host_id == "falcon":
        return HostHardwareConfig(
            cameras=(
                CameraConfig(
                    implementation=CameraImplementation.LIBCAMERA,
                    enabled=False,
                    # TODO: This is inverted and produces BGR888 images
                    image_format="RGB888",
                    image_size=(1920, 1080),
                    # 2×2 binned 16:9 mode with full field of view that scales
                    # cleanly to 1080p
                    sensor_mode="2304:1296",
                    # Lower JPEG quality to cap Wi-Fi bandwidth from the Falcon
                    jpeg_quality=60,
                    camera_frame_id="camera_link",
                    # Low-blur, full-FOV binned mode at exactly 15 fps. The
                    # IMX708 2304×1296 minimum frame time is about 17,849 µs
                    libcamera=LibcameraConfig(
                        # Disable auto-exposure
                        ae_enable=False,
                        # 8 ms exposure time, approximately 1/125 second
                        exposure_time_usec=8000,
                        # Manual exposure mode, available since libcamera 0.5
                        exposure_time_mode=1,
                        # Equal 66.667 ms limits produce exactly 15.000 fps
                        frame_duration_limits_usec=(66667, 66667),
                    ),
                ),
            ),
            ahrs=AhrsConfig(
                # BNO086 interrupt line on the Falcon's GPIO header
                imu=Bno086Config(interrupt_gpio=23),
                # Default Falcon boot-time AHRS mounting calibration policy
                mounting=AhrsMountingConfig(
                    parent_frame_id="base_link",
                    child_frame_id="imu_link",
                    calibration_duration_sec=2.0,
                    stationary_angular_speed_threshold_rads=0.35,
                    min_sample_count=10,
                ),
                enable_zupt=True,
                enable_magnetometer=True,
                # Optional process prefix for deployments that grant scheduler
                # privileges through systemd and want the GPIO/I2C drain loop
                # to outrank perception work
                launch_prefix=os.getenv("OASIS_BNO086_LAUNCH_PREFIX") or None,
            ),
            mcu=MCUConfig(
                node_name="engineer",
                implementation=MCUImplementation.TELEMETRIX,
                serial_port="/dev/ttyACM0",
            ),
        )
    if host_id == "jetson":
        return HostHardwareConfig(
            mcu=MCUConfig(
                node_name="engine",
                implementation=MCUImplementation.TELEMETRIX,
                serial_port="/dev/ttyACM0",
            )
        )
    if host_id == "nas":
        return HostHardwareConfig(
            cameras=(
                CameraConfig(
                    implementation=CameraImplementation.KINECT_V2,
                ),
            )
        )
    if host_id == "station":
        return HostHardwareConfig(
            cameras=(
                CameraConfig(
                    implementation=CameraImplementation.LIBCAMERA,
                    image_format="BGR888",
                    image_size=DEFAULT_IMAGE_SIZE,
                    # Raspberry Pi Camera Module V2 full sensor resolution
                    sensor_mode="3280:2464",
                    libcamera=LibcameraConfig(
                        # 8 ms exposure time, approximately 1/125 second
                        exposure_time_usec=8000,
                        # Manual exposure mode, available since libcamera 0.5
                        exposure_time_mode=1,
                    ),
                ),
            ),
            mcu=MCUConfig(
                node_name="conductor",
                implementation=MCUImplementation.TELEMETRIX,
                serial_port="/dev/ttyACM0",
            ),
        )

    return HostHardwareConfig()
