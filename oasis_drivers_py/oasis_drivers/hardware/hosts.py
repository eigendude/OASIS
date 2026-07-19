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
from oasis_drivers.hardware.config import Ssd1305DisplayConfig


DEFAULT_IMAGE_SIZE: tuple[int, int] = (640, 480)


def get_host_hardware_config(
    host_id: str,
    zone_id: str,
) -> HostHardwareConfig:
    """Return installed hardware for a smarthome host and deployment zone"""

    if not zone_id:
        raise ValueError("zone_id must not be empty")

    if host_id == "airlab":
        # The TCA9548A mux address is fixed at 0x70 by the Raspberry Pi
        # device-tree overlay:
        #
        # dtoverlay=i2c-mux,pca9548,addr=0x70
        #
        # Linux exposes each mux channel as a separate child I2C adapter, so
        # the runtime driver only needs the parent bus and channel.
        #
        # Discover the parent I2C bus, mux child adapters, and attached
        # devices:
        #
        #   i2cdetect -l
        #
        # Scan the parent Raspberry Pi I2C bus:
        #
        #   i2cdetect -y 1
        #
        # List mux child adapters and their channel IDs:
        #
        #   for dev in /sys/class/i2c-dev/i2c-*; do
        #     bus="${dev##*-}"
        #     printf "i2c-%-3s " "${bus}"
        #     cat "${dev}/name"
        #   done
        #
        # Scan every TCA9548A mux channel:
        #
        #   for dev in /sys/class/i2c-dev/i2c-*; do
        #     bus="${dev##*-}"
        #     name="$(cat "${dev}/name")"
        #
        #     case "${name}" in
        #       *mux*)
        #         echo "=== i2c-${bus}: ${name} ==="
        #         i2cdetect -y "${bus}"
        #         ;;
        #     esac
        #   done
        #
        # Expected topology on airlab:
        #
        #   /dev/i2c-1
        #     0x3c  SSD1305 OLED
        #     0x68  DS3231 RTC, shown as UU when claimed by the kernel
        #     0x70  TCA9548A mux, shown as UU when claimed by the kernel
        #
        return HostHardwareConfig(
            ssd1305_display=Ssd1305DisplayConfig(
                i2c_device="/dev/i2c-1",
                i2c_address=0x3C,
                width=128,
                height=32,
                column_offset=4,
                contrast=0xFF,
                threshold=127,
                invert_pixels=False,
                rotation=0,
                update_rate_hz=45.0,
                reconnect_interval_sec=1.0,
                reconnect_settle_sec=0.5,
                enabled=True,
                blank_on_shutdown=True,
                reject_wrong_dimensions=True,
                clip_wrong_dimensions=False,
                enable_partial_updates=True,
            ),
        )
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
            ssd1305_display=Ssd1305DisplayConfig(
                i2c_device="/dev/i2c-1",
                i2c_address=0x3C,
                width=128,
                height=32,
                column_offset=4,
                contrast=0xD0,  # Brownouts start around 0xE0
                threshold=127,
                invert_pixels=False,
                rotation=0,
                update_rate_hz=10.0,
                reconnect_interval_sec=1.0,
                reconnect_settle_sec=0.5,
                enabled=True,
                blank_on_shutdown=True,
                reject_wrong_dimensions=True,
                clip_wrong_dimensions=False,
                enable_partial_updates=True,
            ),
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
