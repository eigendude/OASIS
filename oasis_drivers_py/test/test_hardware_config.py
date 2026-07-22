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

from oasis_drivers.hardware.config import CameraConfig
from oasis_drivers.hardware.config import CameraImplementation
from oasis_drivers.hardware.config import HostHardwareConfig
from oasis_drivers.hardware.config import MCUImplementation
from oasis_drivers.hardware.hosts import get_host_hardware_config
from oasis_home.utils.smarthome_config import SmarthomeConfig
from oasis_home.utils.smarthome_config import normalize_hostname


def test_falcon_hardware_preserves_camera_and_ahrs_configuration(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("OASIS_BNO086_LAUNCH_PREFIX", "chrt --fifo 50")

    hardware: HostHardwareConfig = get_host_hardware_config("falcon", "falcon_zone")

    assert len(hardware.cameras) == 1
    camera: CameraConfig = hardware.cameras[0]
    assert camera.implementation is CameraImplementation.LIBCAMERA
    assert camera.image_format == "RGB888"
    assert camera.image_size == (1920, 1080)
    assert camera.sensor_mode == "2304:1296"
    assert camera.jpeg_quality == 60
    assert camera.camera_frame_id == "camera_link"
    assert camera.libcamera is not None
    assert camera.libcamera.as_parameters() == {
        "AeEnable": False,
        "ExposureTime": 8000,
        "ExposureTimeMode": 1,
        "FrameDurationLimits": [66667, 66667],
    }

    assert hardware.ahrs is not None
    assert hardware.ahrs.imu.interrupt_gpio == 23
    assert hardware.ahrs.mounting.parent_frame_id == "base_link"
    assert hardware.ahrs.mounting.child_frame_id == "imu_link"
    assert hardware.ahrs.mounting.calibration_duration_sec == 2.0
    assert hardware.ahrs.mounting.stationary_angular_speed_threshold_rads == 0.35
    assert hardware.ahrs.mounting.min_sample_count == 10
    assert hardware.ahrs.enable_zupt is True
    assert hardware.ahrs.enable_magnetometer is True
    assert hardware.ahrs.launch_prefix == "chrt --fifo 50"

    assert hardware.mcu is not None
    assert hardware.mcu.node_name == "engineer"
    assert hardware.mcu.implementation is MCUImplementation.TELEMETRIX
    assert hardware.mcu.serial_port == "/dev/ttyACM0"


def test_station_hardware_preserves_camera_and_mcu_configuration() -> None:
    hardware: HostHardwareConfig = get_host_hardware_config("station", "station_zone")

    assert len(hardware.cameras) == 1
    camera: CameraConfig = hardware.cameras[0]
    assert camera.implementation is CameraImplementation.LIBCAMERA
    assert camera.enabled is True
    assert camera.image_format == "BGR888"
    assert camera.image_size == (640, 480)
    assert camera.sensor_mode == "3280:2464"
    assert camera.libcamera is not None
    assert camera.libcamera.as_parameters() == {
        "ExposureTime": 8000,
        "ExposureTimeMode": 1,
    }

    assert hardware.mcu is not None
    assert hardware.mcu.node_name == "conductor"
    assert hardware.mcu.implementation is MCUImplementation.TELEMETRIX


@pytest.mark.parametrize("host_id", ["bar", "kitchen"])
def test_v4l2_hosts_use_the_default_video_device(host_id: str) -> None:
    hardware: HostHardwareConfig = get_host_hardware_config(host_id, f"{host_id}_zone")

    assert len(hardware.cameras) == 1
    camera: CameraConfig = hardware.cameras[0]
    assert camera.implementation is CameraImplementation.V4L2
    assert camera.video_device == "/dev/video0"
    assert camera.image_size == (640, 480)


def test_nas_has_kinect_v2_camera_without_storing_deployment_zone() -> None:
    hardware: HostHardwareConfig = get_host_hardware_config(
        "nas", "logical_kinect_zone"
    )

    assert len(hardware.cameras) == 1
    assert hardware.cameras[0].implementation is CameraImplementation.KINECT_V2


def test_station_power_meter_parameters_preserve_driver_values() -> None:
    hardware: HostHardwareConfig = get_host_hardware_config("airlab", "airlab_zone")

    assert hardware.ssd1305_display is not None
    assert hardware.ssd1305_display.as_parameters() == {
        "i2c_device": "/dev/i2c-1",
        "i2c_address": 0x3C,
        "width": 128,
        "height": 32,
        "column_offset": 4,
        "contrast": 0xFF,
        "threshold": 127,
        "invert_pixels": False,
        "rotation": 0,
        "update_rate_hz": 45.0,
        "reconnect_interval_sec": 1.0,
        "display_power_settle_sec": 0.25,
        "enabled": True,
        "blank_on_shutdown": True,
        "reject_wrong_dimensions": True,
        "clip_wrong_dimensions": False,
        "enable_partial_updates": True,
    }

    hardware = get_host_hardware_config("station", "airlab_zone")
    assert hardware.power_meter is not None
    assert hardware.power_meter.power_meter_ids == (
        "power_meter_0",
        "power_meter_1",
    )
    assert hardware.power_meter.as_parameters() == {
        "publish_rate_hz": 10.0,
        "mux_address": 0x70,
        "power_meter_address": 0x60,
        "power_meter_ids": ["power_meter_0", "power_meter_1"],
        "current_sense_range_amps": 30.0,
        "voltage_divider_resistance_ohms": 2_000_000.0,
        "voltage_sense_resistance_ohms": 8_200.0,
        "expected_crs_sns": 4,
        "disconnect_after_failures": 3,
        "filter_length": 3,
    }


def test_raw_abn_hostname_resolves_to_airlab_display(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    normalized_hostname: str = normalize_hostname("abn-007000")
    assert normalized_hostname == "abn_007000"

    monkeypatch.setattr(SmarthomeConfig, "HOSTNAME", normalized_hostname)
    config: SmarthomeConfig = SmarthomeConfig()
    assert config.HOST_ID == "airlab"

    hardware: HostHardwareConfig = get_host_hardware_config(
        config.HOST_ID,
        config.ZONE_ID,
    )
    assert hardware.ssd1305_display is not None
    assert hardware.ssd1305_display.i2c_device == "/dev/i2c-1"
    assert hardware.ssd1305_display.i2c_address == 0x3C


@pytest.mark.parametrize("host_id", ["door", "unknown"])
def test_hosts_without_driver_hardware_are_empty(host_id: str) -> None:
    hardware: HostHardwareConfig = get_host_hardware_config(host_id, "zone")

    assert hardware.cameras == ()
    assert hardware.ahrs is None
    assert hardware.mcu is None
    assert hardware.power_meter is None
