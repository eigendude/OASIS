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

from dataclasses import dataclass
from enum import Enum


class CameraImplementation(Enum):
    """Driver implementation used for an installed camera"""

    # Raspberry Pi libcamera driver exposed by camera_ros
    LIBCAMERA = "libcamera"

    # Kinect V2 bridge
    KINECT_V2 = "kinect_v2"

    # Linux Video4Linux2 camera driver
    V4L2 = "v4l2"


@dataclass(frozen=True)
class LibcameraConfig:
    """Typed controls passed to a libcamera-backed camera"""

    # Exposure duration in microseconds
    exposure_time_usec: int

    # libcamera exposure mode where 1 selects manual exposure
    exposure_time_mode: int

    # Whether libcamera auto-exposure is explicitly enabled or disabled
    ae_enable: bool | None = None

    # Minimum and maximum frame durations in microseconds
    frame_duration_limits_usec: tuple[int, int] | None = None

    def as_parameters(self) -> dict[str, object]:
        """Convert the controls to camera_ros parameter names and values"""

        parameters: dict[str, object] = {
            "ExposureTime": self.exposure_time_usec,
            "ExposureTimeMode": self.exposure_time_mode,
        }
        if self.ae_enable is not None:
            parameters["AeEnable"] = self.ae_enable
        if self.frame_duration_limits_usec is not None:
            parameters["FrameDurationLimits"] = list(self.frame_duration_limits_usec)
        return parameters


@dataclass(frozen=True)
class CameraConfig:
    """Configuration for one camera physically installed on a host"""

    # Driver implementation used to communicate with the camera
    implementation: CameraImplementation

    # Whether the installed camera should currently be launched
    enabled: bool = True

    # Pixel encoding requested from the camera driver
    image_format: str | None = None

    # Output image width and height in pixels
    image_size: tuple[int, int] | None = None

    # Sensor width and height mode in camera_ros format
    sensor_mode: str | None = None

    # Frame ID attached to camera messages
    camera_frame_id: str | None = None

    # Compressed image quality in the inclusive range 0 through 100
    jpeg_quality: int | None = None

    # Controls forwarded to the libcamera API
    libcamera: LibcameraConfig | None = None

    # Linux device path used by a V4L2 camera
    video_device: str | None = None

    def image_size_list(self) -> list[int]:
        """Return the immutable image dimensions in the driver API format"""

        if self.image_size is None:
            raise ValueError("camera image size is not configured")
        return list(self.image_size)


@dataclass(frozen=True)
class Bno086Config:
    """Configuration for a BNO086 inertial measurement unit"""

    # BCM GPIO number connected to the active-low interrupt output
    interrupt_gpio: int


@dataclass(frozen=True)
class AhrsMountingConfig:
    """Stationary boot calibration policy for a fixed AHRS mounting"""

    # Parent frame for the fixed mounting transform
    parent_frame_id: str

    # Child frame for the fixed mounting transform
    child_frame_id: str

    # Duration of the stationary calibration window in seconds
    calibration_duration_sec: float

    # Maximum stationary angular speed in radians per second
    stationary_angular_speed_threshold_rads: float

    # Minimum number of samples required to solve the mounting transform
    min_sample_count: int


@dataclass(frozen=True)
class AhrsConfig:
    """Cohesive inertial hardware and processing configuration"""

    # Physical inertial measurement unit supplying AHRS samples
    imu: Bno086Config

    # Fixed mounting calibration policy used by the AHRS node
    mounting: AhrsMountingConfig

    # Whether to launch the zero-velocity update detector
    enable_zupt: bool = True

    # Whether an MMC5983MA magnetometer is physically installed
    enable_magnetometer: bool = True

    # Optional process prefix for the component container hosting the IMU
    launch_prefix: str | None = None


class MCUImplementation(Enum):
    """Protocol implementation used to communicate with an MCU"""

    # Standard Firmata serial protocol
    FIRMATA = "firmata"

    # Telemetrix serial protocol
    TELEMETRIX = "telemetrix"


@dataclass(frozen=True)
class MCUConfig:
    """Configuration for one serial microcontroller bridge"""

    # Logical MCU node name used in ROS node and topic names
    node_name: str

    # Serial protocol implementation used by the attached MCU
    implementation: MCUImplementation

    # Linux serial device connected to the MCU
    serial_port: str


@dataclass(frozen=True)
class PowerMeterConfig:
    """Configuration for ACS37800 power meters behind an I2C mux"""

    # Measurement publication frequency in hertz
    publish_rate_hz: float

    # Seven-bit TCA9548A I2C mux address
    mux_address: int

    # Seven-bit ACS37800 I2C address used on each mux channel
    power_meter_address: int

    # Stable logical IDs assigned to meters in mux-channel order
    power_meter_ids: tuple[str, ...]

    # Full-scale current measurement range in amperes
    current_sense_range_amps: float

    # Series resistance on each voltage input leg in ohms
    voltage_divider_resistance_ohms: float

    # Voltage-sense resistor value in ohms
    voltage_sense_resistance_ohms: float

    # Expected crs_sns register value used to validate hardware setup
    expected_crs_sns: int

    # Consecutive communication failures allowed before disconnection
    disconnect_after_failures: int

    def as_parameters(self) -> dict[str, object]:
        """Convert the configuration to ROS parameter names and values"""

        voltage_divider_resistance: float = self.voltage_divider_resistance_ohms
        return {
            "publish_rate_hz": self.publish_rate_hz,
            "mux_address": self.mux_address,
            "power_meter_address": self.power_meter_address,
            "power_meter_ids": list(self.power_meter_ids),
            "current_sense_range_amps": self.current_sense_range_amps,
            "voltage_divider_resistance_ohms": voltage_divider_resistance,
            "voltage_sense_resistance_ohms": self.voltage_sense_resistance_ohms,
            "expected_crs_sns": self.expected_crs_sns,
            "disconnect_after_failures": self.disconnect_after_failures,
        }


@dataclass(frozen=True)
class Ssd1305DisplayConfig:
    """Configuration for a directly attached SSD1305 monochrome OLED"""

    # Linux I2C device path connected directly to the OLED
    i2c_device: str

    # Seven-bit SSD1305 I2C address
    i2c_address: int

    # Hardware validation width; Product 4567 is fixed at 128 pixels
    width: int

    # Hardware validation height; Product 4567 is fixed at 32 pixels
    height: int

    # GDDRAM column offset for a 128-column panel on the 132-column SSD1305
    column_offset: int

    # Display contrast register value in the inclusive range 0 through 255
    contrast: int

    # Monochrome conversion threshold in the inclusive range 0 through 255
    threshold: int

    # Whether thresholded framebuffer pixels are inverted
    invert_pixels: bool

    # Clockwise rotation in degrees, one of 0, 90, 180, or 270
    rotation: int

    # Maximum hardware update rate in hertz
    update_rate_hz: float

    # Consecutive I2C failures before reopening and reinitializing the device
    recover_after_failures: int

    # Requested display power state at node startup
    enabled: bool

    # Whether GDDRAM is cleared before DISPLAY_OFF during shutdown
    blank_on_shutdown: bool

    # Whether images with dimensions incompatible with rotation are rejected
    reject_wrong_dimensions: bool

    # Whether non-matching source images are clipped instead of rejected
    clip_wrong_dimensions: bool

    # Whether updates transmit only pages differing from confirmed hardware
    enable_partial_updates: bool

    def as_parameters(self) -> dict[str, object]:
        """Convert the configuration to ROS parameter names and values"""

        return {
            "i2c_device": self.i2c_device,
            "i2c_address": self.i2c_address,
            "width": self.width,
            "height": self.height,
            "column_offset": self.column_offset,
            "contrast": self.contrast,
            "threshold": self.threshold,
            "invert_pixels": self.invert_pixels,
            "rotation": self.rotation,
            "update_rate_hz": self.update_rate_hz,
            "recover_after_failures": self.recover_after_failures,
            "enabled": self.enabled,
            "blank_on_shutdown": self.blank_on_shutdown,
            "reject_wrong_dimensions": self.reject_wrong_dimensions,
            "clip_wrong_dimensions": self.clip_wrong_dimensions,
            "enable_partial_updates": self.enable_partial_updates,
        }


@dataclass(frozen=True)
class HostHardwareConfig:
    """Complete physical hardware configuration for one OASIS host"""

    # Cameras physically installed on the host
    cameras: tuple[CameraConfig, ...] = ()

    # Optional cohesive AHRS hardware and processing configuration
    ahrs: AhrsConfig | None = None

    # Optional serial microcontroller bridge configuration
    mcu: MCUConfig | None = None

    # Optional ACS37800 power meter installation
    power_meter: PowerMeterConfig | None = None

    # Optional directly attached SSD1305 OLED display
    ssd1305_display: Ssd1305DisplayConfig | None = None
